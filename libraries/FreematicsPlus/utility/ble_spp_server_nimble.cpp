/*
 * ble_spp_server_nimble.cpp
 *
 * ============================================================================
 *  BLE SPP server implementation, built against h2zero/NimBLE-Arduino.
 *  The sole BLE backend as of 2026-09-21 - replaced the classic ESP-IDF
 *  Bluedroid implementation (ble_spp_server.c/.h, removed) for a smaller
 *  RAM footprint that coexists better with WiFi. Verified working on real
 *  hardware with the Freematics Controller phone app 2026-09-21 (device
 *  advertises, connects, command round-trip confirmed).
 *
 *  Same service/characteristic UUIDs as the old Bluedroid implementation
 *  (SERVICE 0xABF0, COMMAND characteristic 0xFFE1 read/write feeding
 *  ble_recv_command()'s queue, STATUS characteristic 0xFFE2 read/notify
 *  written by ble_send_response()) so the phone app needed no changes.
 *
 *  h2zero/NimBLE-Arduino is pinned to the 1.4.x release line (see
 *  platformio.ini's lib_deps comment for why - the last line with the
 *  "classic" BLEDevice/BLEServer/BLECharacteristic API this file uses,
 *  matching this project's arduino-esp32 2.x/IDF 4.4.x platform pin; the
 *  2.x library line renames everything with an Nim prefix and targets
 *  IDF 5.x/arduino-esp32 3.x - do not bump without re-adapting this file).
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <NimBLEDevice.h>
#include "ble_spp_server_nimble.h"

// Command queue depth matches the original's xQueueCreate(4, sizeof(void*)).
#define SPP_CMD_QUEUE_DEPTH 4

static NimBLEServer*         g_server     = nullptr;
static NimBLECharacteristic* g_statusChar = nullptr;
static QueueHandle_t         g_cmdQueue   = nullptr;

// 2026-09-21: pause/resume support (see ble_pause()/ble_resume() below).
// g_advName is saved so ble_resume() can recreate the server/advertising
// with the same name without the caller having to pass it again.
static char     g_advName[32]        = {0};
static bool     g_bleInitialized     = false; // ble_init() has run at least once
static bool     g_blePaused          = false;
static uint32_t g_blePauseStartMs    = 0;

// 2026-09-22: real mutex serializing ble_pause()/ble_resume(), added after a
// SECOND boot-loop of the exact same class as the original 2026-09-21 race
// (see ble_resume()'s own comment below). The 2026-09-21 fix only narrowed
// the race window (clear g_blePaused before the slow re-init work) - it did
// not close it, since two tasks can still genuinely execute the check-then-
// clear on different CPU cores at the same instant. A THIRD independent
// caller was added the night of 2026-09-22 (a periodic watchdog, see
// telelogger.ino's SIGNAL_CHECK_INTERVAL block) specifically to catch a
// resume that got missed - but every additional concurrent caller only
// raises the odds of hitting that same narrow window, which is exactly what
// then happened (confirmed via a live serial capture: BLE worked briefly
// after a fresh boot, then went permanently unreachable, with the device's
// own uptime counter showing a low value consistent with a fresh crash-
// reboot, not a long-running session). A FreeRTOS mutex (not a
// portENTER_CRITICAL spinlock - the re-init work below yields/blocks, which
// a spinlock must never be held across) makes the caller count irrelevant:
// whichever task gets there first runs pause/resume to completion before
// any other caller's check can even begin.
// Created once, synchronously, inside ble_init() (single-threaded, runs
// from setup() before the telemetry task or any WiFi event callback could
// possibly call pause/resume) - deliberately NOT lazily created on first
// use, which would itself be a race if two tasks called pause/resume for
// the first time concurrently.
static SemaphoreHandle_t g_blePauseMutex = nullptr;

// ---------------------------------------------------------------------------
// Server-level connect/disconnect: original's ESP_GATTS_CONNECT_EVT /
// ESP_GATTS_DISCONNECT_EVT handlers just tracked is_connected/spp_conn_id
// and, on disconnect, restarted advertising so the phone can reconnect
// without a device reboot. NimBLE-Arduino does NOT auto-restart advertising
// on disconnect by default, so that restart has to be done explicitly here
// - dropping it would silently turn "phone app disconnects" into "BLE is
// now unreachable until reboot", a regression from the original.
// ---------------------------------------------------------------------------
class SppServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        Serial.printf("[BLE] client connected (peers=%u)\n", (unsigned)pServer->getConnectedCount());
    }
    // 2026-09-23: diagnostic-only addition (no behavior change) - printing
    // peer count and post-restart advertising state to find out why
    // advertising sometimes doesn't come back after a mid-session disconnect
    // (observed a 69s gap with zero reconnect and a calm WiFi link, so it's
    // neither the ble_pause()/ble_resume() cycle nor WiFi coexistence).
    void onDisconnect(BLEServer* pServer) override {
        Serial.printf("[BLE] client disconnected (peers=%u) - restarting advertising\n",
                       (unsigned)pServer->getConnectedCount());
        BLEDevice::startAdvertising();
        Serial.printf("[BLE] advertising active after restart: %d\n",
                       (int)NimBLEDevice::getAdvertising()->isAdvertising());
    }
};

// ---------------------------------------------------------------------------
// Command characteristic (UUID 0xFFE1): phone writes an AT-style command
// string here (e.g. "UPTIME", "APN=internet"); telelogger.ino's
// processBLE() reads it back out via ble_recv_command(). Mirrors the
// original ESP_GATTS_WRITE_EVT handler's malloc + xQueueSend +
// drop-oldest-on-full behaviour exactly, including that ble_recv_command()'s
// caller (processBLE()) owns and must free the returned pointer.
// ---------------------------------------------------------------------------
class CommandCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChar) override {
        std::string value = pChar->getValue();
        if (value.empty()) return;

        char* cmd = (char*)malloc(value.length() + 1);
        if (!cmd) {
            Serial.println("[BLE] command malloc failed");
            return;
        }
        memcpy(cmd, value.data(), value.length());
        cmd[value.length()] = 0;

        if (!g_cmdQueue) {
            free(cmd);
            return;
        }
        if (xQueueSend(g_cmdQueue, &cmd, 0) == errQUEUE_FULL) {
            // Queue full: drop the oldest pending command (free it) and
            // retry, same policy as the original.
            char* stale = nullptr;
            if (xQueueReceive(g_cmdQueue, &stale, 0) == pdTRUE && stale) {
                free(stale);
            }
            xQueueSend(g_cmdQueue, &cmd, 0);
        }
    }

    void onRead(BLECharacteristic* pChar) override {
        // Mirrors the original's ESP_GATTS_READ_EVT case, which responded
        // to a read of the COMMAND characteristic with a literal "OK".
        // Kept only for exact behavioural parity with the Bluedroid
        // version - it's unclear whether the phone app actually relies on
        // reading this characteristic (real responses go over the STATUS
        // characteristic via notify, through ble_send_response()).
        pChar->setValue("OK");
    }
};

// Builds the GATT server/service/characteristics and starts advertising.
// Factored out of ble_init() so ble_resume() (see below) can run the exact
// same sequence again after a pause/resume cycle, instead of duplicating it
// - every field this touches (g_server, g_statusChar, the service/
// characteristic objects, the advertising object) is freshly (re)created by
// NimBLEDevice::init() having just been called, so there is never a stale
// object left over from a previous cycle to worry about (see ble_pause()'s
// comment for why deinit(true), not deinit(false), is used).
static void bleCreateGattServerAndAdvertise(const char* name)
{
    g_server = BLEDevice::createServer();
    g_server->setCallbacks(new SppServerCallbacks());

    // Service UUID 0xABF0 - same 16-bit UUID as the original's
    // spp_service_uuid, so it expands to the same 128-bit Bluetooth-Base
    // UUID the phone app already looks for.
    BLEService* service = g_server->createService(BLEUUID((uint16_t)0xABF0));

    // Command characteristic, UUID 0xFFE1 (ESP_GATT_UUID_SPP_COMMAND_RECEIVE
    // in the original) - read + write + write-without-response, matching
    // char_prop_read_write there.
    BLECharacteristic* cmdChar = service->createCharacteristic(
        BLEUUID((uint16_t)0xFFE1),
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    cmdChar->setCallbacks(new CommandCharCallbacks());

    // Status characteristic, UUID 0xFFE2 (ESP_GATT_UUID_SPP_COMMAND_NOTIFY
    // in the original) - read + notify, matching char_prop_read_notify
    // there. NimBLE-Arduino auto-attaches a CCCD (0x2902) to any
    // NOTIFY/INDICATE characteristic automatically - no separate BLE2902
    // descriptor object needed (that's the older esp32-BLE-Arduino
    // library's pattern, not NimBLE-Arduino's).
    g_statusChar = service->createCharacteristic(
        BLEUUID((uint16_t)0xFFE2),
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    service->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(service->getUUID());
    // Numerically matches the original's spp_adv_params
    // (adv_int_min=0x20, adv_int_max=0x40, both 0.625ms units).
    adv->setMinInterval(0x20);
    adv->setMaxInterval(0x40);
    BLEDevice::startAdvertising();

    Serial.printf("[BLE] NimBLE SPP-like server '%s' advertising\n", name);
}

void ble_init(const char* adv_name)
{
    const char* name = (adv_name && *adv_name) ? adv_name : "FreematicsPlus";
    strncpy(g_advName, name, sizeof(g_advName) - 1);
    g_advName[sizeof(g_advName) - 1] = 0;

    if (!g_cmdQueue) {
        g_cmdQueue = xQueueCreate(SPP_CMD_QUEUE_DEPTH, sizeof(char*));
        if (!g_cmdQueue) {
            Serial.println("[BLE] command queue create failed");
            return;
        }
    }

    if (!g_blePauseMutex) {
        g_blePauseMutex = xSemaphoreCreateMutex();
    }

    BLEDevice::init(name);
    bleCreateGattServerAndAdvertise(name);
    g_bleInitialized = true;
}

// ---------------------------------------------------------------------------
// ble_pause() / ble_resume() / ble_isPausedTooLong()
// ---------------------------------------------------------------------------
// 2026-09-21: fixes the WiFi/BT coexistence boot loop documented in
// FreematicsNetwork.cpp's ClientWIFI::begin() - WiFi.setSleep(false) (the
// standard fix for WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT(15)/NO_AP_FOUND(201)
// disconnects) aborts the device (coex_core_enable(): "Should enable WiFi
// modem sleep when both WiFi and Bluetooth are enabled!!!!!!" -> abort(),
// called from esp_bt_controller_enable()) whenever the BT controller is
// enabled AT THE SAME TIME WiFi has modem sleep disabled - confirmed live
// twice (Bluedroid 2026-09-20, NimBLE 2026-09-21, identical abort both
// times, same underlying esp_bt_controller_enable() either way). The fix:
// never let those two conditions overlap. ble_pause() is called right
// before WiFi.setSleep(false)+WiFi.begin() starts a connection attempt;
// ble_resume() is called only after WiFi.setSleep(true) has restored modem
// sleep, once the attempt is over (success or failure/timeout) - see
// FreematicsNetwork.cpp and telelogger.ino's onWifiEvent()/wifiConnect()
// for the exact call sites.
//
// WHY A FULL NimBLEDevice::deinit(true)/init() CYCLE, NOT A LIGHTER OPTION -
// verified against the actual pinned NimBLE-Arduino 1.4.1 source
// (.pio/libdeps/esp32dev/NimBLE-Arduino/src/, fetched into this worktree by
// a real `pio run` and read directly, not guessed):
//  - A raw esp_bt_controller_disable()/enable() call (bypassing
//    NimBLEDevice entirely, mirroring what the Bluedroid backend's
//    btStop()/btStart() do) was considered and rejected: NimBLE's host
//    (nimble_port task) owns a continuous HCI transport to the controller
//    (esp_nimble_hci_init(), only torn down by
//    esp_nimble_hci_and_controller_deinit() inside NimBLEDevice::deinit())
//    that Bluedroid's host does not have in the same form - whether the
//    nimble host task detects and cleanly recovers from the controller
//    being yanked out from under it without going through that same
//    deinit path is genuinely unverified from source (would need to trace
//    into the closed-over esp_nimble_hci/nimble_port transport internals,
//    not part of this library's own source), and a hang in the host task
//    is a worse failure mode than a clean, well-understood crash - so this
//    option was not taken.
//  - NimBLEDevice::deinit(false) (clearAll=false, keeps g_server/advertising
//    objects alive) was also considered, since NimBLEDevice::init() called
//    again afterward re-inits the controller+host cleanly as a matched
//    pair (verified: NimBLEDevice.cpp's init() blocks in a `while(!m_synced)`
//    spin until the freshly re-created nimble_port host resyncs with the
//    controller before returning, so by the time ble_resume() below calls
//    anything else, the new host instance is confirmed live). BUT: this
//    was rejected because NimBLEServer::m_gattsStarted (set true the first
//    time NimBLEAdvertising::start() auto-calls pServer->start(), see
//    NimBLEAdvertising.cpp line ~405) is never reset by deinit(false)/
//    init(), so NimBLEAdvertising::start()'s own
//    `if(!pServer->m_gattsStarted) pServer->start();` guard would
//    incorrectly skip re-running ble_gatts_start()/ble_gatts_add_svcs() on
//    the freshly reinitialized (and therefore GATT-table-EMPTY) host
//    instance - the device would advertise, but a phone connecting would
//    find no service/characteristics at all. m_gattsStarted is a private
//    NimBLEServer member (NimBLEServer.h) with no public reset API, so
//    there is no clean way to correct this without relying on an
//    unexported implementation detail.
//  - Chosen instead: NimBLEDevice::deinit(true) (clearAll=true) on pause,
//    which - per NimBLEDevice.cpp's deinit() - additionally deletes
//    g_server and the advertising object (verified: their destructors walk
//    down and delete every owned NimBLEService/NimBLECharacteristic/
//    callback object too, e.g. NimBLEServer::~NimBLEServer() deletes each
//    entry in m_svcVec and, via m_deleteCallbacks, the server callbacks
//    object), then ble_resume() calls NimBLEDevice::init() followed by the
//    SAME bleCreateGattServerAndAdvertise() helper ble_init() itself uses -
//    i.e. resume is not a special/lighter path, it is a literal re-run of
//    first-boot init against the freshly (re)created controller+host. This
//    guarantees a brand-new NimBLEServer object (m_gattsStarted starts
//    false in its constructor - NimBLEServer.cpp line ~44) every time, so
//    NimBLEAdvertising::start()'s auto-start-server guard behaves exactly
//    as it did at first boot and genuinely re-registers the GATT table on
//    the new host instance. Slightly more heap churn per pause/resume cycle
//    (fresh NimBLEServer/NimBLEService/NimBLECharacteristic/callback
//    objects each time, old ones cleanly deleted by deinit(true) - no
//    leak) in exchange for not depending on any private/unexported library
//    state. Given WiFi (re)connect attempts are not a tight loop (seconds
//    apart at the very least), this trade is the right one here.
//
// Because the controller (and, with it, the whole NimBLE host+GATT server)
// is fully torn down while paused, any BLE client (the phone app) connected
// at pause time is necessarily dropped - there is no way around that (the
// radio itself goes away). This is the one piece that could NOT be verified
// without real hardware: the reasoning above establishes that ble_resume()
// rebuilds an equivalent, functioning GATT server and starts advertising
// again every single time (not just the first), but whether the phone app
// actually notices the disconnect and reconnects cleanly on ITS side is a
// question for the phone app / BLE stack on that end, untestable from here.
void ble_pause()
{
    if (!g_bleInitialized) return;
    // Mutex (2026-09-22, see its own declaration comment above) - serializes
    // this against ble_resume() and against itself, closing the race the
    // 2026-09-21 flag-order fix only narrowed. portMAX_DELAY: this can only
    // block behind another pause/resume call, which always completes.
    if (g_blePauseMutex) xSemaphoreTake(g_blePauseMutex, portMAX_DELAY);
    if (g_blePaused) {
        if (g_blePauseMutex) xSemaphoreGive(g_blePauseMutex);
        return;
    }
    Serial.println("[BLE] pausing BT controller for WiFi (re)connect");
    NimBLEDevice::deinit(true /* clearAll - see comment above for why */);
    g_server = nullptr;
    g_statusChar = nullptr;
    g_blePauseStartMs = millis();
    g_blePaused = true;
    if (g_blePauseMutex) xSemaphoreGive(g_blePauseMutex);
}

void ble_resume()
{
    if (!g_bleInitialized) return;
    // Mutex-protected (2026-09-22): this used to just clear g_blePaused
    // before the slow re-init work as a race mitigation (2026-09-21) - that
    // narrowed but did NOT close the window between two tasks' `if
    // (!g_blePaused)` check and the clear, confirmed hit a second time the
    // night of 2026-09-22 after a third concurrent caller (a periodic
    // watchdog) was added, with the identical ESP_ERR_INVALID_STATE-abort
    // signature as the original 2026-09-21 boot loop. The mutex makes the
    // number of callers irrelevant - only one can ever be inside the
    // check-and-clear-and-reinit section at a time, so a second (or third)
    // caller simply blocks until the first is done, sees g_blePaused
    // already false, and returns immediately - no re-entrant init possible.
    if (g_blePauseMutex) xSemaphoreTake(g_blePauseMutex, portMAX_DELAY);
    if (!g_blePaused) {
        if (g_blePauseMutex) xSemaphoreGive(g_blePauseMutex);
        return;
    }
    g_blePaused = false;
    Serial.println("[BLE] resuming BT controller after WiFi (re)connect");
    BLEDevice::init(g_advName);
    bleCreateGattServerAndAdvertise(g_advName);
    if (g_blePauseMutex) xSemaphoreGive(g_blePauseMutex);
}

bool ble_isPausedTooLong(uint32_t maxPauseMs)
{
    if (!g_blePaused) return false;
    return millis() - g_blePauseStartMs > maxPauseMs;
}

char* ble_recv_command(int timeout)
{
    char* cmd = nullptr;
    if (g_cmdQueue && xQueueReceive(g_cmdQueue, &cmd, timeout / portTICK_PERIOD_MS) == pdTRUE) {
        return cmd;
    }
    return nullptr;
}

// NimBLE-Arduino's BLECharacteristic::notify() silently no-ops when no
// client has written the CCCD to formally subscribe to notifications
// (NimBLECharacteristic.cpp's notify(): `if (m_subscribedVec.size() == 0)
// return;`). The original Bluedroid code (ble_spp_server.c) called the raw
// esp_ble_gatts_send_indicate() directly, which does NOT check subscription
// state - it always sends to the tracked connection. Confirmed live
// 2026-09-21: the real Freematics Controller App connects and its commands
// (BATT/TEMP/ON?) round-trip and get computed correctly (visible in the
// [BLE] logs from telelogger.ino's processBLE()), but the app displayed 0
// for every value - consistent with it never issuing a CCCD subscribe
// write, so NimBLE's notify() was silently discarding every response.
// This bypasses NimBLE-Arduino's subscription gate by calling the same
// underlying raw NimBLE host API (ble_gattc_notify_custom) the library
// itself uses internally, for every currently connected peer regardless of
// subscription state - restoring the original's "always send" behaviour.
static void bleForceNotify(BLECharacteristic* pChar, const uint8_t* data, size_t len)
{
    if (!g_server || !pChar) return;
    for (uint16_t connHandle : g_server->getPeerDevices()) {
        os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
        if (om) {
            ble_gattc_notify_custom(connHandle, pChar->getHandle(), om);
        }
    }
}

void ble_send_response(void* data, int len, char* ptr_to_free)
{
    if (ptr_to_free) free(ptr_to_free);
    if (len > 0 && g_statusChar && g_server && g_server->getConnectedCount() > 0) {
        g_statusChar->setValue((uint8_t*)data, (size_t)len);
        bleForceNotify(g_statusChar, (const uint8_t*)data, (size_t)len);
    }
}

void ble_send(int spp_index, void* data, int len)
{
    if (spp_index == SPP_IDX_SPP_STATUS_VAL) {
        // Same notify path as ble_send_response(), just without freeing
        // anything - matches the original ble_send(), which never took
        // ownership of `data` either (only ble_send_response()'s
        // ptr_to_free was ever freed).
        ble_send_response(data, len, nullptr);
        return;
    }
    // SPP_IDX_SPP_DATA_NTY_VAL and anything else: documented no-op - this
    // characteristic was never created in the live GATT table, no current
    // caller uses this index.
}
