/*
 * ble_spp_server_nimble.cpp
 *
 * ============================================================================
 *  Alternative BLE SPP server implementation, built against
 *  h2zero/NimBLE-Arduino instead of raw ESP-IDF Bluedroid.
 *
 *  ORIGIN: this file started as an untested draft written 2026-09-15/16 by
 *  an earlier agent session with no build/hardware access (see git history
 *  of this repo / the stale worktree it came from for that original text).
 *  On 2026-09-21, in a separate isolated worktree with PlatformIO available
 *  (but still NO hardware access - no device is reachable and none should
 *  be touched from here), it was wired into a real build behind the
 *  ENABLE_BLE_NIMBLE compile-time switch (see below) and iterated on until
 *  `pio run -e esp32dev -D ENABLE_BLE_NIMBLE=1` compiles cleanly. It has
 *  STILL NEVER been flashed or run on real hardware - compiling clean only
 *  proves the code is well-formed C++ against this project's pinned
 *  NimBLE-Arduino version, not that it works. All of the functional/
 *  hardware caveats below are unchanged from the original draft and still
 *  apply in full.
 *
 *  REQUIRED MANUAL TESTING BEFORE TRUSTING THIS (do on the bench first,
 *  then in the car):
 *   1. Flash a USB build with ENABLE_BLE_NIMBLE=1 (see "HOW TO BUILD/TEST
 *      THIS" below) to real hardware.
 *   2. Confirm the "Freematics Controller" phone app can see the device
 *      advertising, connect, and that commands like UPTIME/BATT/VIN/ON?
 *      round-trip correctly (processBLE()'s command table in telelogger.ino
 *      is unchanged, so this is purely "does the transport work").
 *   3. Re-run the SAME heap/stability measurement that motivated this
 *      rewrite: log ESP.getFreeHeap() / ESP.getMaxAllocHeap() periodically
 *      with BLE active, and count WiFi disconnects over a comparable
 *      window (the original bad numbers with classic Bluedroid: free heap
 *      dropping to ~10-13KB, max allocatable block ~6-8KB, and repeated
 *      WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT/NO_AP_FOUND disconnects). This is
 *      the actual pass/fail criterion - "connects over BLE" alone does not
 *      prove the WiFi instability is fixed.
 *   4. Also confirm normal WiFi telemetry to the server keeps working for a
 *      real drive, not just at idle - the original problem only fully
 *      showed up under sustained operation.
 *
 *  HOW TO BUILD/TEST THIS:
 *   a. lib_deps in firmware_v5/telelogger/platformio.ini already includes
 *      h2zero/NimBLE-Arduino (pinned - see that file for the exact version
 *      and why 1.4.x rather than 2.x was chosen).
 *   b. Build with ENABLE_BLE_NIMBLE=1, e.g.:
 *        pio run -e esp32dev -D ENABLE_BLE_NIMBLE=1
 *      (or set it permanently for an env by adding -DENABLE_BLE_NIMBLE=1 to
 *      that env's build_flags in platformio.ini). ENABLE_BLE=1 must also be
 *      in effect (it is, by default) for telelogger.ino to actually call
 *      ble_init()/etc. at all - ENABLE_BLE_NIMBLE only picks WHICH
 *      implementation answers those calls, it doesn't turn BLE on by
 *      itself.
 *   c. Build + flash over USB, then follow the manual testing steps above.
 *   d. To go back to classic Bluedroid, just omit ENABLE_BLE_NIMBLE (or set
 *      it to 0) and rebuild - no other change needed.
 *
 * ----------------------------------------------------------------------------
 *  WHICH NimBLE OPTION, AND WHY
 * ----------------------------------------------------------------------------
 *  Two options were investigated:
 *
 *  (1) ESP-IDF's own NimBLE host (CONFIG_BT_NIMBLE_ENABLED, a Kconfig-level
 *      swap of the Bluetooth HOST stack, keeping Bluedroid's controller-side
 *      code out of the picture) - REJECTED. The pinned platform
 *      (espressif32 @ 6.5.0 -> arduino-esp32 core 2.0.17) ships PRECOMPILED
 *      static libs (tools/sdk/esp32/lib/libbt.a and friends), not ESP-IDF
 *      source rebuilt from this project's sdkconfig. The sdkconfig baked
 *      into that prebuilt lib says CONFIG_BT_BLUEDROID_ENABLED=y and
 *      # CONFIG_BT_NIMBLE_ENABLED is not set, with no per-project
 *      PlatformIO knob that flips a setting baked into an already-compiled
 *      .a file - doing this for real would mean rebuilding arduino-esp32's
 *      entire IDF component set from source with a different sdkconfig, a
 *      framework-packaging-level change, not something available from this
 *      repo's platformio.ini.
 *
 *  (2) h2zero/NimBLE-Arduino (a full Arduino-API-shaped BLE library
 *      implemented on top of the NimBLE host, installable as a normal
 *      PlatformIO lib_dep, requiring NO framework rebuild) - CHOSEN, by
 *      elimination as much as by merit: it's the only one of the two that
 *      is actually usable from this project without a framework-level
 *      rebuild. It also happens to be the standard, widely-used way people
 *      get NimBLE's smaller RAM footprint on off-the-shelf arduino-esp32
 *      installs, so it's a well-trodden path, not an exotic one.
 *
 *  This file is therefore a real rewrite of ble_spp_server.c's GATT-server
 *  logic against NimBLE-Arduino's BLEDevice/BLEServer/BLECharacteristic
 *  API, not a config flip.
 *
 * ----------------------------------------------------------------------------
 *  WHAT THIS FILE REPLICATES FROM THE ORIGINAL, AND WHAT IT DELIBERATELY
 *  DOESN'T
 * ----------------------------------------------------------------------------
 *  Original ble_spp_server.c's SPP_IDX_NB attribute table nominally
 *  declares FOUR characteristics (command, status, data-receive, data-
 *  notify) but the data-receive/data-notify pair is wrapped in `#if 0` and
 *  was never actually created in the live GATT table - only the SERVICE
 *  (UUID 0xABF0), COMMAND characteristic (UUID 0xFFE1, read/write, feeds
 *  ble_recv_command()'s queue) and STATUS characteristic (UUID 0xFFE2,
 *  read/notify, what ble_send_response() writes to) were ever live. This
 *  file replicates exactly that live surface - same service/characteristic
 *  UUIDs (kept as plain 16-bit values via BLEUUID((uint16_t)0x....), which
 *  expand to the same standard Bluetooth-Base 128-bit UUID the original's
 *  ESP_UUID_LEN_16 attributes did, so the existing "Freematics Controller"
 *  phone app should see the same service/characteristics without any app-
 *  side change) - same two live characteristics, same queue-based command
 *  hand-off, same "drop oldest on overflow" queue behaviour.
 *
 *  Known, deliberate deviations from the original (flagged, not hidden):
 *   - No artificial ~20-byte command-length ceiling. The original's GATT
 *     table declared SPP_COMMAND_VAL with max length SPP_CMD_MAX_LEN (20
 *     bytes) - actually smaller than some real commands processBLE() in
 *     telelogger.ino accepts (e.g. "OTA_TOKEN=<64 hex chars>" is 74+ bytes),
 *     and the original's ESP_GATTS_WRITE_EVT handler's `is_prep==true`
 *     branch (BLE "prepared/long write") only logs, never actually
 *     reassembles multi-part writes into one buffer - so long commands were
 *     very likely already broken/truncated over BLE before this rewrite.
 *     NimBLE-Arduino characteristics don't impose that pre-sized ceiling by
 *     default (bounded instead by CONFIG_BT_NIMBLE_ATT_MAX_LEN, default
 *     512), so this file incidentally may or may not fix that pre-existing
 *     limitation - UNVERIFIED either way, not the point of this rewrite,
 *     just noting the behavioural difference exists.
 *   - ble_send_response()/ble_send() check `getConnectedCount() > 0` before
 *     calling notify(). The original called esp_ble_gatts_send_indicate()
 *     unconditionally (relying on a stale spp_conn_id/spp_gatts_if to make
 *     it a silent no-op when nothing is connected). This guard is meant to
 *     be equivalent-or-safer, not a functional change, but it is an
 *     unverified assumption about NimBLE's notify() behaviour when called
 *     with zero connected peers.
 *   - ble_send(SPP_IDX_SPP_DATA_NTY_VAL, ...) is a documented no-op here.
 *     In the original, calling ble_send() with that index against the
 *     never-created data-notify characteristic would have hit
 *     find_char_and_desr_index() returning 0xff and then indexed
 *     spp_handle_table[0xff] - almost certainly a latent bug/undefined
 *     table read in the original that was never triggered because nothing
 *     calls ble_send() with that index today. This file chooses to no-op
 *     safely instead of reproducing that bug. Irrelevant in practice since
 *     no current caller uses this path, but flagged for completeness.
 *   - Advertising interval (setMinInterval/setMaxInterval below) is set to
 *     numerically match the original's spp_adv_params (adv_int_min=0x20,
 *     adv_int_max=0x40, both in 0.625 ms units).
 *
 * ----------------------------------------------------------------------------
 *  REAL UNCERTAINTY / RISK (read before trusting this)
 * ----------------------------------------------------------------------------
 *   - LIBRARY API VERSION: platformio.ini pins h2zero/NimBLE-Arduino to a
 *     1.4.x release deliberately - that's the last line that keeps the
 *     "classic" API this file uses (global BLEDevice/BLEServer/BLEService/
 *     BLECharacteristic/BLECharacteristicCallbacks/BLEServerCallbacks
 *     names, callback signature `onWrite(BLECharacteristic*)` with no extra
 *     connection-info parameter). h2zero's library later (2.x releases)
 *     renamed most classes with an Nim prefix (NimBLEDevice, NimBLEServer,
 *     NimBLECharacteristic, ...) and changed callback signatures to take an
 *     added `NimBLEConnInfo&` parameter, targeting IDF 5.x/arduino-esp32
 *     3.x - a different platform pin than this project's (espressif32 @
 *     6.5.0 -> arduino-esp32 2.0.17 -> IDF 4.4.x). If the lib_deps pin is
 *     ever bumped into the 2.x line, this file will need small, mostly
 *     mechanical renames (BLEDevice -> NimBLEDevice etc., callback
 *     signatures) - do not bump the pin without re-testing compilation.
 *   - PSRAM: BOARD_HAS_PSRAM=1 is set for this board, but the BT controller
 *     (regardless of Bluedroid vs NimBLE host) does DMA out of internal
 *     DRAM - PSRAM does not directly offload BLE controller/host buffers on
 *     the classic ESP32 (single-core radio DMA constraint). So switching to
 *     NimBLE should shrink the HOST-side static/heap footprint substantially
 *     (commonly cited as roughly half-to-a-third of Bluedroid's, though the
 *     exact number depends on config), but PSRAM presence is not expected
 *     to change that math - this is stated from general ESP32 BLE
 *     architecture knowledge, not something independently re-measured for
 *     this exact library/version here. The heap-comparison test in step 3
 *     above is what actually settles it for this device.
 *   - Coexistence with WiFi: NimBLE is expected to help specifically
 *     because its host-side RAM use is smaller, leaving more free internal
 *     DRAM for WiFi/mbedTLS - but BLE and WiFi still share the same 2.4 GHz
 *     radio time-division on ESP32 either way; if any of the original
 *     instability was RF/coexistence-scheduling related rather than purely
 *     heap-starvation related, NimBLE alone may not fully fix it. That's an
 *     inference, not a certainty - worth watching for during step 3/4
 *     testing.
 *   - Further RAM tuning available but NOT applied here (would require
 *     build_flags changes the user should make deliberately, after
 *     confirming the basic swap works): NimBLE-Arduino exposes its own
 *     compile-time config via build_flags, e.g.
 *     -DCONFIG_BT_NIMBLE_MAX_CONNECTIONS=1,
 *     -DCONFIG_BT_NIMBLE_ROLE_CENTRAL_DISABLED,
 *     -DCONFIG_BT_NIMBLE_ROLE_OBSERVER_DISABLED,
 *     -DCONFIG_BT_NIMBLE_MAX_BONDS=0 - each shrinks static allocations
 *     further for a peripheral-only, single-connection, no-bonding use case
 *     like this one. Left as a documented follow-up, not applied, so the
 *     first test is of the straightforward swap only. Note also that
 *     -DCONFIG_BT_NIMBLE_ROLE_CENTRAL_DISABLED / ROLE_OBSERVER_DISABLED
 *     would need to be REMOVED again if/when the separate, not-yet-written
 *     "scan for nearby phones to auto-match drivers to trips" feature is
 *     implemented - that feature needs the BLE central/scanning role, which
 *     this file does not implement or use at all (peripheral/GATT-server
 *     role only, same as the original). That scanning feature is still
 *     completely unwritten as of this integration pass.
 *   - This file has been made to COMPILE cleanly (2026-09-21, isolated
 *     worktree, esp32dev env, ENABLE_BLE_NIMBLE=1) but has NOT been
 *     runtime- or hardware-tested at all. Compiling clean only rules out
 *     API-shape mistakes: it says nothing about GATT behaviour, timing, or
 *     the actual heap/WiFi-coexistence question this rewrite exists to
 *     answer.
 */

#ifndef ENABLE_BLE_NIMBLE
#define ENABLE_BLE_NIMBLE 0
#endif

#if !ENABLE_BLE_NIMBLE
// Inert by default. PlatformIO's Library Dependency Finder auto-compiles
// every .c/.cpp under libraries/FreematicsPlus/utility/ for the CURRENTLY
// BUILDING environment (esp32dev) regardless of whether anything #includes
// this specific file - library.json for FreematicsPlus declares no
// srcFilter, so the whole utility/ directory is in scope. If this
// translation unit unconditionally #included <NimBLEDevice.h>, today's
// real (Bluedroid) build would break the moment this file was added, which
// is explicitly NOT what was asked for. Everything below only compiles when
// ENABLE_BLE_NIMBLE is defined to a nonzero value (see config.h /
// platformio.ini). Until then this file compiles to an empty translation
// unit, and ble_spp_server.c (guarded the mirror-image way) provides the
// real ble_init()/ble_send()/ble_recv_command()/ble_send_response()
// symbols instead.

#else  // ENABLE_BLE_NIMBLE

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
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
        Serial.println("[BLE] client connected");
    }
    void onDisconnect(BLEServer* pServer) override {
        Serial.println("[BLE] client disconnected - restarting advertising");
        BLEDevice::startAdvertising();
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
    if (!g_bleInitialized || g_blePaused) return;
    Serial.println("[BLE] pausing BT controller for WiFi (re)connect");
    NimBLEDevice::deinit(true /* clearAll - see comment above for why */);
    g_server = nullptr;
    g_statusChar = nullptr;
    g_blePauseStartMs = millis();
    g_blePaused = true;
}

void ble_resume()
{
    if (!g_bleInitialized || !g_blePaused) return;
    Serial.println("[BLE] resuming BT controller after WiFi (re)connect");
    BLEDevice::init(g_advName);
    bleCreateGattServerAndAdvertise(g_advName);
    g_blePaused = false;
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
    // SPP_IDX_SPP_DATA_NTY_VAL and anything else: documented no-op - see
    // header comment block above ("WHAT THIS FILE REPLICATES ... AND WHAT
    // IT DELIBERATELY DOESN'T") for why.
}

#endif // ENABLE_BLE_NIMBLE
