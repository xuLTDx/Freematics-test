/******************************************************************************
* Arduino sketch of a vehicle data data logger and telemeter for Freematics Hub
* Works with Freematics ONE+ Model A and Model B
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
* Visit https://hub.freematics.com to view live and history telemetry data
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include <Arduino.h>
#include <Update.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

// ExplicitnÃƒÂ¡ deklarÃƒÂ¡cia globÃƒÂ¡lneho objektu Update z ESP32 Updater kniÃ…Â¾nice
extern UpdateClass Update;
#include <FreematicsPlus.h>
#include <httpd.h>
#include <mbedtls/sha256.h>
#include <mbedtls/platform.h>
#include "config.h"
#include "telestore.h"
#include "teleclient.h"
#if BOARD_HAS_PSRAM
#include "esp32/himem.h"
#endif
#include "driver/adc.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_sleep.h"
#include "esp_system.h"

// 2026-09-14: __DATE__/__TIME__ are evaluated per translation unit, not once
// for the whole firmware - dataserver.cpp has its own copy, which only
// updates when dataserver.cpp itself gets recompiled. Since most recent work
// only touches this file, PlatformIO's incremental build leaves
// dataserver.cpp's copy stale (confirmed: dataserver.cpp last changed
// 13:15:14, so its __DATE__/__TIME__ froze at whatever it compiled to then,
// even in a firmware.bin built hours later) - /api/info's "fw_build" field
// (dataserver.cpp) reported an old timestamp while the real running build
// (this file's own Serial boot banner) was current. Fix: one shared, always-
// current string, defined here (this file changes on every real firmware
// change) and consumed via extern from dataserver.cpp instead of it
// evaluating __DATE__/__TIME__ itself.
// extern on the definition itself, not just the consuming side's forward
// declaration - a plain "const char x[] = ..." at file scope in C++ (unlike
// C) defaults to INTERNAL linkage, which silently produces exactly this
// "undefined reference to FW_BUILD_STR" at link time from dataserver.cpp's
// extern declaration, since there'd be no external symbol to find.
extern const char FW_BUILD_STR[] = __DATE__ " " __TIME__;

// states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_GPS_ONLINE 0x20
#define STATE_CELL_CONNECTED 0x40
#define STATE_WIFI_CONNECTED 0x80
#define STATE_WORKING 0x100
#define STATE_STANDBY 0x200

typedef struct {
  byte pid;
  byte tier;
  int value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {
  // Tier 1: polled every cycle (fast-changing: speed, RPM, throttle, load)
  {PID_SPEED, 1},
  {PID_RPM, 1},
  {PID_THROTTLE, 1},
  {PID_ENGINE_LOAD, 1},
  // Tier 2: polled once per two cycles (medium rate)
  {PID_FUEL_PRESSURE, 2},
  {PID_TIMING_ADVANCE, 2},
  {PID_INTAKE_MAP, 2},
  {PID_MAF_FLOW, 2},
  // Tier 3: polled once per many cycles (slow-changing)
  {PID_COOLANT_TEMP, 3},
  {PID_INTAKE_TEMP, 3},
  {PID_SHORT_TERM_FUEL_TRIM_1, 3},
  {PID_LONG_TERM_FUEL_TRIM_1, 3},
  {PID_SHORT_TERM_FUEL_TRIM_2, 3},
  {PID_LONG_TERM_FUEL_TRIM_2, 3},
  {PID_RUNTIME, 3},
  // NOTE: PID_FUEL_LEVEL intentionally NOT polled here via standard
  // obd.readPID() - same reason as PID_ODOMETER above. Confirmed via a
  // HexSniff capture (2026-09-12) that the standard fuel-level DID this
  // PID maps to gets zero responses on this vehicle (Gateway-routed
  // diagnostics again), so leaving it in this tier-3 list would time out
  // on every pass and `break` the loop, starving PID_BAROMETRIC and
  // everything listed after it. Read instead via the dedicated UDS block
  // in processOBD() below (DID 0x22B0 on the Instrument Cluster module,
  // 0x714/0x77E - not the Gateway that odometer uses), which handles
  // failure without blocking other PIDs.
  {PID_BAROMETRIC, 3},
  {PID_CONTROL_MODULE_VOLTAGE, 3},
  {PID_ABSOLUTE_ENGINE_LOAD, 3},
  {PID_RELATIVE_THROTTLE_POS, 3},
  {PID_AMBIENT_TEMP, 3},
  {PID_ACC_PEDAL_POS_D, 3},
  {PID_ACC_PEDAL_POS_E, 3},
  {PID_ENGINE_OIL_TEMP, 3},
  {PID_ETHANOL_FUEL, 3},
  {PID_HYBRID_BATTERY_PERCENTAGE, 3},
  {PID_ENGINE_FUEL_RATE, 3},
  {PID_REL_ACCEL_PEDAL, 3},
  // NOTE: PID_ODOMETER intentionally NOT polled here via standard obd.readPID().
  // VAG/Passat B8 ECUs do not support the standard Mode 1 odometer PID, so
  // leaving it in this table caused a timeout on every pass through it, which
  // triggered `break` in the tier-poll loop below and starved every PID that
  // comes after it in this list. Odometer is instead read via the dedicated
  // UDS block in processOBD() (VAG UDS DIDs + GPS fallback), which handles
  // failures without blocking other PIDs.
  // Catalyst temperatures (only supported on petrol engines)
  {PID_CATALYST_TEMP_B1S1, 3},
  {PID_CATALYST_TEMP_B2S1, 3},
  {PID_ENGINE_TORQUE_DEMANDED, 3},
  {PID_ENGINE_TORQUE_PERCENTAGE, 3},
  {PID_ENGINE_REF_TORQUE, 3},
};

CBufferManager bufman;
Task subtask;

#if ENABLE_MEMS
float accBias[3] = {0}; // calibrated reference accelerometer data
float accSum[3] = {0};
float acc[3] = {0};
float gyr[3] = {0};
float mag[3] = {0};
uint8_t accCount = 0;
#endif
int deviceTemp = 0;

// Declared here (before wifiConnect()/wifiReconnectCurrent() below, which
// reference it) rather than further down with the rest of the network setup -
// C++ requires the declaration to precede first use, and Arduino's automatic
// function-prototyping only forward-declares functions, not global objects.
#if SERVER_PROTOCOL == PROTOCOL_UDP
TeleClientUDP teleClient;
#else
TeleClientHTTP teleClient;
#endif

// 2026-09-14: dedicated HTTP client for pull-OTA, independent of teleClient.wifi
// (which is WifiUDP, not HTTP-capable, on SERVER_PROTOCOL=PROTOCOL_UDP builds -
// see performPullOtaCheck()'s comment for the full story of why this exists).
// Declared here for the same C++-declaration-order reason as teleClient above.
#if ENABLE_WIFI
WifiHTTP otaWifiClient;
#endif

// config data
char apn[32];
char simPin[16] = SIM_CARD_PIN;
#if ENABLE_WIFI
char wifiSSID[32] = WIFI_SSID;
char wifiPassword[32] = WIFI_PASSWORD;
char wifiSSID2[32] = WIFI_SSID2;
char wifiPassword2[32] = WIFI_PASSWORD2;

// Which network (0 = primary wifiSSID, 1 = secondary wifiSSID2) was used on
// the most recent connect attempt - lets wifiReconnectCurrent() (below)
// reconnect to the SAME network instead of alternating.
uint8_t wifiCurrentIdx = 0;

// --- Adaptive network preference (2026-09-21) -------------------------
// Confirmed live at the user's home: wifiConnect() below used to
// unconditionally XOR-alternate between wifiSSID/wifiSSID2 on every single
// call, regardless of which one actually worked last. At home, wifiSSID2
// (uLTD_ext, only in range at the workplace) is never reachable, so every
// OTHER (re)connect attempt was guaranteed to fail with NO_AP_FOUND -
// wasted airtime/battery and needlessly fell back to cellular half the
// time even though wifiSSID (NTLIRIS) was right there and working.
//
// Fix: remember which network last actually reached
// ARDUINO_EVENT_WIFI_STA_GOT_IP (onWifiEvent() below sets wifiPreferredIdx/
// resets wifiPreferredFails there) and prefer trying that one first. Only
// after WIFI_PREFERRED_FAIL_THRESHOLD consecutive failed attempts on the
// preferred network does wifiConnect() fall back to the old strict
// alternation between both configured networks - covering the case where
// the device genuinely has moved to the other location. Deliberately just
// three small scalars, not a growing list/log - and does NOT persist
// across a power cycle (no NVS write).
//
// wifiPreferredIdx: network to try first on the next wifiConnect() call.
uint8_t wifiPreferredIdx = 0;
// wifiPreferredFails: consecutive times wifiPreferredIdx has been attempted
// (via wifiConnect()) without succeeding by the time wifiConnect() was
// called again - since wifiConnect() is only ever called while NOT
// currently connected (see its call sites), being called again always
// means the previous attempt did not result in a live connection.
uint8_t wifiPreferredFails = 0;
// wifiLastAttemptIdx: which network the most recent wifiConnect() call
// attempted, so the NEXT call can tell whether to count that as a failure
// of wifiPreferredIdx. 0xFF = no attempt yet this boot (nothing to count).
uint8_t wifiLastAttemptIdx = 0xFF;
#define WIFI_PREFERRED_FAIL_THRESHOLD 3 /* consecutive fails before trying the alternate network first */

// Tries the two configured WiFi networks in turn: each call to this function
// (which only happens when the device is NOT currently connected Ã¢â‚¬â€ see the
// call sites) alternates which network it attempts, so if the primary
// (wifiSSID) is out of range, the next retry cycle tries the secondary
// (wifiSSID2), and so on. Skips a network whose SSID is empty.
// This replaces the direct teleClient.wifi.begin(wifiSSID, wifiPassword)
// calls that used to be scattered across the file (single-network only).
// wifiScanAndLog() (defined later, after `logger`/`state` exist - it needs
// both) is forward-declared here since Arduino's .ino auto-prototyping
// doesn't cover functions using types/globals not yet visible at their own
// definition point. See the definition (near `State state;`) for what it
// does and why.
static void wifiScanAndLog();
static bool wifiScanPending = false;
static int32_t wifiScanCount = 0;
static int32_t wifiScanTargetRssi = -999;
#define PID_WIFI_SCAN_COUNT  0x320
#define PID_WIFI_TARGET_RSSI 0x321

// 2026-09-14: why the device rebooted, sent once per boot. Added after a
// ~18-minute total telemetry silence (WiFi AND cellular both quiet) followed
// two failed direct-OTA push attempts - no way to tell from the outside
// whether that was a genuine crash (panic/watchdog) caused by the
// interrupted OTA write, a brownout, or something unrelated. esp_reset_reason()
// (esp_system.h, standard ESP-IDF) answers this definitively instead of
// guessing: ESP_RST_POWERON=1 (normal power-on), ESP_RST_WDT=8/ESP_RST_TASK_WDT=
// 9/ESP_RST_INT_WDT=7 (a stuck task got killed by a watchdog), ESP_RST_PANIC=2
// (crash/exception), ESP_RST_BROWNOUT=6 (supply voltage sagged), ESP_RST_SW=3
// (esp_restart() called deliberately, e.g. by the OTA handler after a
// successful flash). Full enum: esp-idf esp_system.h RESET_REASON.
static bool resetReasonPending = false;
static int32_t resetReasonCode = 0;
#define PID_RESET_REASON 0x360

// ESP32 WiFi disconnect reason codes (wifi_err_reason_t, esp_wifi_types.h) -
// sent as telemetry (see wifiDisconnectPending below) so repeated WiFi
// drops during real-world (in-car, engine-running) testing can be diagnosed
// remotely instead of guessed at. Common values worth knowing on sight:
//   2   = AUTH_EXPIRE (AP-initiated re-auth failure)
//   3   = AUTH_LEAVE
//   6/7 = ASSOC_EXPIRE / NOT_AUTHED
//   8   = ASSOC_LEAVE (station left - e.g. our own disconnect() call)
//   15  = 4WAY_HANDSHAKE_TIMEOUT (wrong password OR AP not responding in time)
//   200 = BEACON_TIMEOUT (lost the AP's signal - range/interference)
//   201 = NO_AP_FOUND
//   202 = AUTH_FAIL
//   203 = ASSOC_FAIL
//   204 = HANDSHAKE_TIMEOUT
// Full list: esp-idf esp_wifi_types.h WIFI_REASON_*.
static bool wifiDisconnectPending = false;
static int32_t wifiDisconnectReason = 0;
static int32_t wifiDisconnectCount = 0;
#define PID_WIFI_DISCONNECT_REASON 0x322
#define PID_WIFI_DISCONNECT_COUNT  0x323

// Defined later (near wifiScanAndLog(), after `logger`/`state` exist).
static void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info);

// Safety-net ceiling for ble_pause()/ble_resume() (see FreematicsNetwork.cpp's
// ClientWIFI::begin()/setup() and ble_spp_server_nimble.cpp's ble_pause()
// comment for the full WiFi/BT coexistence story). Normally
// ble_resume() fires within WIFI_JOIN_TIMEOUT (15s) via either
// ARDUINO_EVENT_WIFI_STA_GOT_IP below or ClientWIFI::setup()'s own
// success/timeout paths - this is only a backstop for the one wifiConnect()
// call site (the periodic RSSI-check block further down) that does not call
// teleClient.wifi.setup() right after wifiConnect(), so BT can't be left
// paused indefinitely if that path's connection attempt never resolves one
// way or the other. Generous margin over WIFI_JOIN_TIMEOUT on purpose - this
// is a last resort, not the normal resume path.
#define BLE_PAUSE_MAX_MS 30000

void wifiConnect()
{
#if ENABLE_WIFI
  // Backstop: force BT back on if a previous pause somehow never got
  // resumed. WiFi.setSleep(true) MUST happen before ble_resume() (same
  // ordering requirement as every other resume call site - see
  // ClientWIFI::begin()'s comment) so the WiFi/BT coexistence abort can't
  // recur here either.
  if (ble_isPausedTooLong(BLE_PAUSE_MAX_MS)) {
    Serial.println("[WIFI] BLE pause exceeded safety timeout - forcing resume");
    WiFi.setSleep(true);
    ble_resume();
  }
#endif
  static bool scanned = false;
  if (!scanned) {
    scanned = true;
    WiFi.onEvent(onWifiEvent);
    wifiScanAndLog();
  }

  // Adaptive network preference (see the block above wifiCurrentIdx's
  // declaration for the full story). wifiConnect() is only ever called
  // while NOT currently connected (all call sites gate on
  // !teleClient.wifi.connected() / !state.check(STATE_WIFI_CONNECTED)), so
  // being called again always means the PREVIOUS attempt (wifiLastAttemptIdx)
  // did not end up connected - if that was the preferred network, count one
  // more consecutive failure. onWifiEvent() resets wifiPreferredFails to 0
  // on the next successful GOT_IP. 0xFF = no attempt yet this boot.
  if (wifiLastAttemptIdx != 0xFF && wifiLastAttemptIdx == wifiPreferredIdx
      && wifiPreferredFails < 250) {
    wifiPreferredFails++;
  }

  // Once the preferred network has failed WIFI_PREFERRED_FAIL_THRESHOLD
  // times in a row, fall back to the old strict alternation between both
  // configured networks (via wifiAltIdx) until one of them succeeds again -
  // covers the device genuinely having moved to the other location.
  static uint8_t wifiAltIdx = 0;
  uint8_t startIdx;
  if (wifiPreferredFails < WIFI_PREFERRED_FAIL_THRESHOLD) {
    startIdx = wifiPreferredIdx;
  } else {
    startIdx = wifiAltIdx;
    wifiAltIdx ^= 1; // next fallback call (if this SSID is empty, or preferred net keeps failing) tries the other network
  }

  const char* ssid = "";
  const char* pass = "";
  for (int tries = 0; tries < 2; tries++) {
    uint8_t thisIdx = (tries == 0) ? startIdx : (startIdx ^ 1);
    const char* s = thisIdx == 0 ? wifiSSID : wifiSSID2;
    const char* p = thisIdx == 0 ? wifiPassword : wifiPassword2;
    if (s[0]) { ssid = s; pass = p; wifiCurrentIdx = thisIdx; break; }
  }
  if (!ssid[0]) return; // neither network configured
  wifiLastAttemptIdx = wifiCurrentIdx;
  Serial.print("WIFI:");
  Serial.println(ssid);
  teleClient.wifi.begin(ssid, pass);
}

// Reconnects to whichever network wifiConnect() used most recently, WITHOUT
// advancing the alternation. Use this only when we know we were already on
// a working network and just need it back (e.g. the heap-fragmentation
// WiFi restart mid-OTA below) Ã¢â‚¬â€ not when searching for a reachable network,
// which is what wifiConnect() is for.
void wifiReconnectCurrent()
{
  const char* ssid = wifiCurrentIdx == 0 ? wifiSSID : wifiSSID2;
  const char* pass = wifiCurrentIdx == 0 ? wifiPassword : wifiPassword2;
  if (!ssid[0]) return;
  Serial.print("WIFI:");
  Serial.println(ssid);
  teleClient.wifi.begin(ssid, pass);
}
#endif
// Server settings Ã¢â‚¬â€œ loaded from NVS at boot; fall back to compile-time defaults.
// Set via NVS keys SERVER_HOST, SERVER_PORT, WEBHOOK_PATH using the
// Freematics HA integration provisioning flash (config_nvs.bin).
char serverHost[128] = SERVER_HOST;
uint16_t serverPort = SERVER_PORT;
// WEBHOOK_PATH overrides the legacy /hub/api/post/<devid> path when set.
// Nabu Casa cloud hook paths are ~185 characters; 256 bytes ensures they fit.
char webhookPath[256] = "";
// Cellular-specific server overrides (NVS keys CELL_HOST, CELL_PORT, CELL_PATH).
// When non-empty, these replace SERVER_HOST / SERVER_PORT / WEBHOOK_PATH for
// cellular (SIM7600) connections.  Set to hooks.nabu.casa + the Nabu Casa
// cloud-hook token path by the HA integration when Nabu Casa cloud is active,
// so that cellular devices reach the cloud webhook endpoint directly rather than
// the Remote UI proxy (*.ui.nabu.casa) which the SIM7600 TLS stack cannot use.
// WiFi connections use SERVER_HOST / WEBHOOK_PATH, which the HA integration sets
// from get_url(prefer_external=True) Ã¢â‚¬â€œ typically *.ui.nabu.casa (Nabu Casa Remote
// UI).  Using the Remote UI for WiFi ensures WiFi telemetry and WiFi OTA (which
// always uses *.ui.nabu.casa) share the same TLS session, avoiding mbedTLS heap
// fragmentation from repeated TLS host switching on the ESP32.
char cellServerHost[128] = "";
uint16_t cellServerPort = 443;
char cellWebhookPath[256] = "";
// Runtime HTTP server enable.  Defaults to the compile-time ENABLE_HTTPD value
// (1 = on) so OTA firmware updates via WiFi work without requiring NVS
// provisioning.  Can be overridden by NVS key ENABLE_HTTPD written by the HA
// integration config_nvs.bin.  Only takes effect when compiled with ENABLE_HTTPD=1.
uint8_t enableHttpd = ENABLE_HTTPD;
// Runtime BLE enable Ã¢â‚¬â€œ set to 0 via NVS key ENABLE_BLE to disable the BLE
// SPP server and free ~100 KB heap for the TLS webhook client.  Defaults to
// 1 (on) so devices that were never provisioned keep the previous behaviour.
// Only takes effect when the firmware is compiled with ENABLE_BLE=1.
uint8_t enableBle = 1;
nvs_handle_t nvs;
// NVS settings version string (NVS_VER key): written by the HA integration
// when generating the NVS partition so the firmware can report which settings
// are active.  Format: "<FIRMWARE_VERSION>.<settings_timestamp>" e.g.
// "5.1.2026-03-16T16:11:20+00:00".  Empty when NVS was not provisioned by HA.
char nvsVersion[64] = "";

// ---------------------------------------------------------------------------
// Pull-OTA configuration (loaded from NVS by loadConfig(), firmware v5.2+).
// When otaToken is non-empty the firmware periodically GETs
//   https://{otaHost}:{otaPort}/api/freematics/ota_pull/{otaToken}/meta.json
// and downloads / flashes a newer firmware version when one is available.
// ---------------------------------------------------------------------------
// Secret path token provisioned by the HA integration (OTA_TOKEN NVS key).
// Embedded as a URL path component so no Authorization header is needed.
char otaToken[68] = "";   // 64 hex chars + null; empty = feature disabled
// HA server hostname for pull-OTA (OTA_HOST NVS key).  May differ from
// serverHost when Nabu Casa cloud is active (serverHost would be
// hooks.nabu.casa which does not serve the pull-OTA endpoint).
char otaHost[128] = "";
uint16_t otaPort = 6001;   // OTA_PORT NVS key (u16) - matches freematics-ota's default LISTEN_PORT
// Interval between pull-OTA checks in seconds.  0 = disabled (default).
uint16_t otaCheckIntervalS = 0;  // OTA_INTERVAL NVS key (u16)

// ---------------------------------------------------------------------------
// Geofence-based WiFi known locations (2026-09-22)
// ---------------------------------------------------------------------------
// A small table of Business Addresses (Traccar's tc_business_addresses,
// shared with the Kniha jazd trip-purpose feature - see BusinessAddress.java)
// that have a known WiFi network attached. Lets the device later recognize
// "I'm near a place I can reach over WiFi" from GPS alone (not wired into
// standby/connect logic yet - that's a separate, deferred step; this is just
// the data plumbing: fetch, parse, store).
//
// Fetched from ota_server.py's locations.csv proxy (see that file's module
// docstring, and BusinessAddress.java's ssid field comment, for why it's a
// proxy through the existing per-device OTA token rather than a direct
// Traccar login on the device) by syncKnownLocations() below.
//
// Deliberately stores wifiIdx (which of the device's own two configured
// networks this location's SSID matches), not the SSID string itself -
// syncKnownLocations() already filters to entries whose ssid equals wifiSSID
// or wifiSSID2, since the device only ever has a password for those two
// (WPWD=/NVS); a location naming any other SSID could never be connected to
// anyway, so keeping it out of the table just wastes a slot.
#define MAX_KNOWN_LOCATIONS 8
struct KnownLocation {
  float lat;
  float lon;
  float radiusM;    // meters, copied from the Business Address's own radius
  uint8_t wifiIdx;  // 0 = wifiSSID/wifiPassword, 1 = wifiSSID2/wifiPassword2
};
static KnownLocation knownLocations[MAX_KNOWN_LOCATIONS];
static uint8_t knownLocationCount = 0;
// Shared between the optimistic immediate-on-connect sync attempt and the
// periodic SIGNAL_CHECK_INTERVAL retry further down in telemetry() - see
// both call sites' comments. File-scoped (not function-local) specifically
// so both places can see whether a sync has genuinely succeeded yet.
static uint32_t s_lastLocSyncTime = 0;
static bool s_locSynced = false;

#if ENABLE_WIFI
// Persists the current knownLocations[]/knownLocationCount to NVS (LOC_N +
// LOCS keys) so a reboot doesn't lose the table until the next sync - see
// loadConfig()'s matching load side.
static void saveKnownLocationsToNvs()
{
  nvs_set_u8(nvs, "LOC_N", knownLocationCount);
  nvs_set_blob(nvs, "LOCS", knownLocations, knownLocationCount * sizeof(KnownLocation));
  nvs_commit(nvs);
}

// Fetches ota_server.py's locations.csv feed and replaces the in-RAM
// knownLocations[] table wholesale - a full refresh, not an incremental
// merge, so a radius change or a removed/added address on the Traccar side
// is picked up automatically on the very next sync with no separate
// delete-handling logic needed (this was the explicit design question -
// "co ak sa okruh zmeni v traccar, a co ak tam pribudne dalsia adresa" -
// answered by always replacing the whole table, not patching it).
//
// Reuses otaWifiClient/otaHost/otaPort/otaToken: the same trust boundary,
// server and (when the host matches) already-open TLS session as pull-OTA
// checks. Gated on otaToken being provisioned, exactly like
// performPullOtaCheck() - a device with pull-OTA never configured has no
// reason to hit this endpoint either, and ota_server.py's per-device token
// check would reject it anyway.
//
// Deliberately plain CSV, not JSON (see ota_server.py's module docstring
// for why): "lat,lon,radius,ssid" one per line, no header, no quoting -
// parsed the same hand-rolled way as performPullOtaCheck() parses
// meta.json's flat fields (no JSON library in this codebase).
//
// Must only be called from the main loop() task, never from a WiFi event
// callback (onWifiEvent() runs on the small-stack Arduino event task - a
// TLS handshake plus this function's local buffer does not belong there).
// Returns true only on a genuine successful sync (HTTP 200, body parsed) -
// callers use this to decide whether to retry soon (e.g. a transient
// connect failure right at the moment WiFi just came up, before the IP
// stack/DNS have fully settled - confirmed live 2026-09-22: the very first
// sync attempt after boot got "Cannot connect" while the exact same
// otaWifiClient/otaHost/otaPort combination performPullOtaCheck() uses
// (which runs later, not at the instant of connection) works fine) rather
// than waiting the full LOC_SYNC_MIN_INTERVAL.
static bool syncKnownLocations()
{
  if (!otaToken[0] || !otaHost[0]) return false;
  if (!WiFi.isConnected()) return false;

  // 2026-09-22: was char path[80] - too small. "/api/freematics/ota_pull/"
  // (26) + otaToken (up to 67, see its declaration) + "/locations.csv" (14)
  // + null needs up to 108 bytes; snprintf silently truncated the token/
  // suffix to fit 80, producing a malformed path the server's path parser
  // rejected with 404 - confirmed live (first WiFi sync attempt after
  // deploying this feature got exactly that 404).
  char path[128];
  snprintf(path, sizeof(path), "/api/freematics/ota_pull/%s/locations.csv", otaToken);

  if (!otaWifiClient.open(otaHost, otaPort)) {
    // TEMP 2026-09-22 diagnostic: "Cannot connect" reproduced 3x in a row
    // (immediate attempt + two periodic retries, several seconds apart,
    // well after telemetry was flowing normally) with no "[WIFI] Low heap"
    // print from WifiHTTP::open()'s own guards - so it's a genuine
    // client.connect() failure, not heap fragmentation. Narrow down further:
    // WiFi link state and whether DNS resolution of otaHost itself works
    // from this code path.
    IPAddress resolved;
    bool dnsOk = WiFi.hostByName(otaHost, resolved);
    Serial.printf("[LOC-SYNC] Cannot connect - WiFi.status=%d dnsOk=%d resolved=%s\n",
                  (int)WiFi.status(), (int)dnsOk, dnsOk ? resolved.toString().c_str() : "-");
    return false;
  }
  if (!otaWifiClient.send(METHOD_GET, path)) {
    Serial.println("[LOC-SYNC] send failed");
    return false;
  }

  char buf[1024];
  int bytes = 0;
  char* body = otaWifiClient.receive(buf, sizeof(buf) - 1, &bytes);
  if (!body || otaWifiClient.code() != 200) {
    Serial.printf("[LOC-SYNC] HTTP %u\n", (unsigned)otaWifiClient.code());
    return false;
  }
  buf[bytes < (int)sizeof(buf) - 1 ? bytes : (int)sizeof(buf) - 1] = '\0';
  // Do NOT close the connection here - same TLS-session-reuse reasoning as
  // performPullOtaCheck()'s meta.json fetch (see its comment).

  uint8_t newCount = 0;
  char* line = strtok(body, "\n");
  while (line && newCount < MAX_KNOWN_LOCATIONS) {
    // "lat,lon,radius,ssid"
    char* latStr = line;
    char* lonStr = strchr(latStr, ',');
    if (!lonStr) { line = strtok(nullptr, "\n"); continue; }
    *lonStr++ = 0;
    char* radStr = strchr(lonStr, ',');
    if (!radStr) { line = strtok(nullptr, "\n"); continue; }
    *radStr++ = 0;
    char* ssidStr = strchr(radStr, ',');
    if (!ssidStr) { line = strtok(nullptr, "\n"); continue; }
    *ssidStr++ = 0;
    char* cr = strchr(ssidStr, '\r');  // defensive - server sends bare \n
    if (cr) *cr = 0;

    uint8_t wifiIdx;
    if (wifiSSID[0] && !strcmp(ssidStr, wifiSSID)) {
      wifiIdx = 0;
    } else if (wifiSSID2[0] && !strcmp(ssidStr, wifiSSID2)) {
      wifiIdx = 1;
    } else {
      // Not one of this device's configured networks - no password for it,
      // so it could never actually be connected to. Skip.
      line = strtok(nullptr, "\n");
      continue;
    }

    knownLocations[newCount].lat = (float)atof(latStr);
    knownLocations[newCount].lon = (float)atof(lonStr);
    knownLocations[newCount].radiusM = (float)atof(radStr);
    knownLocations[newCount].wifiIdx = wifiIdx;
    newCount++;
    line = strtok(nullptr, "\n");
  }
  knownLocationCount = newCount;
  saveKnownLocationsToNvs();
  Serial.printf("[LOC-SYNC] %u known location(s)\n", (unsigned)knownLocationCount);
  return true;
}
#endif  // ENABLE_WIFI

// live data
String netop;
String ip;
int16_t rssi = 0;
int16_t rssiLast = 0;
char vin[18] = {0};
uint16_t dtc[6] = {0};
float batteryVoltage = 0;
GPS_DATA* gd = 0;

char devid[12] = {0};
char isoTime[32] = {0};

// stats data
uint32_t lastMotionTime = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;
uint32_t lastStatsTime = 0;

int32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;
int32_t dataInterval = 1000;

#if STORAGE != STORAGE_NONE
int fileid = 0;
uint16_t lastSizeKB = 0;
#endif

byte ledMode = 0;

// Runtime LED / buzzer enable flags.  All default to true (on) so that
// un-provisioned devices preserve the original hardware behaviour.
// Set via NVS keys LED_RED_EN, LED_WHITE_EN, and BEEP_EN (u8, 0=off 1=on).
bool enableLedRed = true;    // red/power LED: lights up in standby / power-on state
bool enableLedWhite = true;  // white/network LED: lights up during data transmission
bool enableBeep = true;      // connection beep: short buzz on WiFi/cellular connect

// Runtime OBD enable flag.  Loaded from NVS by loadConfig().
// Defaults to true so un-provisioned devices keep the existing behaviour.
// Set via NVS key OBD_EN (u8, 0=off 1=on).
bool enableObd = true;   // OBD-II PID polling (compile-time ENABLE_OBD must also be 1)

// Deep-standby flag.  When true the device uses ESP32 deep sleep during
// standby instead of the normal active-wait loop.  Loaded from NVS key
// DEEP_STANDBY (u8, 0=off 1=on).  Defaults to false.
bool enableDeepStandby = false;

// Vehicle identification loaded from NVS (VEHICLE_MAKE, VEHICLE_MODEL, VEHICLE_YEAR).
// Stored for informational purposes and future vehicle-specific PID selection.
char vehicleMake[32] = "";
char vehicleModel[32] = "";
char vehicleYear[8] = "";
// Vehicle-specific extra PIDs (VEHICLE_PIDS NVS key).
// Comma-separated list of mode-1 PID hex values, e.g. "22,23,5A".
// The firmware parses this at startup and appends those PIDs to the dynamic poll list.
char vehiclePidsStr[128] = "";
// Runtime dynamic vehicle PID poll list (parsed from vehiclePidsStr).
#define MAX_VEHICLE_PIDS 16
PID_POLLING_INFO vehicleObdData[MAX_VEHICLE_PIDS];
int vehicleObdDataCount = 0;

// Standby-time override loaded from NVS key STANDBY_TIME (u16, seconds).
// 0 means "use the compile-time STATIONARY_TIME_TABLE default" (currently 180 s).
// When set to a value between 5 and 900 it replaces the last (maximum) entry
// of the stationary-time table so the device enters standby sooner.
// 65535 (0xFFFF) is a sentinel meaning "disable standby entirely" - see the
// nvsStandbyTimeS == 0xFFFF handling in process() where the stationary-time
// table's last tier is set to 0 ("never reaches this tier"). Temporary,
// debugging-only use (extended bench/parked testing) - must not be left set
// permanently, since it also keeps WiFi/OBD polling running non-stop.
uint16_t nvsStandbyTimeS = 0;

// Set to true by handlerOTA while an OTA flash is in progress.
// The telemetry task checks this flag and yields the WiFi to the OTA upload.
volatile bool s_ota_active = false;

// --- Missed-data catch-up (store-and-forward across outages) ------------
// wmDoneFileId (NVS key WM_FILE, u32): highest /DATA/<id>.CSV file id whose
// ENTIRE contents were confirmed successfully sent to the server. Written
// only after a file is fully replayed - never mid-file - so an interruption
// just means that one file is retried in full next time (small harmless
// re-send overlap at worst, never a gap). This is deliberately per-FILE, not
// per-record: PID 0 in each CSV line is a boot-relative millis() counter,
// not wall-clock time, and cannot be compared across different boot
// sessions/files, so file-level granularity avoids needing to correlate
// millis() to real time at all.
// Left at 0 (meaning "nothing confirmed yet") when never set - see
// catchUpMissedFiles() for why that is deliberately treated as "everything
// before the current session's own file is caught up" rather than
// replaying the device's entire lifetime SD history on first boot.
uint32_t wmDoneFileId = 0;
// One-shot per boot: runs catchUpMissedFiles() the first time the telemetry
// send loop is about to send a live packet, then never again until reboot.
// Not static: dataserver.cpp's WM_FILE= control-command handler needs to
// re-arm this after a manual watermark override.
bool s_catchupPending = true;

// SD card paths for the two-phase pull-OTA staging mechanism (STORAGE_SD only).
// Phase 1 (during active telemetry): firmware is downloaded to OTA_PENDING_PATH.
// Phase 2 (at next standby transition): firmware is flashed from SD to flash.
// OTA_META_PATH stores the expected byte count so partial downloads can be
// detected and removed at startup without attempting a corrupt flash.
#define OTA_PENDING_PATH "/ota_fw.bin"    // staged firmware binary
#define OTA_META_PATH    "/ota_meta.txt"  // companion: expected byte count (decimal)
#define OTA_NVS_PATH     "/ota_nvs.bin"   // staged NVS settings binary (optional)

// Set by performPullOtaCheck() when a firmware has been fully downloaded to SD.
// Cleared by performPullOtaFlash() on success or unrecoverable error.
// Also set at startup when a previously staged file is found on SD.
static volatile bool s_ota_pending = false;

// Sentinel values for the LED/beep runtime-state PIDs (0x84 / 0x85), the
// connection-type PID (0x88), and the OBD/CAN/standby PIDs (0x89/0x8a/0x8b).
// Initialised to -1 so the first call to process()
// always adds the PIDs to the buffer.  Reset back to -1 by initialize() and
// whenever a new WiFi or cellular connection is established (in telemetry()), so
// the current state is always re-sent after a reconnect Ã¢â‚¬â€ preventing a permanent
// "Unbekannt" IST-Status in Home Assistant when HA is reloaded while the device
// is connected (HA loses diag state but device won't resend unchanged values
// unless the sentinels are reset).
static int8_t s_lastLedWhite = -1;
static int8_t s_lastBeep     = -1;
static int8_t s_lastConnType = -1;  // PID_CONN_TYPE sentinel: 1=WiFi, 2=Cellular
static int8_t  s_lastObd         = -1;   // PID_OBD_STATE sentinel
static int16_t s_lastStandbyTime = -1;   // PID_STANDBY_TIME sentinel (seconds)
static int8_t  s_lastDeepStandby = -1;   // PID_DEEP_STANDBY sentinel

// Inject-on-next-packet flag: set whenever the sentinels are reset (new
// connection established or session start).  The telemetry task checks this
// flag and injects LED/beep/conn-type/SD PIDs directly into the outgoing
// CStorage packet before store.tailer(), guaranteeing these IST-Status values
// are in the FIRST transmitted packet regardless of which buffer getNewest()
// picks up.  Without this, a race between process() (updating sentinels and
// filling buffers) and the telemetry loop (slow OTA check over cellular delays
// getNewest()) causes the sentinel-triggered buffer to be overwritten before it
// is transmitted Ã¢â‚¬â€ leaving HA with "Unbekannt" for LED/beep/SD indefinitely.
static volatile bool s_send_state_pids = false;

// Cached SD card capacity/free space (MiB) Ã¢â‚¬â€ updated by process() each time it
// emits PID_SD_TOTAL_MB / PID_SD_FREE_MB.  Read (not written) by the telemetry
// inject block so it does not need to access the SD SPI bus from the wrong task.
static uint32_t s_cachedSdTotalMb = 0;
static uint32_t s_cachedSdFreeMb  = 0;

// Pull-OTA constants used in initialize(), standby(), performPullOtaFlash(),
// and performPullOtaCheck().  Defined here (before any function body) so that
// all translation-unit uses see them regardless of source order.
// Minimum plausible firmware binary size Ã¢â‚¬â€ rejects short error pages returned
// instead of the real binary.
#define PULL_OTA_MIN_FW_SIZE       65536U   // 64 KB
// Chunk size for SD download and SDÃ¢â€ â€™flash write loops.
#define PULL_OTA_CHUNK_SIZE        4096U    // 4 KB
// Per-chunk receive timeout when streaming from the network socket.
#define PULL_OTA_CHUNK_TIMEOUT_MS  30000U   // 30 s
// Delay after setting s_ota_active to let the telemetry task yield its SSL
// connections before Update.begin() allocates flash partition memory.
#define OTA_TELEMETRY_YIELD_DELAY_MS 1000U

// Shared chunk buffer used by the download (to SD) and flash (from SD) phases.
// File-scope static so both functions share one allocation.
static uint8_t s_otaChunkBuf[PULL_OTA_CHUNK_SIZE];

bool serverSetup(IPAddress& ip);
void serverProcess(int timeout);
void processMEMS(CBuffer* buffer);
bool processGPS(CBuffer* buffer);
void processBLE(int timeout);
// Forward declaration: defined in the Pull-OTA section below; called from
// telemetry() before its point of definition.
bool performPullOtaCheck();
#if STORAGE == STORAGE_SD
// Forward declaration: defined in the Pull-OTA section below.
static bool performPullOtaFlash();
#endif

class State {
public:
  bool check(uint16_t flags) { return (m_state & flags) == flags; }
  void set(uint16_t flags) { m_state |= flags; }
  void clear(uint16_t flags) { m_state &= ~flags; }
  uint16_t m_state = 0;
};

FreematicsESP32 sys;

// ---------------------------------------------------------------------------
// TEMPORARY bench test (2026-09-22) - saved plan's point 1: does pull-OTA's
// meta.json check work at all over cellular? CellHTTP already exists in the
// library (built/hardened for TeleClientHTTP mode - see CHANGELOG.md's
// CCHOPEN/CCHSTART fixes) but has never been wired into anything in THIS
// build (TeleClientUDP - teleClient.cell is CellUDP, not CellHTTP). This
// fetches meta.json only (no firmware download, no flash, no meta.json
// "available" handling) via a SEPARATE CellHTTP object attached to the SAME
// physical modem as the live teleClient.cell (CellUDP) session, via
// CellSIMCOM::attach() (NOT begin() - see its comment in FreematicsNetwork.h
// for why begin()'s power-toggle/purge/handshake are each unsafe to repeat
// on a live modem).
//
// Known, accepted risk (verified by reading xbReceive()/CellHTTP::open()/
// send() in FreematicsNetwork.cpp, not assumed): there is exactly ONE
// physical UART and ONE driver-level RX ring buffer shared by both objects,
// with no per-session demultiplexing at all - CellHTTP::open()/send() call
// m_device->xbPurge() as part of their own (already proven, unmodified) AT
// sequence on the SIM7600 path, and every sendCommand() on either object
// drains whatever is currently queued in that one shared buffer regardless
// of which logical session it belongs to. The modem's radio can genuinely
// run multiple simultaneous data sessions (that part is real), but the
// serial control link between the ESP32 and the modem - where every AT
// command and its response travels - is one shared wire, not something
// this library virtualizes per-object. While this test runs, an incoming
// UDP datagram for teleClient.cell's live session (a server ACK, or even
// another OTA_READY push) could be silently dropped. Consequence is bounded
// and recoverable (Traccar re-delivers a queued command on the device's
// next packet), not a session/registration loss - this is a deliberate,
// informed bench-test tradeoff, not an oversight. Remove once cellular OTA
// is either wired into performPullOtaCheck() for real (needing a proper
// answer to this shared-UART question first) or shelved.
//
// 2026-09-22 live test found the actual modem on this device is a
// SIM7670E-LN, not SIM7600 - CellHTTP::init()/open()/send()/receive() each
// branch on m_type per modem family, and attach() (below) MUST be given the
// real type (via teleClient.cell.type(), already detected by its own
// begin()) or it silently runs the wrong AT-command dialect - confirmed
// live: attach() without a type argument defaults to CELL_SIM7600's
// AT+CCHOPEN family, which this SIM7670E-LN modem does not implement the
// same way, and open() failed outright.
//
// Called once per boot, right after cellular first connects - see its call
// site near "[CELL] In service" further down. Deliberately NOT gated on
// WiFi state - the whole point is testing this independent of it.
static void testCellularOtaMeta()
{
  if (!otaToken[0] || !otaHost[0]) {
    Serial.println("[CELL-OTA-TEST] OTA_TOKEN/OTA_HOST not provisioned, skipping");
    return;
  }

  // TEMP 2026-09-22 diagnostic: AT+CLAC via the ALREADY-working teleClient.cell
  // (CellUDP, live and begin()'d since boot) - decisive test of whether
  // AT+CLAC/AT+HTTPINIT failing is caused by anything about the new
  // otaCellClient/attach() object, or is a genuine modem/firmware response
  // regardless of which object sends it.
  teleClient.cell.rawAT("AT+CLAC\r", 3000);
  Serial.println("[CELL-OTA-TEST] AT+CLAC via teleClient.cell raw buffer:");
  Serial.println(teleClient.cell.rawBuffer());
  Serial.println("[CELL-OTA-TEST] AT+CLAC via teleClient.cell raw buffer END");

  static CellHTTP otaCellClient;
  // 2026-09-22 finding: this device's real modem (SIM7670E-LNGV, firmware
  // V1.9.05) has AT+HTTPINIT/AT+CLAC confirmed non-functional (plain ERROR,
  // reproduced via teleClient.cell directly above - not an artifact of this
  // test's own code). Forcing CELL_SIM7600 (AT+CCH* raw-SSL-socket family)
  // was also tried live: partial support found - AT+CSSLCFG="sslversion"/
  // "authmode" succeed, but "ignorertctime"/"alpnprotocol"/"ciphersuite" all
  // return ERROR (unrecognized parameter names on this firmware), so
  // AT+CCHSTART then fails too. AT+CCHOPEN itself DOES return a real,
  // structured "+CCHOPEN: <session>,<errcode>" response (not blank ERROR)
  // when tried anyway - meaning the AT+CCH* command family is genuinely
  // partially present on this modem, just with a different supported-
  // parameter set than SIM7600E-H's firmware. Reverted to the real detected
  // type here rather than leaving the CELL_SIM7600 force in place, since
  // that was a one-off experiment, not a working configuration - the actual
  // fix (which CSSLCFG parameters this firmware needs) is still unknown,
  // needs real SIMCom A76XX/SIM7670-LNGV documentation this session
  // couldn't read (scanned-image PDFs, no OCR tool available) or a lot more
  // live AT-parameter trial and error neither this session nor the user
  // wanted to keep doing blindly.
  Serial.printf("[CELL-OTA-TEST] Attaching to shared modem (type=%s)...\n", teleClient.cell.deviceName());
  if (!otaCellClient.attach(&sys, teleClient.cell.type())) {
    Serial.println("[CELL-OTA-TEST] attach() failed (OOM)");
    return;
  }
  // TEMP 2026-09-22 diagnostic: one-time purge right before this object's
  // very first command, to test whether stale leftover bytes already
  // sitting in the shared UART driver's RX ring buffer (e.g. from
  // teleClient.cell's own recent traffic) are being misread as the
  // response to THIS object's first AT command - every AT+CLAC/AT+HTTPINIT
  // attempt so far has failed with a bare ERROR that arrives suspiciously
  // fast, consistent with reading already-buffered stale bytes rather than
  // a genuine modem round-trip. attach() deliberately does NOT do this
  // itself (see its own comment - a purge is exactly the kind of
  // other-session-disrupting operation begin() does that attach() exists to
  // avoid), so it's tested here, once, explicitly, only for this bench test.
  sys.xbPurge();
  Serial.println("[CELL-OTA-TEST] Purged shared UART buffer");

  Serial.println("[CELL-OTA-TEST] init() - SSL context setup...");
  otaCellClient.init();

  Serial.printf("[CELL-OTA-TEST] open() %s:%u...\n", otaHost, (unsigned)otaPort);
  if (!otaCellClient.open(otaHost, otaPort)) {
    Serial.println("[CELL-OTA-TEST] open() FAILED");
    return;
  }
  Serial.println("[CELL-OTA-TEST] open() OK");

  // Same build-string encoding as performPullOtaCheck()'s real meta.json
  // request, for a faithful test of the actual production path (not just
  // "does TLS work") - see that function's comment for why spaces are
  // percent-encoded.
  char metaPath[448];
  {
    char buildEnc[48];
    int bi = 0;
    for (const char* p = __DATE__ " " __TIME__; *p && bi < (int)sizeof(buildEnc) - 4; p++) {
      if (*p == ' ') {
        buildEnc[bi++] = '%'; buildEnc[bi++] = '2'; buildEnc[bi++] = '0';
      } else {
        buildEnc[bi++] = *p;
      }
    }
    buildEnc[bi] = 0;
    snprintf(metaPath, sizeof(metaPath),
             "/api/freematics/ota_pull/%s/meta.json?build=%s&variant=%s",
             otaToken, buildEnc, FIRMWARE_VERSION);
  }

  if (!otaCellClient.send(METHOD_GET, otaHost, otaPort, metaPath)) {
    Serial.println("[CELL-OTA-TEST] send() FAILED");
    otaCellClient.close();
    return;
  }
  Serial.println("[CELL-OTA-TEST] send() OK, waiting for response...");

  // 2026-09-22: HTTP_CONN_TIMEOUT (5s) was too short here too, same
  // "SIMCom's documented Max Response Time is much longer than we assumed"
  // reasoning as CCHOPEN_TIMEOUT_SIM7670 (open() first succeeded with a real
  // 786ms handshake once that timeout was fixed, but receive() then still
  // failed at 5s) - reuse the same 60s ceiling rather than inventing a
  // third magic timeout constant.
  int bytes = 0;
  char* body = otaCellClient.receive(&bytes, CCHOPEN_TIMEOUT_SIM7670);
  if (!body) {
    Serial.println("[CELL-OTA-TEST] receive() FAILED (no response)");
    otaCellClient.close();
    return;
  }
  Serial.printf("[CELL-OTA-TEST] SUCCESS: HTTP %u, %d bytes\n", (unsigned)otaCellClient.code(), bytes);
  Serial.println(body);
  otaCellClient.close();
}

class OBD : public COBD
{
protected:
  void idleTasks()
  {
    // do some quick tasks while waiting for OBD response
#if ENABLE_MEMS
    processMEMS(0);
#endif
    processBLE(0);
  }
};

OBD obd;

MEMS_I2C* mems = 0;

#if STORAGE == STORAGE_SPIFFS
SPIFFSLogger logger;
#elif STORAGE == STORAGE_SD
SDLogger logger;
#endif

State state;

// 2026-09-14: exposes SD-log writing (logger.logEvent(), gated on
// STATE_STORAGE_READY - same pattern used everywhere else in this file) to
// teleclient.cpp, a separate translation unit that cannot see the State/
// Logger class definitions above (they're defined directly in this .ino,
// not in a shared header). Root-caused via a full code-trace: the entire
// WiFi/cellular connect path in TeleClientUDP::connect()/notify()
// (teleclient.cpp) only ever did Serial.println() on failure - never
// logger.logEvent() - so every connection failure there was invisible to
// /api/events, making a 40+-minute total telemetry silence look identical
// to "nothing is even trying" from the SD log alone. Not static: needs
// external linkage so teleclient.cpp's extern declaration can reach it.
void logNetEvent(const char* msg)
{
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    logger.logEvent(msg);
    // Flush immediately: this function exists specifically to diagnose
    // crashes/resets that happen moments later (e.g. mid-connect) - an
    // unflushed event is invisible after a reset, defeating the purpose.
    logger.flush();
  }
#endif
}

// Live-diagnostic wrapper for dataserver.cpp's /api/control?cmd=STATE? -
// exposes the raw state.m_state bitmask and the teleClient login flag over
// HTTP without needing a Serial connection.  Added 2026-09-15 while
// debugging "on WiFi, nothing reaches Traccar" with the device outside the
// car (no USB/Serial available) - reading telelogger.ino/teleclient.cpp
// alone could not tell whether the WiFi connect branch in telemetry() was
// ever actually being reached and running.
uint16_t getStateBits()
{
  return state.m_state;
}

bool teleLoginState()
{
  return teleClient.login;
}

// More live-diagnostic getters for dataserver.cpp's /api/info - added
// 2026-09-15 per request: "nech clovek vidi ze nieco ide, statusy, ip, gps
// satelity pozicia, logovanie, kolko odoslalo dat akou cestou" (a status
// view without needing Serial - IP, GPS satellites/position, how much data
// was sent and over which route). JSON output is cheap on memory (just a
// few extra snprintf fields on the existing /api/info buffer), unlike a
// full HTML status page.
const char* teleConnMethod()
{
  if (state.check(STATE_WIFI_CONNECTED)) return "wifi";
  if (state.check(STATE_CELL_CONNECTED)) return "cell";
  return "none";
}

const char* teleNetOperator() { return netop.c_str(); }
const char* teleIpAddress()   { return ip.c_str(); }
int16_t teleRssi()            { return rssi; }
uint32_t teleTxCount()        { return teleClient.txCount; }
uint32_t teleTxBytes()        { return teleClient.txBytes; }
uint32_t teleRxBytes()        { return teleClient.rxBytes; }

int gpsSatCount() { return gd ? (int)gd->sat : 0; }
float gpsLat()    { return gd ? gd->lat : 0; }
float gpsLng()    { return gd ? gd->lng : 0; }
// 0xFFFFFFFF sentinel = no GPS fix data yet at all (gd null or never timestamped).
uint32_t gpsAgeMs() { return (gd && gd->ts) ? (uint32_t)(millis() - gd->ts) : 0xFFFFFFFF; }

// ESP32 WiFi disconnect reason codes (wifi_err_reason_t, esp_wifi_types.h) -
// sent as telemetry (see wifiDisconnectPending, declared earlier alongside
// wifiConnect()) so repeated WiFi drops during real-world (in-car,
// engine-running) testing can be diagnosed remotely instead of guessed at.
// Common values worth knowing on sight:
//   2   = AUTH_EXPIRE (AP-initiated re-auth failure)
//   3   = AUTH_LEAVE
//   6/7 = ASSOC_EXPIRE / NOT_AUTHED
//   8   = ASSOC_LEAVE (station left - e.g. our own disconnect() call)
//   15  = 4WAY_HANDSHAKE_TIMEOUT (wrong password OR AP not responding in time)
//   200 = BEACON_TIMEOUT (lost the AP's signal - range/interference)
//   201 = NO_AP_FOUND
//   202 = AUTH_FAIL
//   203 = ASSOC_FAIL
//   204 = HANDSHAKE_TIMEOUT
// Full list: esp-idf esp_wifi_types.h WIFI_REASON_*.
static void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    wifiDisconnectReason = info.wifi_sta_disconnected.reason;
    wifiDisconnectCount++;
    wifiDisconnectPending = true;
    Serial.printf("[WIFI] Disconnected, reason=%d (count=%d)\n",
        (int)wifiDisconnectReason, (int)wifiDisconnectCount);
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) {
      char diag[48];
      snprintf(diag, sizeof(diag), "WIFI DISCONNECT REASON=%d N=%d",
          (int)wifiDisconnectReason, (int)wifiDisconnectCount);
      logger.logEvent(diag);
    }
#endif
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    // Connection attempt succeeded - resume BT now rather than waiting for
    // whichever ClientWIFI::setup() call happens to poll next. Matters most
    // for the periodic RSSI-check wifiConnect() call site further down,
    // which does not call setup() right after - without this, BT would stay
    // paused until some later, unrelated setup() call happened to run.
    // WiFi.setSleep(true) MUST come before ble_resume() (see
    // ClientWIFI::begin()'s comment for why) - duplicated here rather than
    // shared with ClientWIFI::setup()'s copy since this fires from a
    // different translation unit. ble_resume() is a no-op if BT was already
    // resumed (e.g. setup() got there first) or was never paused, so this
    // is safe to call on every GOT_IP, not just the first.
    WiFi.setSleep(true);
    ble_resume();
    // Adaptive network preference (Problem 1 fix, see the block above
    // wifiCurrentIdx's declaration): the network that just got an IP
    // becomes the preferred one for the next wifiConnect() call, with the
    // failure streak reset - wifiCurrentIdx was set synchronously by
    // wifiConnect() right before it called teleClient.wifi.begin(), so it
    // reflects whichever network this GOT_IP event is actually for.
    wifiPreferredIdx = wifiCurrentIdx;
    wifiPreferredFails = 0;
  }
}

// Sent once (see wifiScanPending, declared earlier alongside wifiConnect())
// as regular telemetry over whatever channel is actually up (cellular
// included) - the whole point of this scan is diagnosing a WiFi
// connectivity problem, so it has to be readable even when WiFi itself is
// the thing not working (SD-card/Serial logging alone wouldn't help there
// without physical device access, same reasoning as the UDS diagnostic
// PIDs elsewhere in this file). wifiScanCount = networks found (0 if
// none); wifiScanTargetRssi = RSSI of the configured SSID (wifiSSID/
// wifiSSID2) if it showed up in the scan, or -999 as a sentinel meaning
// "not seen at all". Scans once (see the `scanned` static in
// wifiConnect()), not on every reconnect attempt - WiFi.scanNetworks()
// takes a couple of seconds, not worth paying on every retry.
static void wifiScanAndLog()
{
  // Explicit STA mode before scanning - scanNetworks() can otherwise return
  // WIFI_SCAN_FAILED (-2) if called before the radio is in a scan-capable
  // mode this early in the connect sequence. Harmless/idempotent if the
  // radio is already in STA mode (which is what the connect below needs
  // anyway).
  WiFi.mode(WIFI_STA);
  // 2026-09-14: WiFi.mode() only *requests* the mode change - the underlying
  // esp_wifi_start() (radio init, RF calibration, buffer allocation) runs on
  // the WiFi event task and is not guaranteed done by the time mode() returns,
  // especially on a fresh cold boot (confirmed via real-drive telemetry:
  // io800=-2/WIFI_SCAN_FAILED on a cold boot in the car, but io800=5 - a
  // clean scan - when the device had already been running a while on the
  // bench with the driver already warm). The mode() call alone doesn't fix
  // that race, so retry the scan itself a few times with a short wait
  // in between, giving the driver time to actually come up, before
  // reporting a real failure.
  Serial.println("[WIFI] Scanning for networks...");
  int n = WiFi.scanNetworks();
  for (byte scanRetry = 0; n < 0 && scanRetry < 3; scanRetry++) {
    Serial.printf("[WIFI] Scan attempt failed (ret=%d), retrying...\n", n);
    delay(300);
    n = WiFi.scanNetworks();
  }
  // Preserve a negative return as-is (WIFI_SCAN_FAILED=-2, WIFI_SCAN_RUNNING
  // =-1) instead of clamping to 0 - "found 0 networks" and "the scan call
  // itself failed" are different problems and worth telling apart in the
  // telemetry/log.
  wifiScanCount = n;
  wifiScanTargetRssi = -999;
  if (n <= 0) {
    Serial.printf("[WIFI] Scan: no networks found (ret=%d)\n", n);
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) {
      char diag0[24];
      snprintf(diag0, sizeof(diag0), "WIFI SCAN=%d", n);
      logger.logEvent(diag0);
    }
#endif
    WiFi.scanDelete();
    wifiScanPending = true;
    return;
  }
  char diag[200];
  int len = snprintf(diag, sizeof(diag), "WIFI SCAN=%d", n);
  // List up to the first few strongest-looking entries (as returned by the
  // scan, typically already sorted by RSSI) - enough to see what's around
  // without risking the event-log line length limit (FileLogger::logEvent
  // truncates the combined line beyond ~132 chars).
  int shown = n < 4 ? n : 4;
  for (int i = 0; i < shown && len < (int)sizeof(diag) - 32; i++) {
    len += snprintf(diag + len, sizeof(diag) - len, " %s(%d)",
        WiFi.SSID(i).c_str(), WiFi.RSSI(i));
  }
  Serial.print("[WIFI] "); Serial.println(diag);
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) logger.logEvent(diag);
#endif
  // Look for the configured SSID(s) specifically among the scan results,
  // regardless of whether they were in the first few logged above.
  for (int i = 0; i < n; i++) {
    String found = WiFi.SSID(i);
    if ((wifiSSID[0] && found == wifiSSID) || (wifiSSID2[0] && found == wifiSSID2)) {
      wifiScanTargetRssi = WiFi.RSSI(i);
      break;
    }
  }
  WiFi.scanDelete();
  wifiScanPending = true;
}

// ---------------------------------------------------------------------------
// Volatile request flags set by the httpd task (handlerControl) to request
// telemetry state changes.  All actual state.m_state modifications are
// applied by the net task at the top of its main loop so that m_state is
// always written from a single task context Ã¢â‚¬â€ same thread-safety pattern
// as s_ota_active.
// ---------------------------------------------------------------------------
static volatile bool s_http_standby_enter = false;
static volatile bool s_http_standby_exit  = false;
// Set by handlerControl (cmd=OTA_CHECK_NOW) to make the periodic pull-OTA
// check in telemetry() run on its very next pass instead of waiting for
// otaCheckIntervalS - lets an external push-decision service (one that
// knows, from Traccar + each device's own /api/info, which devices are
// actually out of date) skip the wait instead of just editing registry.json
// and hoping the device polls soon. Consumed (cleared) by telemetry() the
// moment it fires; does not bypass otaToken[0]/STATE_WIFI_CONNECTED, so it's
// a no-op if OTA isn't provisioned or WiFi isn't up.
static volatile bool s_ota_check_now = false;

// Called from handlerControl (httpd task) to pause or resume the telemetry task.
void httpControlStandby(bool enter) {
    if (enter) {
        s_http_standby_enter = true;
        s_http_standby_exit  = false;
    } else {
        s_http_standby_exit  = true;
        s_http_standby_enter = false;
    }
}

// Returns true when the telemetry task is in (or has been requested to enter)
// standby mode.  Called from handlerControl to answer "ON?" queries.
bool httpIsStandby() {
    return state.check(STATE_STANDBY) || s_http_standby_enter;
}

// Called from handlerControl (httpd task, cmd=OTA_CHECK_NOW) to request an
// immediate pull-OTA check on the telemetry task's next pass.
void httpTriggerOtaCheckNow() {
    s_ota_check_now = true;
}

void printTimeoutStats()
{
  Serial.print("Timeouts: OBD:");
  Serial.print(timeoutsOBD);
  Serial.print(" Network:");
  Serial.println(timeoutsNet);
}

void beep(int duration)
{
    // turn on buzzer at 2000Hz frequency 
    sys.buzzer(2000);
    delay(duration);
    // turn off buzzer
    sys.buzzer(0);
}

#if LOG_EXT_SENSORS
void processExtInputs(CBuffer* buffer)
{
#if LOG_EXT_SENSORS == 1
  uint8_t levels[2] = {(uint8_t)digitalRead(PIN_SENSOR1), (uint8_t)digitalRead(PIN_SENSOR2)};
  buffer->add(PID_EXT_SENSORS, ELEMENT_UINT8, levels, sizeof(levels), 2);
#elif LOG_EXT_SENSORS == 2
  uint16_t reading[] = {adc1_get_raw(ADC1_CHANNEL_0), adc1_get_raw(ADC1_CHANNEL_1)};
  Serial.print("GPIO0:");
  Serial.print((float)reading[0] * 3.15 / 4095 - 0.01);
  Serial.print(" GPIO1:");
  Serial.println((float)reading[1] * 3.15 / 4095 - 0.01);
  buffer->add(PID_EXT_SENSORS, ELEMENT_UINT16, reading, sizeof(reading), 2);
#endif
}
#endif

/*******************************************************************************
  HTTP API
*******************************************************************************/
#if ENABLE_HTTPD
int handlerLiveData(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int n = snprintf(buf, bufsize, "{\"obd\":{\"vin\":\"%s\",\"battery\":%.1f,\"pid\":[", vin, batteryVoltage);
    uint32_t t = millis();
    for (int i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | obdData[i].pid, obdData[i].value, (unsigned int)(t - obdData[i].ts));
    }
    n--;
    n += snprintf(buf + n, bufsize - n, "]}");
#if ENABLE_MEMS
    if (accCount) {
      n += snprintf(buf + n, bufsize - n, ",\"mems\":{\"acc\":[%d,%d,%d],\"stationary\":%u}",
          (int)((accSum[0] / accCount - accBias[0]) * 100), (int)((accSum[1] / accCount - accBias[1]) * 100), (int)((accSum[2] / accCount - accBias[2]) * 100),
          (unsigned int)(millis() - lastMotionTime));
    }
#endif
    if (gd && gd->ts) {
      n += snprintf(buf + n, bufsize - n, ",\"gps\":{\"utc\":\"%s\",\"lat\":%f,\"lng\":%f,\"alt\":%f,\"speed\":%f,\"sat\":%d,\"age\":%u}",
          isoTime, gd->lat, gd->lng, gd->alt, gd->speed, (int)gd->sat, (unsigned int)(millis() - gd->ts));
    }
    buf[n++] = '}';
    param->contentLength = n;
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}
#endif

/*******************************************************************************
  Reading and processing OBD data
*******************************************************************************/
#if ENABLE_OBD
// Forward declaration: defined later, right before processGPS(); parses a
// UDS ReadDataByIdentifier positive response (see the ODOMETER PROCESSING
// block below).
static long parseUdsHexValue(const char* resp, uint16_t did);

// Forward declaration: defined later alongside processGPS(); accumulates
// GPS-based distance since boot, used as a fallback when OBD/UDS odometer
// reads fail (see the ODOMETER PROCESSING block below).
uint32_t gpsOdometerKm();

void processOBD(CBuffer* buffer)
{
  static int idx[2] = {0, 0};
  int tier = 1;
  for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
    if (obdData[i].tier > tier) {
        // reset previous tier index
        idx[tier - 2] = 0;
        // keep new tier number
        tier = obdData[i].tier;
        // move up current tier index
        i += idx[tier - 2]++;
        // check if into next tier
        if (obdData[i].tier != tier) {
            idx[tier - 2]= 0;
            i--;
            continue;
        }
    }
    byte pid = obdData[i].pid;
    if (!obd.isValidPID(pid)) continue;
    int value;
    if (obd.readPID(pid, value)) {
        obdData[i].ts = millis();
        obdData[i].value = value;
        buffer->add((uint16_t)pid | 0x100, ELEMENT_INT32, &value, sizeof(value));
    } else {
        timeoutsOBD++;
        printTimeoutStats();
        break;
    }
    if (tier > 1) break;
  }

  // Poll vehicle-specific extra PIDs (from VEHICLE_PIDS NVS key).
  // Use a static index to cycle through all vehicle PIDs one per call (tier-3 pacing).
  if (vehicleObdDataCount > 0) {
    static int vehiclePidIdx = 0;
    if (vehiclePidIdx >= vehicleObdDataCount) vehiclePidIdx = 0;
    byte vpid = vehicleObdData[vehiclePidIdx].pid;
    if (obd.isValidPID(vpid)) {
      int vval;
      if (obd.readPID(vpid, vval)) {
        vehicleObdData[vehiclePidIdx].ts    = millis();
        vehicleObdData[vehiclePidIdx].value = vval;
        buffer->add((uint16_t)vpid | 0x100, ELEMENT_INT32, &vval, sizeof(vval));
      }
    }
    vehiclePidIdx++;
  }

  // VAG/PSA UDS odo/fuel module (vag_odo_fuel.cpp/psa_odo_fuel.cpp) removed
  // entirely 2026-09-16: confirmed dead end on this hardware. The Freematics
  // ONE+'s CAN goes through a closed-firmware STM32 co-processor
  // ("OBD2USART") that does not support UDS on a non-default CAN header -
  // see DIY_Telemetry_Build/04_Fyzicky_bypass_povodneho_Freematics/SUPIS.md.
  // Every UDS attempt through this AT-command interface is inconclusive by
  // construction, on any vehicle, until that co-processor is bypassed in
  // hardware. Single unified build/OTA target now (no more VAG/PSA split -
  // see FIRMWARE_VERSION in config.h).

  int kph = obdData[0].value;
  if (kph >= 2) lastMotionTime = millis();
}
#endif

// Parses a UDS ReadDataByIdentifier positive response (SID 0x62) for the
// given DID. The response text coming back from obd.link->sendCommand() is
// an ASCII-hex string such as "62 05 05 00 01 86 A0\r>" or "620505000186A0"
// (formatting/spacing/prompt characters vary by adapter) Ã¢â‚¬â€ NOT a decimal
// number, so atoi() on it silently returns garbage (it stops at the first
// non-digit character, e.g. "62" -> 62). This strips everything that isn't
// a hex digit, verifies the response is a positive match for the requested
// DID, and returns the trailing data bytes as a number. Returns -1 on a
// negative response (0x7F), a DID mismatch, or a malformed/short response.
static long parseUdsHexValue(const char* resp, uint16_t did)
{
  if (!resp) return -1;
  char hex[32];
  int n = 0;
  for (const char* p = resp; *p && n < (int)sizeof(hex) - 1; p++) {
    if (isxdigit((unsigned char)*p)) hex[n++] = *p;
  }
  hex[n] = 0;
  if (n < 6) return -1; // need at least SID(2 hex chars) + DID(4 hex chars)

  char sidStr[3] = { hex[0], hex[1], 0 };
  if (strtoul(sidStr, nullptr, 16) != 0x62) return -1; // 0x7F = negative response, or garbage

  char didStr[5] = { hex[2], hex[3], hex[4], hex[5], 0 };
  if (strtoul(didStr, nullptr, 16) != did) return -1; // response for a different DID

  if (n <= 6) return -1; // positive response but no data bytes attached
  return strtol(hex + 6, nullptr, 16);
}


bool initGPS()
{
  // start GNSS receiver
  if (sys.gpsBeginExt()) {
    Serial.println("GNSS:OK(E)");
  } else if (sys.gpsBegin()) {
    Serial.println("GNSS:OK(I)");
  } else {
    Serial.println("GNSS:NO");
    return false;
  }
  return true;
}

// Distance driven since boot, accumulated from consecutive valid GPS fixes
// (haversine great-circle distance between fixes). This is NOT the vehicle's
// true lifetime odometer Ã¢â‚¬â€ it resets to 0 on every reboot/deep-sleep wake Ã¢â‚¬â€
// but it's a usable fallback for vehicles/situations where the OBD/UDS
// odometer read fails, so Traccar still receives an increasing distance
// value instead of nothing at all.
static double s_gpsOdometerKm = 0.0;

uint32_t gpsOdometerKm()
{
  return (uint32_t)s_gpsOdometerKm;
}

static double haversineKm(float lat1, float lng1, float lat2, float lng2)
{
  const double R = 6371.0; // Earth radius, km
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLng = (lng2 - lng1) * DEG_TO_RAD;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
             sin(dLng / 2) * sin(dLng / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// Geofence-WiFi (2026-09-22): true if (lat,lon) falls within any synced
// knownLocations[] entry's radius, with *outWifiIdx set to which of the
// device's own two configured networks (wifiSSID/wifiSSID2) to try there.
// NOT called from anywhere yet - the data plumbing (syncKnownLocations(),
// near the OTA config globals) lands first; wiring this into the actual
// standby/wake decision is a separate, deferred step (see the saved plan:
// after cellular OTA-pull is confirmed and WiFi-off-in-standby exists to
// wake from in the first place).
static bool findNearbyKnownLocation(float lat, float lon, uint8_t* outWifiIdx)
{
  for (uint8_t i = 0; i < knownLocationCount; i++) {
    double distM = haversineKm(lat, lon, knownLocations[i].lat, knownLocations[i].lon) * 1000.0;
    if (distM <= knownLocations[i].radiusM) {
      if (outWifiIdx) *outWifiIdx = knownLocations[i].wifiIdx;
      return true;
    }
  }
  return false;
}

bool processGPS(CBuffer* buffer)
{
  static uint32_t lastGPStime = 0;
  static float lastGPSLat = 0;
  static float lastGPSLng = 0;

  if (!gd) {
    lastGPStime = 0;
    lastGPSLat = 0;
    lastGPSLng = 0;
  }
#if GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    // read parsed GPS data
    if (!sys.gpsGetData(&gd)) {
      return false;
    }
  }
#else
    if (!teleClient.cell.getLocation(&gd)) {
      return false;
    }
#endif
  if (!gd || lastGPStime == gd->time) return false;
  if (gd->date) {
    // generate ISO time string
    char *p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
        (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
        (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100);
    unsigned char tenth = (gd->time % 100) / 10;
    if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
    *p = 'Z';
    *(p + 1) = 0;
  }
  if (gd->lng == 0 && gd->lat == 0) {
    // No position fix yet Ã¢â‚¬â€œ still log satellite count and HDOP so that
    // sensor.gps_satellites / sensor.gps_hdop in HA show the GPS is actively
    // searching even before the first valid position is obtained.
    if (buffer) {
      if (gd->sat) buffer->add(PID_GPS_SAT_COUNT, ELEMENT_UINT8, &gd->sat, sizeof(uint8_t));
      if (gd->hdop) buffer->add(PID_GPS_HDOP, ELEMENT_UINT8, &gd->hdop, sizeof(uint8_t));
    }
    if (gd->date) {
      Serial.print("[GNSS] ");
      Serial.print(isoTime);
      Serial.print(" SATS:");
      Serial.println(gd->sat);
    }
    return false;
  }
  if ((lastGPSLat || lastGPSLng) && (abs(gd->lat - lastGPSLat) > 0.001 || abs(gd->lng - lastGPSLng) > 0.001)) {
    // invalid coordinates data
    lastGPSLat = 0;
    lastGPSLng = 0;
    return false;
  }
  if (lastGPSLat || lastGPSLng) {
    s_gpsOdometerKm += haversineKm(lastGPSLat, lastGPSLng, gd->lat, gd->lng);
  }
  lastGPSLat = gd->lat;
  lastGPSLng = gd->lng;

  float kph = gd->speed * 1.852f;
  if (kph >= 2) lastMotionTime = millis();

  if (buffer) {
    buffer->add(PID_GPS_TIME, ELEMENT_UINT32, &gd->time, sizeof(uint32_t));
    // Date (2026-09-22): PID_GPS_DATE was never sent, only PID_GPS_TIME. The
    // server (FreematicsProtocolDecoder) defaults a position's date to "today"
    // whenever no date field is present in the packet - fine for live traffic,
    // but any record that sits in the SD/RAM backlog and gets transmitted on a
    // later calendar day keeps its original GPS time with the WRONG (transmit)
    // date stitched on, landing the fixTime on a completely different moment
    // than when it was recorded - confirmed 2026-09-22 with backlog records
    // from Sep 21 replayed the next morning that decoded into that afternoon's
    // live drive window and corrupted the trip/stop reports.
    buffer->add(PID_GPS_DATE, ELEMENT_UINT32, &gd->date, sizeof(uint32_t));
    buffer->add(PID_GPS_LATITUDE, ELEMENT_FLOAT, &gd->lat, sizeof(float));
    buffer->add(PID_GPS_LONGITUDE, ELEMENT_FLOAT, &gd->lng, sizeof(float));
    buffer->add(PID_GPS_ALTITUDE, ELEMENT_FLOAT_D1, &gd->alt, sizeof(float)); /* m */
    buffer->add(PID_GPS_SPEED, ELEMENT_FLOAT_D1, &kph, sizeof(kph));
    buffer->add(PID_GPS_HEADING, ELEMENT_UINT16, &gd->heading, sizeof(uint16_t));
    if (gd->sat) buffer->add(PID_GPS_SAT_COUNT, ELEMENT_UINT8, &gd->sat, sizeof(uint8_t));
    if (gd->hdop) buffer->add(PID_GPS_HDOP, ELEMENT_UINT8, &gd->hdop, sizeof(uint8_t));
  }
  
  Serial.print("[GNSS] ");
  Serial.print(gd->lat, 6);
  Serial.print(' ');
  Serial.print(gd->lng, 6);
  Serial.print(' ');
  Serial.print((int)kph);
  Serial.print("km/h");
  Serial.print(" SATS:");
  Serial.print(gd->sat);
  Serial.print(" HDOP:");
  Serial.print(gd->hdop);
  Serial.print(" Course:");
  Serial.println(gd->heading);
  //Serial.println(gd->errors);
  lastGPStime = gd->time;
  return true;
}

bool waitMotionGPS(int timeout)
{
  unsigned long t = millis();
  lastMotionTime = 0;
  do {
      serverProcess(100);
    if (!processGPS(0)) continue;
    if (lastMotionTime) return true;
  } while (millis() - t < timeout);
  return false;
}

#if ENABLE_MEMS
void processMEMS(CBuffer* buffer)
{
  if (!state.check(STATE_MEMS_READY)) return;

  // load and store accelerometer data
  float temp;
#if ENABLE_ORIENTATION
  ORIENTATION ori;
  if (!mems->read(acc, gyr, mag, &temp, &ori)) return;
#else
  if (!mems->read(acc, gyr, mag, &temp)) return;
#endif
  deviceTemp = (int)temp;

  accSum[0] += acc[0];
  accSum[1] += acc[1];
  accSum[2] += acc[2];
  accCount++;

  // Update lastMotionTime whenever the instantaneous bias-corrected
  // acceleration exceeds MOTION_THRESHOLD.  This is the fallback motion
  // source when OBD-II and GPS are both unavailable: without it the
  // stationary-timeout logic in process() would put the device into
  // STANDBY (and disconnect WiFi) after ~3 minutes even while the
  // vehicle is driving.  Uses instantaneous values (same approach as
  // waitMotion()) rather than the per-buffer average so that brief
  // manoeuvres (cornering, braking) are detected even across long
  // sampling windows.
  {
    float motion = 0;
    for (byte i = 0; i < 3; i++) {
      float m = acc[i] - accBias[i];
      motion += m * m;
    }
    if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
      lastMotionTime = millis();
    }
  }

  if (buffer) {
    if (accCount) {
      float value[3];
      value[0] = accSum[0] / accCount - accBias[0];
      value[1] = accSum[1] / accCount - accBias[1];
      value[2] = accSum[2] / accCount - accBias[2];
      buffer->add(PID_ACC, ELEMENT_FLOAT_D2, value, sizeof(value), 3);
#if ENABLE_ORIENTATION
      value[0] = ori.yaw;
      value[1] = ori.pitch;
      value[2] = ori.roll;
      buffer->add(PID_ORIENTATION, ELEMENT_FLOAT_D2, value, sizeof(value), 3);
#endif
    }
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accCount = 0;
  }
}

void calibrateMEMS()
{
  if (state.check(STATE_MEMS_READY)) {
    accBias[0] = 0;
    accBias[1] = 0;
    accBias[2] = 0;
    int n;
    unsigned long t = millis();
    for (n = 0; millis() - t < 1000; n++) {
      float acc[3];
      if (!mems->read(acc)) continue;
      accBias[0] += acc[0];
      accBias[1] += acc[1];
      accBias[2] += acc[2];
      delay(10);
    }
    accBias[0] /= n;
    accBias[1] /= n;
    accBias[2] /= n;
    Serial.print("ACC BIAS:");
    Serial.print(accBias[0]);
    Serial.print('/');
    Serial.print(accBias[1]);
    Serial.print('/');
    Serial.println(accBias[2]);
  }
}
#endif

void printTime()
{
  time_t utc;
  time(&utc);
  struct tm *btm = gmtime(&utc);
  if (btm->tm_year > 100) {
    // valid system time available
    char buf[64];
    sprintf(buf, "%04u-%02u-%02u %02u:%02u:%02u",
      1900 + btm->tm_year, btm->tm_mon + 1, btm->tm_mday, btm->tm_hour, btm->tm_min, btm->tm_sec);
    Serial.print("UTC:");
    Serial.println(buf);
  }
}

/*******************************************************************************
  Initializing all data logging components
*******************************************************************************/
void initialize()
{
  // dump buffer data
  bufman.purge();

  // Reset LED/beep/conn_type sentinels so the current state is re-sent in the
  // first buffer of the new telemetry session.  initialize() is called at the
  // start of every logging session (boot and after each network disconnection),
  // so resetting here ensures the device always reports its live LED/beep state
  // and active transport type to HA after a reconnect Ã¢â‚¬â€ preventing a permanent
  // "Unbekannt" IST-Status when HA is reloaded while the device was connected
  // (HA loses diag state, device never resends unchanged values unless the
  // sentinels are reset).
  s_lastLedWhite    = -1;
  s_lastBeep        = -1;
  s_lastConnType    = -1;
  s_lastObd         = -1;
  s_lastStandbyTime = -1;
  // Signal the telemetry task to inject IST-Status PIDs into the very next
  // transmitted packet so they are not lost to the getNewest() race.
  s_send_state_pids = true;

#if ENABLE_MEMS
  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
  }
#endif

#if GNSS == GNSS_STANDALONE
  if (!state.check(STATE_GPS_READY)) {
    if (initGPS()) {
      state.set(STATE_GPS_READY);
    }
  }
#endif

#if ENABLE_OBD
  // initialize OBD communication (skipped when enableObd=false via NVS key OBD_EN)
  if (enableObd && !state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
    // Pre-wake: send OBD2 diagnostic requests BEFORE obd.init() so the vehicle ECU
    // is active when init() tries to communicate.  Many vehicles (especially VAG:
    // VW/Skoda/Audi/Seat, but also BMW and Mercedes) keep the diagnostic CAN bus
    // silent until they receive an initial request Ã¢â‚¬â€ obd.init() alone may time out
    // because it expects the ECU to already be responsive.
    // Sequence: minimal ELM327 reset + two PID requests (RPM=0x0C, coolant=0x05).
    // The ECU may not reply yet (still waking up), which is fine; the intent is to
    // put a frame on the bus and let the ECU's diagnostic stack initialise before
    // obd.init() fires its own PID_SPEED probe.
    if (obd.link) {
      char wbuf[64];
      obd.link->sendCommand("ATZ\r",  wbuf, sizeof(wbuf), 1000); // reset ELM327
      obd.link->sendCommand("ATE0\r", wbuf, sizeof(wbuf),  500); // disable echo
      obd.link->sendCommand("ATH0\r", wbuf, sizeof(wbuf),  500); // disable headers
      obd.link->sendCommand("010C\r", wbuf, sizeof(wbuf),  500); // RPM Ã¢â‚¬â€œ wake req 1
      obd.link->sendCommand("0105\r", wbuf, sizeof(wbuf),  500); // coolant Ã¢â‚¬â€œ wake req 2
      Serial.println("OBD:pre-wake sent");
      delay(100); // give ECU time to start its diagnostic task
    }
    if (obd.init()) {
      Serial.println("OBD:OK");
      state.set(STATE_OBD_READY);
    } else {
      Serial.println("OBD:NO");
      //state.clear(STATE_WORKING);
      //return;
    }
  }
#endif

#if STORAGE != STORAGE_NONE
  if (!state.check(STATE_STORAGE_READY)) {
    // init storage
    if (logger.init()) {
      state.set(STATE_STORAGE_READY);
    }
  }
  if (state.check(STATE_STORAGE_READY)) {
    fileid = logger.begin();
    if (fileid) {
      // Load the catch-up watermark (see wmDoneFileId's own comment above).
      // ESP_ERR_NVS_NOT_FOUND means this is the first boot ever with this
      // feature - on a device that already has a long SD history (this one
      // has hundreds of files from months of use), catching up from file 1
      // would flood the server with its entire lifetime history. So on first
      // activation only, seed the watermark to "everything up to and
      // including the previous file is already handled" and start real
      // gap-tracking fresh from this boot's own file onward.
      esp_err_t wmErr = nvs_get_u32(nvs, "WM_FILE", &wmDoneFileId);
      if (wmErr == ESP_ERR_NVS_NOT_FOUND) {
        wmDoneFileId = (fileid > 1) ? (uint32_t)(fileid - 1) : 0;
        nvs_set_u32(nvs, "WM_FILE", wmDoneFileId);
        nvs_commit(nvs);
      }
      // Write a diagnostic boot banner so that every CSV log file carries
      // the firmware version, device ID, and the initial subsystem status
      // that would otherwise only appear on the serial console.
      char diag[128];
      logger.timestamp(millis());
      snprintf(diag, sizeof(diag), "BOOT FW=%s ID=%s", FIRMWARE_VERSION, devid);
      logger.logEvent(diag);
      // See PID_RESET_REASON comment above - answers "why did it reboot"
      // definitively, on SD (readable even with no network at all) and as
      // telemetry (see resetReasonPending, consumed in process()).
      resetReasonCode = (int32_t)esp_reset_reason();
      resetReasonPending = true;
      snprintf(diag, sizeof(diag), "RESET_REASON=%ld", (long)resetReasonCode);
      logger.logEvent(diag);
      snprintf(diag, sizeof(diag), "STATE OBD=%c GPS=%c MEMS=%c",
          state.check(STATE_OBD_READY)  ? '1' : '0',
          state.check(STATE_GPS_READY)  ? '1' : '0',
          state.check(STATE_MEMS_READY) ? '1' : '0');
      logger.logEvent(diag);
#if ENABLE_WIFI
      snprintf(diag, sizeof(diag), "WIFI SSID1=%s SSID2=%s",
          wifiSSID[0] ? wifiSSID : "-", wifiSSID2[0] ? wifiSSID2 : "-");
      logger.logEvent(diag);
#endif
      logger.flush();
    }
  }
#endif

#if STORAGE == STORAGE_SD
  // Startup check: detect a firmware staged by a previous session.
  // against the actual /ota_fw.bin file size.  A match means the download
  // completed successfully; set s_ota_pending so the flash happens at the
  // next standby transition.  Any mismatch means a partial download Ã¢â‚¬â€
  // clean up both files to avoid a corrupt flash attempt.
  if (state.check(STATE_STORAGE_READY)) {
    if (SD.exists(OTA_META_PATH)) {
      unsigned long expectedSize = 0;
      {
        File mf = SD.open(OTA_META_PATH, FILE_READ);
        if (mf) {
          char buf[16] = {0};
          mf.readBytesUntil('\n', buf, sizeof(buf) - 1);
          mf.close();
          expectedSize = strtoul(buf, nullptr, 10);
        }
      }
      bool stagingValid = false;
      if (expectedSize >= PULL_OTA_MIN_FW_SIZE && SD.exists(OTA_PENDING_PATH)) {
        File ff = SD.open(OTA_PENDING_PATH, FILE_READ);
        unsigned long actual = ff ? (unsigned long)ff.size() : 0UL;
        if (ff) ff.close();
        stagingValid = (actual == expectedSize);
      }
      if (stagingValid) {
        Serial.println("[OTA-PULL] Staged firmware found on SD Ã¢â‚¬â€ flashing at boot");
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL BOOT_FLASH");
        // Flash immediately at boot: the telemetry task has not started yet so
        // there is no need for s_ota_active synchronisation.  Flashing at boot
        // also ensures the update is applied even on devices that never reach
        // standby naturally (e.g. no OBD / no MEMS stationary timeout).
        if (performPullOtaFlash()) {
          // Flash succeeded; reboot timer is running Ã¢â‚¬â€ block here until it fires.
          while (true) delay(1000);
        }
        // Flash failed (corrupt image etc.): staging files already cleaned up
        // by performPullOtaFlash().  Continue normal boot.
      } else {
        SD.remove(OTA_PENDING_PATH);
        SD.remove(OTA_META_PATH);
        SD.remove(OTA_NVS_PATH);
        Serial.println("[OTA-PULL] Stale/incomplete SD staging files removed");
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL STALE_REMOVED");
      }
    } else if (SD.exists(OTA_PENDING_PATH)) {
      // Firmware file without companion meta Ã¢â‚¬â€ can't verify, remove it.
      SD.remove(OTA_PENDING_PATH);
      SD.remove(OTA_NVS_PATH);
    } else {
      // Normal case: no OTA firmware staged on SD.
      Serial.println("[OTA] SD:none");
    }
  }
#endif

  // re-try OBD if connection not established
#if ENABLE_OBD
  if (state.check(STATE_OBD_READY)) {
    char buf[128];
    if (obd.getVIN(buf, sizeof(buf))) {
      memcpy(vin, buf, sizeof(vin) - 1);
      Serial.print("VIN:");
      Serial.println(vin);
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) {
        char diag[32];
        snprintf(diag, sizeof(diag), "VIN=%s", vin);
        logger.logEvent(diag);
      }
#endif
    }
    int dtcCount = obd.readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      Serial.print("DTC:");
      Serial.println(dtcCount);
    }
  }
#endif

  // check system time
  printTime();

  lastMotionTime = millis();
  state.set(STATE_WORKING);

}

void showStats()
{
  uint32_t t = millis() - teleClient.startTime;
  char buf[32];
  sprintf(buf, "%02u:%02u.%c ", t / 60000, (t % 60000) / 1000, (t % 1000) / 100 + '0');
  Serial.print("[NET] ");
  Serial.print(buf);
  Serial.print("| Packet #");
  Serial.print(teleClient.txCount);
  Serial.print(" | Out: ");
  Serial.print(teleClient.txBytes >> 10);
  Serial.print(" KB | In: ");
  Serial.print(teleClient.rxBytes);
  Serial.print(" bytes | ");
  Serial.print((unsigned int)((uint64_t)(teleClient.txBytes + teleClient.rxBytes) * 3600 / (millis() - teleClient.startTime)));
  Serial.print(" KB/h");

  Serial.println();
}

bool waitMotion(long timeout)
{
#if ENABLE_MEMS
  unsigned long t = millis();
  if (state.check(STATE_MEMS_READY)) {
    do {
      // calculate relative movement
      float motion = 0;
      float acc[3];
      if (!mems->read(acc)) continue;
      if (accCount == 10) {
        accCount = 0;
        accSum[0] = 0;
        accSum[1] = 0;
        accSum[2] = 0;
      }
      accSum[0] += acc[0];
      accSum[1] += acc[1];
      accSum[2] += acc[2];
      accCount++;
      for (byte i = 0; i < 3; i++) {
        float m = (acc[i] - accBias[i]);
        motion += m * m;
      }
#if ENABLE_HTTPD
      serverProcess(100);
#endif
      processBLE(100);
      // check movement
      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
        //lastMotionTime = millis();
        Serial.println(motion);
        return true;
      }
    } while (state.check(STATE_STANDBY) && ((long)(millis() - t) < timeout || timeout == -1));
    return false;
  }
#endif
  serverProcess(timeout);
  return false;
}

/*******************************************************************************
  Collecting and processing data
*******************************************************************************/
void process()
{
  static uint32_t lastGPStick = 0;
  uint32_t startTime = millis();

  CBuffer* buffer = bufman.getFree();
  buffer->state = BUFFER_STATE_FILLING;

#if ENABLE_OBD
  // process OBD data if connected
  if (state.check(STATE_OBD_READY)) {
    processOBD(buffer);
    if (obd.errors >= MAX_OBD_ERRORS) {
      if (!obd.init()) {
        Serial.println("[OBD] ECU OFF");
#if STORAGE != STORAGE_NONE
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OBD ECU_OFF");
#endif
        state.clear(STATE_OBD_READY | STATE_WORKING);
        return;
      }
    }
  } else {
    // STATE_OBD_READY not set Ã¢â‚¬â€œ attempt reconnection.
    // Limit to one attempt every 30 s: obd.init() sends ATZ which hard-resets
    // the ELM327.  Calling it every process() cycle (every 1Ã¢â‚¬â€œ5 s) resets the
    // adapter before it can finish its auto-protocol detection, creating a
    // permanent reconnect loop where OBD-II never connects.
    // Use full init (quick=false): tries readPID twice instead of once, which
    // is more robust for ECUs that are slow to respond after engine start.
    // lastOBDReinit=0 at declaration means the first attempt is deferred until
    // millis() >= 30000 (~30 s after boot), giving initialize()'s own init
    // attempt a clear head-start and the ECU time to become ready.
    static uint32_t lastOBDReinit = 0;
    if (millis() - lastOBDReinit >= 30000) {
      lastOBDReinit = millis();
      if (obd.init(PROTO_AUTO, false)) {
        state.set(STATE_OBD_READY);
        Serial.println("[OBD] ECU ON");
      }
    }
  }
#endif

  if (rssi != rssiLast) {
    int val = (rssiLast = rssi);
    buffer->add(PID_CSQ, ELEMENT_INT32, &val, sizeof(val));
  }
  if (resetReasonPending) {
    resetReasonPending = false;
    buffer->add(PID_RESET_REASON, ELEMENT_INT32, &resetReasonCode, sizeof(resetReasonCode));
  }
#if ENABLE_WIFI
  if (wifiDisconnectPending) {
    wifiDisconnectPending = false;
    buffer->add(PID_WIFI_DISCONNECT_REASON, ELEMENT_INT32, &wifiDisconnectReason, sizeof(wifiDisconnectReason));
    buffer->add(PID_WIFI_DISCONNECT_COUNT, ELEMENT_INT32, &wifiDisconnectCount, sizeof(wifiDisconnectCount));
  }
  if (wifiScanPending) {
    wifiScanPending = false;
    buffer->add(PID_WIFI_SCAN_COUNT, ELEMENT_INT32, &wifiScanCount, sizeof(wifiScanCount));
    buffer->add(PID_WIFI_TARGET_RSSI, ELEMENT_INT32, &wifiScanTargetRssi, sizeof(wifiScanTargetRssi));
  }
#endif
#if ENABLE_OBD
  if (sys.devType > 12) {
    batteryVoltage = (float)(analogRead(A0) * 45) / 4095;
  } else {
    batteryVoltage = obd.getVoltage();
  }
  if (batteryVoltage) {
    uint16_t v = batteryVoltage * 100;
    buffer->add(PID_BATTERY_VOLTAGE, ELEMENT_UINT16, &v, sizeof(v));
  }
#endif

#if LOG_EXT_SENSORS
  processExtInputs(buffer);
#endif

#if ENABLE_MEMS
  processMEMS(buffer);
#endif

  bool success = processGPS(buffer);
#if GNSS_RESET_TIMEOUT
  if (success) {
    lastGPStick = millis();
    state.set(STATE_GPS_ONLINE);
  } else {
    if (millis() - lastGPStick > GNSS_RESET_TIMEOUT * 1000) {
      sys.gpsEnd();
      state.clear(STATE_GPS_ONLINE | STATE_GPS_READY);
      delay(20);
      if (initGPS()) state.set(STATE_GPS_READY);
      lastGPStick = millis();
    }
  }
#endif

  if (!state.check(STATE_MEMS_READY)) {
    deviceTemp = readChipTemperature();
  }
  buffer->add(PID_DEVICE_TEMP, ELEMENT_INT32, &deviceTemp, sizeof(deviceTemp));

  // Report white-LED and beep runtime state so HA can display live IST-Status.
  // Uses the file-scope sentinels s_lastLedWhite / s_lastBeep (both initialised
  // to -1 and reset to -1 on every telemetry reconnect via initialize() and on
  // every new WiFi/cellular connection) so the current state is always re-sent
  // after a connection drop, preventing a permanent "Unbekannt" IST-Status in
  // Home Assistant when the very first transmission attempt fails and the buffer
  // is subsequently purged.
  {
    uint8_t lwv = enableLedWhite ? 1 : 0;
    uint8_t bv  = enableBeep     ? 1 : 0;
    if ((int8_t)lwv != s_lastLedWhite) {
      s_lastLedWhite = (int8_t)lwv;
      buffer->add(PID_LED_WHITE_STATE, ELEMENT_UINT8, &lwv, sizeof(lwv));
    }
    if ((int8_t)bv != s_lastBeep) {
      s_lastBeep = (int8_t)bv;
      buffer->add(PID_BEEP_STATE, ELEMENT_UINT8, &bv, sizeof(bv));
    }
  }

  // Report active transport type (WiFi vs Cellular) via PID_CONN_TYPE (0x88).
  // Only added when a network connection is active (STATE_NET_READY) so the
  // value is always meaningful Ã¢â‚¬â€ the device is either on WiFi or cellular,
  // never in AP-only mode when this PID reaches HA.  The sentinel is reset on
  // every new connection, so the first packet of each session always includes
  // this PID, enabling HA to correctly update "WiFi letzte Verbindung" /
  // "LTE letzte Verbindung" timestamps for both transports.
  if (state.check(STATE_NET_READY)) {
    // 1 = WiFi (STATE_WIFI_CONNECTED), 2 = Cellular (SIM7600 / LTE).
    uint8_t ctv = state.check(STATE_WIFI_CONNECTED) ? 1 : 2;
    if ((int8_t)ctv != s_lastConnType) {
      s_lastConnType = (int8_t)ctv;
      buffer->add(PID_CONN_TYPE, ELEMENT_UINT8, &ctv, sizeof(ctv));
    }
  }

  // Report OBD runtime state (PID 0x89) so HA can display the live
  // IST-Status alongside the configured value from the options flow.
  {
    uint8_t ov = enableObd ? 1 : 0;
    if ((int8_t)ov != s_lastObd) {
      s_lastObd = (int8_t)ov;
      buffer->add(PID_OBD_STATE, ELEMENT_UINT8, &ov, sizeof(ov));
    }
  }

  // Report standby-time override (PID 0x8b).  0 = firmware compile-time default.
  {
    int16_t sv = (int16_t)nvsStandbyTimeS;
    if (sv != s_lastStandbyTime) {
      s_lastStandbyTime = sv;
      buffer->add(PID_STANDBY_TIME, ELEMENT_UINT16, &nvsStandbyTimeS, sizeof(nvsStandbyTimeS));
    }
  }

  // Report deep-standby mode (PID 0x8c): 1 = deep sleep on standby, 0 = normal.
  {
    uint8_t dv = enableDeepStandby ? 1 : 0;
    if ((int8_t)dv != s_lastDeepStandby) {
      s_lastDeepStandby = (int8_t)dv;
      buffer->add(PID_DEEP_STANDBY, ELEMENT_UINT8, &dv, sizeof(dv));
    }
  }

#if STORAGE == STORAGE_SD
  // Report SD card total/free space once per minute so HA can display SD
  // presence and usage without a direct HTTP connection to the device.
  // PID_SD_TOTAL_MB = 0 signals "no card / not ready" to HA.
  // Also purge oldest log files when SD is >= 80% full (auto-cleanup).
  {
    static uint32_t lastSdReportMs = 0;
    uint32_t nowMs = millis();
    if (lastSdReportMs == 0 || nowMs - lastSdReportMs >= 60000UL) {
      lastSdReportMs = nowMs;
      uint32_t sdTotalMb = 0;
      uint32_t sdFreeMb  = 0;
      if (state.check(STATE_STORAGE_READY)) {
        // Purge oldest 20% of log files when SD >= 80% full.
        if (logger.purgeOldFiles()) {
          logger.logEvent("SD:PURGE");
        }
        uint64_t tot = SD.totalBytes();
        uint64_t used = SD.usedBytes();
        sdTotalMb = (uint32_t)(tot >> 20);
        sdFreeMb  = (uint32_t)((tot > used ? tot - used : 0) >> 20);
      }
      // Update module-level cache so the telemetry inject block can include
      // the SD values in the guaranteed first-packet injection without
      // touching the SD SPI bus from the wrong task.
      s_cachedSdTotalMb = sdTotalMb;
      s_cachedSdFreeMb  = sdFreeMb;
      buffer->add(PID_SD_TOTAL_MB, ELEMENT_UINT32, &sdTotalMb, sizeof(sdTotalMb));
      buffer->add(PID_SD_FREE_MB,  ELEMENT_UINT32, &sdFreeMb,  sizeof(sdFreeMb));
    }
  }
#endif

  buffer->timestamp = millis();
  buffer->state = BUFFER_STATE_FILLED;

  // display file buffer stats
  if (startTime - lastStatsTime >= 3000) {
    bufman.printStats();
    lastStatsTime = startTime;
  }

#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    // Prepend a timestamp record (PID 0) before the data payload so that the
    // /api/data endpoint (handlerLogData in dataserver.cpp) can correlate each
    // CSV record with a time offset.  The network serialisation path already
    // does this via store.timestamp(buffer->timestamp); the file path was
    // missing it, causing ts to remain 0 throughout CSV parsing and breaking
    // all time-filtered data queries.
    logger.timestamp(buffer->timestamp);
    buffer->serialize(logger);
    uint16_t sizeKB = (uint16_t)(logger.size() >> 10);
    if (sizeKB != lastSizeKB) {
      logger.flush();
      lastSizeKB = sizeKB;
      Serial.print("[FILE] ");
      Serial.print(sizeKB);
      Serial.println("KB");
    }
    // Detect and log subsystem state transitions (OBD/GPS/WiFi/Cell).
    // prevDiagState is initialised to 0xFFFF so the very first call
    // always writes a STATUS line, establishing the initial state in the CSV.
    {
      static uint16_t prevDiagState = 0xFFFF; /* impossible mask value forces initial STATUS entry */
      const uint16_t DIAG_MASK = STATE_OBD_READY | STATE_GPS_READY | STATE_GPS_ONLINE
                                | STATE_WIFI_CONNECTED | STATE_CELL_CONNECTED;
      uint16_t cur = state.m_state & DIAG_MASK;
      if (cur != prevDiagState) {
        char diag[128];
        snprintf(diag, sizeof(diag),
            "STATUS OBD=%c GPS=%c FIX=%c WIFI=%c CELL=%c t=%lu",
            state.check(STATE_OBD_READY)      ? '1' : '0',
            state.check(STATE_GPS_READY)      ? '1' : '0',
            state.check(STATE_GPS_ONLINE)     ? '1' : '0',
            state.check(STATE_WIFI_CONNECTED) ? '1' : '0',
            state.check(STATE_CELL_CONNECTED) ? '1' : '0',
            millis() / 1000);
        logger.logEvent(diag);
        prevDiagState = cur;
      }
    }
  }
#endif

  const int dataIntervals[] = DATA_INTERVAL_TABLE;
#if ENABLE_OBD || ENABLE_MEMS
  // motion adaptive data interval control
  // Build effective stationary-time table, applying STANDBY_TIME NVS override
  // (nvsStandbyTimeS, 5-900 s) to the last (maximum-standby) entry when set.
  const uint16_t stationaryTimeDefaults[] = STATIONARY_TIME_TABLE;
  const byte stationaryCount = sizeof(stationaryTimeDefaults) / sizeof(stationaryTimeDefaults[0]);
  uint16_t stationaryTime[stationaryCount];
  for (byte i = 0; i < stationaryCount; i++) {
    stationaryTime[i] = stationaryTimeDefaults[i];
  }
  if (nvsStandbyTimeS == 0xFFFF) {
    // Sentinel: disable standby entirely (e.g. for extended bench/parked
    // testing where the vehicle deliberately never moves - normal standby
    // would otherwise suspend WiFi/OBD polling and cut off exactly the
    // telemetry being watched). 0 in the stationaryTime table means "never
    // reaches this tier" per the loop below (motionless < 0 is never true
    // via the || stationaryTime[i] == 0 check).
    stationaryTime[stationaryCount - 1] = 0;
  } else if (nvsStandbyTimeS >= 5) {
    stationaryTime[stationaryCount - 1] = nvsStandbyTimeS;
  }
  unsigned int motionless = (millis() - lastMotionTime) / 1000;
  bool stationary = true;
  for (byte i = 0; i < stationaryCount; i++) {
    dataInterval = dataIntervals[i];
    if (motionless < stationaryTime[i] || stationaryTime[i] == 0) {
      stationary = false;
      break;
    }
  }
  // OBD alive check: fires once per stationary period, as soon as the device
  // first enters the standby countdown (motionless >= stationaryTime[0], i.e.
  // 10 s by default).  Goal: detect "ignition off" early so the Freematics
  // stops polling the OBD2 port immediately.  Some vehicles with factory alarms
  // monitor the OBD2 bus; continued requests after the ignition is cut can
  // trigger a false alarm.
  // The check is skipped when OBD is disabled or the ECU was never connected
  // (STATE_OBD_READY not set), in which case the normal countdown applies.
#if ENABLE_OBD
  {
    static bool s_obdAliveChecked = false; // reset each time motion resumes
    const bool inStationaryPhase = (motionless >= stationaryTime[0]);
    if (!inStationaryPhase) {
      // Vehicle is moving Ã¢â‚¬â€œ clear the flag so we probe again next time it stops.
      s_obdAliveChecked = false;
    } else if (enableObd && state.check(STATE_OBD_READY) && !s_obdAliveChecked) {
      s_obdAliveChecked = true;
      // Probe with "01 00" (Mode 1 PID 0 Ã¢â‚¬â€œ supported PIDs [01-20]).  This is a
      // read-only request with no side effects; 1 s timeout keeps the check
      // non-blocking.  The ELM327 link is accessed directly to control the
      // timeout, bypassing obd.readPID() which uses OBD_TIMEOUT_LONG (10 s).
      char abuf[32] = {};
      const int ret = obd.link ? obd.link->sendCommand("0100\r", abuf, sizeof(abuf), 1000) : 0;
      if (ret <= 0) {
        // No response: ECU is offline (ignition cut).  Enter standby now
        // without waiting for the full countdown to expire.
        Serial.println("OBD:ECU offline at standby-timer start - entering standby immediately");
        state.clear(STATE_WORKING);
        return;
      }
      Serial.println("OBD:ECU alive - standby countdown running");
    }
  }
#endif

  // 2026-09-21: with ENABLE_MEMS=0 (see config.h/platformio.ini history for
  // why), lastMotionTime never updates from real motion at all, so
  // `motionless` grows unboundedly from boot regardless of context - the
  // device would standby on a schedule even while sitting on a USB-powered
  // bench being actively worked on. USB power reads far below real vehicle
  // system voltage (12V nominal, ~9V+ even on a weak/discharged battery)
  // via the same batteryVoltage reading used for JUMPSTART_VOLTAGE below -
  // treat anything under this as "not actually in a vehicle right now" and
  // never enter standby, regardless of the stationary timer.
  if (batteryVoltage > 0 && batteryVoltage < 7.0f) {
    stationary = false;
  }

  if (stationary) {
    // stationery timeout
    Serial.print("Stationary for ");
    Serial.print(motionless);
    Serial.println(" secs");
    // trip ended, go into standby
    state.clear(STATE_WORKING);
    return;
  }
#else
  dataInterval = dataIntervals[0];
#endif
  // Idle loop: wait out the rest of the data interval while servicing
  // low-priority tasks.  Each iteration sleeps at most 100 ms so that the
  // built-in HTTP server is polled regularly (every ~100 ms) even when BLE
  // is disabled.  This ensures that /api/control?cmd=OFF requests (sent by
  // the HA OTA flash manager to pause telemetry before uploading firmware)
  // are received and acted upon during normal telemetry operation.
  do {
    long t = dataInterval - (millis() - startTime);
    long slice = (t > 100) ? 100 : (t > 0 ? t : 0);
    processBLE(slice);
#if ENABLE_HTTPD
    if (enableHttpd) serverProcess(0);
#endif
  } while (millis() - startTime < dataInterval);
}

bool initCell(bool quick = false)
{
  Serial.println("[CELL] Activating...");
  // power on network module
  if (!teleClient.cell.begin(&sys)) {
    Serial.println("[CELL] No supported module");
    return false;
  }
  if (quick) return true;
  Serial.print("CELL:");
  Serial.println(teleClient.cell.deviceName());
  // Retry checkSIM up to 3 times; the SIM may not be ready immediately after power-on
  {
    bool simReady = false;
    for (byte simRetry = 0; simRetry < 3 && !simReady; simRetry++) {
      if (teleClient.cell.checkSIM(simPin)) {
        simReady = true;
      } else if (simRetry < 2) {
        delay(2000);
      }
    }
    if (!simReady) {
      Serial.println("NO SIM CARD");
    }
  }
  Serial.print("IMEI:");
  Serial.println(teleClient.cell.IMEI);
  Serial.println("[CELL] Searching...");
  if (*apn) {
    Serial.print("APN:");
    Serial.println(apn);
  }
  if (teleClient.cell.setup(apn, APN_USERNAME, APN_PASSWORD)) {
    netop = teleClient.cell.getOperatorName();
    if (netop.length()) {
      Serial.print("Operator:");
      Serial.println(netop);
    }

#if GNSS == GNSS_CELLULAR
    if (teleClient.cell.setGPS(true)) {
      Serial.println("CELL GNSS:OK");
    }
#endif

    ip = teleClient.cell.getIP();
    if (ip.length()) {
      Serial.print("[CELL] IP:");
      Serial.println(ip);
    }
    state.set(STATE_CELL_CONNECTED);
  } else {
    char *p = strstr(teleClient.cell.getBuffer(), "+CPSI:");
    if (p) {
      char *q = strchr(p, '\r');
      if (q) *q = 0;
      Serial.print("[CELL] ");
      Serial.println(p + 7);
    } else {
      Serial.print(teleClient.cell.getBuffer());
    }
  }
  timeoutsNet = 0;
  return state.check(STATE_CELL_CONNECTED);
}

/*******************************************************************************
  Missed-data catch-up: replay one /DATA/<fileId>.CSV file's contents as
  live-format packets over the current connection.
  A CSV line ("<HEXPID>,<valuetext>") is byte-identical to the value text a
  live wire packet would carry for the same PID - both come from the same
  CStorage::log() formatting, just with a different delimiter/separator
  (file: ',' and '\n'; wire: ':' and ','). So no typed re-decoding is needed:
  each line is reformatted from "PID,value" to "PID:value" and fed straight
  into replayStore's own dispatch(), bookended by header()/tailer() per
  record (a record = the lines between one PID-0/timestamp line and the
  next). PID 0xFE ("FE,<text>") is FileLogger::logEvent()'s diagnostic-text
  marker, not a sensor reading - skipped during replay.
  Returns true only if every record in the file was transmitted successfully
  (false stops at the first failure, so the caller does not advance the
  watermark past a file that was only partially sent - it will be retried in
  full, not resumed mid-file, next time).
*******************************************************************************/
bool sendCsvFile(CStorageRAM& replayStore, uint32_t fileId)
{
  char path[24];
  sprintf(path, "/DATA/%u.CSV", (unsigned int)fileId);
  File f = SD.open(path, FILE_READ);
  if (!f) {
    // Already purged by SDLogger::purgeOldFiles() under SD space pressure -
    // nothing left to replay; don't let a deleted file block later ones.
    return true;
  }

  char line[64];
  int lineLen = 0;
  bool recordOpen = false;
  bool ok = true;
  int yieldCounter = 0;

  while (ok && f.available()) {
    char buf[256];
    int n = f.readBytes(buf, sizeof(buf));
    for (int i = 0; i < n && ok; i++) {
      char c = buf[i];
      if (c == '\n') {
        line[lineLen] = 0;
        char* comma = strchr(line, ',');
        if (comma) {
          *comma = 0;
          const char* valueText = comma + 1;
          uint16_t pid = (uint16_t)strtoul(line, 0, 16);
          if (pid == 0) {
            // New record boundary (PID 0 = timestamp, see CStorage::timestamp()).
            // Close and send the previous record first, if any.
            if (recordOpen) {
              replayStore.tailer();
              ok = teleClient.transmit(replayStore.buffer(), replayStore.length());
            }
            if (ok) {
              replayStore.header(devid);
              recordOpen = true;
            }
          }
          if (ok && recordOpen && pid != 0xFE) {
            char entry[80];
            int elen = snprintf(entry, sizeof(entry), "%X:%s", pid, valueText);
            replayStore.dispatch(entry, elen);
          }
        }
        lineLen = 0;
      } else if (lineLen < (int)sizeof(line) - 1) {
        line[lineLen++] = c;
      }
    }
    if (++yieldCounter >= 4) { yield(); yieldCounter = 0; }
  }
  if (ok && recordOpen) {
    replayStore.tailer();
    ok = teleClient.transmit(replayStore.buffer(), replayStore.length());
  }
  f.close();
  return ok;
}

// Driver: replays every /DATA file between the watermark and the current
// session's own file (exclusive - the active file is still being live-
// written and is handled by the normal send loop, not this one). Advances
// and persists wmDoneFileId to NVS after each fully-successful file so a
// later interruption resumes at the next unfinished file, not from scratch.
// Called once per boot (see s_catchupPending) right before the send loop
// starts sending live packets, so any gap left by an extended outage
// (Switzerland/ferry-style, no WiFi or cellular for hours+) gets replayed
// in strict file order before today's live data resumes - this ordering
// matters because Traccar's DistanceHandler computes totalDistance from
// consecutive positions in arrival order, so sending newer data before an
// older backlog would corrupt the odometer calibration chain.
// Returns true only once every missed file has been fully sent (or there
// was nothing missed to begin with) - false means the caller must NOT fall
// through to sending live data this iteration (that would let newer data
// overtake a still-incomplete backlog), and must retry catch-up again
// instead, e.g. on the next reconnect.
bool catchUpMissedFiles(CStorageRAM& replayStore)
{
  if (fileid <= 0) return true;
  uint32_t upTo = (uint32_t)fileid - 1;
  if (wmDoneFileId >= upTo) return true;  // nothing missed

  Serial.print("[CATCHUP] replaying files ");
  Serial.print(wmDoneFileId + 1);
  Serial.print("..");
  Serial.println(upTo);

  for (uint32_t id = wmDoneFileId + 1; id <= upTo; id++) {
    if (s_ota_active) return false;  // yield to OTA exactly like the live send loop does
    if (!sendCsvFile(replayStore, id)) {
      Serial.print("[CATCHUP] file ");
      Serial.print(id);
      Serial.println(" failed, will retry later");
      return false;
    }
    wmDoneFileId = id;
    nvs_set_u32(nvs, "WM_FILE", wmDoneFileId);
    nvs_commit(nvs);
    Serial.print("[CATCHUP] file ");
    Serial.print(id);
    Serial.println(" done");
  }
  return true;
}

/*******************************************************************************
  Initializing network, maintaining connection and doing transmissions
*******************************************************************************/
void telemetry(void* inst)
{
  uint32_t lastRssiTime = 0;
  uint8_t connErrors = 0;
  CStorageRAM store;
  store.init(
#if BOARD_HAS_PSRAM
    (char*)heap_caps_malloc(SERIALIZE_BUFFER_SIZE, MALLOC_CAP_SPIRAM),
#else
    (char*)malloc(SERIALIZE_BUFFER_SIZE),
#endif
    SERIALIZE_BUFFER_SIZE
  );
  teleClient.reset();

  for (;;) {
    // Yield the WiFi radio to the OTA flash handler while it is active.
    // Without this, the telemetry task competes for bandwidth and can cause
    // WiFi reconnects that disrupt the HTTP connection used for the upload.
    if (s_ota_active) {
      delay(500);
      continue;
    }

    // Apply standby / resume requests originating from the httpd task.
    // The actual m_state writes happen here (net task only) so that
    // state.m_state is always modified from a single task context.
    if (s_http_standby_enter) {
      s_http_standby_enter = false;
      state.set(STATE_STANDBY);
      state.clear(STATE_WORKING);
      Serial.println("[HTTP] Telemetry paused via /api/control?cmd=OFF");
    }
    if (s_http_standby_exit) {
      s_http_standby_exit = false;
      state.clear(STATE_STANDBY);
      Serial.println("[HTTP] Telemetry resumed via /api/control?cmd=ON");
    }

    if (state.check(STATE_STANDBY)) {
      if (state.check(STATE_CELL_CONNECTED) || state.check(STATE_WIFI_CONNECTED)) {
        teleClient.shutdown();
        netop = "";
        ip = "";
        rssi = 0;
      }
      state.clear(STATE_NET_READY | STATE_CELL_CONNECTED | STATE_WIFI_CONNECTED);
      teleClient.reset();
      bufman.purge();
      // Reset LED/beep/conn_type sentinels so the current state is re-sent in
      // the first buffer after the connection is re-established.  Without this
      // reset the state-change detection would suppress the PIDs (value
      // unchanged) and Home Assistant would keep showing "Unbekannt" for the
      // IST-Status and the connection-type timestamps.
      s_lastLedWhite    = -1;
      s_lastBeep        = -1;
      s_lastConnType    = -1;
      s_lastObd         = -1;
      s_lastStandbyTime = -1;
      s_send_state_pids = true;

      uint32_t t = millis();
      do {
        delay(1000);
      } while (state.check(STATE_STANDBY) && millis() - t < 1000L * PING_BACK_INTERVAL);
      if (state.check(STATE_STANDBY)) {
        // start ping
#if ENABLE_WIFI
        if (wifiSSID[0] || wifiSSID2[0]) {
          wifiConnect();
        }
        if (teleClient.wifi.setup()) {
          Serial.println("[WIFI] Ping...");
          teleClient.ping();
        }
        else
#endif
        {
          if (initCell()) {
            Serial.println("[CELL] Ping...");
            teleClient.ping();
          }
        }
        teleClient.shutdown();
        state.clear(STATE_CELL_CONNECTED | STATE_WIFI_CONNECTED);
      }
      continue;
    }

#if ENABLE_WIFI
    if ((wifiSSID[0] || wifiSSID2[0]) && !state.check(STATE_WIFI_CONNECTED)) {
      if (!teleClient.wifi.connected()) {
        wifiConnect();
      }
      teleClient.wifi.setup(WIFI_JOIN_TIMEOUT);
    }
#endif

    while (state.check(STATE_WORKING)) {
      // Break out immediately when OTA or a standby request is pending so
      // the outer loop can process the flag and shut down SSL connections
      // before Update.begin() is called.  Without this check the inner loop
      // keeps transmitting packets (and holding SSL heap) even after
      // cmd=OFF / s_ota_active is set, which exhausts available heap and
      // triggers abort() during Update.begin().
      if (s_ota_active || s_http_standby_enter) break;

#if ENABLE_WIFI
      if (wifiSSID[0]) {
        if (!state.check(STATE_WIFI_CONNECTED) && teleClient.wifi.connected()) {
          ip = teleClient.wifi.getIP();
          if (ip.length()) {
            Serial.print("[WIFI] IP:");
            Serial.println(ip);
            logNetEvent("NET WIFI_GOT_IP");
          }
          connErrors = 0;
          if (teleClient.connect()) {
            state.set(STATE_WIFI_CONNECTED | STATE_NET_READY);
            if (enableBeep) beep(50);
            // Geofence-WiFi known-locations sync (2026-09-22): "on every
            // successful connection" per the design, rate-limited to
            // LOC_SYNC_MIN_INTERVAL so a flapping WiFi connection (in/out of
            // range while driving) can't open a fresh TLS session to
            // ota_server.py on every single reconnect - see
            // syncKnownLocations()'s own comment and LOC_SYNC_MIN_INTERVAL's
            // definition in config.h for the heap-fragmentation reasoning.
            // Deliberately a separate, independent call from the pull-OTA
            // check further down - not nested inside it, not sharing any of
            // its heap-sensitive sequencing (meta.json -> firmware.bin ->
            // ota_confirm).
            //
            // This is only the OPTIMISTIC immediate attempt - confirmed live
            // 2026-09-22 that it reliably fails ("Cannot connect") this
            // early, most likely the IP stack/DNS resolver not being fully
            // settled the instant WiFi reports connected (performPullOtaCheck()
            // uses the same otaWifiClient/otaHost successfully, but only ever
            // runs well after this point, not at the instant of connection).
            // s_locSynced only becomes true on genuine success, so a real
            // retry happens every SIGNAL_CHECK_INTERVAL via the periodic
            // block further down (search s_locSynced) until one actually
            // succeeds - this call is just a cheap "maybe it's already fine"
            // try, not the only chance.
            if (!s_locSynced || millis() - s_lastLocSyncTime > (uint32_t)LOC_SYNC_MIN_INTERVAL * 1000UL) {
              if (syncKnownLocations()) {
                s_lastLocSyncTime = millis();
                s_locSynced = true;
              }
            }
            // Reset sentinels so the first WiFi packet always re-transmits the
            // LED/beep state and connection type to HA.  Without this, the
            // sentinels retain their values from the previous cellular session
            // and HA would keep showing stale IST-Status values.
            s_lastLedWhite    = -1;
            s_lastBeep        = -1;
            s_lastConnType    = -1;
            s_lastObd         = -1;
            s_lastStandbyTime = -1;
            s_send_state_pids = true;
            // switch off cellular module when wifi connected
            if (state.check(STATE_CELL_CONNECTED)) {
              teleClient.cell.end();
              state.clear(STATE_CELL_CONNECTED);
              Serial.println("[CELL] Deactivated");
            }
          }
        } else if (state.check(STATE_WIFI_CONNECTED) && !teleClient.wifi.connected()) {
          Serial.println("[WIFI] Disconnected");
          state.clear(STATE_WIFI_CONNECTED);
        }
      }
#endif
      if (!state.check(STATE_WIFI_CONNECTED) && !state.check(STATE_CELL_CONNECTED)) {
        connErrors = 0;
        if (!initCell() || !teleClient.connect()) {
          teleClient.cell.end();
          state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
          Serial.println("[CELL] Deactivated");
#if ENABLE_WIFI
          if (wifiSSID[0] || wifiSSID2[0]) {
            // Try WiFi immediately before the cellular backoff delay
            if (!teleClient.wifi.connected()) {
              wifiConnect();
            }
            if (teleClient.wifi.setup(WIFI_JOIN_TIMEOUT)) {
              break;  // WiFi connected; re-enter outer loop to complete setup
            }
          }
#endif
          // avoid turning on/off cellular module too frequently to avoid operator banning
          delay(60000 * 3);
          break;
        }
        Serial.println("[CELL] In service");
        state.set(STATE_NET_READY);
        if (enableBeep) beep(50);
        // TEMPORARY bench test (2026-09-22, saved plan point 1) - see
        // testCellularOtaMeta()'s own comment for the full design/risk
        // story. Runs once per boot, right after cellular first comes up.
        {
          static bool tested = false;
          if (!tested) {
            tested = true;
            testCellularOtaMeta();
          }
        }
        // Reset sentinels so the first cellular packet always re-transmits the
        // LED/beep state and connection type to HA.  Without this, the sentinels
        // retain their values from the previous WiFi session and HA would keep
        // showing stale IST-Status values and incorrect connection timestamps.
        s_lastLedWhite    = -1;
        s_lastBeep        = -1;
        s_lastConnType    = -1;
        s_lastObd         = -1;
        s_lastStandbyTime = -1;
        s_send_state_pids = true;
      }

      if (millis() - lastRssiTime > SIGNAL_CHECK_INTERVAL * 1000) {
#if ENABLE_WIFI
        // Geofence-WiFi sync retry (2026-09-22): the optimistic immediate
        // attempt at the moment WiFi connects (search s_locSynced above)
        // reliably fails that early on this hardware - retry here every
        // SIGNAL_CHECK_INTERVAL (10s) as long as it still hasn't succeeded
        // this session, instead of only on the next connection-state
        // transition (which may never come again if WiFi just stays up).
        // Once s_locSynced is true, this is a no-op until the connection
        // actually drops and reconnects (state.set(STATE_WIFI_CONNECTED...)
        // above does NOT reset s_locSynced - a real address/SSID edit is
        // still only picked up every LOC_SYNC_MIN_INTERVAL, by design).
        if (state.check(STATE_WIFI_CONNECTED) &&
            (!s_locSynced || millis() - s_lastLocSyncTime > (uint32_t)LOC_SYNC_MIN_INTERVAL * 1000UL)) {
          if (syncKnownLocations()) {
            s_lastLocSyncTime = millis();
            s_locSynced = true;
          }
        }
        if (state.check(STATE_WIFI_CONNECTED))
        {
          rssi = teleClient.wifi.RSSI();
        }
        else
#endif
        {
          rssi = teleClient.cell.RSSI();
        }
        if (rssi) {
          Serial.print("RSSI:");
          Serial.print(rssi);
          Serial.println("dBm");
        }
        lastRssiTime = millis();

#if ENABLE_WIFI
        // Problem 2 fix (2026-09-21): retry WiFi periodically even while
        // already successfully connected via cellular, so the device can
        // hand back over to WiFi once back in range, without waiting for a
        // full outage/reboot. This call still goes through wifiConnect() ->
        // teleClient.wifi.begin() -> ClientWIFI::begin(), the same
        // ble_pause()-wrapped path as every other WiFi (re)connect attempt -
        // no parallel/bypassing path. Deliberately gated at
        // WIFI_CELLULAR_RECHECK_INTERVAL (minutes), NOT this block's own
        // SIGNAL_CHECK_INTERVAL (10s) cadence: each attempt now pauses/
        // resumes the BT controller and runs a WiFi join handshake, both of
        // which cost real battery on this car-12V-powered device - retrying
        // every 10s would defeat the whole point of the BT-pause work being
        // "occasional", not continuous. Only "no candidate network configured
        // or connected" needed at all - the actual network scanning/join
        // attempt itself only ever tries wifiSSID/wifiSSID2 (wifiConnect()
        // never scans for open/unknown networks).
        static uint32_t lastWifiCellRecheckTime = 0;
        bool wifiRetryDue = true;
        if (state.check(STATE_CELL_CONNECTED)) {
          wifiRetryDue = millis() - lastWifiCellRecheckTime > (uint32_t)WIFI_CELLULAR_RECHECK_INTERVAL * 1000UL;
        }
        // (else: not on cellular either - e.g. transiently between
        // connections - keep the fast retry cadence so we don't sit idle.)
        if ((wifiSSID[0] || wifiSSID2[0]) && !state.check(STATE_WIFI_CONNECTED) && !teleClient.wifi.connected() && wifiRetryDue) {
          if (state.check(STATE_CELL_CONNECTED)) lastWifiCellRecheckTime = millis();
          wifiConnect();
        }
#endif
      }

      // Periodic pull-OTA check: runs only when WiFi is connected and
      // OTA_TOKEN + OTA_INTERVAL are provisioned.  OTA is WiFi-only; the
      // SIM7600E-H cellular modem cannot reliably connect to the OTA endpoint
      // (TLS error 15 against *.ui.nabu.casa / Cloudflare).  The check is
      // placed here (before the empty-buffer continue) so it fires even when
      // OBD2/GPS are inactive and no telemetry data is being collected.
      // Rate-limited by otaCheckIntervalS; 0 means disabled.
      //
      // For STORAGE_SD: performPullOtaCheck() downloads the binary to SD and
      // returns false (no reboot yet).  The flash happens in standby().
      // For other storage: returns true when direct flash has started (reboot
      // imminent) so the caller blocks here waiting for the reboot timer.
      // 2026-09-21: periodic timer-based polling removed per explicit
      // decision - push (ota_push_watcher comparing the device's reported
      // versionFw against registry.json, then Command.TYPE_CUSTOM
      // OTA_READY) is now the ONLY trigger. otaCheckIntervalS > 0 is kept
      // purely as the existing OTA-provisioned/enabled gate (0 = OTA
      // disabled entirely), no longer as a timer period.
      if (otaToken[0] && otaCheckIntervalS > 0 &&
          state.check(STATE_WIFI_CONNECTED)) {
        bool checkNow = s_ota_check_now;
        if (checkNow) {
          s_ota_check_now = false;
          Serial.println("[OTA-PULL] Checking for firmware update (triggered)...");
          // Do NOT close the telemetry TLS session here.  In virtually all
          // deployments (Nabu Casa / hooks.nabu.casa) the OTA host and the
          // telemetry webhook host are the same *.ui.nabu.casa or
          // hooks.nabu.casa domain.  Calling wifi.close() before the OTA
          // check would tear down the active TLS session and force a new
          // TLS handshake, creating an allocÃ¢â€ â€™freeÃ¢â€ â€™alloc cycle that fragments
          // the mbedTLS heap over time (each cycle leaves behind tiny holes
          // that reduce the maximum contiguous block).  Over ~30 telemetry
          // packets the max block shrinks from ~40 KB to ~20 KB Ã¢â‚¬â€ well below
          // the 38 KB TLS_MIN_FREE_HEAP threshold Ã¢â‚¬â€ causing Guard 2 in
          // WifiHTTP::open() to fire ("Low heap Ã¢â‚¬Â¦ after cleanup, skipping TLS
          // connect") on every subsequent OTA check, rendering OTA unusable.
          //
          // WifiHTTP::open() already handles both cases correctly:
          //   Ã¢â‚¬Â¢ Same host: reuse the existing TLS session (zero TLS cycles).
          //   Ã¢â‚¬Â¢ Different host, heap OK: stop() + connect() atomically.
          //   Ã¢â‚¬Â¢ Different host, heap low: Guard 1 returns false; the post-OTA
          //     check below restarts WiFi to coalesce the heap.
          if (performPullOtaCheck()) {
            // Direct-flash path: firmware flash started; device will reboot
            // shortly.  Block here so the loop doesn't continue transmitting.
            while (true) delay(1000);
          }
#if STORAGE == STORAGE_SD
          // SD-staged path: firmware was written to /ota_fw.bin; standby()
          // will call performPullOtaFlash() when the car turns off.
          // Break out of the inner while immediately so no further transmit
          // attempts are made with the now-fragmented TLS heap.  The outer
          // for(;;) loop sees s_ota_pending=true on the next iteration and
          // enters its delay(1000)/continue idle path.
          // performPullOtaCheck() called wifi.close() internally which reset
          // m_state to HTTP_DISCONNECTED, so the HTTP_ERROR-based guard below
          // would never fire after a successful SD download Ã¢â‚¬â€ break explicitly.
          if (s_ota_pending) {
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            state.clear(STATE_NET_READY | STATE_WIFI_CONNECTED);
            break;
          }
#endif
          // If OTA failed due to genuine heap fragmentation (e.g. a TLS
          // teardown inside performPullOtaCheck() leaked mbedTLS state), the
          // heap may still be too low to re-establish telemetry.  Detect this
          // and restart WiFi to coalesce the heap.
#if ENABLE_WIFI
          if (state.check(STATE_WIFI_CONNECTED) &&
              ESP.getMaxAllocHeap() < TLS_MIN_FREE_HEAP) {
            Serial.printf("[WIFI] Low heap (%u bytes max block) after OTA TLS fail, restarting WiFi\n",
                          (unsigned)ESP.getMaxAllocHeap());
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            state.clear(STATE_NET_READY | STATE_WIFI_CONNECTED);
            break;
          }
#endif
        }
      }

      // One-shot per boot, right before the first live packet: replay any
      // /DATA files left over from an outage that spanned a reboot (or
      // simply never got sent) before resuming normal live transmission.
      // See catchUpMissedFiles()'s own comment for why this must run here,
      // strictly before getNewest() below, not interleaved with it. If it
      // returns false (interrupted, e.g. lost connection mid-replay),
      // s_catchupPending stays true so the NEXT iteration retries catch-up
      // again instead of falling through to live data below - newer data
      // must never be sent while an older backlog is still incomplete.
      if (s_catchupPending) {
        if (catchUpMissedFiles(store)) {
          s_catchupPending = false;
        } else {
          delay(1000);
          continue;
        }
      }

      // get data from buffer
      CBuffer* buffer = bufman.getNewest();
      if (!buffer) {
        delay(50);
        continue;
      }
#if SERVER_PROTOCOL == PROTOCOL_UDP
      store.header(devid);
#endif
      store.timestamp(buffer->timestamp);
      buffer->serialize(store);
      bufman.free(buffer);
      // Inject IST-Status PIDs (LED/beep/conn-type/SD) directly into this
      // packet whenever a new connection has just been established.
      //
      // Without this injection, there is a race between process() and the
      // telemetry loop that reliably loses these PIDs on cellular connections:
      //   1. Sentinel reset Ã¢â€ â€™ process() adds PIDs to Buffer A, updates sentinel.
      //   2. OTA meta-check over cellular takes several seconds while process()
      //      fills Buffers B, C, D Ã¢â‚¬Â¦ (sentinel already matches, no PIDs).
      //   3. getNewest() returns Buffer D (newest), Buffer A is overwritten.
      //   4. Result: HA never receives LED/beep/SD Ã¢â€ â€™ "Unbekannt" forever.
      // WiFi is not immune but the OTA check is much faster there, so the race
      // is rarely observed.  With this injection both transports are reliable.
      if (s_send_state_pids) {
        s_send_state_pids = false;
        {
          uint8_t v = enableLedWhite ? 1 : 0;
          store.log(PID_LED_WHITE_STATE, &v, 1);
          s_lastLedWhite = (int8_t)v;
        }
        {
          uint8_t v = enableBeep ? 1 : 0;
          store.log(PID_BEEP_STATE, &v, 1);
          s_lastBeep = (int8_t)v;
        }
        if (state.check(STATE_NET_READY)) {
          uint8_t v = state.check(STATE_WIFI_CONNECTED) ? 1 : 2;
          store.log(PID_CONN_TYPE, &v, 1);
          s_lastConnType = (int8_t)v;
        }
        {
          uint8_t ov = enableObd ? 1 : 0;
          store.log(PID_OBD_STATE, &ov, 1);
          s_lastObd = (int8_t)ov;
        }
        {
          store.log(PID_STANDBY_TIME, &nvsStandbyTimeS, 1);
          s_lastStandbyTime = (int16_t)nvsStandbyTimeS;
        }
        {
          uint8_t dv = enableDeepStandby ? 1 : 0;
          store.log(PID_DEEP_STANDBY, &dv, 1);
          s_lastDeepStandby = (int8_t)dv;
        }
#if STORAGE == STORAGE_SD
        // s_cachedSdTotalMb/Free are kept current by process(); they are 0
        // before the first SD read which HA correctly interprets as "no card".
        store.log(PID_SD_TOTAL_MB, &s_cachedSdTotalMb, 1);
        store.log(PID_SD_FREE_MB,  &s_cachedSdFreeMb,  1);
#endif
      }
      store.tailer();
      Serial.print("[DAT] ");
      Serial.println(store.buffer());

      // start transmission
      // Snapshot enableLedWhite before the (blocking) transmit call so that
      // if the main task processes a LED_WHITE=0 /api/control command while
      // teleClient.transmit() is running, the LED is still driven LOW after
      // the transmission completes.  Without the snapshot, the check at the
      // second #ifdef PIN_LED block could see enableLedWhite=false and skip
      // the LOW write, leaving the LED stuck on.
#ifdef PIN_LED
      const bool ledWhiteFlash = enableLedWhite;
      if (ledWhiteFlash) digitalWrite(PIN_LED, HIGH);
#endif

      if (teleClient.transmit(store.buffer(), store.length())) {
        // successfully sent
        connErrors = 0;
        showStats();
      } else {
        timeoutsNet++;
        connErrors++;
        printTimeoutStats();
        if (connErrors < MAX_CONN_ERRORS_RECONNECT) {
          // quick reconnect
          if (!teleClient.connect(true)) {
            // Quick reconnect failed while the WiFi radio is still up.
            // Check if this is a heap-fragmentation failure (the TLS handshake
            // cannot allocate its internal buffers) rather than a transient
            // server-side error.  ESP.getMaxAllocHeap() returns the largest
            // contiguous free DRAM block; values below TLS_MIN_FREE_HEAP
            // indicate that mbedtls_ssl_setup()'s 2Ãƒâ€”17 KB record buffers
            // cannot be satisfied even if total free memory is nominally OK.
            // In that case waiting for MAX_CONN_ERRORS_RECONNECT attempts
            // wastes ~40 s in an unrecoverable state.  Disconnect WiFi now to
            // trigger the outer loop's WiFi-restart path (begin + setup), which
            // stops and re-starts the WiFi driver.  The driver's internal DRAM
            // allocations are freed by esp_wifi_stop() and re-initialised by
            // esp_wifi_start(), coalescing the fragmented heap and giving the
            // next TLS handshake a contiguous block to work with.
#if ENABLE_WIFI
            if (state.check(STATE_WIFI_CONNECTED) &&
                ESP.getMaxAllocHeap() < TLS_MIN_FREE_HEAP) {
              Serial.printf("[WIFI] Low heap (%u bytes max block) after TLS fail, restarting WiFi\n",
                            (unsigned)ESP.getMaxAllocHeap());
              teleClient.wifi.end();
              state.clear(STATE_NET_READY | STATE_WIFI_CONNECTED);
              break;
            }
#endif
          }
        }
      }
#ifdef PIN_LED
      if (ledWhiteFlash) digitalWrite(PIN_LED, LOW);
#endif
      store.purge();

      teleClient.inbound();

      if (state.check(STATE_CELL_CONNECTED) && !teleClient.cell.check(1000)) {
        Serial.println("[CELL] Not in service");
        state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
        break;
      }

      if (syncInterval > 10000 && millis() - teleClient.lastSyncTime > syncInterval) {
        Serial.println("[NET] Poor connection");
        timeoutsNet++;
        if (!teleClient.connect()) {
          connErrors++;
        }
      }

      if (connErrors >= MAX_CONN_ERRORS_RECONNECT) {
#if ENABLE_WIFI
        if (state.check(STATE_WIFI_CONNECTED)) {
          teleClient.wifi.end();
          state.clear(STATE_NET_READY | STATE_WIFI_CONNECTED);
          break;
        }
#endif
        if (state.check(STATE_CELL_CONNECTED)) {
          teleClient.cell.end();
          state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
          break;
        }
      }

      if (deviceTemp >= COOLING_DOWN_TEMP) {
        // device too hot, cool down by pause transmission
        Serial.print("HIGH DEVICE TEMP: ");
        Serial.println(deviceTemp);
        bufman.purge();
      }

    }
  }
}

/*******************************************************************************
  Implementing stand-by mode
*******************************************************************************/
void standby()
{
  state.set(STATE_STANDBY);

#if STORAGE == STORAGE_SD
  // Safety net: if s_ota_pending is set (e.g. esp_restart() failed in
  // performPullOtaCheck), apply the staged firmware now before shutting down.
  // Normally the device restarts immediately after staging and the boot-time
  // flash path handles this Ã¢â‚¬â€ this block should rarely execute.
  if (s_ota_pending && state.check(STATE_STORAGE_READY)) {
    s_ota_active = true;
    delay(OTA_TELEMETRY_YIELD_DELAY_MS); // give the telemetry task one scheduling cycle to yield
    if (performPullOtaFlash()) {
      // Flash succeeded; reboot timer is running Ã¢â‚¬â€ block here until it fires.
      while (true) delay(1000);
    }
    // Flash failed: clear flags and fall through to normal standby.
    s_ota_active = false;
    s_ota_pending = false;
  }
#endif

#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    logger.end();
  }
#endif

#if ENABLE_WIFI
  // Geofence-WiFi (2026-09-22): one-time check against the last known GPS fix
  // (gd still points at it here, before GPS is torn down below) - the car is
  // stationary through the whole of standby(), so there's no need to keep GPS
  // on or re-check periodically. Near a known location: leave WiFi as-is (no
  // battery cost, already close to a good network for the next wake). Far
  // from any: turn WiFi off now to protect the vehicle battery during standby;
  // the existing wifiConnect()/reconnect logic in the main loop brings it back
  // once standby ends, same as any other cold start.
  if (state.check(STATE_WIFI_CONNECTED) && knownLocationCount > 0) {
    float lastLat = gpsLat();
    float lastLng = gpsLng();
    if ((lastLat || lastLng) && !findNearbyKnownLocation(lastLat, lastLng, 0)) {
      Serial.println("[WIFI] Not near known location, WiFi OFF for standby");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      state.clear(STATE_NET_READY | STATE_WIFI_CONNECTED);
    }
  }
#endif

#if !GNSS_ALWAYS_ON && GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    Serial.println("[GNSS] OFF");
    sys.gpsEnd(true);
    state.clear(STATE_GPS_READY | STATE_GPS_ONLINE);
    gd = 0;
  }
#endif

  state.clear(STATE_WORKING | STATE_OBD_READY | STATE_STORAGE_READY);
  // this will put co-processor into sleep mode
  Serial.println("STANDBY");
  obd.enterLowPowerMode();

  // Deep standby: use ESP32 deep sleep for lower power consumption.
  // The device restarts after the wake-up timer expires (nvsStandbyTimeS seconds,
  // minimum 5 s).  Wake-up fully reinitialises all subsystems.
  if (enableDeepStandby) {
    uint64_t sleep_us = ((nvsStandbyTimeS >= 5) ? (uint64_t)nvsStandbyTimeS : 180ULL) * 1000000ULL;
    Serial.print("DEEP_SLEEP ");
    Serial.print((unsigned)(sleep_us / 1000000ULL));
    Serial.println("s");
    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();
    // Never reached: device restarts from setup() after wake-up.
  }

#if ENABLE_MEMS
  calibrateMEMS();
  waitMotion(-1);
#elif ENABLE_OBD
  do {
    delay(5000);
  } while (obd.getVoltage() < JUMPSTART_VOLTAGE);
#else
  delay(5000);
#endif
  Serial.println("WAKEUP");
  sys.resetLink();
#if RESET_AFTER_WAKEUP
#if ENABLE_MEMS
  if (mems) mems->end();  
#endif
  ESP.restart();
#endif  
  state.clear(STATE_STANDBY);
}

/*******************************************************************************
  Tasks to perform in idle/waiting time
*******************************************************************************/
void genDeviceID(char* buf)
{
    uint64_t seed = ESP.getEfuseMac() >> 8;
    for (int i = 0; i < 8; i++, seed >>= 5) {
      byte x = (byte)seed & 0x1f;
      if (x >= 10) {
        x = x - 10 + 'A';
        switch (x) {
          case 'B': x = 'W'; break;
          case 'D': x = 'X'; break;
          case 'I': x = 'Y'; break;
          case 'O': x = 'Z'; break;
        }
      } else {
        x += '0';
      }
      buf[i] = x;
    }
    buf[8] = 0;
}

void showSysInfo()
{
  Serial.print("CPU:");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.print("MHz FLASH:");
  Serial.print(ESP.getFlashChipSize() >> 20);
  Serial.println("MB");
  // ESP.getHeapSize() returns the total DRAM heap available to the application.
  // The ESP32's 520 KB SRAM is shared with the WiFi (~60 KB), BLE (~100 KB),
  // and FreeRTOS/driver stacks, so the application heap is typically ~230 KB.
  // PSRAM (external SPI RAM, present on ESP32-WROVER / Freematics ONE+ Model B)
  // is reported separately and is only used when BOARD_HAS_PSRAM is set at
  // compile time (see platformio.ini for details).
  Serial.print("RAM:");
  Serial.print(ESP.getHeapSize() >> 10);
  Serial.print("KB");
  if (psramInit()) {
    Serial.print(" PSRAM:");
    Serial.print(esp_spiram_get_size() >> 20);
    Serial.print("MB");
  }
  Serial.println();

  int rtc = rtc_clk_slow_freq_get();
  if (rtc) {
    Serial.print("RTC:");
    Serial.println(rtc);
  }


  Serial.print("DEVICE ID:");
  Serial.println(devid);
  Serial.print("FW:");
  Serial.print(FIRMWARE_VERSION);
  Serial.print(" Built:");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  if (nvsVersion[0]) {
    Serial.print("NVS:");
    Serial.println(nvsVersion);
  }
#if ENABLE_WIFI
  // Factory-burned MAC, readable even before WiFi.mode()/begin() - useful for
  // finding the device in the router's DHCP client list or for MAC-based
  // access-list entries, independent of whether it has joined a network yet.
  Serial.print("WIFI MAC:");
  Serial.println(WiFi.macAddress());
#endif
}

void loadConfig()
{
  size_t len;
  len = sizeof(apn);
  apn[0] = 0;
  nvs_get_str(nvs, "CELL_APN", apn, &len);
  if (!apn[0]) {
    strcpy(apn, CELL_APN);
  }

  len = sizeof(simPin);
  simPin[0] = 0;
  nvs_get_str(nvs, "SIM_PIN", simPin, &len);
  if (!simPin[0]) {
    strncpy(simPin, SIM_CARD_PIN, sizeof(simPin) - 1);
    simPin[sizeof(simPin) - 1] = 0;
  }

#if ENABLE_WIFI
  len = sizeof(wifiSSID);
  nvs_get_str(nvs, "WIFI_SSID", wifiSSID, &len);
  len = sizeof(wifiPassword);
  nvs_get_str(nvs, "WIFI_PWD", wifiPassword, &len);
  len = sizeof(wifiSSID2);
  nvs_get_str(nvs, "WIFI_SSID2", wifiSSID2, &len);
  len = sizeof(wifiPassword2);
  nvs_get_str(nvs, "WIFI_PWD2", wifiPassword2, &len);
#endif

  // Server settings provisioned via NVS (e.g. via HA integration config_nvs.bin).
  // Override the compile-time SERVER_HOST / SERVER_PORT defaults when present.
  len = sizeof(serverHost);
  serverHost[0] = 0;
  nvs_get_str(nvs, "SERVER_HOST", serverHost, &len);
  if (!serverHost[0]) {
    strncpy(serverHost, SERVER_HOST, sizeof(serverHost) - 1);
    serverHost[sizeof(serverHost) - 1] = 0;
  }
  uint16_t nvsPort = 0;
  nvs_get_u16(nvs, "SERVER_PORT", &nvsPort);
  if (nvsPort) serverPort = nvsPort;

  len = sizeof(webhookPath);
  webhookPath[0] = 0;
  nvs_get_str(nvs, "WEBHOOK_PATH", webhookPath, &len);

  // Cellular-specific server overrides (NVS keys CELL_HOST, CELL_PORT, CELL_PATH).
  // When present, these are used instead of SERVER_HOST / SERVER_PORT /
  // WEBHOOK_PATH for cellular (SIM7600) connections.  Provisioned by the HA
  // integration with hooks.nabu.casa when Nabu Casa cloud is active, so that
  // SIM7600 devices reach the cloud webhook endpoint rather than the Remote UI
  // proxy (*.ui.nabu.casa) which the SIM7600 TLS stack cannot handle.
  len = sizeof(cellServerHost);
  cellServerHost[0] = 0;
  nvs_get_str(nvs, "CELL_HOST", cellServerHost, &len);
  uint16_t nvsCellPort = 0;
  nvs_get_u16(nvs, "CELL_PORT", &nvsCellPort);
  if (nvsCellPort) cellServerPort = nvsCellPort;
  len = sizeof(cellWebhookPath);
  cellWebhookPath[0] = 0;
  nvs_get_str(nvs, "CELL_PATH", cellWebhookPath, &len);

  // Enable HTTP server at runtime when provisioned via config_nvs.bin.
  // Only has an effect when the firmware is compiled with ENABLE_HTTPD=1.
  uint8_t nvsHttpd = 0;
  if (nvs_get_u8(nvs, "ENABLE_HTTPD", &nvsHttpd) == ESP_OK) {
    enableHttpd = nvsHttpd;
  }

#if ENABLE_BLE
  // Enable/disable BLE at runtime.  NVS key ENABLE_BLE is written by the HA
  // integration (0 = off, 1 = on).  Disabling BLE frees ~100 KB of heap,
  // which prevents MBEDTLS_ERR_SSL_ALLOC_FAILED during the TLS handshake for
  // the HTTPS webhook.  The key is absent on un-provisioned devices so the
  // default (enableBle = 1) preserves backwards-compatible behaviour.
  uint8_t nvsBle = 1;
  if (nvs_get_u8(nvs, "ENABLE_BLE", &nvsBle) == ESP_OK) {
    enableBle = nvsBle;
  } else if (webhookPath[0]) {
    // ENABLE_BLE was not provisioned in NVS but webhook mode is active.
    // Auto-disable BLE so the ~100 KB it occupies is available for the TLS
    // handshake (prevents MBEDTLS_ERR_SSL_ALLOC_FAILED on hooks.nabu.casa).
    // Users who want BLE alongside webhooks can set ENABLE_BLE=1 explicitly.
    enableBle = 0;
  }
#endif

  // Enable verbose cellular debug logging at runtime.  NVS key CELL_DEBUG is
  // written by the HA config/options flow (0 = off, 1 = on).  When enabled the
  // firmware prints TX-Preview, hex-dump, AT+CCHSTATUS? and per-packet
  // "Incoming data" diagnostics to the serial console.  Default is 0 (off).
  uint8_t nvsCellDebug = 0;
  if (nvs_get_u8(nvs, "CELL_DEBUG", &nvsCellDebug) == ESP_OK) {
    cellNetDebug = nvsCellDebug;
  }

  // LED and buzzer behaviour overrides written by the HA config/options flow.
  // All default to 1 (enabled) when the NVS key is absent so un-provisioned
  // devices keep the original out-of-box behaviour.
  //
  // LED_RED_EN  Ã¢â‚¬â€œ red/power LED (standby / power-on indicator)
  // LED_WHITE_EN Ã¢â‚¬â€œ white/network LED (data-transmission indicator)
  // BEEP_EN     Ã¢â‚¬â€œ short buzzer beep on each WiFi/cellular connect event
  uint8_t nvsLedRedEn = 1;
  if (nvs_get_u8(nvs, "LED_RED_EN", &nvsLedRedEn) == ESP_OK) {
    enableLedRed = nvsLedRedEn != 0;
  }
  uint8_t nvsLedWhiteEn = 1;
  if (nvs_get_u8(nvs, "LED_WHITE_EN", &nvsLedWhiteEn) == ESP_OK) {
    enableLedWhite = nvsLedWhiteEn != 0;
  }
  uint8_t nvsBeepEn = 1;
  if (nvs_get_u8(nvs, "BEEP_EN", &nvsBeepEn) == ESP_OK) {
    enableBeep = nvsBeepEn != 0;
  }

  // OBD querying enable/disable (NVS key OBD_EN, u8).
  // Defaults to 1 (on) so un-provisioned devices keep existing OBD behaviour.
  uint8_t nvsObdEn = 1;
  if (nvs_get_u8(nvs, "OBD_EN", &nvsObdEn) == ESP_OK) {
    enableObd = nvsObdEn != 0;
  }

  // Deep-standby mode (NVS key DEEP_STANDBY, u8, 0=off 1=on).
  // When enabled the device uses ESP32 deep sleep during standby.
  uint8_t nvsDeepStandby = 0;
  if (nvs_get_u8(nvs, "DEEP_STANDBY", &nvsDeepStandby) == ESP_OK) {
    enableDeepStandby = nvsDeepStandby != 0;
  }

  // Standby-time override (NVS key STANDBY_TIME, u16, seconds, 5-900).
  // 0 means "use compile-time STATIONARY_TIME_TABLE default" (currently 180 s).
  // Values below 5 are clamped to 0 (use default) for safety.
  uint16_t nvsStby = 0;
  if (nvs_get_u16(nvs, "STANDBY_TIME", &nvsStby) == ESP_OK) {
    nvsStandbyTimeS = (nvsStby >= 5) ? nvsStby : 0;
  }

  // Optional data-interval override (milliseconds). Minimum 500 ms to avoid
  // flooding the server or the SD card.  0 / missing = keep compile-time default.
  uint16_t nvsDataInterval = 0;
  if (nvs_get_u16(nvs, "DATA_INTERVAL", &nvsDataInterval) == ESP_OK && nvsDataInterval >= 500) {
    dataInterval = nvsDataInterval;
  }

  // Optional server-sync-interval override (seconds).  0 = keep default.
  uint16_t nvsSyncInterval = 0;
  if (nvs_get_u16(nvs, "SYNC_INTERVAL", &nvsSyncInterval) == ESP_OK && nvsSyncInterval > 0) {
    syncInterval = (int32_t)nvsSyncInterval * 1000;
  }

  // Pull-OTA configuration (firmware v5.2+).
  // OTA_TOKEN: secret path token; when set enables periodic firmware checks.
  len = sizeof(otaToken);
  otaToken[0] = 0;
  nvs_get_str(nvs, "OTA_TOKEN", otaToken, &len);

  // OTA_HOST: HA server for pull-OTA (may differ from serverHost).
  len = sizeof(otaHost);
  otaHost[0] = 0;
  nvs_get_str(nvs, "OTA_HOST", otaHost, &len);
  if (!otaHost[0] && otaToken[0]) {
    // Fall back to serverHost when no separate OTA_HOST is provisioned.
    strncpy(otaHost, serverHost, sizeof(otaHost) - 1);
    otaHost[sizeof(otaHost) - 1] = 0;
  }

  uint16_t nvsOtaPort = 0;
  nvs_get_u16(nvs, "OTA_PORT", &nvsOtaPort);
  if (nvsOtaPort) otaPort = nvsOtaPort;

  uint16_t nvsOtaInterval = 0;
  nvs_get_u16(nvs, "OTA_INTERVAL", &nvsOtaInterval);
  otaCheckIntervalS = nvsOtaInterval;

  // Geofence-WiFi known locations (2026-09-22), cached from the last
  // successful syncKnownLocations() so a reboot right after arriving near a
  // location doesn't lose the table until the next sync. See its own
  // definition (near the OTA config globals) for the full design story.
  // A length/count mismatch (blob missing, or a size left over from a
  // firmware build with a different MAX_KNOWN_LOCATIONS/struct layout)
  // fails safe to an empty table rather than risking garbage entries.
  {
    uint8_t n = 0;
    nvs_get_u8(nvs, "LOC_N", &n);
    if (n > MAX_KNOWN_LOCATIONS) n = MAX_KNOWN_LOCATIONS;
    size_t blobLen = n * sizeof(KnownLocation);
    if (n > 0 && nvs_get_blob(nvs, "LOCS", knownLocations, &blobLen) == ESP_OK
        && blobLen == n * sizeof(KnownLocation)) {
      knownLocationCount = n;
    } else {
      knownLocationCount = 0;
    }
  }

  // NVS settings version (NVS_VER key).  Written by the HA integration when
  // generating the NVS partition image.  Logged at boot so the user can verify
  // which settings version is active on the device (especially after a serial
  // flash or OTA NVS update).  Absent on devices never provisioned by HA.
  len = sizeof(nvsVersion);
  nvsVersion[0] = 0;
  nvs_get_str(nvs, "NVS_VER", nvsVersion, &len);

  // Vehicle identification (NVS keys VEHICLE_MAKE, VEHICLE_MODEL, VEHICLE_YEAR).
  // Optional Ã¢â‚¬â€œ absent on devices not provisioned with vehicle info.
  size_t vlen = sizeof(vehicleMake);
  nvs_get_str(nvs, "VEHICLE_MAKE", vehicleMake, &vlen);
  vlen = sizeof(vehicleModel);
  nvs_get_str(nvs, "VEHICLE_MODEL", vehicleModel, &vlen);
  vlen = sizeof(vehicleYear);
  nvs_get_str(nvs, "VEHICLE_YEAR", vehicleYear, &vlen);

  // Vehicle-specific extra PIDs (NVS key VEHICLE_PIDS).
  // Comma-separated hex values, e.g. "22,23,5A".
  // Parse into vehicleObdData[] so processOBD() can poll them at tier 3.
  vlen = sizeof(vehiclePidsStr);
  vehiclePidsStr[0] = 0;
  nvs_get_str(nvs, "VEHICLE_PIDS", vehiclePidsStr, &vlen);
  vehicleObdDataCount = 0;
  if (vehiclePidsStr[0]) {
    char tmp[sizeof(vehiclePidsStr)];
    strncpy(tmp, vehiclePidsStr, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = 0;
    char *tok = strtok(tmp, ",");
    while (tok && vehicleObdDataCount < MAX_VEHICLE_PIDS) {
      byte pid = (byte)strtol(tok, nullptr, 16);
      // Skip PID 0x00 (the "PIDs supported [01-20]" support bitmap) Ã¢â‚¬â€œ it is
      // handled internally by isValidPID() and not a pollable value PID.
      if (pid > 0) {
        vehicleObdData[vehicleObdDataCount].pid   = pid;
        vehicleObdData[vehicleObdDataCount].tier  = 3;
        vehicleObdData[vehicleObdDataCount].value = 0;
        vehicleObdData[vehicleObdDataCount].ts    = 0;
        vehicleObdDataCount++;
      }
      tok = strtok(nullptr, ",");
    }
    if (vehicleObdDataCount > 0) {
      Serial.printf("VEHICLE:%s %s %s pids=%d\n",
                    vehicleMake, vehicleModel, vehicleYear, vehicleObdDataCount);
    }
  }
}

// ---------------------------------------------------------------------------
// Pull-OTA check (Variant 1: authenticated token endpoint;
//                 Variant 2: /local/ public endpoint)
// ---------------------------------------------------------------------------
// Checks the pull-OTA metadata endpoint and, if a new firmware version is
// available, downloads and flashes it using the Arduino Update library.
//
// The metadata endpoint is:
//   GET https://{otaHost}:{otaPort}/api/freematics/ota_pull/{otaToken}/meta.json
//
// For Variant 2 (/local/ deployment) the same path structure is used:
//   GET https://{otaHost}:{otaPort}/local/FreematicsONE/{devid}/version.json
// The caller stores the appropriate prefix in otaToken (e.g. the /local/ path).
//
// Returns true if a firmware update was successfully applied (device will
// reboot shortly after), false otherwise.
// ---------------------------------------------------------------------------

// Shared chunk buffer used by both the download (to SD) and flash (from SD)
// phases.  See the declaration near the top of the file (after s_ota_pending).
// PULL_OTA_MIN_FW_SIZE / PULL_OTA_CHUNK_SIZE / PULL_OTA_CHUNK_TIMEOUT_MS /
// OTA_TELEMETRY_YIELD_DELAY_MS are also defined near the top.

#if STORAGE == STORAGE_SD
// ---------------------------------------------------------------------------
// _applyNvsFromSD()
//
// Writes a staged NVS partition image (/ota_nvs.bin) directly to the NVS
// flash partition so that updated settings (WiFi credentials, LED behaviour,
// BLE, data interval, etc.) take effect on the next boot without requiring a
// serial re-flash.
//
// Called by performPullOtaFlash() AFTER a successful firmware flash, just
// before the reboot timer fires.  If the NVS staging file does not exist the
// function returns true immediately (NVS update is optional).
//
// On any error the staging file is removed and false is returned; the caller
// logs the outcome and continues with the reboot (firmware was already
// flashed successfully, so only the settings update failed).
// ---------------------------------------------------------------------------
static bool _applyNvsFromSD()
{
  if (!SD.exists(OTA_NVS_PATH)) {
    return true; // no NVS update staged; not an error
  }

  File nvsFile = SD.open(OTA_NVS_PATH, FILE_READ);
  if (!nvsFile) {
    Serial.println("[OTA-PULL] Cannot open NVS staging file");
    SD.remove(OTA_NVS_PATH);
    return false;
  }
  size_t nvsSize = nvsFile.size();

  // Sanity check: the NVS image must be at least 4 KB and at most the full
  // partition size (20 KB).  Anything outside that range is corrupt.
  if (nvsSize < 4096 || nvsSize > 0x5000) {
    nvsFile.close();
    Serial.printf("[OTA-PULL] NVS staging file has invalid size: %u Ã¢â‚¬â€ skipping\n",
                  (unsigned)nvsSize);
    SD.remove(OTA_NVS_PATH);
    return false;
  }

  // Read the entire NVS image into a heap buffer BEFORE touching the NVS flash
  // partition.  This is critical: if we erased the partition first and then
  // encountered an SD read error mid-write, the device would reboot with a
  // blank NVS Ã¢â‚¬â€ losing WiFi credentials, OTA_TOKEN, and all other settings.
  // By buffering the full image first we guarantee that either (a) the SD read
  // succeeds and we can safely erase + write, or (b) we leave the existing NVS
  // intact and return false so the caller logs "rebooting with old NVS".
  //
  // Allocate nvsSize rounded up to a 4-byte boundary so the alignment padding
  // applied to the last write chunk (esp_partition_write requires 4-byte-aligned
  // sizes) never writes beyond the allocated region.
  size_t nvsBufSize = (nvsSize + 3) & ~3UL;
  uint8_t* nvsBuf = (uint8_t*)malloc(nvsBufSize);
  if (!nvsBuf) {
    nvsFile.close();
    Serial.println("[OTA-PULL] NVS: not enough RAM to buffer image Ã¢â‚¬â€ skipping NVS update");
    SD.remove(OTA_NVS_PATH);
    return false;
  }
  // Pre-fill the alignment-padding tail with 0xFF so it is ready for the write.
  if (nvsBufSize > nvsSize) {
    memset(nvsBuf + nvsSize, 0xFF, nvsBufSize - nvsSize);
  }

  size_t readTotal = 0;
  bool readOk = true;
  while (readTotal < nvsSize) {
    int toRead = (int)(nvsSize - readTotal);
    if (toRead > (int)PULL_OTA_CHUNK_SIZE) toRead = (int)PULL_OTA_CHUNK_SIZE;
    int n = nvsFile.read(nvsBuf + readTotal, toRead);
    if (n <= 0) {
      Serial.printf("[OTA-PULL] NVS SD read error at offset %u\n", (unsigned)readTotal);
      readOk = false;
      break;
    }
    readTotal += (size_t)n;
  }
  nvsFile.close();
  SD.remove(OTA_NVS_PATH);

  if (!readOk || readTotal != nvsSize) {
    free(nvsBuf);
    Serial.println("[OTA-PULL] NVS staging file read incomplete Ã¢â‚¬â€ NVS unchanged");
    return false;
  }

  const esp_partition_t* nvsPart = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "nvs");
  if (!nvsPart) {
    free(nvsBuf);
    Serial.println("[OTA-PULL] NVS partition not found in partition table");
    return false;
  }

  // Deinit the NVS library before touching the underlying flash so no
  // in-memory pages are written back after we erase the partition.
  nvs_flash_deinit();

  esp_err_t err = esp_partition_erase_range(nvsPart, 0, nvsPart->size);
  if (err != ESP_OK) {
    free(nvsBuf);
    Serial.printf("[OTA-PULL] NVS partition erase failed: %d\n", (int)err);
    return false;
  }

  // Write the buffered NVS image to the flash partition in PULL_OTA_CHUNK_SIZE
  // (4 KB) chunks.  The SD file has already been read and removed above.
  // The tail of nvsBuf was pre-filled with 0xFF so alignment padding for the
  // last chunk is already in place Ã¢â‚¬â€ no per-chunk memset is needed.
  size_t written = 0;
  bool writeOk = true;
  while (written < nvsSize) {
    size_t toWrite = nvsSize - written;
    if (toWrite > PULL_OTA_CHUNK_SIZE) toWrite = PULL_OTA_CHUNK_SIZE;
    // Round up to 4-byte boundary for esp_partition_write; tail bytes are 0xFF.
    size_t aligned = (toWrite + 3) & ~3UL;
    err = esp_partition_write(nvsPart, written, nvsBuf + written, aligned);
    if (err != ESP_OK) {
      Serial.printf("[OTA-PULL] NVS partition write failed at offset %u: %d\n",
                    (unsigned)written, (int)err);
      writeOk = false;
      break;
    }
    written += toWrite;
  }
  free(nvsBuf);

  if (!writeOk) {
    return false;
  }

  Serial.printf("[OTA-PULL] NVS settings applied: %u bytes written\n", (unsigned)written);
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL NVS OK");
#endif
  return true;
}

// ---------------------------------------------------------------------------
// performPullOtaFlash()
//
// Reads the firmware binary staged on the SD card (/ota_fw.bin) and writes
// it to the OTA flash partition using the Arduino Update library.  Called by
// standby() at the next power-down / sleep transition Ã¢â‚¬â€ NEVER during active
// telemetry Ã¢â‚¬â€ so no live data is lost.
//
// Pre-conditions (checked here):
//   Ã¢â‚¬Â¢ s_ota_pending == true  (set by performPullOtaCheck() after download)
//   Ã¢â‚¬Â¢ OTA_META_PATH exists on SD and contains the expected byte count
//   Ã¢â‚¬Â¢ OTA_PENDING_PATH exists on SD and its size matches OTA_META_PATH
//
// Returns true if the flash succeeded and the reboot timer is running.
// On any error, staging files are cleaned up and false is returned so that
// standby() can clear s_ota_pending and continue normally.
// ---------------------------------------------------------------------------
static bool performPullOtaFlash()
{
  // --- Verify staging files ------------------------------------------------
  if (!SD.exists(OTA_PENDING_PATH)) {
    Serial.println("[OTA-PULL] Staged firmware not found on SD");
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_NOT_FOUND");
#endif
    SD.remove(OTA_META_PATH);
    return false;
  }

  // Read expected size from companion meta file.
  unsigned long expectedSize = 0;
  {
    File mf = SD.open(OTA_META_PATH, FILE_READ);
    if (mf) {
      char buf[16] = {0};
      mf.readBytesUntil('\n', buf, sizeof(buf) - 1);
      mf.close();
      expectedSize = strtoul(buf, nullptr, 10);
    }
  }
  {
    File ff = SD.open(OTA_PENDING_PATH, FILE_READ);
    unsigned long actual = ff ? (unsigned long)ff.size() : 0UL;
    if (ff) ff.close();
    if (expectedSize < PULL_OTA_MIN_FW_SIZE || actual != expectedSize) {
      Serial.printf("[OTA-PULL] SD staging size mismatch: file=%lu expected=%lu Ã¢â‚¬â€ removing\n",
                    actual, expectedSize);
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_SIZE");
#endif
      SD.remove(OTA_PENDING_PATH);
      SD.remove(OTA_META_PATH);
      return false;
    }
  }

  size_t fwSize = (size_t)expectedSize;
  Serial.printf("[OTA-PULL] Flashing staged firmware from SD: %u bytes\n", (unsigned)fwSize);
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    char diag[48];
    snprintf(diag, sizeof(diag), "OTA-PULL FLASH START SIZE=%u", (unsigned)fwSize);
    logger.logEvent(diag);
  }
#endif

  if (!Update.begin(fwSize)) {
    Serial.printf("[OTA-PULL] Update.begin failed: %s\n", Update.errorString());
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=UPD_BEGIN");
#endif
    return false;
  }

  File fwFile = SD.open(OTA_PENDING_PATH, FILE_READ);
  if (!fwFile) {
    Serial.println("[OTA-PULL] Cannot open staged firmware for reading");
    Update.abort();
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_OPEN_READ");
#endif
    return false;
  }

  size_t written = 0;
  uint32_t t0 = millis();
  size_t lastLogAt = 0;

  while (written < fwSize) {
    int toRead = (int)(fwSize - written);
    if (toRead > (int)PULL_OTA_CHUNK_SIZE) toRead = (int)PULL_OTA_CHUNK_SIZE;
    int n = fwFile.read(s_otaChunkBuf, toRead);
    if (n <= 0) {
      Serial.printf("[OTA-PULL] SD read error at offset %u\n", (unsigned)written);
      Update.abort();
      fwFile.close();
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_READ");
#endif
      return false;
    }
    size_t w = Update.write(s_otaChunkBuf, (size_t)n);
    if (w != (size_t)n) {
      Serial.printf("[OTA-PULL] Flash write error at offset %u\n", (unsigned)written);
      Update.abort();
      fwFile.close();
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FLASH_WRITE");
#endif
      return false;
    }
    written += (size_t)n;

    // Log progress every 10%.
    if (written - lastLogAt >= (fwSize / 10 ? fwSize / 10 : 1)) {
      lastLogAt = written;
      Serial.printf("[OTA-PULL] Flash %u / %u bytes (%.0f%%) in %u ms\n",
                    (unsigned)written, (unsigned)fwSize,
                    100.0f * written / fwSize, (unsigned)(millis() - t0));
    }
  }
  fwFile.close();

  if (!Update.end()) {
    Serial.printf("[OTA-PULL] Update.end failed: %s\n", Update.errorString());
    SD.remove(OTA_PENDING_PATH);
    SD.remove(OTA_META_PATH);
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=UPD_END");
#endif
    return false;
  }

  // --- Flash successful: clean up SD, log, apply NVS settings if staged -----
  SD.remove(OTA_PENDING_PATH);
  SD.remove(OTA_META_PATH);

  Serial.printf("[OTA-PULL] Flash from SD successful: %u bytes in %u ms\n",
                (unsigned)written, (unsigned)(millis() - t0));

  // Apply NVS settings update if staged.  This is done AFTER a successful
  // firmware flash so a settings-write failure never prevents the firmware
  // update.  On failure the device still reboots into the new firmware
  // (with old NVS settings); the settings will be retried on the next OTA
  // check interval (the HA side keeps nvs_version unsynchronised until the
  // device successfully downloads and applies nvs.bin).
  {
    bool _nvsStaged = SD.exists(OTA_NVS_PATH);
    if (_applyNvsFromSD()) {
      if (_nvsStaged) {
        Serial.println("[OTA-PULL] Settings (NVS) updated successfully");
      } else {
        Serial.println("[OTA-PULL] No NVS settings staged Ã¢â‚¬â€ rebooting with firmware only");
      }
    } else {
      Serial.println("[OTA-PULL] Settings (NVS) update failed Ã¢â‚¬â€ rebooting with old NVS");
    }
  }

#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    char diag[64];
    snprintf(diag, sizeof(diag), "OTA-PULL FLASH OK FW=%s SIZE=%u",
             FIRMWARE_VERSION, (unsigned)written);
    logger.logEvent(diag);
    logger.flush();  // ensure log entry survives the reboot
  }
#endif

  // s_ota_active remains true; device reboots shortly.
  static esp_timer_handle_t s_ota_flash_timer = NULL;
  if (!s_ota_flash_timer) {
    esp_timer_create_args_t args = {};
    args.callback        = [](void*) { esp_restart(); };
    args.dispatch_method = ESP_TIMER_TASK;
    args.name            = "ota_flash_reboot";
    esp_timer_create(&args, &s_ota_flash_timer);
  } else {
    esp_timer_stop(s_ota_flash_timer);
  }
  esp_timer_start_once(s_ota_flash_timer, 1500000);
  return true;
}
#endif  // STORAGE == STORAGE_SD

// ---------------------------------------------------------------------------
// _maskOtaHost()
//
// Returns a privacy-safe representation of an OTA hostname for serial
// logging.  Shows only the first 8 characters followed by "..." when the
// hostname is longer than 12 characters, preventing NabuCasa subdomain IDs
// from appearing verbatim in serial output.
// ---------------------------------------------------------------------------
static String _maskOtaHost(const char* host) {
  if (!host || !host[0]) return String("(none)");
  size_t n = strlen(host);
  if (n > 12) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.8s...", host);
    return String(buf);
  }
  return String(host);
}

// ---------------------------------------------------------------------------
// performPullOtaCheck()
//
// Fetches the pull-OTA metadata endpoint and, when a newer firmware version
// is available, downloads it.  Works over WiFi only.
//
// OTA over cellular is not supported: TLS on SIM7600E-H modems is unreliable
// (TLS error 15 against Cloudflare and Nabu Casa endpoints) and cannot be
// resolved with reasonable effort.  Use WiFi (or a mobile hotspot) for OTA.
//
// For STORAGE == STORAGE_SD the binary is saved to the SD card staging file
// (OTA_PENDING_PATH) and the flash is DEFERRED to the next standby
// transition.  s_ota_pending is set to true on a successful download so
// that standby() calls performPullOtaFlash().
//
// For other storage configurations (STORAGE_NONE / STORAGE_SPIFFS) the old
// direct-flash path is used as a fallback.
//
// Returns true only when the direct-flash path has started (reboot imminent).
// Returns false in all other cases, including the SD-staging success case
// (caller must NOT block waiting for a reboot when false is returned).
// ---------------------------------------------------------------------------
// 2026-09-14: this used to depend on teleClient.wifi being HTTP-capable
// (WifiHTTP: .open/.send/.receiveHeaders/.rawClient/.code), which only holds
// for HTTP-capable SERVER_PROTOCOL builds - with SERVER_PROTOCOL=PROTOCOL_UDP
// (this device's actual build), teleClient.wifi is WifiUDP instead, which has
// none of those methods, so the real implementation below never got compiled
// at all for this build - performPullOtaCheck() silently resolved to the
// no-op stub further down, and pull-OTA never worked, no matter how long
// WiFi stayed connected. Root-caused 2026-09-14 by reading the actual
// #if/#else split, not by guessing at network/timing causes (which is what
// consumed most of a day's debugging before this was found).
// Fix: a dedicated otaWifiClient (WifiHTTP, declared where teleClient is)
// used ONLY by pull-OTA, completely independent of SERVER_PROTOCOL/
// teleClient.wifi's type - so this now compiles and runs on every
// ENABLE_WIFI build regardless of the main telemetry transport.
#if ENABLE_WIFI
bool performPullOtaCheck()
{
  if (!otaToken[0] || !otaHost[0]) return false;

  // OTA is only supported over WiFi.  Cellular is not supported because
  // TLS on SIM7600E-H modems cannot reliably connect to the OTA endpoint
  // (TLS error 15 against Cloudflare / Nabu Casa Remote UI domains).
#if ENABLE_WIFI
  if (!WiFi.isConnected()) return false;
#else
  return false;  // No WiFi compiled in Ã¢â‚¬â€ OTA unavailable
#endif

#if STORAGE == STORAGE_SD
  // If a firmware is already staged on SD (from a previous download that set
  // s_ota_pending before the esp_restart() fired), the restart did not happen
  // yet.  Trigger it now so the boot-time flash runs cleanly.
  if (s_ota_pending) {
    Serial.println("[OTA-PULL] Firmware already staged on SD, restarting to flash");
    esp_restart();
    return false;  // unreachable
  }
#endif

  // ---- Step 1: Fetch metadata JSON ----------------------------------------
  // Build the metadata path: /api/freematics/ota_pull/{token}/meta.json
  // Query string reports the currently-running build (__DATE__ __TIME__,
  // e.g. "Sep 14 2026 12:04:32") so the server can tell whether the
  // firmware it would offer is already installed, instead of blindly
  // offering "available:true" forever and re-flashing the same build every
  // OTA_INTERVAL cycle. Spaces are the only character in that string not
  // safe raw in a query value - percent-encoded here.
  //
  // variant=FIRMWARE_VERSION (e.g. "5.3-odo-PSA") is also sent - this is the
  // device's own periodic pull-check, a completely separate path from the
  // push-decision service's (ota_push_watcher.py) variant safety check,
  // which only guards ITS OWN trigger and does nothing to stop this pull
  // path from fetching whatever a mismatched/stale token happens to point
  // at. ota_server.py's meta.json handler refuses when this doesn't match
  // the registry entry's own expected_variant, so a device can never pull
  // another vehicle's firmware over its own token by itself either.
  char metaPath[448];
  {
    char buildEnc[48];
    int bi = 0;
    for (const char* p = __DATE__ " " __TIME__; *p && bi < (int)sizeof(buildEnc) - 4; p++) {
      if (*p == ' ') {
        buildEnc[bi++] = '%'; buildEnc[bi++] = '2'; buildEnc[bi++] = '0';
      } else {
        buildEnc[bi++] = *p;
      }
    }
    buildEnc[bi] = 0;
    snprintf(metaPath, sizeof(metaPath),
             "/api/freematics/ota_pull/%s/meta.json?build=%s&variant=%s",
             otaToken, buildEnc, FIRMWARE_VERSION);
  }

  Serial.printf("[OTA-PULL] URL: https://%s:%u/api/freematics/ota_pull/%.8s.../meta.json\n",
                _maskOtaHost(otaHost).c_str(), (unsigned)otaPort, otaToken);

  char metaBuf[2048];
  int metaBytes = 0;
  char* metaBody = nullptr;

#if ENABLE_WIFI
  // WifiHTTP::open() handles all session-reuse and heap-guard logic:
  //   Ã¢â‚¬Â¢ Same host as telemetry: reuse the existing TLS session (zero cost).
  //   Ã¢â‚¬Â¢ Different host, heap OK: stop() + connect() atomically.
  //   Ã¢â‚¬Â¢ Low heap: Guard 1 or Guard 2 fires, returns false Ã¢â€ â€™ caller restarts WiFi.
  if (!otaWifiClient.open(otaHost, otaPort)) {
    Serial.printf("[OTA-PULL] Cannot connect to %s:%u\n", _maskOtaHost(otaHost).c_str(), (unsigned)otaPort);
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=CONNECT");
#endif
    return false;
  }

  if (!otaWifiClient.send(METHOD_GET, metaPath)) {
    Serial.println("[OTA-PULL] META send failed");
    otaWifiClient.close();
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=META_SEND");
#endif
    return false;
  }

  metaBody = otaWifiClient.receive(metaBuf, sizeof(metaBuf) - 1, &metaBytes);
  if (!metaBody || otaWifiClient.code() != 200) {
    Serial.printf("[OTA-PULL] META HTTP %u\n", (unsigned)otaWifiClient.code());
    otaWifiClient.close();
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) {
      char _ota_diag[48];
      snprintf(_ota_diag, sizeof(_ota_diag), "OTA-PULL ERR=META_HTTP%d", (int)otaWifiClient.code());
      logger.logEvent(_ota_diag);
    }
#endif
    return false;
  }
  metaBuf[metaBytes < (int)sizeof(metaBuf) - 1 ? metaBytes : (int)sizeof(metaBuf) - 1] = '\0';
  // Do NOT close the WiFi connection here: keep it alive so both the next
  // telemetry packet and the next OTA check (60 s later) can reuse the same
  // TLS session.  receive() has already updated m_state (HTTP_CONNECTED when
  // the server sent keep-alive, HTTP_DISCONNECTED when it sent Connection:close).
  // In the keep-alive case this avoids one needless TLS teardown+handshake cycle
  // per OTA check interval.  In the close case open() will call client.stop()
  // immediately before client.connect(), preventing heap fragmentation by other
  // tasks from grabbing the freed TLS block between the two calls.
#endif  // ENABLE_WIFI

  // ---- Parse metadata JSON ------------------------------------------------

  // Parse "available": true / false
  if (!strstr(metaBody, "\"available\":true") && !strstr(metaBody, "\"available\": true")) {
    Serial.println("[OTA-PULL] No update available");
    return false;
  }

  // Parse optional nvs_only flag Ã¢â‚¬â€ set by the HA server when the firmware
  // binary was already delivered (version matches in ota_pull_state.json) but
  // the NVS settings partition is outdated (e.g. WiFi credentials or LED/beep
  // state changed, or nvs.bin failed to download alongside the firmware in a
  // previous cycle due to heap fragmentation after the large firmware transfer).
  // In this mode the device skips firmware download entirely, fetches only the
  // small NVS binary with a fresh heap (no prior fragmentation), applies it
  // directly to the NVS flash partition, and restarts to activate the settings.
  bool nvsOnly = (strstr(metaBody, "\"nvs_only\":true")  != nullptr ||
                  strstr(metaBody, "\"nvs_only\": true") != nullptr);

  // Parse optional nvs_url field (present in both normal and nvs_only updates).
  char nvsPath[256] = "";
  {
    char* nvsField = strstr(metaBody, "\"nvs_url\":");
    if (nvsField) {
      char* start = strchr(nvsField + 10, '"');
      if (start) {
        start++;
        char* end = strchr(start, '"');
        if (end && (size_t)(end - start) < sizeof(nvsPath) - 1) {
          memcpy(nvsPath, start, end - start);
          nvsPath[end - start] = '\0';
        }
      }
    }
  }

  // ---- NVS-only update path -----------------------------------------------
  // Entered when firmware is current but NVS settings are outdated.
  // Downloads only the NVS binary (heap is fresh Ã¢â‚¬â€ no prior firmware download),
  // applies it directly, and restarts to activate the new settings.
#if STORAGE == STORAGE_SD
  if (nvsOnly) {
    if (!state.check(STATE_STORAGE_READY)) {
      Serial.println("[OTA-PULL] NVS-only: SD not ready, skipping");
      return false;
    }
    Serial.println("[OTA-PULL] NVS-only update: downloading settings binary");
    if (SD.exists(OTA_NVS_PATH)) SD.remove(OTA_NVS_PATH);
    if (nvsPath[0]) {
#if ENABLE_WIFI
      if (otaWifiClient.open(otaHost, otaPort) &&
          otaWifiClient.send(METHOD_GET, nvsPath)) {
        int _nvsCL = 0;
        int _nvsHC = otaWifiClient.receiveHeaders(&_nvsCL);
        if (_nvsHC == 200 && _nvsCL > 0) {
          File _nvsFile = SD.open(OTA_NVS_PATH, FILE_WRITE);
          if (_nvsFile) {
            WiFiClientSecure& _nvsRaw = otaWifiClient.rawClient();
            size_t _nvsWr = 0, _nvsExp = (size_t)_nvsCL;
            bool _nvsOk = true;
            while (_nvsWr < _nvsExp) {
              uint32_t _t1 = millis();
              while (!_nvsRaw.available() && millis() - _t1 < PULL_OTA_CHUNK_TIMEOUT_MS) delay(1);
              if (!_nvsRaw.available()) { _nvsOk = false; break; }
              int _nr = (int)(_nvsExp - _nvsWr);
              if (_nr > (int)PULL_OTA_CHUNK_SIZE) _nr = (int)PULL_OTA_CHUNK_SIZE;
              int _n = _nvsRaw.read(s_otaChunkBuf, _nr);
              if (_n <= 0) { _nvsOk = false; break; }
              if ((size_t)_nvsFile.write(s_otaChunkBuf, (size_t)_n) != (size_t)_n) { _nvsOk = false; break; }
              _nvsWr += (size_t)_n;
            }
            _nvsFile.close();
            if (!_nvsOk || _nvsWr != _nvsExp) {
              Serial.println("[OTA-PULL] NVS download incomplete Ã¢â‚¬â€ settings unchanged");
              SD.remove(OTA_NVS_PATH);
            } else {
              Serial.printf("[OTA-PULL] NVS staged: %u bytes\n", (unsigned)_nvsWr);
            }
          }
        } else {
          Serial.printf("[OTA-PULL] NVS HTTP %d Ã¢â‚¬â€ settings unchanged\n", _nvsHC);
        }
        otaWifiClient.close();
      } else {
        Serial.println("[OTA-PULL] NVS connect/send failed Ã¢â‚¬â€ settings unchanged");
        otaWifiClient.close();
      }
#endif  // ENABLE_WIFI
    }
    // Only restart if NVS was successfully staged on SD.  If the download
    // failed leave the device running and let it retry on the next interval.
    if (!SD.exists(OTA_NVS_PATH)) {
      Serial.println("[OTA-PULL] NVS-only: download failed, will retry on next check");
      return false;
    }
    if (_applyNvsFromSD()) {
      Serial.println("[OTA-PULL] Settings (NVS) applied Ã¢â‚¬â€ restarting");
      esp_restart();
      return false;  // unreachable
    } else {
      // Apply failed (e.g. RAM allocation error, SD read error, flash write
      // error).  The NVS partition is unchanged; leave the device running so
      // it retries the download + apply on the next OTA check interval.
      Serial.println("[OTA-PULL] Settings (NVS) apply failed Ã¢â‚¬â€ will retry on next check");
      return false;
    }
  }
#endif  // STORAGE == STORAGE_SD
  if (nvsOnly) {
    // No SD storage available Ã¢â‚¬â€ cannot download NVS binary; skip silently.
    Serial.println("[OTA-PULL] NVS-only update skipped (no SD storage)");
    return false;
  }

  // Parse firmware "size" field.
  size_t fwSize = 0;
  char* sizeField = strstr(metaBody, "\"size\":");
  if (!sizeField) {
    Serial.println("[OTA-PULL] META: missing size field");
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=META_NOSIZE");
#endif
    return false;
  }
  fwSize = (size_t)atol(sizeField + 7);
  if (fwSize < PULL_OTA_MIN_FW_SIZE) { // sanity check: firmware must be at least 64 KB
    Serial.printf("[OTA-PULL] META: implausible size %u\n", (unsigned)fwSize);
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=META_SIZE");
#endif
    return false;
  }

  Serial.printf("[OTA-PULL] Update available: %u bytes\n", (unsigned)fwSize);
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    char _ota_diag[48];
    snprintf(_ota_diag, sizeof(_ota_diag), "OTA-PULL START SIZE=%u", (unsigned)fwSize);
    logger.logEvent(_ota_diag);
  }
#endif

  // Parse optional sha256 field for post-download integrity verification.
  // When present, the downloaded firmware file is checked against this digest
  // immediately after the download loop Ã¢â‚¬â€ before the companion meta file is
  // written.  A mismatch (e.g. due to Transfer-Encoding: chunked manglings
  // or cellular bit-errors) is caught here and the staging file is removed so
  // the next OTA interval triggers a clean retry.  Missing field Ã¢â€ â€™ skip check.
  char fwSha256Hex[65] = "";
  {
    char* sf = strstr(metaBody, "\"sha256\":");
    if (sf) {
      char* s = strchr(sf + 9, '"');
      if (s) {
        s++;
        char* e = strchr(s, '"');
        if (e && (size_t)(e - s) == 64) {
          memcpy(fwSha256Hex, s, 64);
          fwSha256Hex[64] = '\0';
          // Validate: all 64 chars must be valid hex [0-9a-fA-F] (upper or lower).
          // Non-hex content (e.g. malformed meta) is treated as absent (skip verify).
          for (int _i = 0; _i < 64; _i++) {
            char _c = fwSha256Hex[_i];
            if (!((_c >= '0' && _c <= '9') || (_c >= 'a' && _c <= 'f') ||
                  (_c >= 'A' && _c <= 'F'))) {
              fwSha256Hex[0] = '\0';
              break;
            }
          }
        }
      }
    }
  }

  // ---- Step 2: Download firmware ------------------------------------------
  char fwPath[384];
  snprintf(fwPath, sizeof(fwPath),
           "/api/freematics/ota_pull/%s/firmware.bin", otaToken);

#if STORAGE == STORAGE_SD
  // --- SD-staging path (default): download to /ota_fw.bin, flash at standby -
  // This leaves active telemetry running; no data is lost.
  if (state.check(STATE_STORAGE_READY)) {
    // a. Clean up any leftover files.
    if (SD.exists(OTA_PENDING_PATH)) SD.remove(OTA_PENDING_PATH);
    if (SD.exists(OTA_META_PATH)) SD.remove(OTA_META_PATH);

    File fwFile = SD.open(OTA_PENDING_PATH, FILE_WRITE);
    if (!fwFile) {
      Serial.println("[OTA-PULL] Cannot create SD staging file");
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_OPEN");
#endif
      return false;
    }

    size_t written = 0;
    uint32_t dlStart = millis();
    size_t lastLogAt = 0;
    bool dlOk = true;

    // Streaming SHA256 context Ã¢â‚¬â€ updated with every chunk written to SD.
    // Verification happens after the download loop and before the meta file
    // is written, so a corrupted download is detected without any flash attempt.
    mbedtls_sha256_context sha256Ctx;
    bool doSha256 = (fwSha256Hex[0] != '\0');
    if (doSha256) {
      mbedtls_sha256_init(&sha256Ctx);
      mbedtls_sha256_starts_ret(&sha256Ctx, 0);
    }


#if ENABLE_WIFI
    if (!otaWifiClient.open(otaHost, otaPort)) {
      Serial.printf("[OTA-PULL] FW connect failed to %s:%u\n", _maskOtaHost(otaHost).c_str(), (unsigned)otaPort);
      fwFile.close();
      SD.remove(OTA_PENDING_PATH);
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FW_CONNECT");
#endif
      return false;
    }

    if (!otaWifiClient.send(METHOD_GET, fwPath)) {
      Serial.println("[OTA-PULL] FW send failed");
      otaWifiClient.close();
      fwFile.close();
      SD.remove(OTA_PENDING_PATH);
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FW_SEND");
#endif
      return false;
    }

    int contentLength = 0;
    int httpCode = otaWifiClient.receiveHeaders(&contentLength);
    if (httpCode != 200) {
      Serial.printf("[OTA-PULL] FW HTTP %d\n", httpCode);
      otaWifiClient.close();
      fwFile.close();
      SD.remove(OTA_PENDING_PATH);
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) {
        char _ota_diag[48];
        snprintf(_ota_diag, sizeof(_ota_diag), "OTA-PULL ERR=FW_HTTP%d", httpCode);
        logger.logEvent(_ota_diag);
      }
#endif
      return false;
    }
    if (contentLength > 0 && (size_t)contentLength != fwSize) {
      Serial.printf("[OTA-PULL] FW size mismatch: meta=%u header=%d\n",
                    (unsigned)fwSize, contentLength);
      fwSize = (size_t)contentLength;
    }

    WiFiClientSecure& rawSock = otaWifiClient.rawClient();

    while (written < fwSize) {
      uint32_t chunkStart = millis();
      while (!rawSock.available() && millis() - chunkStart < PULL_OTA_CHUNK_TIMEOUT_MS) delay(1);
      if (!rawSock.available()) {
        Serial.printf("[OTA-PULL] Recv timeout at offset %u\n", (unsigned)written);
#if STORAGE != STORAGE_NONE
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=RECV_TIMEOUT");
#endif
        dlOk = false;
        break;
      }
      int toRead = (int)(fwSize - written);
      if (toRead > (int)PULL_OTA_CHUNK_SIZE) toRead = (int)PULL_OTA_CHUNK_SIZE;
      int n = rawSock.read(s_otaChunkBuf, toRead);
      if (n <= 0) {
        Serial.printf("[OTA-PULL] Read error at offset %u\n", (unsigned)written);
#if STORAGE != STORAGE_NONE
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=RECV_READ");
#endif
        dlOk = false;
        break;
      }
      size_t nw = fwFile.write(s_otaChunkBuf, (size_t)n);
      if (nw != (size_t)n) {
        // First write attempt failed or was incomplete; retry once.
        Serial.printf("[OTA-PULL] SD write retry at offset %u (got %u/%u)\n",
                      (unsigned)written, (unsigned)nw, (unsigned)n);
        delay(50);
        if (nw < (size_t)n) {
          size_t nw2 = fwFile.write(s_otaChunkBuf + nw, (size_t)n - nw);
          nw += nw2;
        }
        if (nw != (size_t)n) {
          Serial.printf("[OTA-PULL] SD write error at offset %u\n", (unsigned)written);
#if STORAGE != STORAGE_NONE
          if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_WRITE");
#endif
          dlOk = false;
          break;
        }
        Serial.printf("[OTA-PULL] SD write retry succeeded at offset %u\n", (unsigned)written);
      }
      written += (size_t)n;
      if (doSha256) mbedtls_sha256_update_ret(&sha256Ctx, s_otaChunkBuf, (size_t)n);
      if (written - lastLogAt >= (fwSize / 10 ? fwSize / 10 : 1)) {
        lastLogAt = written;
        Serial.printf("[OTA-PULL] %u / %u bytes (%.0f%%) in %u ms\n",
                      (unsigned)written, (unsigned)fwSize,
                      100.0f * written / fwSize, (unsigned)(millis() - dlStart));
      }
    }
    fwFile.close();
    otaWifiClient.close();
#endif  // ENABLE_WIFI

    if (!dlOk || written != fwSize) {
      Serial.printf("[OTA-PULL] Download incomplete (%u / %u bytes) Ã¢â‚¬â€ staging file removed\n",
                    (unsigned)written, (unsigned)fwSize);
      SD.remove(OTA_PENDING_PATH);
      if (doSha256) mbedtls_sha256_free(&sha256Ctx);
      return false;
    }

    // Verify SHA256 of the downloaded firmware against the expected digest from
    // meta.json.  Done BEFORE writing the companion meta file so that a corrupt
    // download (e.g. chunked-encoding artefacts from the reverse-proxy, or a
    // partial transfer) is detected immediately and the staging file is removed
    // Ã¢â‚¬â€ the next OTA check interval will retry with a clean download.
    if (doSha256) {
      static const int SHA256_DIGEST_BYTES = 32;  // SHA-256 produces a 256-bit (32-byte) digest
      uint8_t digest[SHA256_DIGEST_BYTES];
      mbedtls_sha256_finish_ret(&sha256Ctx, digest);
      mbedtls_sha256_free(&sha256Ctx);
      char actualHex[SHA256_DIGEST_BYTES * 2 + 1];
      for (int i = 0; i < SHA256_DIGEST_BYTES; i++) snprintf(actualHex + i * 2, 3, "%02x", digest[i]);
      actualHex[SHA256_DIGEST_BYTES * 2] = '\0';
      if (strncmp(actualHex, fwSha256Hex, SHA256_DIGEST_BYTES * 2) != 0) {
        Serial.printf("[OTA-PULL] SHA256 mismatch Ã¢â‚¬â€ staging file removed (retry at next interval)\n"
                      "[OTA-PULL]   expected: %s\n"
                      "[OTA-PULL]   actual:   %s\n",
                      fwSha256Hex, actualHex);
#if STORAGE != STORAGE_NONE
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SHA256");
#endif
        SD.remove(OTA_PENDING_PATH);
        return false;
      }
      Serial.println("[OTA-PULL] SHA256 OK");
    }

    // Notify HA that the firmware was downloaded and verified on the device.
    // This is a WiFi-only, best-effort call Ã¢â‚¬â€ OTA proceeds even if it fails.
    // The HA server only updates "OTA letzte ÃƒÅ“bertragung" upon receiving this
    // request, ensuring the status reflects a device-confirmed download rather
    // than merely a completed server-side transmission.
#if ENABLE_WIFI
    // After streaming a large firmware binary over TLS, the mbedTLS heap is
    // often fragmented below TLS_MIN_FREE_HEAP (~34 KB max block after close).
    // Restart WiFi to coalesce freed TLS/TCP buffers so the confirm request
    // and any subsequent NVS download can open a fresh TLS session.
    if (ESP.getMaxAllocHeap() < TLS_MIN_FREE_HEAP) {
      Serial.printf("[OTA-PULL] Heap fragmented (%u bytes), restarting WiFi\n",
                    (unsigned)ESP.getMaxAllocHeap());
      otaWifiClient.end();
      wifiReconnectCurrent();
      if (!otaWifiClient.setup(WIFI_JOIN_TIMEOUT)) {
        Serial.println("[OTA-PULL] WiFi reconnect timeout after heap recovery");
      }
    }
    {
      char _confirmPath[128];
      snprintf(_confirmPath, sizeof(_confirmPath),
               "/api/freematics/ota_pull/%s/ota_confirm", otaToken);
      if (otaWifiClient.open(otaHost, otaPort) &&
          otaWifiClient.send(METHOD_GET, _confirmPath)) {
        int _confirmCL = 0;
        int _confirmCode = otaWifiClient.receiveHeaders(&_confirmCL);
        Serial.printf("[OTA-PULL] Confirm %s (HTTP %d)\n",
                      _confirmCode == 200 ? "OK" : "FAILED", _confirmCode);
      } else {
        Serial.println("[OTA-PULL] Confirm request failed (non-fatal)");
      }
      otaWifiClient.close();
    }
#endif

    // Write companion meta file: expected byte count for integrity check.
    {
      File metaFile = SD.open(OTA_META_PATH, FILE_WRITE);
      if (metaFile) {
        char metaBufOut[16];
        snprintf(metaBufOut, sizeof(metaBufOut), "%u\n", (unsigned)fwSize);
        metaFile.print(metaBufOut);
        metaFile.close();
      }
    }

    // ---- Step 3 (optional): Download NVS settings binary -------------------
    if (SD.exists(OTA_NVS_PATH)) SD.remove(OTA_NVS_PATH);
    if (nvsPath[0]) {
      Serial.printf("[OTA-PULL] Downloading NVS settings from %s\n", nvsPath);
#if ENABLE_WIFI
      if (otaWifiClient.open(otaHost, otaPort) &&
          otaWifiClient.send(METHOD_GET, nvsPath)) {
        int nvsContentLen = 0;
        int nvsHttpCode = otaWifiClient.receiveHeaders(&nvsContentLen);
        if (nvsHttpCode == 200 && nvsContentLen > 0) {
          File nvsFile = SD.open(OTA_NVS_PATH, FILE_WRITE);
          if (nvsFile) {
            WiFiClientSecure& rawSock2 = otaWifiClient.rawClient();
            size_t nvsWritten = 0;
            size_t nvsExpected = (size_t)nvsContentLen;
            bool nvsOk = true;
            while (nvsWritten < nvsExpected) {
              uint32_t t1 = millis();
              while (!rawSock2.available() && millis() - t1 < PULL_OTA_CHUNK_TIMEOUT_MS) delay(1);
              if (!rawSock2.available()) { nvsOk = false; break; }
              int toRead = (int)(nvsExpected - nvsWritten);
              if (toRead > (int)PULL_OTA_CHUNK_SIZE) toRead = (int)PULL_OTA_CHUNK_SIZE;
              int nr = rawSock2.read(s_otaChunkBuf, toRead);
              if (nr <= 0) { nvsOk = false; break; }
              if ((size_t)nvsFile.write(s_otaChunkBuf, (size_t)nr) != (size_t)nr) { nvsOk = false; break; }
              nvsWritten += (size_t)nr;
            }
            nvsFile.close();
            if (nvsOk && nvsWritten == nvsExpected) {
              Serial.printf("[OTA-PULL] NVS staged: %u bytes\n", (unsigned)nvsWritten);
#if STORAGE != STORAGE_NONE
              if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL NVS DL OK");
#endif
            } else {
              Serial.println("[OTA-PULL] NVS download incomplete Ã¢â‚¬â€ skipping settings update");
              SD.remove(OTA_NVS_PATH);
            }
          }
        } else {
          Serial.printf("[OTA-PULL] NVS HTTP %d Ã¢â‚¬â€ skipping settings update\n", nvsHttpCode);
        }
        otaWifiClient.close();
      } else {
        Serial.println("[OTA-PULL] NVS connect/send failed Ã¢â‚¬â€ skipping settings update");
        otaWifiClient.close();
      }
#endif  // ENABLE_WIFI
    }  // nvsPath[0]

    s_ota_pending = true;
    Serial.printf("[OTA-PULL] Download complete: %u bytes in %u ms\n"
                  "[OTA-PULL] Firmware staged on SD (%s) Ã¢â‚¬â€ restarting to flash\n",
                  (unsigned)written, (unsigned)(millis() - dlStart), OTA_PENDING_PATH);
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) {
      char _ota_diag[64];
      snprintf(_ota_diag, sizeof(_ota_diag), "OTA-PULL DL OK SIZE=%u", (unsigned)fwSize);
      logger.logEvent(_ota_diag);
    }
#endif
    // Restart immediately so the device boots with a clean heap.  The boot-time
    // staging check (setup(), just after SD init) will detect /ota_fw.bin and
    // /ota_meta.txt and call performPullOtaFlash() before any TLS session is
    // opened, ensuring the flash and optional NVS update succeed reliably.
    // This also resolves the "Low heap" WiFi/LTE failure loop that would
    // otherwise block all network connections until the next natural standby.
    esp_restart();
    return false;  // unreachable: esp_restart() does not return
  }
#endif  // STORAGE == STORAGE_SD

  // ---- Fallback: stream directly to flash (STORAGE_NONE / STORAGE_SPIFFS) --
#if ENABLE_WIFI
  // Signal the telemetry task to pause WiFi I/O before Update.begin().
  s_ota_active = true;
  delay(1500);

  if (!otaWifiClient.open(otaHost, otaPort)) {
    Serial.printf("[OTA-PULL] FW connect failed to %s:%u\n", _maskOtaHost(otaHost).c_str(), (unsigned)otaPort);
    s_ota_active = false;
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FW_CONNECT");
#endif
    return false;
  }

  if (!otaWifiClient.send(METHOD_GET, fwPath)) {
    Serial.println("[OTA-PULL] FW send failed");
    otaWifiClient.close();
    s_ota_active = false;
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FW_SEND");
#endif
    return false;
  }

  int contentLength = 0;
  int httpCode = otaWifiClient.receiveHeaders(&contentLength);
  if (httpCode != 200) {
    Serial.printf("[OTA-PULL] FW HTTP %d\n", httpCode);
    otaWifiClient.close();
    s_ota_active = false;
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) {
      char _ota_diag[48];
      snprintf(_ota_diag, sizeof(_ota_diag), "OTA-PULL ERR=FW_HTTP%d", httpCode);
      logger.logEvent(_ota_diag);
    }
#endif
    return false;
  }
  if (contentLength > 0 && (size_t)contentLength != fwSize) {
    Serial.printf("[OTA-PULL] FW size mismatch: meta=%u header=%d\n",
                  (unsigned)fwSize, contentLength);
    fwSize = (size_t)contentLength;
  }

  if (!Update.begin(fwSize)) {
    Serial.printf("[OTA-PULL] Update.begin failed: %s\n", Update.errorString());
    otaWifiClient.close();
    s_ota_active = false;
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=UPD_BEGIN");
#endif
    return false;
  }

  WiFiClientSecure& rawSock = otaWifiClient.rawClient();
  size_t written = 0;
  uint32_t dlStart = millis();
  size_t lastLogAt = 0;

  while (written < fwSize) {
    uint32_t chunkStart = millis();
    while (!rawSock.available() && millis() - chunkStart < PULL_OTA_CHUNK_TIMEOUT_MS) delay(1);
    if (!rawSock.available()) {
      Serial.printf("[OTA-PULL] Recv timeout at offset %u\n", (unsigned)written);
      Update.abort();
      otaWifiClient.close();
      s_ota_active = false;
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=RECV_TIMEOUT");
#endif
      return false;
    }

    int toRead = (int)(fwSize - written);
    if (toRead > (int)PULL_OTA_CHUNK_SIZE) toRead = (int)PULL_OTA_CHUNK_SIZE;
    int n = rawSock.read(s_otaChunkBuf, toRead);
    if (n <= 0) {
      Serial.printf("[OTA-PULL] Read error at offset %u\n", (unsigned)written);
      Update.abort();
      otaWifiClient.close();
      s_ota_active = false;
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=RECV_READ");
#endif
      return false;
    }

    size_t w = Update.write(s_otaChunkBuf, (size_t)n);
    if (w != (size_t)n) {
      Serial.printf("[OTA-PULL] Flash write error at offset %u\n", (unsigned)written);
      Update.abort();
      otaWifiClient.close();
      s_ota_active = false;
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FLASH_WRITE");
#endif
      return false;
    }
    written += (size_t)n;

    if (written - lastLogAt >= (fwSize / 10 ? fwSize / 10 : 1)) {
      lastLogAt = written;
      Serial.printf("[OTA-PULL] %u / %u bytes (%.0f%%) in %u ms\n",
                    (unsigned)written, (unsigned)fwSize,
                    100.0f * written / fwSize, (unsigned)(millis() - dlStart));
    }
  }

  otaWifiClient.close();
  Serial.printf("[OTA-PULL] Download complete: %u bytes in %u ms\n",
                (unsigned)written, (unsigned)(millis() - dlStart));

  if (!Update.end()) {
    Serial.printf("[OTA-PULL] Update.end failed: %s\n", Update.errorString());
    s_ota_active = false;
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=UPD_END");
#endif
    return false;
  }

  Serial.println("[OTA-PULL] Flash successful, rebooting in 1.5 s");
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    char _ota_diag[64];
    snprintf(_ota_diag, sizeof(_ota_diag),
             "OTA-PULL OK FW=%s SIZE=%u", FIRMWARE_VERSION, (unsigned)written);
    logger.logEvent(_ota_diag);
  }
#endif
  // Notify HA that firmware was written to flash (best-effort, WiFi-only).
  // The HA server only updates "OTA letzte ÃƒÅ“bertragung" upon receiving this
  // request, so the attribute accurately reflects a device-confirmed flash.
  {
    char _confirmPath[128];
    snprintf(_confirmPath, sizeof(_confirmPath),
             "/api/freematics/ota_pull/%s/ota_confirm", otaToken);
    if (otaWifiClient.open(otaHost, otaPort) &&
        otaWifiClient.send(METHOD_GET, _confirmPath)) {
      int _confirmCL = 0;
      int _confirmCode = otaWifiClient.receiveHeaders(&_confirmCL);
      Serial.printf("[OTA-PULL] Confirm %s (HTTP %d)\n",
                    _confirmCode == 200 ? "OK" : "FAILED", _confirmCode);
    } else {
      Serial.println("[OTA-PULL] Confirm request failed (non-fatal)");
    }
    otaWifiClient.close();
  }
  // s_ota_active remains true; device reboots shortly.
  static esp_timer_handle_t s_pull_ota_timer = NULL;
  if (!s_pull_ota_timer) {
    esp_timer_create_args_t args = {};
    args.callback        = [](void*) { esp_restart(); };
    args.dispatch_method = ESP_TIMER_TASK;
    args.name            = "pull_ota_restart";
    esp_timer_create(&args, &s_pull_ota_timer);
  } else {
    esp_timer_stop(s_pull_ota_timer);
  }
  esp_timer_start_once(s_pull_ota_timer, 1500000);
  return true;
#endif  // ENABLE_WIFI (inner - line ~4445, the STORAGE_NONE/SPIFFS direct-flash fallback)
  return false;
}
#else   // !ENABLE_WIFI (outer - whole function; see #if ENABLE_WIFI near performPullOtaCheck's top)
bool performPullOtaCheck()
{
  return false;  // Pull-OTA requires WiFi (ENABLE_WIFI) - no longer tied to SERVER_PROTOCOL.
}
#endif  // ENABLE_WIFI (outer)

void processBLE(int timeout)
{
#if ENABLE_BLE
  if (!enableBle) {
    if (timeout) delay(timeout);
    return;
  }
  static byte echo = 0;
  char* cmd;
  if (!(cmd = ble_recv_command(timeout))) {
    return;
  }

  char *p = strchr(cmd, '\r');
  if (p) *p = 0;
  char buf[48];
  int bufsize = sizeof(buf);
  int n = 0;
  if (echo) n += snprintf(buf + n, bufsize - n, "%s\r", cmd);
  Serial.print("[BLE] ");
  Serial.print(cmd);
  if (!strcmp(cmd, "UPTIME") || !strcmp(cmd, "TICK")) {
    n += snprintf(buf + n, bufsize - n, "%lu", millis());
  } else if (!strcmp(cmd, "BATT")) {
    n += snprintf(buf + n, bufsize - n, "%.2f", (float)(analogRead(A0) * 42) / 4095);
  } else if (!strcmp(cmd, "RESET")) {
#if STORAGE
    logger.end();
#endif
    ESP.restart();
    // never reach here
  } else if (!strcmp(cmd, "OFF")) {
    state.set(STATE_STANDBY);
    state.clear(STATE_WORKING);
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ON")) {
    state.clear(STATE_STANDBY);
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ON?")) {
    n += snprintf(buf + n, bufsize - n, "%u", state.check(STATE_STANDBY) ? 0 : 1);
  } else if (!strcmp(cmd, "APN?")) {
    n += snprintf(buf + n, bufsize - n, "%s", *apn ? apn : "DEFAULT");
  } else if (!strncmp(cmd, "APN=", 4)) {
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "CELL_APN", strcmp(cmd + 4, "DEFAULT") ? cmd + 4 : "") == ESP_OK
        && nvs_commit(nvs) == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strcmp(cmd, "PIN?")) {
    n += snprintf(buf + n, bufsize - n, "%s", *simPin ? "SET" : "NONE");
  } else if (!strncmp(cmd, "PIN=", 4)) {
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "SIM_PIN", strcmp(cmd + 4, "CLEAR") ? cmd + 4 : "") == ESP_OK
        && nvs_commit(nvs) == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strcmp(cmd, "NET_OP")) {
    if (state.check(STATE_WIFI_CONNECTED)) {
#if ENABLE_WIFI
      n += snprintf(buf + n, bufsize - n, "%s", wifiSSID[0] ? wifiSSID : "-");
#endif
    } else {
      snprintf(buf + n, bufsize - n, "%s", netop.length() ? netop.c_str() : "-");
      char *p = strchr(buf + n, ' ');
      if (p) *p = 0;
      n += strlen(buf + n);
    }
  } else if (!strcmp(cmd, "NET_IP")) {
    n += snprintf(buf + n, bufsize - n, "%s", ip.length() ? ip.c_str() : "-");
  } else if (!strcmp(cmd, "NET_PACKET")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.txCount);
  } else if (!strcmp(cmd, "NET_DATA")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.txBytes);
  } else if (!strcmp(cmd, "NET_RATE")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.startTime ? (unsigned int)((uint64_t)(teleClient.txBytes + teleClient.rxBytes) * 3600 / (millis() - teleClient.startTime)) : 0);
  } else if (!strcmp(cmd, "RSSI")) {
    n += snprintf(buf + n, bufsize - n, "%d", rssi);
#if ENABLE_WIFI
  } else if (!strcmp(cmd, "SSID?")) {
    n += snprintf(buf + n, bufsize - n, "%s", wifiSSID[0] ? wifiSSID : "-");
  } else if (!strncmp(cmd, "SSID=", 5)) {
    const char* p = cmd + 5;
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "WIFI_SSID", strcmp(p, "-") ? p : "") == ESP_OK
        && nvs_commit(nvs) == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strcmp(cmd, "WPWD?")) {
    n += snprintf(buf + n, bufsize - n, "%s", wifiPassword[0] ? wifiPassword : "-");
  } else if (!strncmp(cmd, "WPWD=", 5)) {
    const char* p = cmd + 5;
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "WIFI_PWD", strcmp(p, "-") ? p : "") == ESP_OK
        && nvs_commit(nvs) == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strncmp(cmd, "OTA_TOKEN=", 10)) {
    // Provision (or clear) the pull-OTA authentication token in NVS.
    // Use "-" or empty string to clear the token (disables OTA checks).
    // Also updates the runtime variable so the next OTA check fires without
    // a reboot (assuming OTA_INTERVAL is also set and otaHost is reachable).
    const char* p = cmd + 10;
    const bool clr = (p[0] == '\0' || (p[0] == '-' && p[1] == '\0'));
    esp_err_t e = nvs_set_str(nvs, "OTA_TOKEN", clr ? "" : p);
    if (e == ESP_OK) e = nvs_commit(nvs);
    if (e == ESP_OK) {
      size_t tlen = sizeof(otaToken);
      otaToken[0] = 0;
      nvs_get_str(nvs, "OTA_TOKEN", otaToken, &tlen);
      // Ensure otaHost is set so performPullOtaCheck() can open a connection.
      // If no dedicated OTA_HOST key is stored, fall back to serverHost.
      if (!otaHost[0] && otaToken[0]) {
        strncpy(otaHost, serverHost, sizeof(otaHost) - 1);
        otaHost[sizeof(otaHost) - 1] = 0;
      }
    }
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
  } else if (!strncmp(cmd, "OTA_HOST=", 9)) {
    // Set the hostname (or IP) of the HA server that serves the pull-OTA
    // endpoint.  Stored in NVS as OTA_HOST and applied at runtime.
    // Use "-" to clear (firmware then falls back to serverHost).
    const char* p = cmd + 9;
    const bool clr = (p[0] == '-' && p[1] == '\0');
    esp_err_t e = nvs_set_str(nvs, "OTA_HOST", clr ? "" : p);
    if (e == ESP_OK) e = nvs_commit(nvs);
    if (e == ESP_OK) {
      size_t hlen = sizeof(otaHost);
      otaHost[0] = 0;
      nvs_get_str(nvs, "OTA_HOST", otaHost, &hlen);
      // Same fallback as OTA_TOKEN handler: use serverHost when OTA_HOST is cleared.
      if (!otaHost[0] && otaToken[0]) {
        strncpy(otaHost, serverHost, sizeof(otaHost) - 1);
        otaHost[sizeof(otaHost) - 1] = 0;
      }
    }
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
  } else if (!strncmp(cmd, "OTA_INTERVAL=", 13)) {
    // Set the pull-OTA check interval in seconds (0 = disable).
    // Applied at runtime so the next check fires after this interval,
    // without requiring a reboot.
    uint16_t interval = (uint16_t)atoi(cmd + 13);
    esp_err_t e = nvs_set_u16(nvs, "OTA_INTERVAL", interval);
    if (e == ESP_OK) e = nvs_commit(nvs);
    if (e == ESP_OK) otaCheckIntervalS = interval;
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
#else
  } else if (!strcmp(cmd, "SSID?") || !strcmp(cmd, "WPWD?")) {
    n += snprintf(buf + n, bufsize - n, "-");
#endif
#if ENABLE_MEMS
  } else if (!strcmp(cmd, "TEMP")) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)deviceTemp);
  } else if (!strcmp(cmd, "ACC")) {
    n += snprintf(buf + n, bufsize - n, "%.1f/%.1f/%.1f", acc[0], acc[1], acc[2]);
  } else if (!strcmp(cmd, "GYRO")) {
    n += snprintf(buf + n, bufsize - n, "%.1f/%.1f/%.1f", gyr[0], gyr[1], gyr[2]);
  } else if (!strcmp(cmd, "GF")) {
    n += snprintf(buf + n, bufsize - n, "%f", (float)sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]));
#endif
  } else if (!strcmp(cmd, "ATE0")) {
    echo = 0;
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ATE1")) {
    echo = 1;
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "FS")) {
    n += snprintf(buf + n, bufsize - n, "%u",
#if STORAGE == STORAGE_NONE
    0
#else
    logger.size()
#endif
      );
  } else if (!memcmp(cmd, "01", 2)) {
    byte pid = hex2uint8(cmd + 2);
    for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
      if (obdData[i].pid == pid) {
        n += snprintf(buf + n, bufsize - n, "%d", obdData[i].value);
        pid = 0;
        break;
      }
    }
    if (pid) {
      int value;
      if (obd.readPID(pid, value)) {
        n += snprintf(buf + n, bufsize - n, "%d", value);
      } else {
        n += snprintf(buf + n, bufsize - n, "N/A");
      }
    }
  } else if (!strcmp(cmd, "VIN")) {
    n += snprintf(buf + n, bufsize - n, "%s", vin[0] ? vin : "N/A");
  } else if (!strcmp(cmd, "LAT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%f", gd->lat);
  } else if (!strcmp(cmd, "LNG") && gd) {
    n += snprintf(buf + n, bufsize - n, "%f", gd->lng);
  } else if (!strcmp(cmd, "ALT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)gd->alt);
  } else if (!strcmp(cmd, "SAT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%u", (unsigned int)gd->sat);
  } else if (!strcmp(cmd, "SPD") && gd) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)(gd->speed * 1852 / 1000));
  } else if (!strcmp(cmd, "CRS") && gd) {
    n += snprintf(buf + n, bufsize - n, "%u", (unsigned int)gd->heading);
  } else if (!strncmp(cmd, "LED_WHITE=", 10)) {
    // Enable (1) or disable (0) the white/network LED at runtime and in NVS.
    uint8_t v = (uint8_t)atoi(cmd + 10);
    enableLedWhite = (v != 0);
    esp_err_t e = nvs_set_u8(nvs, "LED_WHITE_EN", v);
    if (e == ESP_OK) e = nvs_commit(nvs);
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
  } else if (!strncmp(cmd, "LED_RED=", 8)) {
    // Enable (1) or disable (0) the red/power LED at runtime and in NVS.
    uint8_t v = (uint8_t)atoi(cmd + 8);
    enableLedRed = (v != 0);
    esp_err_t e = nvs_set_u8(nvs, "LED_RED_EN", v);
    if (e == ESP_OK) e = nvs_commit(nvs);
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
  } else if (!strncmp(cmd, "BEEP=", 5)) {
    // Enable (1) or disable (0) the connection beep at runtime and in NVS.
    uint8_t v = (uint8_t)atoi(cmd + 5);
    enableBeep = (v != 0);
    esp_err_t e = nvs_set_u8(nvs, "BEEP_EN", v);
    if (e == ESP_OK) e = nvs_commit(nvs);
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
  } else {
    n += snprintf(buf + n, bufsize - n, "ERROR");
  }
  Serial.print(" -> ");
  Serial.println((p = strchr(buf, '\r')) ? p + 1 : buf);
  if (n < bufsize - 1) {
    buf[n++] = '\r';
  } else {
    n = bufsize - 1;
  }
  buf[n] = 0;
  ble_send_response(buf, n, cmd);
#else
  if (timeout) delay(timeout);
#endif
}

// ---------------------------------------------------------------------------
// printOtaStatus()
//
// Prints the current pull-OTA configuration to the serial console.  Called
// at boot from setup() and after any live OTA config update via the HTTP
// control API (OTA_TOKEN=, OTA_HOST=, OTA_INTERVAL= commands) so users see
// the current OTA state immediately without needing to reboot.
//
// Outputs one of:
//   OTA:disabled                                    Ã¢â‚¬â€œ OTA_TOKEN not in NVS
//   OTA:TOKEN=38f90170... HOST=Ã¢â‚¬Â¦ PORT=Ã¢â‚¬Â¦ INTERVAL=Xs Ã¢â‚¬â€œ fully active
//   OTA:TOKEN=38f90170... HOST=Ã¢â‚¬Â¦ PORT=Ã¢â‚¬Â¦ INTERVAL=0s (checks disabled)
//                                                   Ã¢â‚¬â€œ token set, INTERVAL=0
// The first 8 hex characters of the token are shown so it is recognisable
// in the serial log without exposing the full 64-character secret.
// ---------------------------------------------------------------------------
void printOtaStatus()
{
  if (otaToken[0]) {
    // Show only the first 8 hex chars of the token so the log entry is
    // unambiguous (no angle-bracket confusion) while keeping the secret safe.
    char _tok8[9];
    strncpy(_tok8, otaToken, 8);
    _tok8[8] = '\0';
    Serial.printf("OTA:TOKEN=%s... HOST=%s PORT=%u INTERVAL=%us%s\n",
                  _tok8,
                  otaHost[0] ? _maskOtaHost(otaHost).c_str() : "(server fallback)",
                  (unsigned)otaPort,
                  (unsigned)otaCheckIntervalS,
                  otaCheckIntervalS == 0 ? " (checks disabled)" : "");
  } else {
    Serial.println("OTA:disabled");
  }
}

// ---------------------------------------------------------------------------
// installMbedtlsPsramAllocator()
//
// Why this exists: the precompiled mbedTLS static libs shipped with the
// pinned `platform = espressif32 @ 6.5.0` (arduino-esp32 core 2.0.17) were
// built with CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC=1 baked into their
// sdkconfig.h (confirmed by reading
// tools/sdk/esp32/qio_qspi/include/sdkconfig.h in the local PlatformIO
// package cache - line 512 as of this writing). That forces every one of
// mbedTLS's own internal allocations - most importantly each TLS session's
// RX/TX record buffers, CONFIG_MBEDTLS_SSL_MAX_CONTENT_LEN=16384 bytes
// EACH direction, so ~32KB+ per active TLS session - onto the ~213KB
// internal DRAM heap, even though this board has 8MB of PSRAM
// (CONFIG_ESP32_SPIRAM_SUPPORT / CONFIG_SPIRAM_USE_MALLOC are both already
// enabled in the same sdkconfig.h, and plain malloc() over 4096 bytes
// already redirects to PSRAM automatically via
// CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096 - mbedTLS just doesn't go
// through plain malloc() for this, it has its own dedicated allocator
// hook, which is what's pinned internal-only). Internal DRAM is also where
// BLE/HTTPD/MEMS/OBD/WiFi buffers live and fragment it over time, so by the
// time the OTA-pull path (performPullOtaCheck(), and WifiHTTP::open()'s
// Guard 2) or a telemetry reconnect tries to open a new WiFiClientSecure,
// ESP.getMaxAllocHeap() (largest contiguous free block) is often already
// below TLS_MIN_FREE_HEAP (38KB, see FreematicsNetwork.h) - logged at every
// call site that checks it as "Low heap (N bytes max block) ... skipping
// TLS connect" / "restarting WiFi".
//
// `framework = arduino` with a precompiled platform like this cannot
// override baked-in Kconfig values such as CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC
// via platformio.ini/build_flags/sdkconfig edits - a PlatformIO maintainer
// confirmed directly on the community forum that "changes to the sdkconfig
// will have no effect" for this setup. The fix below is not a workaround
// hack: mbedTLS exposes exactly one supported, documented runtime hook for
// this exact situation, mbedtls_platform_set_calloc_free() (declared in
// mbedtls/platform.h, confirmed present and callable in this build because
// MBEDTLS_PLATFORM_MEMORY is unconditionally defined and neither
// MBEDTLS_PLATFORM_CALLOC_MACRO nor _FREE_MACRO are defined in this port's
// esp_config.h - those two macros are the only thing that would compile out
// the runtime setter in favor of a compile-time-fixed pair). It is callable
// once at startup, before any TLS use, to swap mbedTLS's own calloc/free
// pair for a custom one - no rebuild of the precompiled libs required.
//
// Security tradeoff (intentionally not resolved here, flagging instead of
// asserting): Espressif's own components/mbedtls/Kconfig help text for
// CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC says the internal-only default exists
// for a SECURITY reason, not a reliability one - on plain ESP32, PSRAM
// contents are not hardware-encrypted the way flash can be with flash
// encryption enabled, so moving TLS session buffers into PSRAM could matter
// IF this device ever has flash/PSRAM encryption enabled. This repo was
// searched (config.h, NVS/partition-related code, comments) for any mention
// of flash encryption, PSRAM encryption, or secure boot and none was found
// either way - whether encryption is enabled on the physical device has NOT
// been investigated or confirmed here, one way or the other. If it turns
// out to be enabled, this tradeoff should be revisited.
//
// Only installs the hook if PSRAM is actually present (checked once here,
// at install time, not per-allocation) - on a board with no PSRAM this is a
// correct no-op and mbedTLS keeps using its normal internal-heap allocator.
// No new config flag/toggle is added: with PSRAM present (always true on
// this device) installing the hook is safe, and without PSRAM it's a no-op,
// so there's nothing meaningful for a runtime switch to disable.
//
// Must run before any TLS connection is ever attempted - i.e. before WiFi
// connects and before the telemetry task (which itself opens the first
// WiFiClientSecure) is created - which is why this is called from setup()
// immediately after Serial.begin(), early enough to log the outcome but
// before anything else in setup() touches WiFi/TLS.
// ---------------------------------------------------------------------------
static void *mbedtlsPsramCalloc(size_t n, size_t size)
{
  // heap_caps_calloc() zero-initializes like standard calloc() and handles
  // the n*size overflow check internally, matching calloc() semantics
  // exactly (mbedTLS relies on the zero-init guarantee in some places).
  void *p = heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM);
  if (!p) {
    // PSRAM exhausted (or this allocation doesn't fit) - fall back to the
    // normal internal-heap calloc() rather than returning NULL, which
    // mbedTLS would otherwise treat as a hard, unrecoverable allocation
    // failure.
    p = calloc(n, size);
  }
  return p;
}

static void mbedtlsPsramFree(void *ptr)
{
  // In IDF, free(p) is equivalent to heap_caps_free(p) - it frees memory
  // correctly regardless of which capability/region it was allocated from,
  // so this single free function is correct for both the PSRAM path and the
  // internal-calloc() fallback path above.
  heap_caps_free(ptr);
}

void installMbedtlsPsramAllocator()
{
  if (ESP.getPsramSize() == 0) {
    Serial.println("MBEDTLS:no PSRAM detected, using default internal-heap allocator");
    return;
  }
  int ret = mbedtls_platform_set_calloc_free(mbedtlsPsramCalloc, mbedtlsPsramFree);
  Serial.printf("MBEDTLS:PSRAM allocator %s (psram=%uKB)\n",
                ret == 0 ? "installed" : "FAILED",
                (unsigned)(ESP.getPsramSize() >> 10));
}

void setup()
{
  // Drive the LED pin LOW immediately so that the GPIO output register
  // retained from a previous run (HIGH when the device was in standby)
  // does not keep the red LED on for several hundred milliseconds while
  // NVS is being initialised and loadConfig() is called.  The correct
  // on/off state based on the NVS LED_RED_EN setting is applied further
  // below, after loadConfig() has run.
#ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
#endif

  delay(500);

  // Initialize NVS.  Only erase and reinitialize for the two errors that ESP-IDF
  // documents as requiring a full partition erase: no free pages (partition is
  // full) and new-version-found (written by a newer NVS implementation).
  // For all other errors the partition may still be partially readable; erasing
  // it would destroy provisioned WiFi/server credentials unnecessarily and
  // prevent WiFi from connecting on subsequent boots.
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    if (nvs_flash_erase() == ESP_OK) {
      err = nvs_flash_init();
    }
  }
  if (err == ESP_OK && nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK) {
    loadConfig();
  }

  // initialize USB serial
  Serial.begin(115200);

  // Redirect mbedTLS's internal TLS-session buffer allocations to PSRAM
  // before anything below can possibly touch WiFi/TLS (fixes the
  // TLS_MIN_FREE_HEAP low-heap failures in the OTA-pull and telemetry-
  // reconnect paths - see installMbedtlsPsramAllocator()'s comment above
  // for the full root-cause writeup). Must run this early: it needs to be
  // installed before the telemetry task is created further below, since
  // that task is the one that actually opens the first WiFiClientSecure.
  installMbedtlsPsramAllocator();

  // Set the LED to the state determined by the NVS LED_RED_EN setting loaded
  // above.  When LED_RED_EN=1 (default) the LED is driven HIGH as a visual
  // power-on / initialising indicator; when LED_RED_EN=0 it stays LOW.
#ifdef PIN_LED
  digitalWrite(PIN_LED, enableLedRed ? HIGH : LOW);
#endif

  // generate unique device ID
  genDeviceID(devid);

#if CONFIG_MODE_TIMEOUT
  configMode();
#endif

#if LOG_EXT_SENSORS == 1
  pinMode(PIN_SENSOR1, INPUT);
  pinMode(PIN_SENSOR2, INPUT);
#elif LOG_EXT_SENSORS == 2
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
#endif

  // show system information
  showSysInfo();

  bufman.init();

  //Serial.print(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >> 10);
  //Serial.println("KB");

  state.set(STATE_WORKING);

  // Create the telemetry (WiFi/cellular connect + transmit) task as early as
  // possible in setup(), right after bufman.init() and before OBD/MEMS/
  // HTTPD/BLE run - all of which allocate from and fragment the same ~213KB
  // internal DRAM that xTaskCreate's stack must come out of (task stacks
  // can't come from PSRAM here). Originally this was created LAST, after
  // BLE+HTTPD+MEMS+OBD had already run; measured live on 2026-09-15, that
  // left only free=19512 maxblock=14836 bytes internal - too fragmented for
  // the requested 16384-byte stack, so Task::create() failed EVERY time,
  // silently (its return value used to be discarded): the telemetry task
  // never started, so WiFi/cellular never connected, nothing was ever sent,
  // while httpd/GPS/OBD kept working fine - looking exactly like "WiFi is
  // connected but nothing is ever sent". Moving creation here gives it first
  // claim on a much less fragmented heap. Retries + logs the outcome either
  // way so a future regression here can never be silently invisible again.
  {
    Serial.printf("HEAP:free=%u maxblock=%u (internal, before telemetry task stack)\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
    bool telemetryStarted = false;
    for (byte tries = 0; tries < 5 && !telemetryStarted; tries++) {
      if (tries) delay(500);
      telemetryStarted = subtask.create(telemetry, "telemetry", 2, 16384);
    }
    Serial.print("TASK:telemetry=");
    Serial.println(telemetryStarted ? "OK" : "FAILED(heap?)");
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) {
      logger.logEvent(telemetryStarted ? "TASK TELEMETRY_OK" : "TASK TELEMETRY_FAILED");
    }
#endif
  }

#if ENABLE_OBD
  if (sys.begin()) {
    Serial.print("TYPE:");
    Serial.println(sys.devType);
    obd.begin(sys.link);
  }
#else
  sys.begin(false, true);
#endif

#if ENABLE_MEMS
if (!state.check(STATE_MEMS_READY)) do {
  Serial.print("MEMS:");
  mems = new ICM_42627;
  byte ret = mems->begin();
  if (ret) {
    state.set(STATE_MEMS_READY);
    Serial.println("ICM-42627");
    break;
  }
  delete mems;
  mems = new ICM_20948_I2C;
  ret = mems->begin();
  if (ret) {
    state.set(STATE_MEMS_READY);
    Serial.println("ICM-20948");
    break;
  } 
  delete mems;
  /*
  mems = new MPU9250;
  ret = mems->begin();
  if (ret) {
    state.set(STATE_MEMS_READY);
    Serial.println("MPU-9250");
    break;
  }
  */
  mems = 0;
  Serial.println("NO");
} while (0);
#endif

#if ENABLE_HTTPD
  if (enableHttpd) {
    IPAddress ip;
    if (serverSetup(ip)) {
      Serial.print("HTTPD:");
      Serial.println(ip);
    } else {
      Serial.println("HTTPD:NO");
    }
  }
#endif

#if ENABLE_BLE
  if (enableBle) {
    // init BLE
    ble_init("FreematicsPlus");
  }
#endif

  // Measured to answer "with the telemetry task's 16KB already reserved
  // earlier, is there still enough internal heap left for BLE+HTTPD+MEMS+
  // OBD to run safely, or should BLE be dropped instead of just reordered?"
  Serial.printf("HEAP:free=%u maxblock=%u (internal, after BLE+HTTPD+MEMS+OBD init)\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());

  // Print pull-OTA configuration so users can verify NVS provisioning via the
  // serial console.  This makes silent failures immediately obvious:
  // "OTA:disabled"        Ã¢â€ â€™ OTA_TOKEN not in NVS (re-flash with HA serial-flash button).
  // "INTERVAL=0 (checks disabled)" Ã¢â€ â€™ token set but OTA_INTERVAL not provisioned;
  //                           re-flash or use Send Config to set OTA_INTERVAL > 0.
  printOtaStatus();

  // initialize components
  initialize();

#ifdef PIN_LED
  digitalWrite(PIN_LED, LOW);
#endif
}

void loop()
{
  // error handling
  if (!state.check(STATE_WORKING)) {
    standby();
#ifdef PIN_LED
    if (enableLedRed) digitalWrite(PIN_LED, HIGH);
#endif
    initialize();
#ifdef PIN_LED
    digitalWrite(PIN_LED, LOW);
#endif
    return;
  }

  // collect and log data
  process();
}
