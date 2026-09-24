/*
 * bt_presence.cpp - crew detection by phone, classic Bluetooth (2026-09-24)
 *
 * Phones advertise over BLE only with rotating private addresses and no
 * name (measured on a POCO F7 Ultra: a new Xiaomi address every few
 * minutes), but their classic-Bluetooth address is fixed. A phone with
 * Bluetooth on answers an HCI Remote Name Request to that address from
 * anyone - no pairing, no app, not discoverable. So: for each known phone
 * address (NVS BT_KNOWN, later from Traccar) ask "what is your name?" once
 * per burst; an answer means the phone is within ~10 m.
 *
 * No Bluetooth host stack at all (no NimBLE, no Bluedroid): the controller
 * is driven directly over VHCI with the three HCI commands needed. Runs
 * INSTEAD of NimBLE (BT_MODE=1); switching modes needs a reboot, because the
 * unused mode's controller memory is released for good.
 *
 * Coexistence: the controller is only enabled for the few seconds of a
 * burst, and never during a WiFi connect attempt (ble_coex_begin(), see
 * ble_spp_server_nimble.cpp) - same abort risk as with BLE.
 */
#include <Arduino.h>
#include "esp_bt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <FreematicsPlus.h>
#include <httpd.h>
#include "config.h"

#if ENABLE_BLE

#define BT_KNOWN_MAX 8
#define BT_BURST_EVERY_MS 60000  // TODO: 1 min for the first bursts of a trip, then 5 min
#define BT_EVT_MAX 96            // Remote Name Request Complete / EIR events are 255 bytes; cut
#define BT_FOUND_MAX 12
#define HCI_OP_RESET 0x0C03
#define HCI_OP_WRITE_PAGE_TIMEOUT 0x0C18
#define HCI_OP_WRITE_INQUIRY_MODE 0x0C45
#define HCI_OP_INQUIRY 0x0401
#define HCI_OP_REMOTE_NAME_REQ 0x0419
#define HCI_EV_INQUIRY_COMPLETE 0x01
#define HCI_EV_INQUIRY_RESULT_EXT 0x2F
#define HCI_EV_INQUIRY_RESULT_RSSI 0x22
#define HCI_EV_REMOTE_NAME_COMPLETE 0x07
#define HCI_EV_CMD_COMPLETE 0x0E
#define HCI_EV_CMD_STATUS 0x0F

struct BtKnown {
  uint8_t addr[6];        // most significant first
  char name[33];
  uint32_t lastSeenMs;    // 0 = never
  uint16_t hits, tries;
  int16_t lastStatus;     // HCI status of the last request (0 = answered, 4 = page timeout), -1 none
  uint16_t lastMs;        // how long the last request took
};

struct HciEvt {
  uint8_t len;
  uint8_t data[BT_EVT_MAX];  // event code, param len, params...
};

static BtKnown s_known[BT_KNOWN_MAX];
static uint8_t s_knownCount = 0;

// Devices found by inquiry - only phones in discoverable mode (Bluetooth
// settings open) answer it. This is how a new phone's fixed address is
// learned without typing it in.
struct BtFound {
  uint8_t addr[6];
  uint32_t cod;           // class of device; major class 0x02 = phone
  int8_t rssi;
  char name[33];
  uint32_t lastSeenMs;
};
static BtFound s_found[BT_FOUND_MAX];
static uint8_t s_foundCount = 0;
static portMUX_TYPE s_btMux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_evtQ = 0;
static uint32_t s_cycles = 0;
static int s_lastError = 0;
static bool (*s_active)() = 0;
static bool s_started = false;

static void vhciSendAvailable() {}

// controller -> host, controller task context
static int vhciRecv(uint8_t* data, uint16_t len)
{
  if (len >= 3 && data[0] == 0x04 && s_evtQ) {  // HCI event packet
    HciEvt e;
    e.len = len - 1 > BT_EVT_MAX ? BT_EVT_MAX : len - 1;
    memcpy(e.data, data + 1, e.len);
    xQueueSend(s_evtQ, &e, 0);
  }
  return 0;
}

static esp_vhci_host_callback_t s_vhciCb = { vhciSendAvailable, vhciRecv };

static void hciCmd(uint16_t op, const uint8_t* p, uint8_t n)
{
  uint8_t b[4 + 16];
  b[0] = 0x01;  // HCI command packet
  b[1] = op & 0xFF;
  b[2] = op >> 8;
  b[3] = n;
  if (n) memcpy(b + 4, p, n);
  for (int i = 0; i < 100 && !esp_vhci_host_check_send_available(); i++) delay(1);
  esp_vhci_host_send_packet(b, 4 + n);
}

// Waits for event `code` (for command complete/status: of opcode `op`).
// Returns the event's first parameter byte (a status for all used here), -1 on timeout.
static int hciWait(uint8_t code, uint16_t op, uint32_t timeoutMs, HciEvt* out = 0)
{
  uint32_t t = millis();
  HciEvt e;
  while (millis() - t < timeoutMs) {
    if (xQueueReceive(s_evtQ, &e, pdMS_TO_TICKS(50)) != pdTRUE) continue;
    if (e.data[0] != code || e.len < 3) continue;
    if (code == HCI_EV_CMD_COMPLETE && (e.len < 6 || (e.data[3] | (e.data[4] << 8)) != op)) continue;
    if (code == HCI_EV_CMD_COMPLETE) { if (out) *out = e; return e.data[5]; }
    if (code == HCI_EV_CMD_STATUS && (e.len < 6 || (e.data[4] | (e.data[5] << 8)) != op)) continue;
    if (out) *out = e;
    return e.data[2];
  }
  return -1;
}

static bool btUp()
{
  esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  cfg.mode = ESP_BT_MODE_CLASSIC_BT;
  esp_err_t err = esp_bt_controller_init(&cfg);
  if (err != ESP_OK) { s_lastError = 1000 + err; return false; }
  err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
  if (err != ESP_OK) { s_lastError = 2000 + err; esp_bt_controller_deinit(); return false; }
  esp_vhci_host_register_callback(&s_vhciCb);
  xQueueReset(s_evtQ);
  hciCmd(HCI_OP_RESET, 0, 0);
  if (hciWait(HCI_EV_CMD_COMPLETE, HCI_OP_RESET, 2000) != 0) { s_lastError = 3; return false; }
  // page timeout 0x2000 slots * 0.625 ms = 5.12 s (the spec default): an
  // idle phone may page-scan only every 1.28-2.56 s (R1/R2)
  // 2026-09-24 bench: POCO F7 Ultra answered in 0.2-0.8 s when awake, but in
  // a pocket (screen off) 1/9 at 5.12 s and 0/3 even at 20.48 s - it barely
  // page-scans then. Longer timeouts don't help.
  const uint8_t pto[2] = { 0x00, 0x20 };
  hciCmd(HCI_OP_WRITE_PAGE_TIMEOUT, pto, 2);
  hciWait(HCI_EV_CMD_COMPLETE, HCI_OP_WRITE_PAGE_TIMEOUT, 1000);
  // inquiry results with RSSI and extended inquiry response (name inside)
  const uint8_t mode = 0x02;
  hciCmd(HCI_OP_WRITE_INQUIRY_MODE, &mode, 1);
  hciWait(HCI_EV_CMD_COMPLETE, HCI_OP_WRITE_INQUIRY_MODE, 1000);
  return true;
}

// ~5 s general inquiry; records every responder (address, class, RSSI, EIR name)
static void doInquiry()
{
  const uint8_t p[5] = { 0x33, 0x8B, 0x9E, 4 /* x1.28 s = 5.12 s */, 0 /* unlimited responses */ };
  xQueueReset(s_evtQ);
  hciCmd(HCI_OP_INQUIRY, p, sizeof(p));
  uint32_t t = millis();
  HciEvt e;
  int results = 0, cmdStatus = -1, complete = -1;
  while (millis() - t < 7000) {
    if (xQueueReceive(s_evtQ, &e, pdMS_TO_TICKS(100)) != pdTRUE) continue;
    if (e.data[0] == HCI_EV_INQUIRY_COMPLETE) { complete = e.len >= 3 ? e.data[2] : -2; break; }
    if (e.data[0] == HCI_EV_CMD_STATUS && e.len >= 3) {
      cmdStatus = e.data[2];
      if (cmdStatus != 0) break;  // inquiry refused
      continue;
    }
    // 0x22 (result with RSSI, no EIR) has the same layout up to the RSSI byte
    if ((e.data[0] != HCI_EV_INQUIRY_RESULT_EXT && e.data[0] != HCI_EV_INQUIRY_RESULT_RSSI) || e.len < 17) {
      if (e.data[0] != HCI_EV_CMD_COMPLETE) Serial.printf("[BT] inquiry: other event 0x%02X len=%u\n", e.data[0], e.len);
      continue;
    }
    results++;
    // params: num(1) bdaddr(6) psrm(1) reserved(1) cod(3) clock(2) rssi(1) eir(240)
    uint8_t addr[6];
    for (int i = 0; i < 6; i++) addr[i] = e.data[3 + 5 - i];
    uint32_t cod = e.data[11] | (e.data[12] << 8) | ((uint32_t)e.data[13] << 16);
    char name[33] = {0};
    for (int i = 17; e.data[0] == HCI_EV_INQUIRY_RESULT_EXT && i + 1 < e.len && e.data[i];) {  // EIR: len, type, data
      uint8_t l = e.data[i], type = e.data[i + 1];
      if ((type == 0x09 || type == 0x08) && l > 1) {
        int n = l - 1;
        if (n > (int)sizeof(name) - 1) n = sizeof(name) - 1;
        if (i + 2 + n > e.len) n = e.len - i - 2;  // cut by BT_EVT_MAX
        if (n > 0) memcpy(name, e.data + i + 2, n);
        name[n > 0 ? n : 0] = 0;
        break;
      }
      i += l + 1;
    }
    Serial.printf("[BT] found %02X:%02X:%02X:%02X:%02X:%02X cod=%06X rssi=%d '%s'\n",
        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], (unsigned)cod, (int8_t)e.data[16], name);
    portENTER_CRITICAL(&s_btMux);
    int j = 0;
    while (j < s_foundCount && memcmp(s_found[j].addr, addr, 6)) j++;
    if (j == s_foundCount) {
      if (s_foundCount < BT_FOUND_MAX) s_foundCount++;
      else { j = 0; for (int k = 1; k < BT_FOUND_MAX; k++) if (s_found[k].lastSeenMs < s_found[j].lastSeenMs) j = k; }
      memset(&s_found[j], 0, sizeof(BtFound));
      memcpy(s_found[j].addr, addr, 6);
    }
    s_found[j].cod = cod;
    s_found[j].rssi = (int8_t)e.data[16];
    if (name[0]) strcpy(s_found[j].name, name);
    s_found[j].lastSeenMs = millis();
    portEXIT_CRITICAL(&s_btMux);
  }
  Serial.printf("[BT] inquiry: status=%d complete=%d results=%d %ums\n",
      cmdStatus, complete, results, (unsigned)(millis() - t));
}

static void btDown()
{
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) esp_bt_controller_disable();
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) esp_bt_controller_deinit();
}

// One name request. Returns the HCI status (0 = answered), -1 no reply at all.
static int askName(const uint8_t* addr, char* name, int nameSize, uint32_t* tookMs)
{
  uint8_t p[10];
  for (int i = 0; i < 6; i++) p[i] = addr[5 - i];  // BD_ADDR is little-endian on the wire
  p[6] = 0x02;  // page scan repetition mode R2 (covers R0/R1 scanners too, just pages longer)
  p[7] = 0x00;
  p[8] = p[9] = 0x00;  // clock offset unknown
  uint32_t t = millis();
  xQueueReset(s_evtQ);
  hciCmd(HCI_OP_REMOTE_NAME_REQ, p, sizeof(p));
  int st = hciWait(HCI_EV_CMD_STATUS, HCI_OP_REMOTE_NAME_REQ, 1000);
  if (st != 0) { *tookMs = millis() - t; return st; }
  HciEvt e;
  st = hciWait(HCI_EV_REMOTE_NAME_COMPLETE, 0, 8000, &e);
  *tookMs = millis() - t;
  if (st == 0) {
    // params: status(1) bdaddr(6) name(248, NUL-terminated) - cut to what fits
    int n = e.len - 9;
    if (n > nameSize - 1) n = nameSize - 1;
    if (n < 0) n = 0;
    memcpy(name, e.data + 9, n);
    name[n] = 0;
  }
  return st;
}

static void presenceTask(void*)
{
  uint32_t last = 0;
  for (;;) {
    delay(1000);
    if (s_active && !s_active()) continue;
    if (last && millis() - last < BT_BURST_EVERY_MS) continue;
    if (!ble_coex_begin()) continue;  // WiFi connecting - try again in a second
    last = millis();
    if (btUp()) {
      uint32_t burstStart = millis();
      doInquiry();
      // names of devices just found without one (discoverable = they answer at once)
      for (int i = 0; i < BT_FOUND_MAX; i++) {
        uint8_t addr[6];
        bool ask = false;
        portENTER_CRITICAL(&s_btMux);
        if (i < s_foundCount && !s_found[i].name[0] && s_found[i].lastSeenMs >= burstStart) {
          memcpy(addr, s_found[i].addr, 6);
          ask = true;
        }
        portEXIT_CRITICAL(&s_btMux);
        if (!ask) continue;
        char name[33] = {0};
        uint32_t took = 0;
        if (askName(addr, name, sizeof(name), &took) == 0 && name[0]) {
          Serial.printf("[BT] found name '%s' (%ums)\n", name, (unsigned)took);
          portENTER_CRITICAL(&s_btMux);
          if (i < s_foundCount && !memcmp(s_found[i].addr, addr, 6)) strcpy(s_found[i].name, name);
          portEXIT_CRITICAL(&s_btMux);
        }
      }
      for (int i = 0; i < s_knownCount; i++) {
        uint8_t addr[6];
        portENTER_CRITICAL(&s_btMux);
        memcpy(addr, s_known[i].addr, 6);
        portEXIT_CRITICAL(&s_btMux);
        char name[33] = {0};
        uint32_t took = 0;
        int st = askName(addr, name, sizeof(name), &took);
        Serial.printf("[BT] %02X:%02X:%02X:%02X:%02X:%02X status=%d %ums '%s'\n",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], st, (unsigned)took, name);
        portENTER_CRITICAL(&s_btMux);
        BtKnown& k = s_known[i];
        k.tries++;
        k.lastStatus = st;
        k.lastMs = took;
        if (st == 0) {
          k.hits++;
          k.lastSeenMs = millis();
          if (name[0]) strcpy(k.name, name);
        }
        portEXIT_CRITICAL(&s_btMux);
      }
    }
    btDown();
    ble_coex_end();
    s_cycles++;
  }
}

// "AA:BB:CC:DD:EE:FF,11:22:..." -> table (keeps stats of addresses still listed)
void bt_presence_set_known(const char* list)
{
  BtKnown next[BT_KNOWN_MAX];
  uint8_t cnt = 0;
  const char* p = list;
  while (p && *p && cnt < BT_KNOWN_MAX) {
    unsigned b[6];
    if (sscanf(p, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
      memset(&next[cnt], 0, sizeof(BtKnown));
      for (int i = 0; i < 6; i++) next[cnt].addr[i] = (uint8_t)b[i];
      next[cnt].lastStatus = -1;
      portENTER_CRITICAL(&s_btMux);
      for (int j = 0; j < s_knownCount; j++)
        if (!memcmp(s_known[j].addr, next[cnt].addr, 6)) next[cnt] = s_known[j];
      portEXIT_CRITICAL(&s_btMux);
      cnt++;
    }
    p = strchr(p, ',');
    if (p) p++;
  }
  portENTER_CRITICAL(&s_btMux);
  memcpy(s_known, next, sizeof(BtKnown) * cnt);
  s_knownCount = cnt;
  portEXIT_CRITICAL(&s_btMux);
}

bool bt_presence_start(bool (*active)())
{
  if (s_started) return true;
  s_active = active;
  s_evtQ = xQueueCreate(8, sizeof(HciEvt));
  if (!s_evtQ) return false;
  // BLE-only controller memory is never used in this mode
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
  s_started = xTaskCreatePinnedToCore(presenceTask, "btpresence", 4096, 0, 1, 0, 0) == pdPASS;
  return s_started;
}

#if ENABLE_HTTPD
int handlerBtPresence(UrlHandlerParam* param)
{
  char *buf = param->pucBuffer;
  int bufsize = param->bufSize;
  static BtKnown copy[BT_KNOWN_MAX];
  portENTER_CRITICAL(&s_btMux);
  uint8_t cnt = s_knownCount;
  memcpy(copy, s_known, sizeof(BtKnown) * cnt);
  portEXIT_CRITICAL(&s_btMux);
  uint32_t now = millis();
  int n = snprintf(buf, bufsize, "{\"on\":%u,\"cycles\":%u,\"err\":%d,\"heap\":%u,\"maxblock\":%u,\"dev\":[",
      (unsigned)s_started, (unsigned)s_cycles, s_lastError,
      (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
  for (int i = 0; i < cnt && n < bufsize - 160; i++) {
    const BtKnown& k = copy[i];
    char nm[sizeof(k.name)];
    for (int j = 0; j < (int)sizeof(nm); j++) {
      char c = k.name[j];
      nm[j] = (c == '"' || c == '\\' || (c > 0 && c < 32)) ? '_' : c;
      if (!c) break;
    }
    n += snprintf(buf + n, bufsize - n,
        "%s{\"a\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"n\":\"%s\",\"hits\":%u,\"tries\":%u,\"st\":%d,\"ms\":%u,\"ago\":%d}",
        i ? "," : "", k.addr[0], k.addr[1], k.addr[2], k.addr[3], k.addr[4], k.addr[5], nm,
        (unsigned)k.hits, (unsigned)k.tries, (int)k.lastStatus, (unsigned)k.lastMs,
        k.lastSeenMs ? (int)((now - k.lastSeenMs) / 1000) : -1);
  }
  static BtFound fcopy[BT_FOUND_MAX];
  portENTER_CRITICAL(&s_btMux);
  uint8_t fcnt = s_foundCount;
  memcpy(fcopy, s_found, sizeof(BtFound) * fcnt);
  portEXIT_CRITICAL(&s_btMux);
  n += snprintf(buf + n, bufsize - n, "],\"found\":[");
  for (int i = 0; i < fcnt && n < bufsize - 140; i++) {
    const BtFound& f = fcopy[i];
    char nm[sizeof(f.name)];
    for (int j = 0; j < (int)sizeof(nm); j++) {
      char c = f.name[j];
      nm[j] = (c == '"' || c == '\\' || (c > 0 && c < 32)) ? '_' : c;
      if (!c) break;
    }
    n += snprintf(buf + n, bufsize - n,
        "%s{\"a\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"n\":\"%s\",\"cod\":%u,\"r\":%d,\"ago\":%u}",
        i ? "," : "", f.addr[0], f.addr[1], f.addr[2], f.addr[3], f.addr[4], f.addr[5], nm,
        (unsigned)f.cod, (int)f.rssi, (unsigned)((now - f.lastSeenMs) / 1000));
  }
  n += snprintf(buf + n, bufsize - n, "]}");
  param->contentLength = n;
  param->contentType = HTTPFILETYPE_JSON;
  return FLAG_DATA_RAW;
}
#endif

#endif
