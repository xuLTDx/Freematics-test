// Sniff mode - see sniff.h. The capture part is HexSniff v2 (HexSniff repo,
// firmware/src/main.cpp, 2026-09-29) moved over unchanged in behaviour:
// listen-only TWAI, speed probe, PSRAM frame queue, the same line format.
#include "sniff.h"
#include <SD.h>
#include <Preferences.h>
#include <nvs.h>
#include "driver/twai.h"
#include "esp_timer.h"
#include "canodo.h"
#include "sdwriter.h"

extern nvs_handle_t nvs;
extern const char FW_BUILD_STR[];
uint32_t txqBootId();
void sdLock();    // telestore.cpp - the SD card is shared with the logger
void sdUnlock();

#define CAN_TX_PIN GPIO_NUM_26
#define CAN_RX_PIN GPIO_NUM_34
#define SNIFF_DIR "/SNIFF"
#define LIVE_FRAMES 200
// receive task -> SD writer (PSRAM): 65536 frames = ~1.5 MB, ~40 s of a full
// 500 kbit/s bus (Peugeot 2026-10-12: ~1580 frames/s) if the card stalls;
// smaller if that much PSRAM is not free
static const uint32_t QUEUE_TRY[] = {65536, 32768, 8192};
#define SPEED_PROBE_MS 2000        // listen this long per bus speed
#define STATS_EVERY_MS 60000
#define NVS_SAVE_EVERY_MIN 5       // remaining minutes written to NVS this often

struct Frame {
  int64_t us;                      // esp_timer time of reception
  uint32_t id;
  uint8_t ext, dlc, tx;            // tx = sent by us (SNIFF_SEND)
  uint8_t data[8];
};

static Frame* s_queue;
static uint32_t s_queueCap;
static volatile uint32_t s_qHead, s_qTail;
static volatile uint32_t s_dropped;          // queue full: lost before SD
static portMUX_TYPE s_qMux = portMUX_INITIALIZER_UNLOCKED;
static Frame s_live[LIVE_FRAMES];
static uint32_t s_liveHead, s_liveCount;
static volatile uint32_t s_frames;

static const uint32_t s_speeds[] = {500, 250, 125};
static int s_speedIdx;
static bool s_speedFound;
static uint32_t s_probeStart, s_probeFrames;
static volatile bool s_canRunning;
static volatile bool s_pauseRx, s_rxPaused;
static SemaphoreHandle_t s_canMux;

static volatile bool s_active;
static volatile int8_t s_req = -1;           // -1 none, 0 off, 1 on
static volatile uint16_t s_reqMin;
static uint16_t s_minLeft;
static uint32_t s_minuteMs;
static uint8_t s_sinceSave;

static volatile bool s_fileOk;
static uint32_t s_fileNo;
static char s_path[32];
static volatile uint32_t s_written;   // frame lines the card confirmed (sdwriter.cpp)
static volatile uint32_t s_sdErrors;  // failed writes; each block is written again until it is confirmed
static volatile uint32_t s_sdLost;    // frames that never reached the card (card gone / no file)
static uint32_t s_lastStats;
static volatile bool s_closing;       // the writer ends the file after the queued frames
static const char* volatile s_closeTag = "# end";
static String s_vehicle = "UNSPECIFIED";
static int s_sdLimitPct = SNIFF_SD_LIMIT_PCT;
static volatile bool s_refused;       // last start refused: card too full

// ---------------------------------------------------------------- CAN ----
static twai_timing_config_t timingFor(uint32_t kbit)
{
  switch (kbit) {
    case 250: return TWAI_TIMING_CONFIG_250KBITS();
    case 125: return TWAI_TIMING_CONFIG_125KBITS();
    default:  return TWAI_TIMING_CONFIG_500KBITS();
  }
}

static bool canStart(twai_mode_t mode)
{
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, mode);
  g.rx_queue_len = 128;
  g.tx_queue_len = 2;
  twai_timing_config_t t = timingFor(s_speeds[s_speedIdx]);
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  s_canRunning = true;
  return true;
}

static void canStop()
{
  s_canRunning = false;
  twai_stop();
  twai_driver_uninstall();
}

static void pushFrame(const Frame& f)
{
  portENTER_CRITICAL(&s_qMux);
  uint32_t next = (s_qHead + 1) % s_queueCap;
  if (next == s_qTail) {
    s_dropped++;
  } else {
    s_queue[s_qHead] = f;
    s_qHead = next;
  }
  s_live[s_liveHead] = f;
  s_liveHead = (s_liveHead + 1) % LIVE_FRAMES;
  if (s_liveCount < LIVE_FRAMES) s_liveCount++;
  s_frames++;
  portEXIT_CRITICAL(&s_qMux);
}

static void rxTask(void*)
{
  for (;;) {
    if (s_pauseRx || !s_canRunning) {
      s_rxPaused = true;
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    s_rxPaused = false;
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(20)) != ESP_OK) continue;
    Frame f = {};
    f.us = esp_timer_get_time();
    f.id = m.identifier;
    f.ext = m.extd;
    f.dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
    memcpy(f.data, m.data, f.dlc);
    pushFrame(f);
  }
}

#ifdef TEST_SNIFF_LOAD
// bench: synthetic frames at ~TEST_SNIFF_LOAD per second while the mode is on
// (no bus needed). ID 0x7AA, data = 64-bit sequence number, so a downloaded
// file can be checked for gaps (tools: sniff_check.py).
static void loadTask(void*)
{
  static uint64_t seq = 0;
  const int perTick = TEST_SNIFF_LOAD / 500;  // 2 ms ticks
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(2));
    if (!s_canRunning) continue;
    for (int i = 0; i < perTick; i++) {
      Frame f = {};
      f.us = esp_timer_get_time();
      f.id = 0x7AA;
      f.dlc = 8;
      memcpy(f.data, &seq, 8);
      seq++;
      pushFrame(f);
    }
  }
}
#endif

static void pauseRx()
{
  s_pauseRx = true;
  for (int i = 0; i < 100 && !s_rxPaused; i++) delay(2);
}

// ----------------------------------------------------------------- SD ----
// The file is written by the SD writer task (sdwriter.cpp), which pulls the
// bytes from sniffFill() and confirms them back (sniffConfirmed) only after
// the card took them - a block that failed is written again, never skipped.
static int formatFrame(char* out, int cap, const Frame& f)
{
  int n = snprintf(out, cap, "%lu.%03lu,%lX,%c,%u,", (unsigned long)(f.us / 1000),
                   (unsigned long)(f.us % 1000), (unsigned long)f.id, f.ext ? 'x' : 's', f.dlc);
  for (int i = 0; i < f.dlc && n < cap - 4; i++) n += snprintf(out + n, cap - n, i ? " %02X" : "%02X", f.data[i]);
  if (f.tx && n < cap - 4) n += snprintf(out + n, cap - n, ",TX");
  return n;
}

// text lines (header, bus speed, vehicle, sd errors) waiting for the writer
static char s_text[2048];
static int s_textLen;
static portMUX_TYPE s_tMux = portMUX_INITIALIZER_UNLOCKED;

static void addLine(const char* line)
{
  int n = strlen(line);
  portENTER_CRITICAL(&s_tMux);
  if (s_textLen + n + 1 <= (int)sizeof(s_text)) {
    memcpy(s_text + s_textLen, line, n);
    s_textLen += n;
    s_text[s_textLen++] = '\n';
  }
  portEXIT_CRITICAL(&s_tMux);
}

static void statsLine(char* line, int cap)
{
  twai_status_info_t st = {};
  if (s_canRunning) twai_get_status_info(&st);
  // frames = received (incl. our TX); written + sd_lost + dropped + still queued = frames
  snprintf(line, cap, "# stats t=%lu unix=%lu frames=%lu written=%lu dropped=%lu sd_err=%lu sd_lost=%lu rx_missed=%lu rx_overrun=%lu bus_err=%lu speed=%lu%s",
           (unsigned long)millis(), (unsigned long)time(nullptr), (unsigned long)s_frames, (unsigned long)s_written,
           (unsigned long)s_dropped, (unsigned long)s_sdErrors, (unsigned long)s_sdLost,
           (unsigned long)st.rx_missed_count, (unsigned long)st.rx_overrun_count, (unsigned long)st.bus_error_count,
           (unsigned long)s_speeds[s_speedIdx], s_speedFound ? "k" : "k (probing)");
}

// SD writer task: the next bytes for the file (whole lines only). frames =
// frame lines in them; last = the file ends with these bytes (stop/restart).
int sniffFill(char* buf, int cap, uint32_t* frames, bool* last)
{
  int n = 0;
  *frames = 0;
  *last = false;
  portENTER_CRITICAL(&s_tMux);
  if (s_textLen && s_textLen <= cap) {
    memcpy(buf, s_text, s_textLen);
    n = s_textLen;
    s_textLen = 0;
  }
  portEXIT_CRITICAL(&s_tMux);
  char line[200];
  if (millis() - s_lastStats >= STATS_EVERY_MS && n + (int)sizeof(line) < cap) {
    s_lastStats = millis();
    statsLine(line, sizeof(line));
    Serial.println(line);
    int l = strlen(line);
    memcpy(buf + n, line, l);
    n += l;
    buf[n++] = '\n';
  }
  while (s_qTail != s_qHead && n + 100 < cap) {
    Frame f = s_queue[s_qTail];
    s_qTail = (s_qTail + 1) % s_queueCap;
    n += formatFrame(buf + n, cap - n, f);
    buf[n++] = '\n';
    (*frames)++;
  }
  if (s_closing && s_qTail == s_qHead && n + 2 * (int)sizeof(line) < cap) {
    // the last stats line counts the frames of this very block as written
    // (they are, if this block is confirmed; if not it is written again)
    s_written += *frames;
    statsLine(line, sizeof(line));
    s_written -= *frames;
    int l = strlen(line);
    memcpy(buf + n, line, l);
    n += l;
    buf[n++] = '\n';
    const char* e = s_closeTag;
    l = strlen(e);
    memcpy(buf + n, e, l);
    n += l;
    buf[n++] = '\n';
    *last = true;
  }
  return n;
}

void sniffConfirmed(uint32_t frames) { s_written += frames; }

void sniffNoteSdError(const char* op, int err)
{
  s_sdErrors++;
  char line[64];
  snprintf(line, sizeof(line), "# sderr t=%lu op=%s errno=%d", (unsigned long)millis(), op, err);
  addLine(line);
}

// SD writer: the card is gone - count what never reached it, switch off
void sniffSdDead(uint32_t lostFrames)
{
  uint32_t h = s_qHead;
  s_sdLost += lostFrames + (h + s_queueCap - s_qTail) % s_queueCap;
  s_qTail = h;
  s_fileOk = false;
  s_closing = false;
  s_req = 0;  // off at the next sniffTick()
}

static int sdUsedPct()
{
  sdLock();
  uint64_t total = SD.totalBytes(), used = SD.usedBytes();
  sdUnlock();
  return total ? (int)(used * 100 / total) : 100;
}

// new session file /SNIFF/S<n>.LOG, n above every existing file (the card
// may hold HexSniff's own S1..S100 - never overwrite one: found 2026-10-11,
// the first session here overwrote S1.LOG). Old files: sniffPurgeOld().
static void openFile()
{
  uint32_t n = 0;
  nvs_get_u32(nvs, "SNIFF_NO", &n);
  char path[32];
  sdLock();
  if (!SD.exists(SNIFF_DIR)) SD.mkdir(SNIFF_DIR);
  File dir = SD.open(SNIFF_DIR);
  File f;
  while (dir && (f = dir.openNextFile())) {
    const char* fn = f.name();
    const char* p = strrchr(fn, '/');
    if (p) fn = p + 1;
    if (fn[0] == 'S' && (uint32_t)atoi(fn + 1) > n) n = atoi(fn + 1);
    f.close();
  }
  sdUnlock();
  n++;
  nvs_set_u32(nvs, "SNIFF_NO", n);
  nvs_commit(nvs);
  s_fileNo = n;
  snprintf(path, sizeof(path), SNIFF_DIR "/S%lu.LOG", (unsigned long)n);
  strcpy(s_path, path);
  s_written = s_sdErrors = s_sdLost = 0;
  portENTER_CRITICAL(&s_tMux);
  s_textLen = 0;
  portEXIT_CRITICAL(&s_tMux);
  time_t now = time(nullptr);
  char line[200];
  snprintf(line, sizeof(line), "# telelogger sniff (TWAI listen-only) vehicle=%s fw=%s boot=%lu",
           s_vehicle.c_str(), FW_BUILD_STR, (unsigned long)txqBootId());
  addLine(line);
  // maps the <ms> of the frame lines to the wall clock (0 = clock not set yet);
  // also the file's age for sniffPurgeOld()
  snprintf(line, sizeof(line), "# clock unix=%lu at_ms=%lu", (unsigned long)(now > 1735689600 ? now : 0),
           (unsigned long)(esp_timer_get_time() / 1000));
  addLine(line);
  addLine("# format: <ms>.<us>,<ID hex>,<s=11-bit|x=29-bit>,<dlc>,<data hex>[,TX]");
  s_fileOk = sdwSniffOpen(path);
  Serial.printf("[SNIFF] file %s %s\n", path, s_fileOk ? "open" : "FAILED");
}

// the file's end: the writer drains the queue, adds the last stats line and
// tag, closes. Called with the controller stopped.
static void closeFile(const char* tag)
{
  if (!s_fileOk) return;
  s_closeTag = tag;
  s_closing = true;
  sdwSniffClose(5000);
  if (!sdwSniffOpenNow()) {
    s_fileOk = false;
    s_closing = false;
  } else {
    Serial.println("[SNIFF] file not closed within 5 s (writer still busy)");
  }
}

// 2026-10-12 (user): sniff files go after 60 days. Age = the "# clock unix="
// header line (the session start); only with a valid clock, and a file
// without a clock (unix=0) is never deleted. Returns files removed.
int sniffPurgeOld(time_t now)
{
  if (now < 1735689600) return 0;
  int removed = 0;
  char names[16][16];
  int nNames = 0;
  sdLock();
  File dir = SD.open(SNIFF_DIR);
  File f;
  while (dir && nNames < 16 && (f = dir.openNextFile())) {
    const char* fn = f.name();
    const char* p = strrchr(fn, '/');
    if (p) fn = p + 1;
    uint32_t no = fn[0] == 'S' ? (uint32_t)atoi(fn + 1) : 0;
    if (no && !(s_fileOk && no == s_fileNo)) {
      char head[400];
      int l = f.read((uint8_t*)head, sizeof(head) - 1);
      head[l > 0 ? l : 0] = 0;
      const char* c = strstr(head, "# clock unix=");
      unsigned long unix = c ? strtoul(c + 13, nullptr, 10) : 0;
      if (unix > 1735689600 && (unsigned long)now > unix && (unsigned long)now - unix > SNIFF_KEEP_DAYS * 86400UL) {
        snprintf(names[nNames++], sizeof(names[0]), "%s", fn);
      }
    }
    f.close();
  }
  if (dir) dir.close();
  for (int i = 0; i < nNames; i++) {
    char path[32];
    snprintf(path, sizeof(path), SNIFF_DIR "/%s", names[i]);
    if (SD.remove(path)) {
      removed++;
      Serial.printf("[SNIFF] removed %s (older than %u days)\n", path, (unsigned)SNIFF_KEEP_DAYS);
    }
  }
  sdUnlock();
  return removed;
}

// ------------------------------------------------------------- control ---
static void saveMinutes()
{
  nvs_set_u16(nvs, "SNIFF_MIN", s_active ? s_minLeft : 0);
  nvs_commit(nvs);
  s_sinceSave = 0;
}

static bool startNow(uint16_t minutes)
{
  // the card must keep room for telemetry (2026-10-12: refuse at >= 70 %)
  int used = sdUsedPct();
  if (used >= s_sdLimitPct) {
    s_refused = true;
    Serial.printf("[SNIFF] NOT started: SD %d %% full (limit %d %%)\n", used, s_sdLimitPct);
    return false;
  }
  s_refused = false;
  if (sdwSniffOpenNow()) {  // the previous file is still being finished
    sdwSniffClose(5000);
    if (sdwSniffOpenNow()) return false;
  }
  if (!s_queue) {
    for (uint32_t cap : QUEUE_TRY) {
      s_queue = (Frame*)ps_malloc(sizeof(Frame) * cap);
      if (s_queue) { s_queueCap = cap; break; }
    }
    if (!s_queue) {  // no PSRAM: a small queue
      s_queue = (Frame*)malloc(sizeof(Frame) * 512);
      s_queueCap = 512;
    }
    Serial.printf("[SNIFF] queue %lu frames\n", (unsigned long)s_queueCap);
    s_canMux = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(rxTask, "sniffrx", 3072, nullptr, 10, nullptr, 0);
#ifdef TEST_SNIFF_LOAD
    xTaskCreate(loadTask, "sniffload", 2048, nullptr, 5, nullptr);
#endif
  }
  canOdoEnd();  // the controller is ours now
  s_qHead = s_qTail = 0;
  s_frames = s_dropped = 0;
  s_liveCount = s_liveHead = 0;
  s_speedIdx = 0;
  s_speedFound = false;
  s_closing = false;
  s_lastStats = millis();
  openFile();
  xSemaphoreTake(s_canMux, portMAX_DELAY);
  bool ok = canStart(TWAI_MODE_LISTEN_ONLY);
  xSemaphoreGive(s_canMux);
  s_probeStart = millis();
  s_probeFrames = 0;
  s_minLeft = minutes;
  s_minuteMs = millis();
  s_active = true;
  saveMinutes();
  Serial.printf("[SNIFF] ON %u min, CAN listen-only %s\n", minutes, ok ? "started" : "FAILED");
  return true;
}

void sniffStopNow()
{
  if (!s_active) return;
  s_active = false;
  xSemaphoreTake(s_canMux, portMAX_DELAY);
  pauseRx();
  canStop();
  s_pauseRx = false;
  xSemaphoreGive(s_canMux);
  closeFile("# end");
  saveMinutes();  // the odometer reads come back via vwOdoTick() (telelogger.ino), only with VW_ODO=1
  Serial.println("[SNIFF] OFF");
}

// before a deliberate restart (prepareRestart(), telelogger.ino): the file
// gets its last frames, stats line and "# restart"; the minutes left stay in
// NVS, so the mode comes back after the restart with a new file
void sniffCloseForRestart()
{
  if (!s_active) return;
  saveMinutes();  // the exact minutes left (otherwise up to NVS_SAVE_EVERY_MIN old)
  s_active = false;
  xSemaphoreTake(s_canMux, portMAX_DELAY);
  pauseRx();
  canStop();
  s_pauseRx = false;
  xSemaphoreGive(s_canMux);
  closeFile("# restart");
  Serial.println("[SNIFF] closed for restart");
}

void sniffSetSdLimit(int pct) { s_sdLimitPct = pct; }

static void speedTick()
{
  if (s_speedFound) return;
  if (s_frames > s_probeFrames) {
    s_speedFound = true;
    char line[64];
    snprintf(line, sizeof(line), "# bus speed %lu kbit/s", (unsigned long)s_speeds[s_speedIdx]);
    addLine(line);
    Serial.println(line);
    return;
  }
  if (millis() - s_probeStart < SPEED_PROBE_MS) return;
  xSemaphoreTake(s_canMux, portMAX_DELAY);
  pauseRx();
  canStop();
  s_speedIdx = (s_speedIdx + 1) % (sizeof(s_speeds) / sizeof(s_speeds[0]));
  canStart(TWAI_MODE_LISTEN_ONLY);
  s_probeFrames = s_frames;
  s_probeStart = millis();
  s_pauseRx = false;
  xSemaphoreGive(s_canMux);
}

bool sniffInit()
{
  Preferences p;  // vehicle tag: the same NVS key HexSniff used
  if (p.begin("hexsniff", true)) {
    s_vehicle = p.getString("vehicle", s_vehicle);
    p.end();
  }
  uint16_t m = 0;
  nvs_get_u16(nvs, "SNIFF_MIN", &m);
  if (m) sniffRequest(true, m);  // was on before the restart: continue
  return m != 0;
}

void sniffRequest(bool on, uint16_t minutes)
{
  if (minutes > SNIFF_MAX_MIN) minutes = SNIFF_MAX_MIN;
  s_reqMin = minutes ? minutes : SNIFF_DEFAULT_MIN;
  s_req = on ? 1 : 0;
}

int sniffTick()
{
  int ev = 0;
  int8_t req = s_req;
  if (req >= 0) {
    s_req = -1;
    if (req == 1) {
      if (s_active) {
        s_minLeft = s_reqMin;  // already on: new duration
        saveMinutes();
      } else {
        ev = startNow(s_reqMin) ? SNIFF_EV_STARTED : SNIFF_EV_REFUSED;
        if (ev == SNIFF_EV_REFUSED) saveMinutes();  // not on: not resumed after a restart either
      }
    } else if (s_active) {
      sniffStopNow();
      ev = SNIFF_EV_STOPPED;
    }
  }
  if (!s_active) return ev;
  speedTick();
  if (millis() - s_minuteMs >= 60000) {
    s_minuteMs += 60000;
    if (s_minLeft) s_minLeft--;
    if (!s_minLeft) {
      Serial.println("[SNIFF] time is up");
      sniffStopNow();
      return SNIFF_EV_STOPPED;
    }
    if (++s_sinceSave >= NVS_SAVE_EVERY_MIN) saveMinutes();
  }
  return ev;
}

bool sniffActive() { return s_active; }
bool sniffWanted() { return s_active || s_req == 1; }
uint16_t sniffMinutesLeft() { return s_active ? s_minLeft : 0; }
uint32_t sniffFileNo() { return s_fileNo; }

// ----------------------------------------------------------------- web ---
int sniffStatus(char* buf, int size)
{
  twai_status_info_t st = {};
  if (s_active && s_canRunning) twai_get_status_info(&st);
  return snprintf(buf, size,
      "{\"on\":%d,\"refused\":%d,\"queue\":%lu,\"min\":%u,\"file\":%lu,\"sd\":%d,\"speed\":%lu,\"found\":%d,\"frames\":%lu,"
      "\"written\":%lu,\"sderr\":%lu,\"sdlost\":%lu,"
      "\"dropped\":%lu,\"missed\":%lu,\"overrun\":%lu,\"buserr\":%lu,\"vehicle\":\"%s\"}",
      s_active ? 1 : 0, s_refused ? 1 : 0, (unsigned long)s_queueCap, (unsigned)sniffMinutesLeft(),
      (unsigned long)s_fileNo, s_fileOk ? 1 : 0,
      (unsigned long)s_speeds[s_speedIdx], s_speedFound ? 1 : 0, (unsigned long)s_frames,
      (unsigned long)s_written, (unsigned long)s_sdErrors, (unsigned long)s_sdLost,
      (unsigned long)s_dropped, (unsigned long)st.rx_missed_count, (unsigned long)st.rx_overrun_count,
      (unsigned long)st.bus_error_count, s_vehicle.c_str());
}

int sniffLive(char* buf, int size, uint32_t id, uint32_t mask)
{
  int n = 0;
  char line[96];
  portENTER_CRITICAL(&s_qMux);
  uint32_t count = s_liveCount, head = s_liveHead;
  portEXIT_CRITICAL(&s_qMux);
  for (uint32_t i = 0; i < count; i++) {
    const Frame& f = s_live[(head + LIVE_FRAMES - count + i) % LIVE_FRAMES];
    if (mask && (f.id & mask) != (id & mask)) continue;
    int l = formatFrame(line, sizeof(line), f);
    if (n + l + 2 >= size) {  // keep the newest: drop from the start
      int cut = n / 2;
      char* nl = (char*)memchr(buf + cut, '\n', n - cut);
      if (!nl) break;
      cut = nl + 1 - buf;
      memmove(buf, buf + cut, n - cut);
      n -= cut;
    }
    memcpy(buf + n, line, l);
    n += l;
    buf[n++] = '\n';
  }
  if (n < size) buf[n] = 0;
  return n;
}

int sniffSend(uint32_t id, const uint8_t* data, int len, char* out, int cap)
{
  if (!s_active) return snprintf(out, cap, "ERR sniff off");
  xSemaphoreTake(s_canMux, portMAX_DELAY);
  pauseRx();
  canStop();
  bool ok = canStart(TWAI_MODE_NORMAL);
  twai_message_t m = {};
  m.identifier = id;
  m.extd = id > 0x7FF;
  m.ss = 1;                                   // single shot
  m.data_length_code = len;
  memcpy(m.data, data, len);
  Frame sent = {};
  sent.us = esp_timer_get_time();
  sent.id = id;
  sent.ext = m.extd;
  sent.dlc = len;
  sent.tx = 1;
  memcpy(sent.data, data, len);
  if (ok) ok = twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK;
  pushFrame(sent);
  uint32_t before = s_frames;
  s_pauseRx = false;                          // replies are captured by the normal path
  xSemaphoreGive(s_canMux);
  delay(1000);

  int n = snprintf(out, cap, "%s %lX, frames in the next second:\n", ok ? "Sent" : "SEND FAILED", (unsigned long)id);
  portENTER_CRITICAL(&s_qMux);
  uint32_t count = s_frames - before, head = s_liveHead, live = s_liveCount;
  portEXIT_CRITICAL(&s_qMux);
  if (count > live) count = live;
  for (uint32_t i = 0; i < count && n < cap - 100; i++) {
    n += formatFrame(out + n, cap - n, s_live[(head + LIVE_FRAMES - count + i) % LIVE_FRAMES]);
    out[n++] = '\n';
  }

  xSemaphoreTake(s_canMux, portMAX_DELAY);    // back to listen-only
  pauseRx();
  canStop();
  canStart(TWAI_MODE_LISTEN_ONLY);
  s_pauseRx = false;
  xSemaphoreGive(s_canMux);
  return n;
}

bool sniffSetVehicle(const char* tag)
{
  String clean;
  for (const char* p = tag; *p && clean.length() < 40; p++) {
    if (isalnum((unsigned char)*p) || *p == '_' || *p == '-') clean += *p;
  }
  if (!clean.length()) return false;
  s_vehicle = clean;
  Preferences p;
  if (p.begin("hexsniff", false)) {
    p.putString("vehicle", s_vehicle);
    p.end();
  }
  if (s_active) {
    String line = "# vehicle=" + s_vehicle + " from t=" + String(millis());
    addLine(line.c_str());
  }
  return true;
}

#ifdef TEST_SNIFF_LOAD
// ------------------------------------------------------------- bench ----
// SNIFF_CHECK=<n>: the box reads /SNIFF/S<n>.LOG itself (a 60 min load test
// is ~240 MB, hours over WiFi) and checks the generator's sequence: every
// frame line well-formed, the 0x7AA counter without a gap or repeat, the
// count equal to the last stats line's "written", sderr lines, the end tag.
static char s_checkResult[320] = "none";
static volatile uint32_t s_checkFile;
static volatile bool s_checkRunning;

static void checkTask(void*)
{
  char path[48];
  snprintf(path, sizeof(path), SDW_MOUNT SNIFF_DIR "/S%lu.LOG", (unsigned long)s_checkFile);
  uint32_t lines = 0, frames = 0, bad = 0, gaps = 0, repeats = 0, sderr = 0, lastWritten = 0;
  uint64_t prev = 0, firstGapAt = 0;
  bool havePrev = false, endTag = false;
  static char buf[16384];
  char line[200];
  int ll = 0;
  sdLock();
  FILE* f = fopen(path, "r");
  sdUnlock();
  if (!f) {
    snprintf(s_checkResult, sizeof(s_checkResult), "S%lu: open failed", (unsigned long)s_checkFile);
    s_checkRunning = false;
    vTaskDelete(nullptr);
  }
  for (;;) {
    sdLock();
    int n = fread(buf, 1, sizeof(buf), f);
    sdUnlock();
    if (n <= 0) break;
    for (int i = 0; i < n; i++) {
      char c = buf[i];
      if (c != '\n') { if (ll < (int)sizeof(line) - 1) line[ll++] = c; else bad++; continue; }
      line[ll] = 0;
      ll = 0;
      lines++;
      if (line[0] == '#') {
        if (!strncmp(line, "# sderr", 7)) sderr++;
        if (!strncmp(line, "# end", 5) || !strncmp(line, "# restart", 9)) endTag = true;
        const char* w = strstr(line, " written=");
        if (!strncmp(line, "# stats", 7) && w) lastWritten = strtoul(w + 9, nullptr, 10);
        continue;
      }
      // <ms>.<us>,7AA,s,8,XX XX XX XX XX XX XX XX
      char* p1 = strchr(line, ',');
      char* p4 = p1 ? strchr(p1 + 1, ',') : nullptr;
      p4 = p4 ? strchr(p4 + 1, ',') : nullptr;
      p4 = p4 ? strchr(p4 + 1, ',') : nullptr;
      if (!p1 || !p4) { bad++; continue; }
      frames++;
      if (strncmp(p1 + 1, "7AA,", 4)) continue;
      // "XX XX XX XX XX XX XX XX", little-endian counter (no sscanf: ~10x faster)
      const char* h = p4 + 1;
      uint64_t seq = 0;
      bool ok = strlen(h) == 23;
      for (int k = 0; ok && k < 8; k++) {
        int v = 0;
        for (int m = 0; m < 2; m++) {
          char c = h[k * 3 + m];
          v <<= 4;
          if (c >= '0' && c <= '9') v |= c - '0';
          else if (c >= 'A' && c <= 'F') v |= c - 'A' + 10;
          else ok = false;
        }
        if (k < 7 && h[k * 3 + 2] != ' ') ok = false;
        seq |= (uint64_t)v << (8 * k);
      }
      if (!ok) { bad++; continue; }
      if (havePrev) {
        if (seq == prev + 1) {}
        else if (seq <= prev) repeats++;
        else { if (!gaps) firstGapAt = prev; gaps++; }
      }
      prev = seq;
      havePrev = true;
    }
  }
  sdLock();
  fclose(f);
  sdUnlock();
  if (ll) bad++;  // a last line without its newline
  snprintf(s_checkResult, sizeof(s_checkResult),
      "S%lu: lines=%lu frames=%lu stats_written=%lu match=%d bad=%lu gaps=%lu first_gap_after=%llu repeats=%lu sderr=%lu end=%d",
      (unsigned long)s_checkFile, (unsigned long)lines, (unsigned long)frames, (unsigned long)lastWritten,
      frames == lastWritten ? 1 : 0, (unsigned long)bad, (unsigned long)gaps, (unsigned long long)firstGapAt,
      (unsigned long)repeats, (unsigned long)sderr, endTag ? 1 : 0);
  Serial.println(s_checkResult);
  s_checkRunning = false;
  vTaskDelete(nullptr);
}

int sniffCheck(uint32_t fileNo, char* out, int cap)
{
  if (fileNo) {
    if (s_checkRunning) return snprintf(out, cap, "busy");
    if (s_fileOk && fileNo == s_fileNo) return snprintf(out, cap, "S%lu is open - stop sniff first", (unsigned long)fileNo);
    s_checkFile = fileNo;
    s_checkRunning = true;
    snprintf(s_checkResult, sizeof(s_checkResult), "S%lu: running", (unsigned long)fileNo);
    xTaskCreate(checkTask, "sniffchk", 4096, nullptr, 1, nullptr);
  }
  return snprintf(out, cap, "%s", s_checkResult);
}

// SNIFF_FAKEOLD=<days>: a small sniff file whose header says it started
// <days> ago - the 60-day purge test without waiting 60 days
int sniffFakeOld(uint32_t days, char* out, int cap)
{
  uint32_t n = 0;
  nvs_get_u32(nvs, "SNIFF_NO", &n);
  sdLock();
  File dir = SD.open(SNIFF_DIR);
  File f;
  while (dir && (f = dir.openNextFile())) {
    const char* fn = f.name();
    const char* p = strrchr(fn, '/');
    if (p) fn = p + 1;
    if (fn[0] == 'S' && (uint32_t)atoi(fn + 1) > n) n = atoi(fn + 1);
    f.close();
  }
  n++;
  nvs_set_u32(nvs, "SNIFF_NO", n);
  nvs_commit(nvs);
  char path[32];
  snprintf(path, sizeof(path), SNIFF_DIR "/S%lu.LOG", (unsigned long)n);
  File w = SD.open(path, FILE_WRITE);
  bool ok = (bool)w;
  if (ok) {
    w.printf("# telelogger sniff (TWAI listen-only) vehicle=FAKEOLD fw=%s boot=%lu\n", FW_BUILD_STR,
             (unsigned long)txqBootId());
    w.printf("# clock unix=%lu at_ms=0\n", (unsigned long)(time(nullptr) - (time_t)days * 86400));
    w.printf("# end\n");
    w.close();
  }
  sdUnlock();
  return snprintf(out, cap, "%s %s", path, ok ? "created" : "FAILED");
}
#endif
