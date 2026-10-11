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

extern nvs_handle_t nvs;
extern const char FW_BUILD_STR[];
uint32_t txqBootId();
void sdLock();    // telestore.cpp - the SD card is shared with the logger
void sdUnlock();

#define CAN_TX_PIN GPIO_NUM_26
#define CAN_RX_PIN GPIO_NUM_34
#define SNIFF_DIR "/SNIFF"
#define LIVE_FRAMES 200
#define QUEUE_FRAMES 8192          // receive task -> SD writer (PSRAM)
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

static File s_file;
static bool s_fileOk;
static uint32_t s_fileNo;
static uint32_t s_lastStats;
static volatile bool s_closeFile;
static String s_vehicle = "UNSPECIFIED";

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

static void pauseRx()
{
  s_pauseRx = true;
  for (int i = 0; i < 100 && !s_rxPaused; i++) delay(2);
}

// ----------------------------------------------------------------- SD ----
static int formatFrame(char* out, int cap, const Frame& f)
{
  int n = snprintf(out, cap, "%lu.%03lu,%lX,%c,%u,", (unsigned long)(f.us / 1000),
                   (unsigned long)(f.us % 1000), (unsigned long)f.id, f.ext ? 'x' : 's', f.dlc);
  for (int i = 0; i < f.dlc && n < cap - 4; i++) n += snprintf(out + n, cap - n, i ? " %02X" : "%02X", f.data[i]);
  if (f.tx && n < cap - 4) n += snprintf(out + n, cap - n, ",TX");
  return n;
}

static void writeLine(const char* line)
{
  if (s_fileOk) s_file.println(line);
}

static void statsLine(char* line, int cap)
{
  twai_status_info_t st = {};
  if (s_canRunning) twai_get_status_info(&st);
  snprintf(line, cap, "# stats t=%lu unix=%lu frames=%lu dropped=%lu rx_missed=%lu rx_overrun=%lu bus_err=%lu speed=%lu%s",
           (unsigned long)millis(), (unsigned long)time(nullptr), (unsigned long)s_frames, (unsigned long)s_dropped,
           (unsigned long)st.rx_missed_count, (unsigned long)st.rx_overrun_count, (unsigned long)st.bus_error_count,
           (unsigned long)s_speeds[s_speedIdx], s_speedFound ? "k" : "k (probing)");
}

// new session file /SNIFF/S<n>.LOG, n above every existing file (the card
// may hold HexSniff's own S1..S100 - never overwrite one: found 2026-10-11,
// the first session here overwrote S1.LOG). Nothing is ever deleted: a long
// VCDS session is ~10-25 MB, the card 32 GB (user, 2026-10-11).
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
  n++;
  nvs_set_u32(nvs, "SNIFF_NO", n);
  nvs_commit(nvs);
  s_fileNo = n;
  snprintf(path, sizeof(path), SNIFF_DIR "/S%lu.LOG", (unsigned long)n);
  s_file = SD.open(path, FILE_WRITE);
  s_fileOk = (bool)s_file;
  if (s_fileOk) {
    time_t now = time(nullptr);
    char line[200];
    snprintf(line, sizeof(line), "# telelogger sniff (TWAI listen-only) vehicle=%s fw=%s boot=%lu",
             s_vehicle.c_str(), FW_BUILD_STR, (unsigned long)txqBootId());
    writeLine(line);
    // maps the <ms> of the frame lines to the wall clock (0 = clock not set yet)
    snprintf(line, sizeof(line), "# clock unix=%lu at_ms=%lu", (unsigned long)(now > 1735689600 ? now : 0),
             (unsigned long)(esp_timer_get_time() / 1000));
    writeLine(line);
    writeLine("# format: <ms>.<us>,<ID hex>,<s=11-bit|x=29-bit>,<dlc>,<data hex>[,TX]");
    s_file.flush();
  }
  sdUnlock();
  Serial.printf("[SNIFF] file %s %s\n", path, s_fileOk ? "open" : "FAILED");
}

// drains the queue to SD; closes the file when asked (stop)
static void writerTask(void*)
{
  char line[200];
  uint32_t lastFlush = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!s_fileOk) {
      s_qTail = s_qHead;  // nothing to write to: the live view still works
      continue;
    }
    sdLock();
    int n = 0;
    while (s_qTail != s_qHead && n < 2048) {
      Frame f = s_queue[s_qTail];
      s_qTail = (s_qTail + 1) % s_queueCap;
      formatFrame(line, sizeof(line), f);
      writeLine(line);
      n++;
    }
    if (millis() - s_lastStats >= STATS_EVERY_MS) {
      s_lastStats = millis();
      statsLine(line, sizeof(line));
      writeLine(line);
      Serial.println(line);
    }
    if (millis() - lastFlush > 1000 || s_closeFile) {
      lastFlush = millis();
      s_file.flush();
    }
    if (s_closeFile && s_qTail == s_qHead) {
      statsLine(line, sizeof(line));
      writeLine(line);
      writeLine("# end");
      s_file.close();
      s_fileOk = false;
      s_closeFile = false;
    }
    sdUnlock();
  }
}

// ------------------------------------------------------------- control ---
static void saveMinutes()
{
  nvs_set_u16(nvs, "SNIFF_MIN", s_active ? s_minLeft : 0);
  nvs_commit(nvs);
  s_sinceSave = 0;
}

static void startNow(uint16_t minutes)
{
  if (!s_queue) {
    s_queue = (Frame*)ps_malloc(sizeof(Frame) * QUEUE_FRAMES);
    s_queueCap = QUEUE_FRAMES;
    if (!s_queue) {  // no PSRAM: a smaller queue
      s_queue = (Frame*)malloc(sizeof(Frame) * 512);
      s_queueCap = 512;
    }
    s_canMux = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(rxTask, "sniffrx", 3072, nullptr, 10, nullptr, 0);
    xTaskCreate(writerTask, "sniffsd", 4096, nullptr, 1, nullptr);
  }
  canOdoEnd();  // the controller is ours now
  s_qHead = s_qTail = 0;
  s_frames = s_dropped = 0;
  s_liveCount = s_liveHead = 0;
  s_speedIdx = 0;
  s_speedFound = false;
  s_closeFile = false;
  openFile();
  xSemaphoreTake(s_canMux, portMAX_DELAY);
  bool ok = canStart(TWAI_MODE_LISTEN_ONLY);
  xSemaphoreGive(s_canMux);
  s_probeStart = millis();
  s_probeFrames = 0;
  s_lastStats = millis();
  s_minLeft = minutes;
  s_minuteMs = millis();
  s_active = true;
  saveMinutes();
  Serial.printf("[SNIFF] ON %u min, CAN listen-only %s\n", minutes, ok ? "started" : "FAILED");
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
  s_closeFile = true;
  for (int i = 0; i < 50 && s_closeFile && s_fileOk; i++) delay(20);  // writer closes it
  saveMinutes();
  canOdoBegin();  // back to the odometer reads
  Serial.println("[SNIFF] OFF");
}

static void speedTick()
{
  if (s_speedFound) return;
  if (s_frames > s_probeFrames) {
    s_speedFound = true;
    char line[64];
    snprintf(line, sizeof(line), "# bus speed %lu kbit/s", (unsigned long)s_speeds[s_speedIdx]);
    sdLock();
    writeLine(line);
    sdUnlock();
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
        startNow(s_reqMin);
        ev = SNIFF_EV_STARTED;
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
      "{\"on\":%d,\"min\":%u,\"file\":%lu,\"sd\":%d,\"speed\":%lu,\"found\":%d,\"frames\":%lu,"
      "\"dropped\":%lu,\"missed\":%lu,\"overrun\":%lu,\"buserr\":%lu,\"vehicle\":\"%s\"}",
      s_active ? 1 : 0, (unsigned)sniffMinutesLeft(), (unsigned long)s_fileNo, s_fileOk ? 1 : 0,
      (unsigned long)s_speeds[s_speedIdx], s_speedFound ? 1 : 0, (unsigned long)s_frames,
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
    sdLock();
    writeLine(line.c_str());
    sdUnlock();
  }
  return true;
}
