// Delivery queue - see txqueue.h for the design.
#include <Arduino.h>
#include <SD.h>
#include <nvs.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "txqueue.h"

extern nvs_handle_t nvs;
void sdLock();    // telestore.cpp - the SD card is shared with the logger
void sdUnlock();

// ---------------------------------------------------------------------------
// The Arduino core runs a PSRAM test at every boot (psramInit() ->
// testSPIRAM(), a weak hook) that writes every 8th word of the whole PSRAM -
// measured 2026-09-26: 12 % of the reserve destroyed by each restart, 0 %
// with the test skipped. Test the top 64 KB instead: the queue reserve is the
// first PSRAM allocation, at the bottom. A failing chip is still caught here
// and by esp_spiram_init() itself (the core then runs without PSRAM).
extern "C" bool testSPIRAM(void)
{
  volatile uint32_t* p = (volatile uint32_t*)(0x3F800000 + 0x400000 - 0x10000);
  const int n = 0x10000 / 4;
  for (int i = 0; i < n; i++) p[i] = (uint32_t)i * 2654435761u;
  for (int i = 0; i < n; i++) {
    if (p[i] != (uint32_t)i * 2654435761u) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
#define Q_MAGIC   0x51584654  // "TFXQ"
#define E_MAGIC   0x52434552  // "RECR"
#define W_MAGIC   0x50415257  // "WRAP"
#define Q_VERSION 1
#define ST_PENDING  0
#define ST_INFLIGHT 1
#define ST_ACKED    2

struct QHdr {
  uint32_t magic, version, addr, cap, head, tail, count, lastPos, crc;
};
struct QEnt {
  uint32_t magic, boot, ts, file;
  uint16_t len;
  uint8_t state, pad;
  uint32_t packet, crc;
};
struct SdCursor {  // NVS blob TXQ_CUR: the oldest record not confirmed yet
  uint32_t magic, file, boot, ts, pos, sdActive;
};

static SemaphoreHandle_t s_mux;
static uint8_t* s_mem;
static uint32_t s_cap;
static bool s_psram;
static QHdr* H;
#define DSTART ((uint32_t)sizeof(QHdr))

static uint32_t s_boot, s_file, s_lastTs;
static uint32_t s_packetNo;
static uint32_t s_lost;           // records dropped without any other copy
static uint32_t s_lastThin;       // parked thinning (no SD, queue >= 80 %)
static uint32_t s_lastPriority;
static uint32_t s_lastSave;
static SdCursor s_saved;

// SD replay (records the queue does not hold any more)
static bool s_sdActive;
static uint32_t s_sdFile, s_sdBoot, s_sdTs, s_sdPos;
struct Flight { uint32_t no, sent; bool used, sd, sdEof, sdDone; uint32_t sdNextTs, sdNextPos; };
static Flight s_flight[TXQ_MAX_INFLIGHT];
static uint32_t s_lastBuiltNo;

static void lock() { if (s_mux) xSemaphoreTakeRecursive(s_mux, portMAX_DELAY); }
static void unlock() { if (s_mux) xSemaphoreGiveRecursive(s_mux); }

static uint32_t crc32b(uint32_t crc, const uint8_t* p, int n)
{
  crc = ~crc;
  while (n--) {
    crc ^= *p++;
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
  }
  return ~crc;
}
static uint32_t hdrCrc() { return crc32b(0, (const uint8_t*)H, offsetof(QHdr, crc)); }
static void hdrSeal() { H->crc = hdrCrc(); }
static QEnt* ent(uint32_t pos) { return (QEnt*)(s_mem + pos); }
static uint32_t entCrc(const QEnt* e)
{
  uint32_t c = crc32b(0, (const uint8_t*)&e->boot, 12);
  c = crc32b(c, (const uint8_t*)&e->len, 2);
  return crc32b(c, (const uint8_t*)(e + 1), e->len);
}
static uint32_t entSize(uint32_t len) { return (sizeof(QEnt) + len + 3) & ~3u; }
// position of the entry at pos, following an explicit or implicit wrap
static uint32_t norm(uint32_t pos)
{
  if (pos + sizeof(QEnt) > s_cap || ent(pos)->magic == W_MAGIC) return DSTART;
  return pos;
}
static uint32_t nextPos(uint32_t pos) { return pos + entSize(ent(pos)->len); }

static void resetRing()
{
  memset(H, 0, sizeof(QHdr));
  H->magic = Q_MAGIC;
  H->version = Q_VERSION;
  H->addr = (uint32_t)s_mem;
  H->cap = s_cap;
  H->head = H->tail = H->lastPos = DSTART;
  hdrSeal();
}

void txqEarlyInit()
{
  s_mux = xSemaphoreCreateRecursiveMutex();
  s_mem = (uint8_t*)heap_caps_malloc(TXQ_RESERVE_BYTES, MALLOC_CAP_SPIRAM);
  s_psram = s_mem != nullptr;
  s_cap = TXQ_RESERVE_BYTES;
  if (!s_mem) {
    // no PSRAM (chip failed): a small queue in internal RAM, never a crash
    s_mem = (uint8_t*)malloc(TXQ_FALLBACK_BYTES);
    s_cap = TXQ_FALLBACK_BYTES;
  }
  H = (QHdr*)s_mem;
}

// Walk what survived a restart; stop at the first damaged entry.
static bool validateRing()
{
  if (!s_psram || H->magic != Q_MAGIC || H->version != Q_VERSION || H->addr != (uint32_t)s_mem ||
      H->cap != s_cap || H->crc != hdrCrc() || H->head >= s_cap || H->tail >= s_cap) {
    return false;
  }
  uint32_t pos = H->tail, n = 0, last = H->tail;
  for (uint32_t i = 0; i < H->count; i++) {
    pos = norm(pos);
    QEnt* e = ent(pos);
    if (e->magic != E_MAGIC || e->len > 2048 || pos + entSize(e->len) > s_cap || e->crc != entCrc(e)) break;
    if (e->state == ST_INFLIGHT) e->state = ST_PENDING;
    last = pos;
    pos = nextPos(pos);
    n++;
  }
  if (n != H->count) Serial.printf("[TXQ] reserve damaged after %u of %u records\n", (unsigned)n, (unsigned)H->count);
  H->count = n;
  H->head = n ? pos : DSTART;
  if (!n) H->tail = DSTART;
  H->lastPos = last;
  hdrSeal();
  return true;
}

void txqBegin(uint32_t bootId)
{
  lock();
  s_boot = bootId;
  bool kept = validateRing();
  if (!kept) resetRing();
  size_t len = sizeof(s_saved);
  bool haveSaved = nvs_get_blob(nvs, "TXQ_CUR", &s_saved, &len) == ESP_OK &&
                   len == sizeof(s_saved) && s_saved.magic == Q_MAGIC;
  s_sdActive = false;
  if (haveSaved && s_saved.file && (s_saved.sdActive || !kept)) {
    // the reserve did not survive (power loss) or an SD replay was running:
    // continue from the oldest record that was not confirmed
    s_sdActive = true;
    s_sdFile = s_saved.file;
    s_sdBoot = s_saved.boot;
    s_sdTs = s_saved.ts;
    s_sdPos = s_saved.pos;
  }
  Serial.printf("[TXQ] %s %u KB, %u records kept%s, SD replay %s\n", s_psram ? "PSRAM" : "internal RAM",
      (unsigned)(s_cap >> 10), (unsigned)H->count, kept ? "" : " (reserve lost)",
      s_sdActive ? "from saved cursor" : "off");
  unlock();
}

void txqSetFile(uint32_t fileId) { lock(); s_file = fileId; unlock(); }
uint32_t txqBootId() { return s_boot; }
bool txqUsingPsram() { return s_psram; }

static uint32_t usedBytes()
{
  if (!H->count) return 0;
  if (H->head > H->tail) return H->head - H->tail;
  return (s_cap - H->tail) + (H->head - DSTART);
}
uint32_t txqUsedPercent() { lock(); uint32_t p = usedBytes() * 100 / (s_cap - DSTART); unlock(); return p; }

static void dropTail()
{
  uint32_t pos = norm(H->tail);
  QEnt* e = ent(pos);
  if (e->state != ST_ACKED) {
    if (e->file) {
      if (!s_sdActive) {  // the SD copy takes over from here
        s_sdActive = true;
        s_sdFile = e->file;
        s_sdBoot = e->boot;
        s_sdTs = e->ts;
        s_sdPos = 0;
        Serial.printf("[TXQ] full - SD replay from file %u ts %u\n", (unsigned)e->file, (unsigned)e->ts);
      }
    } else {
      s_lost++;
    }
  }
  H->tail = nextPos(pos);
  if (--H->count == 0) H->head = H->tail = H->lastPos = DSTART;
}

// advance the tail over records the server has confirmed
static void releaseAcked()
{
  while (H->count && ent(norm(H->tail))->state == ST_ACKED) {
    uint32_t pos = norm(H->tail);
    H->tail = nextPos(pos);
    if (--H->count == 0) H->head = H->tail = H->lastPos = DSTART;
  }
}

static bool findSpace(uint32_t need, uint32_t& at)
{
  if (!H->count) {
    H->head = H->tail = H->lastPos = DSTART;
    at = DSTART;
    return DSTART + need <= s_cap;
  }
  if (H->head > H->tail) {  // free: [head, end) and [start, tail)
    if (H->head + need <= s_cap) { at = H->head; return true; }
    if (DSTART + need < H->tail) {
      if (H->head + sizeof(QEnt) <= s_cap) ent(H->head)->magic = W_MAGIC;
      at = DSTART;
      return true;
    }
    return false;
  }
  // wrapped (head <= tail, head == tail = full): free is [head, tail)
  if (H->head + need < H->tail) { at = H->head; return true; }
  return false;
}

// only what the user listed (2026-09-26): time code (PID 0 + GPS time/date),
// position, speed, heading, and in an engine event record the start/stop and
// its time - plus the no-fix flag that says the position is the box's last fix
static int essentials(const char* rec, int len, char* out, int cap)
{
  static const char* keep[] = {"0", "10", "11", "A", "B", "D", "E", "385", "380", "381"};
  int o = 0;
  const char* p = rec;
  const char* end = rec + len;
  while (p < end) {
    const char* q = (const char*)memchr(p, ',', end - p);
    if (!q) q = end;
    const char* colon = (const char*)memchr(p, ':', q - p);
    if (colon) {
      for (const char* k : keep) {
        size_t kl = strlen(k);
        if ((size_t)(colon - p) == kl && !memcmp(p, k, kl) && o + (q - p) + 1 < cap) {
          if (o) out[o++] = ',';
          memcpy(out + o, p, q - p);
          o += q - p;
          break;
        }
      }
    }
    p = q + 1;
  }
  return o;
}

static float fieldValue(const char* rec, int len, const char* key, float def)
{
  char pat[8];
  int pl = snprintf(pat, sizeof(pat), ",%s:", key);
  for (int i = 0; i + pl <= len; i++) {
    if (!memcmp(rec + i, pat, pl)) return atof(rec + i + pl);
  }
  return def;
}

bool txqPush(uint32_t ts, const char* rec, int len)
{
  if (len <= 0) return false;
  char tmp[512];
  lock();
  s_lastTs = ts;
  if (!s_file) {
    // no SD copy: the queue is all there is - make it last
    uint32_t used = usedBytes() * 100 / (s_cap - DSTART);
    bool event = fieldValue(rec, len, "380", -1) >= 0;
    if (used >= 80 && !event) {
      // thinned always (driving or parked): one record per TXQ_THIN_MS
      if (s_lastThin && ts - s_lastThin < TXQ_THIN_MS) { unlock(); return true; }
      s_lastThin = ts;
    }
    if (used >= 50) {  // engine events too (start/stop kept), but never thinned
      len = essentials(rec, len, tmp, sizeof(tmp));
      rec = tmp;
    }
  }
  if (len > 2048) len = 2048;
  uint32_t need = entSize(len), at;
  if (need + DSTART > s_cap) { unlock(); return false; }
  while (!findSpace(need, at)) {
    if (!H->count) { unlock(); return false; }
    dropTail();
  }
  QEnt* e = ent(at);
  e->boot = s_boot;
  e->ts = ts;
  e->file = s_file;
  e->len = len;
  e->state = ST_PENDING;
  e->pad = 0;
  e->packet = 0;
  memcpy(e + 1, rec, len);
  e->crc = entCrc(e);
  e->magic = E_MAGIC;
  H->head = at + need;
  H->lastPos = at;
  H->count++;
  hdrSeal();
  unlock();
  return true;
}

// ---------------------------------------------------------------------------
static int flightsUsed()
{
  int n = 0;
  for (auto& f : s_flight) if (f.used) n++;
  return n;
}
static Flight* newFlight()
{
  for (auto& f : s_flight) {
    if (!f.used) { memset(&f, 0, sizeof(f)); f.used = true; return &f; }
  }
  return nullptr;
}

static void expire()
{
  uint32_t now = millis();
  for (auto& f : s_flight) {
    if (!f.used || now - f.sent < TXQ_ACK_TIMEOUT_MS) continue;
    f.used = false;  // SD replay simply restarts from its cursor
    if (!f.sd) {
      uint32_t pos = H->tail;
      for (uint32_t i = 0; i < H->count; i++) {
        pos = norm(pos);
        QEnt* e = ent(pos);
        if (e->state == ST_INFLIGHT && e->packet == f.no) e->state = ST_PENDING;
        pos = nextPos(pos);
      }
    }
  }
}

static int header(char* out, int cap, const char* devid, uint32_t boot, uint32_t no)
{
  return snprintf(out, cap, "%s#%X:%u,%X:%u", devid, PID_TX_BOOT, (unsigned)boot, PID_TX_PACKET, (unsigned)no);
}
static int tailer(char* out, int n, int cap)
{
  uint8_t sum = 0;
  for (int i = 0; i < n; i++) sum += (uint8_t)out[i];
  return n + snprintf(out + n, cap - n, "*%X", sum);
}

// ---- SD replay: read records back from /DATA/<file>.CSV ----
static bool sdStopKey(uint32_t file, uint32_t ts)
{
  // stop where the queue takes over (its oldest record that has an SD copy)
  uint32_t pos = H->tail;
  for (uint32_t i = 0; i < H->count; i++) {
    pos = norm(pos);
    QEnt* e = ent(pos);
    if (e->file) return file > e->file || (file == e->file && ts >= e->ts);
    pos = nextPos(pos);
  }
  return false;
}

static int buildSdPacket(char* out, int cap, const char* devid, Flight* f)
{
  char path[24];
  sprintf(path, "/DATA/%u.CSV", (unsigned)s_sdFile);
  sdLock();
  File file = SD.open(path, FILE_READ);
  if (!file) {
    sdUnlock();
    // deleted (SD space cleanup) or not written: go on with the next file
    if (s_sdFile < s_file) { s_sdFile++; s_sdTs = 0; s_sdPos = 0; s_sdBoot = 0; }
    else s_sdActive = false;
    return 0;
  }
  if (s_sdPos) file.seek(s_sdPos);
  char line[160];
  static char recBuf[TXQ_PACKET_MAX + 700];  // a record is never cut: start one only below TXQ_PACKET_MAX
  int recLen = 0;         // records accepted into the packet
  int cur = -1;           // start of the record being read in recBuf, -1 = skipping
  bool curHasData = false;
  uint32_t curTs = 0, curPos = 0;
  bool eof = false, done = false;
  uint32_t nextTs = 0, nextPos = 0;
  while (true) {
    uint32_t linePos = file.position();
    int n = file.available() ? file.readBytesUntil('\n', line, sizeof(line) - 1) : -1;
    if (n < 0) { eof = true; break; }
    line[n] = 0;
    char* comma = strchr(line, ',');
    if (!comma) continue;
    *comma = 0;
    const char* key = line;
    const char* val = comma + 1;
    if (!strcmp(key, "FE")) {
      if (!strncmp(val, "BOOTID=", 7)) s_sdBoot = strtoul(val + 7, 0, 10);
      continue;
    }
    if (!strcmp(key, "0")) {
      uint32_t ts = strtoul(val, 0, 10);
      if (cur >= 0 && !curHasData) recLen = cur;  // an empty record: leave it out
      cur = -1;  // the previous record (if any) is complete
      if (!s_sdBoot) continue;  // file without a boot id (old firmware): not replayable
      if (ts < s_sdTs) continue;
      if (sdStopKey(s_sdFile, ts)) { done = true; nextTs = ts; nextPos = linePos; break; }
      if (recLen >= TXQ_PACKET_MAX) { nextTs = ts; nextPos = linePos; break; }
      cur = recLen;
      curHasData = false;
      if (recLen) recBuf[recLen++] = ',';
      recLen += snprintf(recBuf + recLen, sizeof(recBuf) - recLen, "0:%u", (unsigned)ts);
      curTs = ts;
      curPos = linePos;
      continue;
    }
    if (cur < 0) continue;
    curHasData = true;
    int kl = strlen(key), vl = strlen(val);
    if (recLen + kl + vl + 2 < (int)sizeof(recBuf)) {
      recBuf[recLen++] = ',';
      memcpy(recBuf + recLen, key, kl); recLen += kl;
      recBuf[recLen++] = ':';
      memcpy(recBuf + recLen, val, vl); recLen += vl;
    }
  }
  if (cur >= 0 && !curHasData) recLen = cur;
  (void)curTs; (void)curPos;
  file.close();
  sdUnlock();
  if (!recLen) {
    if (done || (eof && s_sdFile >= s_file)) { s_sdActive = false; return 0; }
    if (eof) { s_sdFile++; s_sdTs = 0; s_sdPos = 0; s_sdBoot = 0; }
    return 0;
  }
  f->sd = true;
  f->sdEof = eof;
  f->sdDone = done;
  f->sdNextTs = nextTs;
  f->sdNextPos = nextPos;
  int o = header(out, cap, devid, s_sdBoot, f->no);
  out[o++] = ',';
  memcpy(out + o, recBuf, recLen);
  o += recLen;
  return tailer(out, o, cap);
}

#ifdef TEST_TXQ
volatile bool g_txqTestPause = false;  // bench: an outage without touching the server
#endif

int txqNextPacket(char* out, int cap, const char* devid)
{
#ifdef TEST_TXQ
  if (g_txqTestPause) return 0;
#endif
  lock();
  expire();
  if (flightsUsed() >= TXQ_MAX_INFLIGHT) { unlock(); return 0; }
  Flight* f = newFlight();
  f->no = ++s_packetNo;
  f->sent = millis();
  int n = 0;

  // 1. the newest record first when a backlog is being worked off - the map
  //    and a theft/tow position must not wait behind hours of old data
  if (H->count > 1 && millis() - s_lastPriority > 5000) {
    QEnt* last = ent(H->lastPos);
    QEnt* first = ent(norm(H->tail));
    if (last->state == ST_PENDING && last->ts - first->ts > 20000 && last->boot == first->boot) {
      s_lastPriority = millis();
      n = header(out, cap, devid, last->boot, f->no);
      out[n++] = ',';
      memcpy(out + n, last + 1, last->len);
      n += last->len;
      last->state = ST_INFLIGHT;
      last->packet = f->no;
    }
  }
  // 2. SD replay (older than anything in the queue), one packet at a time
  if (!n && s_sdActive) {
    bool sdBusy = false;
    for (auto& g : s_flight) if (g.used && g.sd) sdBusy = true;
    if (!sdBusy) n = buildSdPacket(out, cap, devid, f);
    if (n) { s_lastBuiltNo = f->no; unlock(); return n; }
    if (!f->sd && s_sdActive && sdBusy) { f->used = false; unlock(); return 0; }
    if (s_sdActive) { f->used = false; unlock(); return 0; }  // wait for its ACK / next file
  }
  // 3. queued records, oldest first, same boot in one packet
  if (!n) {
    uint32_t pos = H->tail, boot = 0;
    for (uint32_t i = 0; i < H->count; i++) {
      pos = norm(pos);
      QEnt* e = ent(pos);
      if (e->state == ST_PENDING) {
        if (!n) {
          boot = e->boot;
          n = header(out, cap, devid, boot, f->no);
        } else if (e->boot != boot) {
          break;
        }
        if (n + 1 + e->len + 8 > cap || n + 1 + e->len > TXQ_PACKET_MAX + 64) break;
        out[n++] = ',';
        memcpy(out + n, e + 1, e->len);
        n += e->len;
        e->state = ST_INFLIGHT;
        e->packet = f->no;
      }
      pos = nextPos(pos);
    }
  }
  if (!n) { f->used = false; s_packetNo--; unlock(); return 0; }
  n = tailer(out, n, cap);
  s_lastBuiltNo = f->no;
  unlock();
  return n;
}

void txqSendFailed()
{
  lock();
  for (auto& f : s_flight) {
    if (f.used && f.no == s_lastBuiltNo) f.sent = millis() - TXQ_ACK_TIMEOUT_MS;  // expire now
  }
  expire();
  unlock();
}

void txqOnAck(uint32_t packetNo)
{
  lock();
  for (auto& f : s_flight) {
    if (!f.used || f.no != packetNo) continue;
    f.used = false;
    if (f.sd) {
      if (f.sdDone) {
        s_sdActive = false;
        Serial.println("[TXQ] SD replay done");
      } else if (f.sdEof) {
        if (s_sdFile >= s_file) { s_sdActive = false; Serial.println("[TXQ] SD replay done"); }
        else { s_sdFile++; s_sdTs = 0; s_sdPos = 0; s_sdBoot = 0; }
      } else {
        s_sdTs = f.sdNextTs;
        s_sdPos = f.sdNextPos;
      }
    } else {
      uint32_t pos = H->tail;
      for (uint32_t i = 0; i < H->count; i++) {
        pos = norm(pos);
        QEnt* e = ent(pos);
        if (e->state == ST_INFLIGHT && e->packet == packetNo) e->state = ST_ACKED;
        pos = nextPos(pos);
      }
      releaseAcked();
      hdrSeal();
    }
  }
  unlock();
}

uint32_t txqPending()
{
  lock();
  uint32_t n = 0, pos = H->tail;
  for (uint32_t i = 0; i < H->count; i++) {
    pos = norm(pos);
    if (ent(pos)->state != ST_ACKED) n++;
    pos = nextPos(pos);
  }
  if (s_sdActive) n++;
  unlock();
  return n;
}

void txqSave(bool force)
{
  if (!force && millis() - s_lastSave < 30000) return;
  s_lastSave = millis();
  lock();
  SdCursor c = {Q_MAGIC, 0, 0, 0, 0, 0};
  if (s_sdActive) {
    c = {Q_MAGIC, s_sdFile, s_sdBoot, s_sdTs, s_sdPos, 1};
  } else {
    uint32_t pos = H->tail;
    for (uint32_t i = 0; i < H->count; i++) {
      pos = norm(pos);
      QEnt* e = ent(pos);
      if (e->state != ST_ACKED && e->file) { c = {Q_MAGIC, e->file, e->boot, e->ts, 0, 0}; break; }
      pos = nextPos(pos);
    }
    if (!c.file && s_file) c = {Q_MAGIC, s_file, s_boot, s_lastTs + 1, 0, 0};
  }
  unlock();
  if (memcmp(&c, &s_saved, sizeof(c))) {
    s_saved = c;
    nvs_set_blob(nvs, "TXQ_CUR", &c, sizeof(c));
    nvs_commit(nvs);
  }
}

#ifdef TEST_TXQ
void txqTestLoseReserve()
{
  txqSave(true);
  lock();
  H->magic = 0;  // next boot: "reserve lost", continue from the saved cursor
  unlock();
}
#endif

void txqResendFrom(uint32_t fileId)
{
  lock();
  s_sdActive = fileId != 0;
  s_sdFile = fileId;
  s_sdBoot = 0;  // read from the file's BOOTID line
  s_sdTs = 0;
  s_sdPos = 0;
  for (auto& f : s_flight) if (f.used && f.sd) f.used = false;
  unlock();
  txqSave(true);
}

void txqStatus(char* buf, int size)
{
  lock();
  snprintf(buf, size, "TXQ %s used=%u%% records=%u pending=%u inflight=%d sd=%s%u/%u lost=%u",
      s_psram ? "psram" : "ram", (unsigned)(usedBytes() * 100 / (s_cap - DSTART)), (unsigned)H->count,
      (unsigned)txqPending(), flightsUsed(), s_sdActive ? "on:" : "off:", (unsigned)s_sdFile, (unsigned)s_sdTs,
      (unsigned)s_lost);
  unlock();
}
