// SD writer - see sdwriter.h.
#include "sdwriter.h"
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>

void sdLock();    // telestore.cpp - every other SD user takes the same lock
void sdUnlock();
void txqSetFile(uint32_t fileId);
void txqSdConfirmed(const uint32_t* ts, int n);
// sniff.cpp
int sniffFill(char* buf, int cap, uint32_t* frames, bool* last);
void sniffConfirmed(uint32_t frames);
void sniffNoteSdError(const char* op, int err);
void sniffSdDead(uint32_t lostFrames);

// a file that is only ever appended to, with its confirmed length
struct WFile {
  int fd = -1;
  char path[48];              // with the mount point
  uint32_t confirmed = 0;     // bytes the card has confirmed (write + fsync)
};

static uint32_t s_errors, s_lostBytes, s_lastErrMs, s_lastErrErrno;
static char s_lastErrOp[12] = "-";
static volatile bool s_dead;
#ifdef TEST_SD_FAULT
static uint32_t s_faultEvery, s_faultCount;
#endif

static void noteError(const char* op, int err)
{
  s_errors++;
  s_lastErrMs = millis();
  s_lastErrErrno = err;
  strncpy(s_lastErrOp, op, sizeof(s_lastErrOp) - 1);
  Serial.printf("[SDW] %s failed errno=%d\n", op, err);
}

// The card's answer for n bytes at the end of the file: write all + fsync.
// On any failure the file is reopened and cut back to the confirmed length,
// so the next try writes the same bytes at the same place.
static bool wfAppend(WFile& f, const char* a, int na, const char* b, int nb, const char* opTag)
{
  if (f.fd < 0) {
    f.fd = open(f.path, O_WRONLY);
    if (f.fd < 0) { noteError("reopen", errno); return false; }
    if (ftruncate(f.fd, f.confirmed) != 0 || lseek(f.fd, f.confirmed, SEEK_SET) != (off_t)f.confirmed) {
      noteError("truncate", errno);
      close(f.fd);
      f.fd = -1;
      return false;
    }
  }
  bool ok = true;
  const char* op = opTag;
#ifdef TEST_SD_FAULT
  if (s_faultEvery && ++s_faultCount % s_faultEvery == 0) {
    // bench: half the bytes reach the file, then "the card fails"
    if (write(f.fd, a, na / 2) < 0) {}
    fsync(f.fd);
    ok = false;
    op = "fault";
    errno = EIO;
  }
#endif
  if (ok && na > 0 && write(f.fd, a, na) != na) { ok = false; op = "write"; }
  if (ok && nb > 0 && write(f.fd, b, nb) != nb) { ok = false; op = "write"; }
  if (ok && fsync(f.fd) != 0) { ok = false; op = "fsync"; }
  if (ok) {
    f.confirmed += na + nb;
    return true;
  }
  noteError(op, errno);
  close(f.fd);  // the next try reopens and truncates
  f.fd = -1;
  return false;
}

static bool wfOpen(WFile& f, const char* path, bool mustBeNew)
{
  snprintf(f.path, sizeof(f.path), SDW_MOUNT "%s", path);
  // ESP-IDF FAT: O_CREAT alone means "open existing" - O_APPEND or O_EXCL is what creates
  f.fd = open(f.path, O_WRONLY | O_CREAT | (mustBeNew ? O_EXCL : O_APPEND), 0644);
  if (f.fd < 0) { noteError("open", errno); return false; }
  off_t end = lseek(f.fd, 0, SEEK_END);
  f.confirmed = end > 0 ? (uint32_t)end : 0;
  return true;
}

// a failure time for the dead check: never later than any later millis()
// (2026-10-12 bench: "millis() | 1" was 1 ms in the future for an even
// millis(), now - since wrapped to 4294967295 and a working card was given up)
static uint32_t failTime() { uint32_t t = millis(); return t ? t : 1; }

static void wfClose(WFile& f)
{
  if (f.fd >= 0) close(f.fd);
  f.fd = -1;
}

// ------------------------------------------------------------ telemetry ---
static char* s_ring;                  // PSRAM, SDW_DATA_RING bytes
static volatile uint32_t s_head;      // bytes ever appended (ring index = % size)
static volatile uint32_t s_tail;      // bytes confirmed on the card
static SemaphoreHandle_t s_ringMux;   // producers (any task); never held across SD
#define MARKS 512
static struct { uint32_t off, ts; } s_marks[MARKS];
static volatile uint32_t s_markHead, s_markTail;
static WFile s_data;
static volatile bool s_dataOpen;
static uint32_t s_lastDataWrite;
static uint32_t s_dataFailSince;      // first failed write of the current bytes, 0 = none
static volatile bool s_kick;

// sniff
static char* s_block;                 // PSRAM, SDW_SNIFF_BLOCK bytes
static int s_blockLen;
static uint32_t s_blockFrames;
static bool s_blockLast;
static WFile s_sniff;
static volatile bool s_sniffOpen;
static uint32_t s_sniffFailSince;

// requests from other tasks, done by the writer task
enum { REQ_NONE, REQ_DATA_OPEN, REQ_DATA_CLOSE, REQ_SNIFF_OPEN, REQ_SNIFF_CLOSE };
static volatile int s_req;
static char s_reqPath[40];
static volatile int s_reqResult;      // 1 ok, -1 failed
static SemaphoreHandle_t s_reqMux;    // one request at a time
static TaskHandle_t s_task;

void sdwKick()
{
  s_kick = true;
  if (s_task) xTaskNotifyGive(s_task);
}

void sdwDataWrite(const char* s, int n)
{
  if (!s_ring || !s_dataOpen || s_dead || n <= 0) {
    if (n > 0 && s_dataOpen) s_lostBytes += n;
    return;
  }
  xSemaphoreTake(s_ringMux, portMAX_DELAY);
  uint32_t h = s_head;
  if (h - s_tail + n > SDW_DATA_RING) {
    s_lostBytes += n;  // the card has not taken the last ~15 min: counted, never overwritten
  } else {
    for (int i = 0; i < n; i++) s_ring[(h + i) % SDW_DATA_RING] = s[i];
    s_head = h + n;
  }
  xSemaphoreGive(s_ringMux);
}

void sdwDataMark(uint32_t ts)
{
  if (!s_dataOpen || s_dead) return;
  xSemaphoreTake(s_ringMux, portMAX_DELAY);
  if (s_markHead - s_markTail >= MARKS) s_markTail++;  // oldest: never confirmed, counted lost if dropped
  s_marks[s_markHead % MARKS].off = s_head;
  s_marks[s_markHead % MARKS].ts = ts;
  s_markHead++;
  xSemaphoreGive(s_ringMux);
}

// writer task: everything appended so far, one write+fsync
static void dataWrite()
{
  uint32_t h = s_head, t = s_tail;
  if (h == t) return;
  uint32_t a = t % SDW_DATA_RING, n = h - t;
  uint32_t na = n, nb = 0;
  if (a + n > SDW_DATA_RING) { na = SDW_DATA_RING - a; nb = n - na; }
  s_lastDataWrite = millis();
  sdLock();
  bool ok = wfAppend(s_data, s_ring + a, na, s_ring, nb, "write");
  sdUnlock();
  if (!ok) {
    if (!s_dataFailSince) s_dataFailSince = failTime();
    char ev[64];
    int l = snprintf(ev, sizeof(ev), "FE,SD_ERR t=%lu op=%s errno=%lu\n", (unsigned long)s_lastErrMs,
                     s_lastErrOp, (unsigned long)s_lastErrErrno);
    sdwDataWrite(ev, l);  // goes out with the retry
    return;
  }
  s_dataFailSince = 0;
  xSemaphoreTake(s_ringMux, portMAX_DELAY);
  s_tail = h;
  uint32_t ts[32];
  int nts = 0;
  while (s_markTail != s_markHead && (int32_t)(s_marks[s_markTail % MARKS].off - h) <= 0) {
    ts[nts++] = s_marks[s_markTail % MARKS].ts;
    s_markTail++;
    if (nts == 32) {
      xSemaphoreGive(s_ringMux);
      txqSdConfirmed(ts, nts);
      nts = 0;
      xSemaphoreTake(s_ringMux, portMAX_DELAY);
    }
  }
  xSemaphoreGive(s_ringMux);
  if (nts) txqSdConfirmed(ts, nts);
}

static void sniffWrite()
{
  if (!s_blockLen) {
    uint32_t frames = 0;
    bool last = false;
    s_blockLen = sniffFill(s_block, SDW_SNIFF_BLOCK, &frames, &last);
    s_blockFrames = frames;
    s_blockLast = last;
    if (!s_blockLen && !last) return;
  }
  if (s_blockLen) {
    sdLock();
    bool ok = wfAppend(s_sniff, s_block, s_blockLen, nullptr, 0, "write");
    sdUnlock();
    if (!ok) {
      if (!s_sniffFailSince) s_sniffFailSince = failTime();
      sniffNoteSdError(s_lastErrOp, s_lastErrErrno);  // a "# sderr" line in a later block
      return;  // the same block again next time
    }
    s_sniffFailSince = 0;
    sniffConfirmed(s_blockFrames);
  }
  s_blockLen = 0;
  s_blockFrames = 0;
  if (s_blockLast) {
    s_blockLast = false;
    sdLock();
    wfClose(s_sniff);
    sdUnlock();
    s_sniffOpen = false;
  }
}

// giving up a file: one more try to cut it back to its confirmed length, so
// no half-written block of the failed write stays in it
static void wfCutAndClose(WFile& f)
{
  if (f.fd < 0 && f.path[0]) f.fd = open(f.path, O_WRONLY);
  if (f.fd >= 0 && ftruncate(f.fd, f.confirmed) != 0) noteError("truncate", errno);
  wfClose(f);
}

static void goDead()
{
  // the card does not take anything any more: telemetry without an SD copy
  // (the delivery queue's no-SD rules), sniff off; every byte is counted
  s_dead = true;
  xSemaphoreTake(s_ringMux, portMAX_DELAY);
  s_lostBytes += s_head - s_tail;
  s_tail = s_head;
  s_markTail = s_markHead;
  xSemaphoreGive(s_ringMux);
  txqSetFile(0);
  if (s_sniffOpen) {
    sniffSdDead(s_blockFrames);  // the unwritten block and everything still queued
    s_blockLen = 0;
    s_blockFrames = 0;
    s_blockLast = false;
    sdLock();
    wfCutAndClose(s_sniff);
    sdUnlock();
    s_sniffOpen = false;
  }
  sdLock();
  wfCutAndClose(s_data);
  sdUnlock();
  Serial.println("[SDW] card not answering - telemetry continues without SD copy, sniff stopped");
}

static void handleRequest()
{
  int r = s_req;
  if (r == REQ_NONE) return;
  int result = -1;
  sdLock();
  if (r == REQ_DATA_OPEN) {
    wfClose(s_data);
    xSemaphoreTake(s_ringMux, portMAX_DELAY);
    s_head = s_tail = 0;
    s_markHead = s_markTail = 0;
    xSemaphoreGive(s_ringMux);
    s_dataFailSince = 0;
    if (wfOpen(s_data, s_reqPath, false)) { s_dataOpen = true; result = 1; }
  } else if (r == REQ_DATA_CLOSE) {
    wfClose(s_data);
    s_dataOpen = false;
    result = 1;
  } else if (r == REQ_SNIFF_OPEN) {
    wfClose(s_sniff);
    s_blockLen = 0;
    s_blockLast = false;
    s_sniffFailSince = 0;
    if (wfOpen(s_sniff, s_reqPath, true)) { s_sniffOpen = true; result = 1; }
  }
  sdUnlock();
  s_reqResult = result;
  s_req = REQ_NONE;
}

static void writerTask(void*)
{
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
    handleRequest();
    if (s_dead) continue;
    // 1. telemetry first
    if (s_dataOpen && s_head != s_tail &&
        (s_kick || millis() - s_lastDataWrite >= SDW_SYNC_MS)) {
      s_kick = false;
      dataWrite();
    }
    // 2. sniff, a block at a time, telemetry checked in between; after a
    //    failure a 200 ms pause before the same block is tried again
    for (int i = 0; s_sniffOpen && i < 8; i++) {
      if (s_sniffFailSince && millis() - s_lastErrMs < 200) break;
      sniffWrite();
      if (s_sniffFailSince) break;
      if (!s_blockLen && !s_blockLast) break;    // nothing more queued
      if (s_dataOpen && s_head != s_tail && millis() - s_lastDataWrite >= SDW_SYNC_MS) break;
    }
    uint32_t now = millis();
    if ((s_dataFailSince && now - s_dataFailSince > SDW_DEAD_MS) ||
        (s_sniffFailSince && now - s_sniffFailSince > SDW_DEAD_MS)) goDead();
  }
}

void sdwStart()
{
  if (s_task) return;
  s_ring = (char*)ps_malloc(SDW_DATA_RING);
  s_block = (char*)ps_malloc(SDW_SNIFF_BLOCK);
  if (!s_ring) s_ring = (char*)malloc(SDW_DATA_RING / 8);  // no PSRAM: never reached on this board
  if (!s_block) s_block = (char*)malloc(4096);
  s_ringMux = xSemaphoreCreateMutex();
  s_reqMux = xSemaphoreCreateMutex();
  xTaskCreate(writerTask, "sdwriter", 6144, nullptr, 3, &s_task);
}

static bool request(int r, const char* path, uint32_t timeoutMs)
{
  if (!s_task) return false;
  xSemaphoreTake(s_reqMux, portMAX_DELAY);
  if (path) strncpy(s_reqPath, path, sizeof(s_reqPath) - 1);
  s_reqResult = 0;
  s_req = r;
  xTaskNotifyGive(s_task);
  for (uint32_t t = millis(); s_req != REQ_NONE && millis() - t < timeoutMs;) delay(5);
  bool ok = s_reqResult == 1;
  xSemaphoreGive(s_reqMux);
  return ok;
}

bool sdwDataOpen(const char* path)
{
  if (s_dead) return false;
  return request(REQ_DATA_OPEN, path, 3000);
}

bool sdwDataSync(uint32_t timeoutMs)
{
  if (!s_dataOpen || s_dead) return s_head == s_tail;
  sdwKick();
  for (uint32_t t = millis(); s_head != s_tail && millis() - t < timeoutMs;) {
    delay(10);
    if (s_head != s_tail) sdwKick();
  }
  return s_head == s_tail;
}

void sdwDataClose(uint32_t timeoutMs)
{
  if (!s_dataOpen) return;
  if (!sdwDataSync(timeoutMs)) {
    xSemaphoreTake(s_ringMux, portMAX_DELAY);
    s_lostBytes += s_head - s_tail;
    s_tail = s_head;
    xSemaphoreGive(s_ringMux);
  }
  request(REQ_DATA_CLOSE, nullptr, 2000);
}

bool sdwDataDead() { return s_dead; }

bool sdwSniffOpen(const char* path)
{
  if (s_dead) return false;
  return request(REQ_SNIFF_OPEN, path, 3000);
}

void sdwSniffClose(uint32_t timeoutMs)
{
  // sniff.cpp has asked sniffFill() for its last bytes; wait for them
  for (uint32_t t = millis(); s_sniffOpen && millis() - t < timeoutMs;) {
    if (s_task) xTaskNotifyGive(s_task);
    delay(20);
  }
}

bool sdwSniffOpenNow() { return s_sniffOpen; }

int sdwStatus(char* buf, int size)
{
  return snprintf(buf, size,
      "SDW dead=%d data=%d pending=%lu lost=%lu errors=%lu last_err=%s/%lu at_ms=%lu sniff=%d psram_free=%u",
      s_dead ? 1 : 0, s_dataOpen ? 1 : 0, (unsigned long)(s_head - s_tail), (unsigned long)s_lostBytes,
      (unsigned long)s_errors, s_lastErrOp, (unsigned long)s_lastErrErrno, (unsigned long)s_lastErrMs,
      s_sniffOpen ? 1 : 0, (unsigned)ESP.getFreePsram());
}

#ifdef TEST_SD_FAULT
void sdwTestFault(uint32_t everyN)
{
  s_faultEvery = everyN;
  s_faultCount = 0;
}
#endif
