#include <FreematicsPlus.h>
#include "telestore.h"
#include "sdwriter.h"

void CStorage::log(uint16_t pid, uint8_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, (unsigned int)values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%u", (unsigned int)values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, uint16_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, (unsigned int)values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%u", (unsigned int)values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, uint32_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%u", values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, int32_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%d", pid, m_delimiter, values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%d", values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, float values[], uint8_t count, const char* fmt)
{
    char buf[256];
    char *p = buf + snprintf(buf, sizeof(buf), "%X%c", pid, m_delimiter);
    for (byte m = 0; m < count && (p - buf) < sizeof(buf) - 3; m++) {
        if (m > 0) *(p++) = ';';
        int l = snprintf(p, sizeof(buf) - (p - buf), fmt, values[m]);
        char *q = strchr(p, '.');
        if (q && atoi(q + 1) == 0) {
            *q = 0;
            if (*p == '-' && *(p + 1) == '0') {
                *p = '0';
                *(++p) = 0;
            } else {
                p = q;
            }
        } else {
            p += l;
        }
    }
    dispatch(buf, (int)(p - buf));
}

void CStorage::timestamp(uint32_t ts)
{
    log(PID_TIMESTAMP, &ts, 1);
}

void CStorage::dispatch(const char* buf, byte len)
{
    // output data via serial
    Serial.write((uint8_t*)buf, len);
    Serial.write(' ');
    m_samples++;
}

byte CStorage::checksum(const char* data, int len)
{
    byte sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return sum;
}

void CStorageRAM::dispatch(const char* buf, byte len)
{
    // reserve some space for checksum
    int remain = m_cacheSize - m_cacheBytes - len - 3;
    if (remain < 0) {
        // m_cache full
        return;
    }
    // store data in m_cache
    memcpy(m_cache + m_cacheBytes, buf, len);
    m_cacheBytes += len;
    m_cache[m_cacheBytes++] = ',';
    m_samples++;
}

void CStorageRAM::header(const char* devid)
{
    m_cacheBytes = sprintf(m_cache, "%s#", devid);
}

void CStorageRAM::tailer()
{
    if (m_cache[m_cacheBytes - 1] == ',') m_cacheBytes--;
    m_cacheBytes += sprintf(m_cache + m_cacheBytes, "*%X", (unsigned int)checksum(m_cache, m_cacheBytes));
}

void CStorageRAM::untailer()
{
    char *p = strrchr(m_cache, '*');
    if (p) {
        *p = ',';
        m_cacheBytes = p + 1 - m_cache;
    }
}

static SemaphoreHandle_t s_logMutex = 0;  // one logger instance per firmware

// the delivery queue reads records back from SD (txqueue.cpp): same mutex
void sdLock()
{
  if (!s_logMutex) s_logMutex = xSemaphoreCreateRecursiveMutex();
  if (s_logMutex) xSemaphoreTakeRecursive(s_logMutex, portMAX_DELAY);
}

void sdUnlock()
{
  if (s_logMutex) xSemaphoreGiveRecursive(s_logMutex);
}

void FileLogger::lock()
{
    if (!s_logMutex) s_logMutex = xSemaphoreCreateRecursiveMutex();
    if (s_logMutex) xSemaphoreTakeRecursive(s_logMutex, portMAX_DELAY);
}

void FileLogger::unlock()
{
    if (s_logMutex) xSemaphoreGiveRecursive(s_logMutex);
}

void FileLogger::dispatch(const char* buf, byte len)
{
    lock();
    if (m_id == 0) {
        unlock();
        return;
    }
    if (m_file.write((uint8_t*)buf, len) == len) {
        m_file.write('\n');
        m_size += (len + 1);
    } else {
        // Reopen and retry instead of giving up for the rest of the boot (the
        // old behaviour: end() - every later record and event silently lost).
        m_writeErrors++;
        Serial.printf("[SD] write error #%u - reopening\n", (unsigned)m_writeErrors);
        if (reopen() && m_file.write((uint8_t*)buf, len) == len) {
            m_file.write('\n');
            m_size += (len + 1);
            char ev[40];
            int n = snprintf(ev, sizeof(ev), "FE,SD_WRITE_ERROR n=%u", (unsigned)m_writeErrors);
            if (m_file.write((uint8_t*)ev, n) == (size_t)n) {
                m_file.write('\n');
                m_size += (n + 1);
            }
        }
        // still failing: this line is lost, the next write tries again
    }
    unlock();
}

void FileLogger::logEvent(const char* text)
{
    // Prefix with PID 0xFE so the line is properly formatted for CSV yet
    // never matched by normal data queries (handlerLogData skips unknown PIDs).
    // The raw /api/log file view shows the full human-readable text.
    // "FE," + up to 245 chars of text (the delivery queue's status line is
    // ~200); longer text is not written at all.
    char buf[250];  // byte length: dispatch() takes at most 255
    int n = snprintf(buf, sizeof(buf), "FE,%s", text);
    if (n > 0 && n < (int)sizeof(buf)) {
        dispatch(buf, (byte)n);
    }
}

int FileLogger::getFileID(File& root)
{
    if (root) {
        File file;
        int id = 0;
        while(file = root.openNextFile()) {
            char *p = strrchr(file.name(), '/');
            unsigned int n = atoi(p ? p + 1 : file.name());
            if (n > id) id = n;
        }
        return id + 1;
    } else {
        return 0;
    }
}

// A reset in the middle of an SD write sometimes leaves the card (which
// keeps power - no SD power switch on this board) not answering CMD0 for a
// while: "NO SD CARD" for the whole session. Confirmed live that MISO is
// idle (0xFF, not busy, not driven by the OBD co-processor sharing the bus)
// and that the card does come back given time - three retries within
// ~1.4 s failed, a later one succeeded. So retry with growing gaps (~6 s
// total, only on the failing path). The Stop Tran token (0xFD) + 80 clocks
// per the SD SPI protocol end any half-finished multi-block write first.
static void sdUnstick()
{
    SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, LOW);
    SPI.transfer(0xFD);
    for (uint32_t t = millis(); millis() - t < 500; ) {
        if (SPI.transfer(0xFF) == 0xFF) break;  // busy (DO held low) is over
    }
    digitalWrite(PIN_SD_CS, HIGH);
    for (int i = 0; i < 10; i++) SPI.transfer(0xFF);  // 80 clocks, deselected
    SPI.endTransaction();
}

bool SDLogger::init()
{
    // creates the mutex here, in the main task, before STATE_STORAGE_READY
    // lets any other task write
    lock();
    unlock();
    SPI.begin();
    // 10 open files (default 5): the SD writer's telemetry + sniff file,
    // downloads over HTTP, SD replay and OTA can be open at the same time
    bool ok = SD.begin(PIN_SD_CS, SPI, SPI_FREQ, SDW_MOUNT, 10);
    uint32_t gap = 200;
    for (int attempt = 1; !ok && attempt <= 6; attempt++, gap *= 2) {
        Serial.printf("SD:init failed, recovery attempt %d\n", attempt);
        SD.end();
        sdUnstick();
        delay(gap > 3200 ? 3200 : gap);
        ok = SD.begin(PIN_SD_CS, SPI, SPI_FREQ, SDW_MOUNT, 10);
    }
    if (ok) sdwStart();
    if (ok) {
        unsigned int total = SD.totalBytes() >> 20;
        unsigned int used = SD.usedBytes() >> 20;
        Serial.print("SD:");
        Serial.print(total);
        Serial.print(" MB total, ");
        Serial.print(used);
        Serial.println(" MB used");
        return true;
    } else {
        Serial.println("NO SD CARD");
        return false;
    }
}

uint32_t SDLogger::begin(uint32_t lastId)
{
    lock();
    char path[24];
    if (!SD.exists("/DATA")) SD.mkdir("/DATA");
    if (lastId) {
        // 2026-09-27: listing /DATA (900+ files) took 14.4 s of every boot,
        // all of it before the first record. The caller remembers the last
        // id (NVS); only check that the next one is really free.
        m_id = lastId + 1;
        for (;;) {
            sprintf(path, "/DATA/%u.CSV", m_id);
            if (!SD.exists(path)) break;
            m_id++;
        }
    } else {
        File root = SD.open("/DATA");
        m_id = getFileID(root);
        if (m_id == 0) m_id = 1;
    }
    unlock();  // the SD writer takes the SD lock itself
    sprintf(path, "/DATA/%u.CSV", m_id);
    Serial.print("File: ");
    Serial.println(path);
    if (!sdwDataOpen(path)) {
        Serial.println("File error");
        m_id = 0;
    }
    m_size = 0;
    m_dataCount = 0;
    return m_id;
}

// a line into the SD writer's ring (it adds nothing on its own: the "\n"
// is ours); the writer confirms it with write + fsync
void SDLogger::dispatch(const char* buf, byte len)
{
    if (m_id == 0) return;
    char line[258];
    memcpy(line, buf, len);
    line[len] = '\n';
    sdwDataWrite(line, len + 1);
    m_size += len + 1;
}

bool SDLogger::reopen() { return m_id != 0; }  // the writer reopens on its own

void SDLogger::flush() { sdwKick(); }

bool SDLogger::sync(uint32_t timeoutMs) { return m_id == 0 || sdwDataSync(timeoutMs); }

void SDLogger::end()
{
    if (m_id) sdwDataClose(5000);
    m_id = 0;
    m_size = 0;
}

bool SDLogger::purgeOldFiles(uint32_t keepFrom, int pct, bool dryRun, char* report, int reportCap)
{
    // Check if cleanup is needed: only purge when SD is >= pct % full.
    sdLock();
    uint64_t total = SD.totalBytes();
    uint64_t used = total ? SD.usedBytes() : 0;
    if (total == 0 || used * 100 < total * (uint64_t)pct) { sdUnlock(); return false; }
    uint64_t needFree = total / 5;  // until 20 % is free
    uint64_t freeNow = total - used;

    // Build a sorted list of file IDs under /DATA (ascending = oldest first).
    // We collect up to 512 IDs to avoid dynamic allocation; in practice the
    // directory never has that many files before space is exhausted.
    static const int MAX_IDS = 512;
    static uint32_t ids[MAX_IDS];
    static uint32_t sizes[MAX_IDS];
    int count = 0;

    File root = SD.open("/DATA");
    if (!root) { sdUnlock(); return false; }
    File file;
    while ((file = root.openNextFile()) && count < MAX_IDS) {
        const char* name = file.name();
        // name may include the path prefix depending on ESP32 Arduino version
        const char* slash = strrchr(name, '/');
        const char* base = slash ? slash + 1 : name;
        unsigned int n = (unsigned int)atoi(base);
        if (n != 0) { ids[count] = n; sizes[count] = file.size(); count++; }
        file.close();
    }
    root.close();

    if (count == 0) { sdUnlock(); return false; }

    // Sort ascending (insertion sort - small count, runs on MCU).
    for (int i = 1; i < count; i++) {
        uint32_t key = ids[i], sz = sizes[i];
        int j = i - 1;
        while (j >= 0 && ids[j] > key) { ids[j + 1] = ids[j]; sizes[j + 1] = sizes[j]; j--; }
        ids[j + 1] = key;
        sizes[j + 1] = sz;
    }

    // Delete the oldest 20% of files (at least 1) that nothing needs any more.
    int toDelete = count / 5;
    if (toDelete < 1) toDelete = 1;
    bool any = false;
    char path[24];
    int rn = 0;
    if (report && reportCap) rn = snprintf(report, reportCap, "used=%u%% keepFrom=%u %s:",
        (unsigned)(used * 100 / total), (unsigned)keepFrom, dryRun ? "would delete" : "deleted");
    for (int i = 0; i < count && toDelete > 0; i++) {
        if (ids[i] == m_id) continue;                 // never the file being written
        if (keepFrom && ids[i] >= keepFrom) break;     // still to be sent: never
        sprintf(path, "/DATA/%u.CSV", ids[i]);
        if (dryRun || SD.remove(path)) {
            Serial.printf("[SD] %s %s\n", dryRun ? "Would purge" : "Purged", path);
            if (report && rn < reportCap - 12) rn += snprintf(report + rn, reportCap - rn, " %u", (unsigned)ids[i]);
            any = true;
            toDelete--;
            freeNow += sizes[i];
        }
        if (freeNow >= needFree) break;  // freed enough
    }
    sdUnlock();
    return any;
}

bool SPIFFSLogger::init()
{
    bool mounted = SPIFFS.begin();
    if (!mounted) {
        Serial.println("Formatting SPIFFS...");
        mounted = SPIFFS.begin(true);
    }
    if (mounted) {
        Serial.print("SPIFFS:");
        Serial.print(SPIFFS.totalBytes());
        Serial.print(" bytes total, ");
        Serial.print(SPIFFS.usedBytes());
        Serial.println(" bytes used");
    } else {
        Serial.println("No SPIFFS");
    }
    return mounted;
}

uint32_t SPIFFSLogger::begin()
{
    File root = SPIFFS.open("/");
    m_id = getFileID(root);
    char path[24];
    sprintf(path, "/DATA/%u.CSV", m_id);
    Serial.print("File: ");
    Serial.println(path);
    m_file = SPIFFS.open(path, FILE_WRITE);
    if (!m_file) {
        Serial.println("File error");
        m_id = 0;
    }
    m_dataCount = 0;
    return m_id;
}

void SPIFFSLogger::purge()
{
    // remove oldest file when unused space is insufficient
    File root = SPIFFS.open("/");
    File file;
    int idx = 0;
    while(file = root.openNextFile()) {
        if (!strncmp(file.name(), "/DATA/", 6)) {
            unsigned int n = atoi(file.name() + 6);
            if (n != 0 && (idx == 0 || n < idx)) idx = n;
        }
    }
    if (idx) {
        m_file.close();
        char path[32];
        sprintf(path, "/DATA/%u.CSV", idx);
        SPIFFS.remove(path);
        Serial.print(path);
        Serial.println(" removed");
        sprintf(path, "/DATA/%u.CSV", m_id);
        m_file = SPIFFS.open(path, FILE_APPEND);
        if (!m_file) m_id = 0;
    }
}
