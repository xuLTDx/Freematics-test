#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>

class CStorage;

// the SD card's one lock (telestore.cpp): every SD user outside the SD writer
// task (downloads, OTA files, purge, SD replay) takes it around its SD calls
void sdLock();
void sdUnlock();

// An SD file whose every call takes the SD lock - for the OTA files in
// telelogger.ino (2026-10-12: they were the last SD users without it). Only
// the SD call itself is locked, never the network I/O between them.
class LockedFile {
public:
    LockedFile() {}
    LockedFile(const File& f) : m_f(f) {}
    explicit operator bool() { sdLock(); bool b = (bool)m_f; sdUnlock(); return b; }
    void close() { sdLock(); m_f.close(); sdUnlock(); }
    size_t size() { sdLock(); size_t n = m_f.size(); sdUnlock(); return n; }
    void flush() { sdLock(); m_f.flush(); sdUnlock(); }
    int read(uint8_t* buf, size_t n) { sdLock(); int r = m_f.read(buf, n); sdUnlock(); return r; }
    size_t write(const uint8_t* buf, size_t n) { sdLock(); size_t r = m_f.write(buf, n); sdUnlock(); return r; }
    size_t readBytesUntil(char t, char* buf, size_t n) { sdLock(); size_t r = m_f.readBytesUntil(t, buf, n); sdUnlock(); return r; }
    size_t print(const char* s) { sdLock(); size_t r = m_f.print(s); sdUnlock(); return r; }
    size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)))
    {
        char b[160];
        va_list ap;
        va_start(ap, fmt);
        int n = vsnprintf(b, sizeof(b), fmt, ap);
        va_end(ap);
        return n > 0 ? write((const uint8_t*)b, n < (int)sizeof(b) ? n : sizeof(b) - 1) : 0;
    }
private:
    File m_f;
};
inline LockedFile sdOpen(const char* path, const char* mode) { sdLock(); LockedFile f(SD.open(path, mode)); sdUnlock(); return f; }
inline bool sdExists(const char* path) { sdLock(); bool b = SD.exists(path); sdUnlock(); return b; }
inline bool sdRemove(const char* path) { sdLock(); bool b = SD.remove(path); sdUnlock(); return b; }

class CStorage {
public:
    virtual bool init() { return true; }
    virtual void uninit() {}
    virtual void log(uint16_t pid, uint8_t values[], uint8_t count);
    virtual void log(uint16_t pid, uint16_t values[], uint8_t count);
    virtual void log(uint16_t pid, uint32_t values[], uint8_t count);
    virtual void log(uint16_t pid, int32_t values[], uint8_t count);
    virtual void log(uint16_t pid, float values[], uint8_t count, const char* fmt = "%f");
    virtual void timestamp(uint32_t ts);
    virtual void purge() { m_samples = 0; }
    virtual uint16_t samples() { return m_samples; }
    virtual void dispatch(const char* buf, byte len);
protected:
    byte checksum(const char* data, int len);
    virtual void header(const char* devid) {}
    virtual void tailer() {}
    int m_samples = 0;
    char m_delimiter = ':';
};

class CStorageRAM: public CStorage {
public:
    void init(char* cache, unsigned int cacheSize)
    {
        m_cacheSize = cacheSize;
        m_cache = cache;
    }
    void uninit()
    {
        if (m_cache) {
            delete m_cache;
            m_cache = 0;
            m_cacheSize = 0;
        }
    }
    void purge() { m_cacheBytes = 0; m_samples = 0; }
    unsigned int length() { return m_cacheBytes; }
    char* buffer() { return m_cache; }
    void dispatch(const char* buf, byte len);
    void header(const char* devid);
    void tailer();
    void untailer();
protected:
    unsigned int m_cacheSize = 0;
    unsigned int m_cacheBytes = 0;
    char* m_cache = 0;
};

// 2026-09-25: every FileLogger operation is serialized by one recursive
// mutex. The main task writes records while the telemetry task writes events
// and flush()es (SDLogger::flush() closes and reopens the file) - unguarded,
// a record write hitting a closed file ended SD logging for the rest of the
// boot (both drives on 2026-09-25 lost their SD log after 2-4 minutes;
// reproduced on the bench within 1 s with events every 20 ms).
class FileLogger : public CStorage {
public:
    FileLogger() { m_delimiter = ','; }
    virtual void dispatch(const char* buf, byte len);
    virtual uint32_t size() { return m_size; }
    virtual void end()
    {
        lock();
        m_file.close();
        m_id = 0;
        m_size = 0;
        unlock();
    }
    virtual void flush()
    {
        lock();
        m_file.flush();
        unlock();
    }
    uint32_t writeErrors() { return m_writeErrors; }
    // Write a human-readable diagnostic/event line to the log file.
    // The line is formatted as "FE,<text>" (PID 0xFE is reserved for events
    // and is never queried by handlerLogData(), so it is silently skipped
    // during data queries while remaining fully visible in the raw file view).
    void logEvent(const char* text);
protected:
    void lock();
    void unlock();
    virtual bool reopen() { return false; }  // SDLogger: close + open for append
    int getFileID(File& root);
    uint32_t m_writeErrors = 0;
    uint32_t m_dataTime = 0;
    uint32_t m_dataCount = 0;
    uint32_t m_size = 0;
    uint32_t m_id = 0;
    File m_file;
};

// 2026-10-12: the SD copy goes through the SD writer task (sdwriter.cpp):
// dispatch() only puts the line into its RAM ring (never waits for the card),
// the writer writes and fsyncs it and confirms the records to the delivery
// queue. flush() = write soon, sync() = wait until the card has everything.
class SDLogger : public FileLogger {
public:
    bool init();
    uint32_t begin(uint32_t lastId = 0);  // lastId: last file id used (0 = list /DATA)
    void dispatch(const char* buf, byte len);
    void flush();
    bool sync(uint32_t timeoutMs);
    void end();
    // Purge oldest log files when the SD card is >= pct % full, oldest first,
    // until 20 % is free. Never deleted: the file being written (m_id) and
    // every file >= keepFrom (records not confirmed by the server yet - the
    // delivery queue replays them from SD; txqOldestFileNeeded()). dryRun:
    // only list them in report. Returns true if any were (or would be) removed.
    // Named purgeOldFiles() (not purge()) to avoid shadowing CStorage::purge().
    bool purgeOldFiles(uint32_t keepFrom, int pct = 80, bool dryRun = false, char* report = nullptr, int reportCap = 0);
protected:
    bool reopen();
};

class SPIFFSLogger : public FileLogger {
public:
    bool init();
    uint32_t begin();
private:
    void purge();
};
