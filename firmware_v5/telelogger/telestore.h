#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>

class CStorage;

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

class SDLogger : public FileLogger {
public:
    bool init();
    uint32_t begin();
    void flush();
    // Purge oldest log files when SD card is >= 80% full.
    // Deletes files in ascending ID order (oldest first) until at least 20%
    // of the total SD capacity is free.  The file currently being written
    // (m_id) is never deleted.  Returns true if any files were removed.
    // Named purgeOldFiles() (not purge()) to avoid shadowing CStorage::purge().
    bool purgeOldFiles();
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
