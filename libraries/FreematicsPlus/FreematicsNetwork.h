/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
*************************************************************************/

#ifndef FREEMATICS_NETWORK
#define FREEMATICS_NETWORK

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "FreematicsBase.h"

#define XBEE_BAUDRATE 115200
#define HTTP_CONN_TIMEOUT 5000
// TLS handshake over cellular can take longer than a data exchange; allow
// up to 15 seconds before declaring the connection attempt failed.
#define HTTP_TLS_HANDSHAKE_TIMEOUT 15000
// 2026-09-22: SIM7670/SIM7672-family AT+CCHOPEN's documented Max Response
// Time is 120000ms (SIMCom SSL Application Note V1.00 - same figure as
// AT+CCHSTART/CCHSSLCFG/CCHSEND/CCHRECV/CCHCLOSE), and AT+CSSLCFG's
// "negotiatetime" (TLS handshake timeout) parameter defaults to 300s -
// confirmed live that HTTP_TLS_HANDSHAKE_TIMEOUT's 15s was simply too short
// on this chip family (CCHOPEN produced zero response at all within 15s,
// not an error - it was still legitimately waiting). Not raising the
// shared HTTP_TLS_HANDSHAKE_TIMEOUT itself, since that value is already
// proven correct in production for CELL_SIM7600's own CCHOPEN wait - this
// is a separate, SIM7670-specific ceiling instead. 60s, not the full
// documented 120s, as a middle ground: long enough for a genuinely slow
// cellular TLS handshake, short enough that a truly failed attempt doesn't
// block the caller for two full minutes.
#define CCHOPEN_TIMEOUT_SIM7670 60000
// Minimum expected duration (ms) for a TLS 1.2 handshake over a cellular
// link (TCP 3-way + TLS exchange with a remote server).  +CCHOPEN:0,0
// arriving faster than this is suspicious and may indicate a plain TCP
// connection that silently bypassed the SSL layer.  Used as a hard-reject
// threshold in open(): any +CCHOPEN response arriving in less than this many
// milliseconds causes the session to be closed and open() to return false.
// Some SIM7600E-H firmware revisions silently fall back to plain TCP even
// with the explicit 5-param ssl_ctx_id form; accepting such sessions leads to
// repeated "400 The plain HTTP request was sent to HTTPS port" responses.
//
// 150 ms is chosen to be safely above the plain-TCP fast-path (typically
// <100 ms: modem acknowledges AT+CCHOPEN + TCP connect without TLS overhead)
// while accepting legitimate TLS handshakes to cloud endpoints such as
// hooks.nabu.casa (AWS ALB, EU) which complete in ~150–400 ms over cellular.
// The previous 300 ms threshold was too aggressive: it falsely rejected real
// TLS connections to hooks.nabu.casa when combined with Connection: close
// (which forces a new TLS handshake for every packet), breaking all cellular
// telemetry delivery.
#define MIN_TLS_HANDSHAKE_MS 150

// Runtime cellular debug flag.  Set to 1 via NVS key CELL_DEBUG (written by
// the HA config/options flow) to enable verbose cellular diagnostic logging:
// TX-Preview, hex-dump, AT+CCHSTATUS? and per-packet "Incoming data" lines.
// Controlled at runtime so no firmware recompile is needed.  Default is 0.
extern uint8_t cellNetDebug;

#define RECV_BUF_SIZE 512

// Minimum total free DRAM required before attempting a new TLS handshake.
// A TLS session (mbedTLS context, handshake buffers, etc.) uses roughly
// 20–50 KB spread across many small allocations.  When the heap is already
// fragmented by prior TLS teardown/creation cycles, individual allocations
// inside mbedtls_ssl_setup() etc. fail with MBEDTLS_ERR_SSL_ALLOC_FAILED
// (-32512), breaking ALL subsequent HTTPS connections until the WiFi stack
// is restarted.  This threshold gives a conservative safety margin so that
// WifiHTTP::open() declines the connect attempt early (without tearing down
// the existing session) and the caller can request a WiFi restart instead.
// 38 KB is chosen as the minimum largest-contiguous-block size: mbedTLS
// allocates ~2×17 KB TLS record buffers inside mbedtls_ssl_setup() and these
// must fit in contiguous DRAM.  Values below ~34 KB will consistently fail;
// the 38 KB margin accounts for additional smaller context allocations.
// The previous 40 KB threshold was too close to the ~40-41 KB max contiguous
// block available after a clean WiFi restart (no active TLS sessions), causing
// Guard 2 in WifiHTTP::open() to fire immediately after WiFi reconnect and
// preventing telemetry from re-establishing its TLS session.  38 KB still
// provides a 4 KB margin above the ~34 KB minimum and allows the post-close
// heap (~40-41 KB on this hardware) to pass the guard reliably.
// ESP.getMaxAllocHeap() returns the largest single contiguous free block,
// which is the correct metric for fragmentation detection (total free heap
// can be high while no individual block is large enough for TLS).
#define TLS_MIN_FREE_HEAP (38 * 1024)

typedef enum {
  METHOD_GET = 0,
  METHOD_POST,
} HTTP_METHOD;

typedef enum {
    HTTP_DISCONNECTED = 0,
    HTTP_CONNECTED,
    HTTP_SENT,
    HTTP_ERROR,
} HTTP_STATES;

typedef struct {
    float lat;
    float lng;
    uint8_t year; /* year past 2000, e.g. 15 for 2015 */
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} NET_LOCATION;

class HTTPClient
{
public:
    HTTP_STATES state() { return m_state; }
    uint16_t code() { return m_code; }
protected:
    String genHeader(HTTP_METHOD method, const char* path, const char* payload, int payloadSize);
    // Generate an HTTP request header with an optional Authorization: Bearer header.
    // When bearerToken is non-null and non-empty it is appended before the
    // terminal CRLF-CRLF so the server can authenticate the request.
    String genHeaderWithAuth(HTTP_METHOD method, const char* path,
                             const char* payload, int payloadSize,
                             const char* bearerToken);
    HTTP_STATES m_state = HTTP_DISCONNECTED;
    uint16_t m_code = 0;
    String m_host;
};

class ClientWIFI
{
public:
    bool begin(const char* ssid, const char* password);
    void end();
    bool setup(unsigned int timeout = 5000);
    String getIP();
    int getSignal() { return 0; }
    const char* deviceName() { return "WiFi"; }
    void listAPs();
    bool connected() { return WiFi.isConnected(); }
    int RSSI() { return WiFi.RSSI(); }
protected:
};

class WifiUDP : public ClientWIFI
{
public:
    bool open(const char* host, uint16_t port);
    void close();
    bool send(const char* data, unsigned int len);
    int receive(char* buffer, int bufsize, unsigned int timeout = 100);
    String queryIP(const char* host);
private:
    IPAddress udpIP;
    uint16_t udpPort;
    WiFiUDP udp;
};

class WifiHTTP : public HTTPClient, public ClientWIFI
{
public:
    bool open(const char* host = 0, uint16_t port = 0);
    void close();
    bool send(HTTP_METHOD method, const char* path, const char* payload = 0, int payloadSize = 0);
    char* receive(char* buffer, int bufsize, int* pbytes = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
    // Expose the underlying TLS socket so callers can stream a large response
    // body (e.g. a firmware binary) in chunks without buffering it all at once.
    // Used by performPullOtaCheck() in telelogger.ino.
    WiFiClientSecure& rawClient() { return client; }
    // Parse the HTTP response headers that are already in the socket, return the
    // HTTP status code and (via *contentLength) the Content-Length value.
    // Leaves the socket positioned at the first byte of the body.
    // Returns -1 on timeout or parse failure.
    int receiveHeaders(int* contentLength, unsigned int timeout = HTTP_CONN_TIMEOUT);
private:
    WiFiClientSecure client;
};

typedef enum {
    CELL_SIM7600 = 0,
    CELL_SIM7670 = 1,
    CELL_SIM7070 = 2,
    CELL_SIM5360 = 3
} CELL_TYPE;

class CellSIMCOM
{
public:
    virtual bool begin(CFreematics* device);
    // Attaches to an ALREADY-running modem that another CellSIMCOM-derived
    // object (sharing the same physical UART/CFreematics device) has
    // already brought up via begin() - unlike begin(), does NOT toggle the
    // power-control pin, does NOT purge the shared UART's pending RX bytes,
    // and does NOT run the model/IMEI detection handshake (ATE0/ATI or the
    // SIMCOMATI fallback). Only sets m_device (and lazily allocates this
    // object's own m_buffer via getBuffer(), same as begin()).
    //
    // Use this for a second object that needs to issue its own AT commands
    // on the shared UART (e.g. telelogger.ino's one-off CellHTTP OTA check)
    // without disturbing a live session already running via a different
    // object (e.g. CellUDP telemetry) on the same modem. begin()'s three
    // side effects are each individually unsafe to repeat on a live modem:
    // xbTogglePower() pulses the power-control pin (harmless at begin()'s
    // 200 ms on an already-running modem in practice, but unverified/
    // unnecessary risk to take again), xbPurge() discards whatever the
    // OTHER object's in-flight AT exchange has already put in the shared
    // UART driver's RX ring buffer, and the handshake's own AT commands
    // compete with the other object's for that same buffer - see
    // FreematicsNetwork.cpp's xbReceive()/sendCommand() for why: there is
    // exactly one physical UART and one driver-level RX ring buffer, with
    // no per-session demultiplexing, so whichever object happens to call
    // xbReceive() next drains whatever is currently queued regardless of
    // which logical session it belongs to.
    //
    // Does NOT eliminate that shared-buffer risk for the attached object's
    // own subsequent AT commands (there is no protocol-level session
    // multiplexing in this driver) - it only avoids ADDING begin()'s three
    // extra rounds of it on top.
    //
    // `type` MUST be the modem type an earlier begin() on a sibling object
    // already detected (see its type() getter below) - attach() does not
    // run begin()'s own detection handshake, so m_type would otherwise stay
    // at its default (CELL_SIM7600) regardless of the real modem, silently
    // running the wrong AT-command dialect for open()/send()/receive()
    // (confirmed live: a SIM7670E-LN modem, left at the CELL_SIM7600
    // default, exercised the CCHOPEN/CCH* command family the modem does not
    // implement the same way, and open() failed).
    bool attach(CFreematics* device, CELL_TYPE type);
    // TEMP 2026-09-22 diagnostic: raw AT passthrough, to test a command
    // directly through an ALREADY-working, already-begin()'d object (e.g.
    // teleClient.cell) and rule software-session/attach() effects in or out
    // vs a genuine modem/firmware-level response. Remove once the SIM7670
    // AT+HTTPINIT/AT+CLAC ERROR root cause is found.
    bool rawAT(const char* cmd, unsigned int timeout = 1000) { return sendCommand(cmd, timeout); }
    char* rawBuffer() { return m_buffer; }
    virtual void end();
    virtual bool setup(const char* apn, const char* username = 0, const char* password = 0, unsigned int timeout = 60000);
    virtual bool setGPS(bool on);
    virtual String getIP();
    int RSSI();
    String getOperatorName();
    bool checkSIM(const char* pin = 0);
    virtual String queryIP(const char* host);
    virtual bool getLocation(GPS_DATA** pgd);
    bool check(unsigned int timeout = 0);
    char* getBuffer();
    // The modem type begin() (or a sibling object's begin()) detected -
    // needed by attach() callers to pass the correct type through.
    CELL_TYPE type() { return m_type; }
    const char* deviceName() { return m_model; }
    char IMEI[16] = {0};
protected:
    bool sendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = 0);
    virtual void inbound();
    virtual void checkGPS();
    float parseDegree(const char* s);
    char* m_buffer = 0;
    char m_model[12] = {0};
    CFreematics* m_device = 0;
    GPS_DATA* m_gps = 0;
    CELL_TYPE m_type = CELL_SIM7600;
    int m_incoming = 0;
};

class CellUDP : public CellSIMCOM
{
public:
    bool open(const char* host, uint16_t port);
    bool close();
    bool send(const char* data, unsigned int len);
    char* receive(int* pbytes = 0, unsigned int timeout = 5000);
protected:
    String udpIP;
    uint16_t udpPort = 0;
};

class CellHTTP : public HTTPClient, public CellSIMCOM
{
public:
    void init();
    bool open(const char* host = 0, uint16_t port = 0);
    bool close();
    bool send(HTTP_METHOD method, const char* host, uint16_t port, const char* path, const char* payload = 0, int payloadSize = 0);
    char* receive(int* pbytes = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
    // Streaming receive for large responses (SIM7600 only).
    // receiveHeaders() reads the first data chunk from the modem socket, parses
    // the HTTP status code and Content-Length header, and buffers any body bytes
    // already present in that chunk so they are returned by the first
    // receiveBodyBytes() call without an additional AT command round-trip.
    // Returns the HTTP status code (e.g. 200), or -1 on failure / unsupported modem.
    int receiveHeaders(int* contentLength = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
    // Read the next chunk of the response body after receiveHeaders().
    // Returns the number of bytes written to buf (> 0), 0 at end-of-stream,
    // or -1 on error / unsupported modem type.
    int receiveBodyBytes(char* buf, int maxLen, unsigned int timeout = HTTP_CONN_TIMEOUT);
protected:
    // Override to detect +CHTTPSCLSE: URCs that arrive during any AT command
    // and mark the session disconnected before send() tries to use it.
    void inbound() override;
private:
    // Body bytes buffered during receiveHeaders() that have not yet been
    // returned by receiveBodyBytes().  m_buffer[0..m_streamBodyLen-1] holds
    // the initial body slice; m_streamBodyPos is the next unread offset.
    // Both are reset to 0 at the start of each receiveHeaders() call.
    int m_streamBodyLen = 0;
    int m_streamBodyPos = 0;
};

#endif
