// Delivery queue (2026-09-26) - the one path every record takes to the server.
//
// Every record goes to SD and, at the same time, into this queue. The
// telemetry task sends queued records to Traccar in packets; a record is only
// dropped from the queue once the server has ACKed the packet it was in (the
// server ACKs after the records are stored, and drops re-sent ones by their
// identity). Identity = (boot number, PID 0 = ms since that boot).
//
// The queue lives in a PSRAM reserve that survives ESP.restart() and panics
// (the Arduino boot PSRAM test, which overwrites every 8th word, is replaced
// - see testSPIRAM() in txqueue.cpp); it does NOT survive a power loss. What
// the queue cannot hold (overflow, power loss, a broken reserve) is read back
// from SD from a cursor. Without SD the queue is the only copy: from 50 % new
// records are cut to time code, position, speed and heading; from 80 % they
// are also thinned to one per 10 s (engine start/stop records never thinned).
// Without PSRAM a small internal-RAM queue is used instead.
#pragma once
#include <Arduino.h>

#ifndef TXQ_RESERVE_BYTES  // -D override for bench tests only (overflow -> SD replay)
#define TXQ_RESERVE_BYTES (1024 * 1024)  // PSRAM: ~1 h of driving at 1 record/s
#endif
#define TXQ_FALLBACK_BYTES (16 * 1024)   // internal RAM when there is no PSRAM
#define TXQ_PACKET_MAX 700               // record bytes per UDP packet (a last record may exceed it)
#define TXQ_PACKET_BUF 1600              // caller's packet buffer
#define TXQ_MAX_INFLIGHT 4               // packets sent and not yet ACKed
#define TXQ_ACK_TIMEOUT_MS 8000          // then the packet's records are sent again
#define TXQ_THIN_MS 10000                // no SD, queue >= 80 %: one record per 10 s

// PID numbers of the delivery protocol (packet header and record flags)
#define PID_TX_BOOT     0x383  // packet header: boot number
#define PID_TX_PACKET   0x384  // packet header: packet number, echoed in the ACK
#define PID_TX_NOFIX    0x385  // record: no new GPS fix, A/B = the box's last fix

void txqEarlyInit();                 // first thing in setup(): take the PSRAM reserve
void txqBegin(uint32_t bootId);      // after NVS is open: check what survived
void txqSetFile(uint32_t fileId);    // the SD file new records are written to (0 = none)
// rec = "0:<ts>,<pid>:<value>,..." as it goes on the wire, without devid/checksum
bool txqPush(uint32_t ts, const char* rec, int len);
int txqNextPacket(char* out, int cap, const char* devid);  // 0 = nothing to send now
void txqSendFailed();                // the last packet from txqNextPacket() did not go out
void txqOnAck(uint32_t packetNo);
void txqSave(bool force);            // persist the "oldest not confirmed" cursor (NVS)
uint32_t txqPending();               // records not yet confirmed (queue; +1 while SD replay runs)
uint32_t txqBootId();
bool txqUsingPsram();
uint32_t txqUsedPercent();
void txqStatus(char* buf, int size); // one line for logs / web UI
void txqResendFrom(uint32_t fileId); // manual: replay /DATA/<fileId>.CSV and later from SD
#ifdef TEST_TXQ
void txqTestLoseReserve();           // bench: as if power was lost (reserve not valid at next boot)
#endif
