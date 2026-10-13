// SD writer (2026-10-12) - the only task that writes to the SD card.
//
// Rule (the same as the delivery queue's ACK): data leaves RAM only once the
// card has confirmed it. A write counts as confirmed when write() took every
// byte AND fsync() returned 0 - plain POSIX calls on the FatFs VFS, not the
// Arduino File class, whose 4 KB stdio buffer said "written" for bytes that
// never reached the card (bench 2026-10-12, S20: a 84-frame hole with
// sd_lost=0). The writer keeps each file's confirmed length; after a failure
// the file is reopened, cut back to that length (no half-written line stays)
// and the same bytes are written again - so a file never has a hole or a
// duplicate, and every byte that did not make it is counted.
//
// Two inputs, served in this order:
//   1. telemetry: the /DATA/<n>.CSV lines (records and FE events) from any
//      task through a PSRAM ring; synced at most once a second, so a power
//      loss costs at most ~1 s of the SD copy. Records are confirmed one by
//      one to the delivery queue (txqSdConfirmed), which only replays from SD
//      what the card really holds.
//   2. sniff: blocks pulled from sniff.cpp (sniffFill), written between
//      telemetry writes.
// The card gone for SDW_DEAD_MS: telemetry continues without an SD copy
// (txqSetFile(0), the delivery queue's no-SD rules), sniff is stopped.
#pragma once
#include <Arduino.h>

#define SDW_MOUNT "/sd"              // SD.begin() mount point
#define SDW_DATA_RING (64 * 1024)   // telemetry bytes waiting for the card (PSRAM)
#define SDW_SNIFF_BLOCK (32 * 1024) // one sniff write
#define SDW_SYNC_MS 1000            // telemetry: at most one write+fsync per second
#define SDW_DEAD_MS 30000           // no successful write this long = card gone

void sdwStart();                     // after SD.begin(), once

// telemetry channel - /DATA/<n>.CSV
bool sdwDataOpen(const char* path);  // a new file; waits for the writer (false = failed)
void sdwDataWrite(const char* s, int n);  // into the ring, never waits for the card
void sdwDataMark(uint32_t ts);       // a delivery-queue record ends here (confirmed later)
bool sdwDataSync(uint32_t timeoutMs);     // wait until everything is on the card
void sdwDataClose(uint32_t timeoutMs);    // sync, then close
void sdwKick();                      // write now (no waiting)
bool sdwDataDead();                  // the card stopped taking telemetry

// sniff channel - the writer opens/closes the file; sniff.cpp supplies the bytes
bool sdwSniffOpen(const char* path); // waits for the writer (false = failed)
void sdwSniffClose(uint32_t timeoutMs);   // after sniffFill() returned its last bytes
bool sdwSniffOpenNow();

// one line for logs / API: errors, lost bytes, dead flag, last error time
int sdwStatus(char* buf, int size);

#ifdef TEST_SD_FAULT
void sdwTestFault(uint32_t everyN);  // bench: every n-th write fails half-way (0 = off)
#endif
