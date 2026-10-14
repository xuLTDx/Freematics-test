// SD file fingerprint (2026-10-14) - FleetNest checks "SD = database" without the card ever
// leaving the car (FleetNest docs/navrh-sd-kontrola.md). For the open /DATA file: how many
// records the card has confirmed, the first and last PID 0 (ts) and the CRC-32 of those ts
// (4 bytes little-endian each, in the order written - the same as zlib.crc32 on the server).
// Pure C, no ESP32 headers: the same code is compiled on the PC to test it against the server.
#pragma once
#include <stdint.h>

struct SdDigest {
  uint32_t file, boot, count, first, last, crc;  // crc = running register (see sdDigestCrc)
};

static inline void sdDigestStart(SdDigest* d, uint32_t file, uint32_t boot)
{
  d->file = file; d->boot = boot; d->count = 0; d->first = 0; d->last = 0;
  d->crc = 0xFFFFFFFFu;
}

// one record the card confirmed (ts ascending)
static inline void sdDigestAdd(SdDigest* d, uint32_t ts)
{
  if (!d->count) d->first = ts;
  d->last = ts;
  d->count++;
  uint32_t c = d->crc;
  for (int i = 0; i < 4; i++) {
    c ^= (ts >> (8 * i)) & 0xFFu;
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  d->crc = c;
}

// the CRC-32 as zlib reports it
static inline uint32_t sdDigestCrc(const SdDigest* d) { return ~d->crc; }

// PID 0x398 value: file;boot;count;first;last;crc
static inline void sdDigestValues(const SdDigest* d, uint32_t out[6])
{
  out[0] = d->file; out[1] = d->boot; out[2] = d->count;
  out[3] = d->first; out[4] = d->last; out[5] = sdDigestCrc(d);
}
