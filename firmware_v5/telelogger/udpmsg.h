// Server messages in one received buffer (2026-10-14).
//
// The server sends each message as its own UDP datagram, "<body>*<CS>" (CS = 2 hex digits, the
// 8-bit sum of the body). Over WiFi one read returns one datagram; over the cellular modem one
// read returns whatever the modem has buffered - often an ACK and a command (or an event reply
// and a command) back to back. The old code handled a read as ONE message: it stopped at the
// ACK and dropped the command behind it, and an event reply with a command behind it failed the
// checksum and was sent again (login / ping three times in the same second). One parser for
// both links: split the buffer into messages, check each checksum, hand each one on.
// Pure C, no ESP32 headers - the same code is compiled on the PC to test it.
#pragma once
#include <stdint.h>

typedef void (*UdpMsgFn)(char* body, void* ctx);  // body NUL-terminated, checksum already OK

static inline int udpHexNibble(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Calls fn for every message with a correct checksum, in order. Returns how many; *bad counts
// the ones whose checksum did not match (they are skipped). The buffer is modified ('*' -> 0).
static inline int udpSplit(char* buf, int len, UdpMsgFn fn, void* ctx, int* bad)
{
  int ok = 0, start = 0;
  if (bad) *bad = 0;
  while (start < len) {
    while (start < len && (buf[start] == '\r' || buf[start] == '\n' || buf[start] == ' ' || buf[start] == 0))
      start++;
    int star = -1;                            // '*' followed by two hex digits = end of a message
    for (int i = start; i + 2 < len; i++) {
      if (buf[i] == '*' && udpHexNibble(buf[i + 1]) >= 0 && udpHexNibble(buf[i + 2]) >= 0) {
        star = i;
        break;
      }
    }
    if (star < 0) break;                      // no complete message left
    uint8_t sum = 0;
    for (int i = start; i < star; i++) sum += (uint8_t)buf[i];
    uint8_t cs = (uint8_t)(udpHexNibble(buf[star + 1]) * 16 + udpHexNibble(buf[star + 2]));
    if (cs == sum) {
      buf[star] = 0;
      fn(buf + start, ctx);
      ok++;
    } else if (bad) {
      (*bad)++;
    }
    start = star + 3;
  }
  return ok;
}
