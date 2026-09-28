// Odometer (and fuel, unconfirmed) read over the CAN module soldered to the
// Molex pins (WCMCU-230, CTX = GPIO26, CRX = GPIO34), bypassing the OBD
// co-processor, which cannot send UDS to a non-default header.
//
// VW Passat B8 (HexSniff PassatB8/decoded.md):
//   odometer: Gateway 0x710 -> 0x77A, UDS 22 02BD, km = data bytes 1..3.
//     Confirmed in the car 2026-09-28 with this code: 157638 km = dashboard.
//   fuel: Instruments 0x714 -> 0x77E, UDS 22 22B0 ("calculated volume").
//     Where the litres sit in the reply is NOT confirmed (HexSniff: 54 data
//     bytes, value at [33:35]; OVMS got 53 bytes, [33:35] is nonsense, [34:36]
//     gave 43.9-44.8 l but rose while driving) - sent raw for confirmation.
// The car only answers with the ignition on, so reads are made only while
// the engine runs (see telelogger.ino).
#pragma once

#include <Arduino.h>

// Result of the last read, sent to the server so a failed read can be
// diagnosed without the serial port.
enum CanOdoResult {
  CANODO_OK = 0,
  CANODO_NOT_READY = 1,
  CANODO_TX_FAILED = 2,     // request could not be queued
  CANODO_NO_REPLY = 3,      // no reply within the timeout
  CANODO_NRC = 4,           // ECU refused: code in CanOdoDiag.nrc
  CANODO_CF_TIMEOUT = 5,    // multi-frame reply incomplete
  CANODO_CF_ORDER = 6,
  CANODO_BAD_REPLY = 7,
  CANODO_IMPLAUSIBLE = 8,
  CANODO_FC_FAILED = 9,
};

struct CanOdoDiag {
  uint8_t result;           // CanOdoResult
  uint8_t nrc;              // ECU negative response code (result 4)
  uint8_t state;            // twai_state_t: 0 stopped, 1 running, 2 bus-off, 3 recovering
  uint32_t framesAll;       // every frame received since boot (any ID)
  uint32_t frames77A;       // frames from 0x77A since boot
  uint32_t txErr;           // controller TX error counter
  uint32_t rxErr;           // controller RX error counter
  uint32_t busErr;          // bus errors since boot
  uint32_t txFailed;        // frames that failed to go out since boot
  uint32_t lastId;          // ID of the last frame received (any ID), 0 = none yet
  uint8_t raw[16];          // gateway reply as received (payload, first 16 bytes)
  uint8_t rawLen;
};

// Installs and starts the ESP32 CAN controller (TWAI) at 500 kbit/s, all
// frames accepted (counted for the diagnosis).
bool canOdoBegin();

// Odometer: one request/response (ISO-TP, flow control sent by us). Every
// frame goes out single-shot, so an unanswered frame is not repeated on the
// bus. Returns true and the km on success; `diag` is filled either way.
bool canOdoRead(uint32_t* km, CanOdoDiag* diag);

// Fuel block 22 22B0 from the instrument cluster: the data bytes after
// 62 22 B0 into `data` (up to `cap`), their count in `len`.
bool canFuelRead(uint8_t* data, size_t cap, size_t* len, CanOdoDiag* diag);
