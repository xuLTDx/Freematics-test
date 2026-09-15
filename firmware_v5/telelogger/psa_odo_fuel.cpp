// PSA/Stellantis (K0 platform) UDS odometer polling + exploratory fuel-level
// capture - see psa_odo_fuel.h for the vehicle scope and why fuel has no
// confirmed DID yet. Same cross-translation-unit pattern as vag_odo_fuel.cpp
// (obdLinkUp/obdSendCommand/logNetEvent wrappers into telelogger.ino) - see
// that file's header comment for the full reasoning.
//
// PSA addressing, sourced 2026-09-15 from ludwig-v/arduino-psa-diag's
// ECU_LIST.md and corroborated by every tx_id/rx_id in Barracuda09/
// PyPSADiag's json/*/*.json ECU database:
//   request ID = response ID + 0x100  (VAG uses response + 0x08)
//   BSI (body/gateway module, incl. "BSI_KILO"): request 0x752, response 0x652
// Session: needs extended session (10 03, positive reply 50 03) + a
// TesterPresent keep-alive for reads; UNLIKE VAG, no SecurityAccess (0x27)
// is required to READ these DIDs - PyPSADiag's setZonesToRead() only issues
// 1003 before queuing DID reads, never a 27/67 exchange. 0x27 is reserved
// for writes (coding), which this module never does.
//
// ODOMETER: BSI DID 0xDBA2 ("Mileage"), request "22 DB A2", response
// "62 DB A2 <data>", value = raw integer / 10 -> km. Source:
// github.com/Barracuda09/PyPSADiag json/BMF/BSI2010.json (credits
// forum-peugeot.com user "weirdcactus" for the original capture) + the
// /10 decode rule in EcuZoneLineEdit.py. NOT verified: exact payload byte
// length (expect 3 bytes/24-bit per the analogous COMBINE DID 0xD70A,
// which explicitly documents a 24-bit mileage field at byte offset 8), or
// whether the K0 platform's specific BSI2010 variant answers this DID
// identically to the generic BSI2010 entries in the PyPSADiag database.
//
// FUEL LEVEL: NOT a UDS DID at all (0x2100/0x2101 were investigated and
// ruled out 2026-09-15 - PyPSADiag's INJ_UDS_UNI.json shows 0x2101 is a
// telecoding config bitmask, e.g. cooling-fan/gearbox/LPG flags, not live
// data; 0xDBA2's own BSI2010.json is the ONLY read_only zone in that whole
// database - no live fuel DID exists in any public PSA UDS database).
// Instead it's a BROADCAST CAN frame the BSI transmits on its own, no
// request needed: CAN ID 0x612, signal "INFO_NIV_CARB", byte offset 3,
// factor 0.5, unit litres, range 0-127L -> litres = data[3] * 0.5.
// Source: commaai/opendbc's psa_aee2010_r3.dbc (AEE2010 - same electrical
// generation as our BSI2010), corroborated by prototux/PSA-RE's older
// AEE2004 612.yml leaving that same byte as "UNKNOWN_3" (consistent
// position across two independent, differently-sourced trees). NOT
// verified: whether 0x612 is actually forwarded to the OBD connector on
// K0 (opendbc's port targets Peugeot 208 2019-25, not Traveller/Zafira),
// or the exact text format this chip's ATMA monitor uses for a captured
// frame - both are why this module logs the raw capture unconditionally.

#include <Arduino.h>
#include <ctype.h>
#include <FreematicsPlus.h>
#include "config.h"
#include "telestore.h"
#include "teleclient.h"
#include "psa_odo_fuel.h"

// --- Thin wrappers into telelogger.ino, shared with vag_odo_fuel.cpp ---
extern bool obdLinkUp();
extern int obdSendCommand(const char* cmd, char* buf, int bufsize, int timeout);
extern void logNetEvent(const char* msg);

// Diagnostic PIDs, range 0x360+ to avoid colliding with the VAG module's
// 0x300-0x352 range (see vag_odo_fuel.cpp) - both modules can in principle
// be linked into the same build (only one calls its process function at
// runtime, gated by which vehicle build this is), so their PID ranges must
// never overlap.
#define PID_PSA_ODO_DIAG     0x360 // 0=link down, 1=no/timeout, 2=parse fail, 3=ok
#define PID_PSA_ODO_RET      0x361
#define PID_PSA_ODO_LEN      0x362
#define PID_PSA_ODO_SESS     0x363 // same coding as VAG's PID_ODO_SESS
#define PID_PSA_ODO_SESS_NRC 0x364
// Broadcast frame 0x612 capture. MON_HIT=1 if "612" appeared anywhere in
// the ATMA capture window (same technique as VAG's MON_HIT), LEN=captured
// text length, DL=parsed decilitres (litres*10) if the byte-3 extraction
// succeeded - 0 if not (parse failure or frame never seen).
#define PID_PSA_FUEL_MON_HIT 0x370
#define PID_PSA_FUEL_MON_LEN 0x371
#define PID_PSA_FUEL_DIAG    0x372 // 0=link down, 1=frame not seen, 2=seen but parse failed, 3=ok
#define PID_PSA_FUEL_LEVEL_DL 0x373 // litres * 10, e.g. 235 = 23.5 l

// Classifies a UDS session-control (1003) response - same coding as VAG's
// classifySessionResponse() in vag_odo_fuel.cpp, duplicated here rather
// than shared so each vehicle module stays self-contained (see that file's
// header comment on why per-vehicle modules don't share internals).
static int classifySessionResponse(int ret, const char* resp, int* nrcOut)
{
  *nrcOut = 0;
  if (ret <= 0) return 0;
  char hex[16];
  int n = 0;
  for (const char* p = resp; *p && n < (int)sizeof(hex) - 1; p++) {
    if (isxdigit((unsigned char)*p)) hex[n++] = *p;
  }
  hex[n] = 0;
  if (n < 2) return 2; // e.g. "NO DATA" - no hex digits at all
  char sidStr[3] = { hex[0], hex[1], 0 };
  long sid = strtol(sidStr, nullptr, 16);
  if (sid == 0x50) {
    if (n >= 4) {
      char subStr[3] = { hex[2], hex[3], 0 };
      if (strtol(subStr, nullptr, 16) == 0x03) return 3;
    }
    return 2;
  }
  if (sid == 0x7F && n >= 6) {
    char nrcStr[3] = { hex[4], hex[5], 0 };
    *nrcOut = (int)strtol(nrcStr, nullptr, 16);
    return 1;
  }
  return 2;
}

// Parses a UDS ReadDataByIdentifier positive response (SID 0x62) for the
// given DID, treating ALL trailing data bytes as one big-endian integer -
// appropriate here since we don't yet know a byte-range subset like VAG's
// parseUdsByteRange() needed (that one skips surrounding unrelated bytes
// in a larger multi-field block; PSA's Mileage DID is not known to have
// any such surrounding bytes). Returns -1 on a negative response (0x7F), a
// DID mismatch, or a malformed/short response.
static long parseUdsWholeValue(const char* resp, uint16_t did)
{
  if (!resp) return -1;
  char hex[32];
  int n = 0;
  for (const char* p = resp; *p && n < (int)sizeof(hex) - 1; p++) {
    if (isxdigit((unsigned char)*p)) hex[n++] = *p;
  }
  hex[n] = 0;
  if (n < 6) return -1; // need at least SID(2 hex chars) + DID(4 hex chars)

  char sidStr[3] = { hex[0], hex[1], 0 };
  if (strtoul(sidStr, nullptr, 16) != 0x62) return -1;

  char didStr[5] = { hex[2], hex[3], hex[4], hex[5], 0 };
  if (strtoul(didStr, nullptr, 16) != did) return -1;

  if (n <= 6) return -1; // positive response but no data bytes attached
  return strtol(hex + 6, nullptr, 16);
}

// Attempts to extract byte index `byteIdx` (0-based, within the 8 data
// bytes) of a captured broadcast frame with 11-bit CAN ID `id3hex` (3 hex
// chars, e.g. "612") from raw ATMA capture text. Assumes the common
// ELM327 monitor-mode layout: <3-hex ID><1-hex DLC nibble><data bytes as
// hex>, with arbitrary/no spacing (all non-hex characters between them
// are stripped first, so this tolerates spaces or none). This exact
// layout is UNVERIFIED for this chip's ATMA implementation - see the file
// header on why the raw capture is always logged regardless. Returns -1
// if the ID isn't found, or there aren't enough hex characters after it
// for the requested byte (e.g. a shorter/malformed capture).
static int extractBroadcastByte(const char* capture, const char* id3hex, int byteIdx)
{
  if (!capture) return -1;
  const char* pos = strstr(capture, id3hex);
  if (!pos) return -1;
  const char* p = pos + 3;
  char hex[40];
  int n = 0;
  for (; *p && n < (int)sizeof(hex) - 1; p++) {
    if (isxdigit((unsigned char)*p)) hex[n++] = *p;
  }
  hex[n] = 0;
  int start = 1 + byteIdx * 2; // skip 1 DLC nibble, then byteIdx*2 hex digits
  if (start + 2 > n) return -1;
  char byteStr[3] = { hex[start], hex[start + 1], 0 };
  return (int)strtol(byteStr, nullptr, 16);
}

void processPsaOdoFuel(CBuffer* buffer)
{
  // =========================================================================
  // ODOMETER (BSI 0x752/0x652, DID 0xDBA2 "Mileage") - once per minute, same
  // cadence/reasoning as the VAG module (value changes slowly, each read
  // costs several extra AT commands).
  // =========================================================================
  static uint32_t lastOdoCheck = 0;
  if (millis() - lastOdoCheck >= 60000) {
    lastOdoCheck = millis();
    uint32_t odometerKm = 0;
    char rawBuf[24] = "-";
    int ret1 = 0;
    long parsed1 = -1;
    char sessResp1[24] = "-";
    int sessRet1 = 0;

    if (obdLinkUp()) {
      char ignore[32];
      obdSendCommand("ATSP6\r", ignore, sizeof(ignore), 200);    // standard CAN 11-bit/500k
      obdSendCommand("ATSH752\r", ignore, sizeof(ignore), 100);  // request header -> BSI
      obdSendCommand("ATCRA652\r", ignore, sizeof(ignore), 100); // filter response

      sessResp1[0] = 0;
      sessRet1 = obdSendCommand("1003\r", sessResp1, sizeof(sessResp1), 200);

      char responseBuf[64];
      responseBuf[0] = 0;
      ret1 = obdSendCommand("22DBA2\r", responseBuf, sizeof(responseBuf), 150);
      strncpy(rawBuf, responseBuf, sizeof(rawBuf) - 1);
      if (ret1 > 0) {
        parsed1 = parseUdsWholeValue(responseBuf, 0xDBA2);
        // Unverified byte width (see file header) - dividing the whole
        // trailing value by 10 assumes no extra bytes beyond the mileage
        // field itself. If parsed1 comes back absurdly large, that's the
        // first thing to check against a real capture.
        if (parsed1 > 0) { odometerKm = (uint32_t)(parsed1 / 10); }
      }

      // Restore defaults for the ordinary Mode 1 PID tier-poll loop.
      obdSendCommand("ATCRA\r", ignore, sizeof(ignore), 100);
      obdSendCommand("ATSH7E0\r", ignore, sizeof(ignore), 100);
      obdSendCommand("ATSP0\r", ignore, sizeof(ignore), 200);
    }

    // Format: PSAODO FW=<version> KM=<value> SESS=<ret>:<resp> R1=<ret> RAW1=<raw>
    {
      char diag[220];
      snprintf(diag, sizeof(diag), "PSAODO FW=%s KM=%lu SESS=%d:%s R1=%d RAW1=%s",
          FIRMWARE_VERSION, (unsigned long)odometerKm, sessRet1, sessResp1, ret1, rawBuf);
      Serial.print("[PSAODO] "); Serial.println(diag);
      logNetEvent(diag);
    }

    {
      int32_t diagCode = !obdLinkUp() ? 0 : (ret1 <= 0 ? 1 : (parsed1 <= 0 ? 2 : 3));
      int32_t diagRet = ret1;
      int32_t diagLen = (int32_t)strlen(rawBuf);
      buffer->add(PID_PSA_ODO_DIAG, ELEMENT_INT32, &diagCode, sizeof(diagCode));
      buffer->add(PID_PSA_ODO_RET, ELEMENT_INT32, &diagRet, sizeof(diagRet));
      buffer->add(PID_PSA_ODO_LEN, ELEMENT_INT32, &diagLen, sizeof(diagLen));

      int sessNrc = 0;
      int32_t sessCode = classifySessionResponse(sessRet1, sessResp1, &sessNrc);
      int32_t sessNrc32 = sessNrc;
      buffer->add(PID_PSA_ODO_SESS, ELEMENT_INT32, &sessCode, sizeof(sessCode));
      buffer->add(PID_PSA_ODO_SESS_NRC, ELEMENT_INT32, &sessNrc32, sizeof(sessNrc32));
    }

    if (odometerKm > 0) {
      buffer->add(PID_ODOMETER | 0x100, ELEMENT_INT32, &odometerKm, sizeof(odometerKm));
    }
  }
  // =========================================================================

  // =========================================================================
  // FUEL - broadcast frame 0x612 monitor capture (see file header for the
  // DBC source/formula: litres = data[3] * 0.5). No request needed - just
  // listen for the frame the BSI already sends on its own, same technique
  // as the VAG module's ATMA-based Alt 2. Same 60s cadence as odometer.
  // =========================================================================
  static uint32_t lastFuelCheck = 0;
  if (millis() - lastFuelCheck >= 60000) {
    lastFuelCheck = millis();
    char monResp[220] = "";
    int monLen = 0;
    bool monSawFrame = false;
    int fuelDeciLiters = 0;

    if (obdLinkUp()) {
      char ignore[32];
      obdSendCommand("ATSP6\r", ignore, sizeof(ignore), 200);
      obdSendCommand("ATH1\r", ignore, sizeof(ignore), 100); // headers on, so the captured ID is visible

      monLen = obdSendCommand("ATMA\r", monResp, sizeof(monResp), 400);
      char stopBuf[8];
      obdSendCommand(" \r", stopBuf, sizeof(stopBuf), 200); // stop monitor mode, drain/resync

      monSawFrame = strstr(monResp, "612") != nullptr;
      if (monSawFrame) {
        int rawByte = extractBroadcastByte(monResp, "612", 3);
        // rawByte * 0.5 = litres -> * 5 = decilitres, avoids float math.
        if (rawByte >= 0) { fuelDeciLiters = rawByte * 5; }
      }

      // Restore this device's normal defaults (ATSP0 auto-detect; ATH0 is
      // COBD::init()'s default, every other PID read assumes headers off).
      obdSendCommand("ATH0\r", ignore, sizeof(ignore), 100);
      obdSendCommand("ATSP0\r", ignore, sizeof(ignore), 200);
    }

    // Two diag lines (kept under FileLogger's ~132 char/line limit each).
    // Format: PSAFUEL FW=<version> DL=<decilitres> MON=<len>:<hit 0/1>
    //         PSAFUEL2 RAW=<first ~110 chars of the ATMA capture>
    {
      char diag1[100];
      snprintf(diag1, sizeof(diag1), "PSAFUEL FW=%s DL=%d MON=%d:%d",
          FIRMWARE_VERSION, fuelDeciLiters, monLen, monSawFrame ? 1 : 0);
      Serial.print("[PSAFUEL] "); Serial.println(diag1);
      logNetEvent(diag1);

      char diag2[128];
      snprintf(diag2, sizeof(diag2), "PSAFUEL2 RAW=%.110s", monResp);
      Serial.print("[PSAFUEL] "); Serial.println(diag2);
      logNetEvent(diag2);
    }

    {
      int32_t monHit = monSawFrame ? 1 : 0;
      int32_t monLen32 = monLen;
      buffer->add(PID_PSA_FUEL_MON_HIT, ELEMENT_INT32, &monHit, sizeof(monHit));
      buffer->add(PID_PSA_FUEL_MON_LEN, ELEMENT_INT32, &monLen32, sizeof(monLen32));

      int32_t diagCode = !obdLinkUp() ? 0 : (!monSawFrame ? 1 : (fuelDeciLiters <= 0 ? 2 : 3));
      buffer->add(PID_PSA_FUEL_DIAG, ELEMENT_INT32, &diagCode, sizeof(diagCode));

      if (fuelDeciLiters > 0) {
        int32_t dl = fuelDeciLiters;
        buffer->add(PID_PSA_FUEL_LEVEL_DL, ELEMENT_INT32, &dl, sizeof(dl));
        buffer->add(PID_FUEL_LEVEL | 0x100, ELEMENT_INT32, &fuelDeciLiters, sizeof(fuelDeciLiters));
      }
    }
  }
  // =========================================================================
}
