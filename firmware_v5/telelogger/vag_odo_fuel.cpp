// VAG (VW/Audi Group) UDS odometer + fuel-level polling - extracted out of
// telelogger.ino so vehicle-specific logic doesn't accumulate in one giant
// shared file as more vehicles (e.g. PSA/Stellantis) are added. Same pattern
// OVMS itself uses (components/vehicle_* per vehicle, shared framework
// stays common) - see OVMS v3 Developer Guide, "OVMS Vehicle Modules".
//
// This file never touches the .ino-local `obd`/`state`/`logger` globals
// directly (their types - OBD, State, the Logger typedef - are defined
// inside telelogger.ino itself, not in a shared header). Instead it goes
// through the thin wrapper functions declared extern below (obdLinkUp/
// obdSendCommand/obdReadStdPID, logNetEvent, gpsOdometerKm) - the
// same cross-translation-unit pattern already used for teleclient.cpp.

#include <Arduino.h>
#include <ctype.h>
#include <FreematicsPlus.h>
#include "config.h"
#include "telestore.h"
#include "teleclient.h"
#include "vag_odo_fuel.h"

// --- Thin wrappers into telelogger.ino, defined there alongside logNetEvent() ---
extern bool obdLinkUp();
extern int obdSendCommand(const char* cmd, char* buf, int bufsize, int timeout);
extern bool obdReadStdPID(byte pid, int& value);
extern void logNetEvent(const char* msg);
extern uint32_t gpsOdometerKm();

// Diagnostic PIDs for the ODO/FUEL UDS blocks below, sent as regular
// telemetry fields alongside GPS/OBD data over the SAME cellular/WiFi
// channel already used for everything else. Traccar's FreematicsProtocolDecoder
// has no case for these keys, so they fall through to its default branch and
// get stored as generic "io<key>" position attributes - readable any time the
// car has signal (not just on WiFi/USB), without any decoder change needed.
// Range 0x300+ chosen to avoid colliding with standard OBD PIDs (max 0xFF,
// sent |0x100 => up to 0x1FF) or the existing custom ones (0x81/0x82/0x11f/0x12f/0x1a6).
// diagCode: 0 = obd.link not up, 1 = no/timeout response, 2 = response
// received but byte-range parse failed, 3 = success (matches the value
// actually used for KM/DL that run).
#define PID_ODO_DIAG  0x300
#define PID_ODO_RET   0x301
#define PID_ODO_LEN   0x302
#define PID_FUEL_DIAG 0x310
#define PID_FUEL_RET  0x311
#define PID_FUEL_LEN  0x312
// Session-control (1003) outcome, sent alongside the above so it's readable
// remotely too - previously this only went into the SD event log's SESS=
// text, which needs WiFi/USB access to the device to read at all, defeating
// the point of diagnosing a real-drive failure. sessCode: 0 = no/timeout
// response, 1 = negative response (session request rejected - sessNrc holds
// the UDS NRC byte, e.g. 0x22 conditionsNotCorrect, 0x33 securityAccess
// required), 2 = some other unexpected response (garbage, or "NO DATA" -
// no hex digits at all), 3 = positive response confirmed (50 03).
#define PID_ODO_SESS      0x303
#define PID_ODO_SESS_NRC  0x304
#define PID_FUEL_SESS     0x313
#define PID_FUEL_SESS_NRC 0x314
// Raw text of the 1003 (session control) response, packed 4 ASCII chars per
// uint32 (big-endian: first char in the MSB) since CBuffer has no string
// element type. Added 2026-09-14 after realizing sessCode=2 alone cannot
// distinguish "NO DATA" (module never answered) from "CAN ERROR"/"BUS ERROR"
// (our own CAN transmit failing) - both classify identically, but the raw
// text tells them apart immediately. Covers up to 12 chars, enough for any
// of the ELM327 standard error strings without needing SD/WiFi access to
// read the SD event log's SESS=/RAW1= text. Decode server-side: each uint32
// -> 4 bytes -> ASCII chars (0 = no more text).
#define PID_ODO_SESS_RAW1  0x305
#define PID_ODO_SESS_RAW2  0x306
#define PID_ODO_SESS_RAW3  0x307
#define PID_FUEL_SESS_RAW1 0x315
#define PID_FUEL_SESS_RAW2 0x316
#define PID_FUEL_SESS_RAW3 0x317

// 2026-09-14: three more independently-testable alternatives to the primary
// ATSH+ATCRA approach above, added after that approach got "NO DATA" on
// 100% of real-drive attempts despite HexSniff proving the module/DID pair
// itself works over a real VCDS session. Run every cycle (not gated behind
// a "try once" flag) so comparison data accumulates from ordinary driving
// instead of needing another physical round-trip per hypothesis:
//   Alt 1 (SESS2): identical "1003" request, but the receive filter is set
//   via ATCF+ATCM (custom filter/mask) instead of the ATCRA convenience
//   command - same classification scheme as PID_ODO_SESS. Per the ELM327
//   spec these are meant to be equivalent (ATCRA is documented as being
//   implemented in terms of CF/CM), so a *different* result here would
//   itself be the interesting finding: it'd mean this chip's ATCRA
//   handling doesn't match spec.
//   Alt 2 (MON_HIT/MON_LEN): passive ATMA (monitor-all) capture for a short
//   window right after a fresh 1003 trigger - the same technique HexSniff
//   itself uses (no ATSH/ATCRA at all, just listen to everything). MON_HIT
//   = 1 if the module's own response ID ("77A"/"77E") text appears anywhere
//   in the captured raw stream, 0 if not. If MON_HIT=1 while SESS/SESS2
//   both still say "no response", the problem is in our filtered-request
//   path, not in the module or the physical request itself.
#define PID_ODO_SESS2      0x330
#define PID_ODO_MON_HIT    0x331
#define PID_ODO_MON_LEN    0x332
#define PID_FUEL_SESS2     0x340
#define PID_FUEL_MON_HIT   0x341
#define PID_FUEL_MON_LEN   0x342
// Alt 3, added 2026-09-14 after reading excieve/odographe's obd-core session.rs
// (a real, independent OBD/UDS session manager) line by line: it NEVER uses
// ATCRA anywhere - it relies purely on ATH1 (headers on) + ATS0 (no spaces)
// plus its own software-side check of the response line's leading CAN-ID
// against the expected responder, instead of the ELM327's hardware receive
// filter. It also skips 1003 (DiagnosticSessionControl) entirely for its own
// vehicle's body-module DID read - goes straight from ATSH<header> to the
// 22xx read. Both are genuinely different from anything tried here so far,
// worth testing independently: does turning off reliance on ATCRA (Alt3a),
// and does skipping 1003 (Alt3b), change anything. ATH1/ATS0 are reset back
// to ATH0/ATS1 (this device's normal defaults, see COBD::init()) right after,
// same restore-regardless-of-outcome pattern as the ATSH header switch below
// - every other PID read in this firmware assumes headers-off formatting.
#define PID_ODO_ALT3_SESS  0x333
#define PID_ODO_ALT3_HIT   0x334
#define PID_FUEL_ALT3_SESS 0x343
#define PID_FUEL_ALT3_HIT  0x344
// One-time (not per-cycle) ELM327/STN chip identification (ATI response),
// packed the same way as SESS_RAW above, so we know for certain which chip
// firmware this is instead of assuming "cheap ELM327 clone" - matters
// because the fix that actually works can depend heavily on that.
#define PID_CHIP_ID1 0x350
#define PID_CHIP_ID2 0x351
#define PID_CHIP_ID3 0x352

// Classifies a UDS session-control (1003) response so the outcome can be
// sent as a compact telemetry value instead of the raw text. See the
// PID_ODO_SESS comment above for the meaning of the returned code.
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

// Packs up to 4 ASCII characters of s[offset..offset+3] into one uint32
// (big-endian, missing/absent chars as 0) - see the PID_ODO_SESS_RAW1..3
// comment above for why this exists instead of just trusting sessCode.
static uint32_t packChars4(const char* s, int offset)
{
  uint32_t v = 0;
  int len = s ? (int)strlen(s) : 0;
  for (int i = 0; i < 4; i++) {
    int idx = offset + i;
    char c = (idx >= 0 && idx < len) ? s[idx] : 0;
    v = (v << 8) | (uint8_t)c;
  }
  return v;
}

// Like parseUdsHexValue() (telelogger.ino), but for DIDs whose value isn't
// the *entire* trailing data block - e.g. the Passat B8 Gateway (0x19) DID
// 0x02BD returns 10 data bytes packed together (status byte, the odometer
// value, then more unidentified bytes ending in a rolling/alive counter that
// changes on every poll - see vehicles/VAG/VW/PassatB8/decoded.md in the
// HexSniff repo for the full byte-by-byte breakdown). This extracts
// `byteLen` bytes starting at `byteOffset` within the data (0 = the first
// data byte right after the DID) as a single big-endian unsigned value,
// ignoring everything before/after that slice.
static long parseUdsByteRange(const char* resp, uint16_t did, int byteOffset, int byteLen)
{
  if (!resp) return -1;
  // 128 hex chars = up to a 64-byte UDS response (SID+DID+62 data bytes) -
  // comfortably covers the largest response seen so far (the Passat B8
  // "Calculated volume" DID 0x22B0, 56 bytes total, needing offsets up to
  // ~35 = hex index ~76). Bump this if a vehicle ever needs a bigger DID.
  char hex[128];
  int n = 0;
  for (const char* p = resp; *p && n < (int)sizeof(hex) - 1; p++) {
    if (isxdigit((unsigned char)*p)) hex[n++] = *p;
  }
  hex[n] = 0;
  if (n < 6) return -1;

  char sidStr[3] = { hex[0], hex[1], 0 };
  if (strtoul(sidStr, nullptr, 16) != 0x62) return -1;

  char didStr[5] = { hex[2], hex[3], hex[4], hex[5], 0 };
  if (strtoul(didStr, nullptr, 16) != did) return -1;

  int start = 6 + byteOffset * 2;
  int end = start + byteLen * 2;
  if (end > n) return -1; // response too short for the requested byte range

  char slice[16];
  memcpy(slice, hex + start, byteLen * 2);
  slice[byteLen * 2] = 0;
  return strtol(slice, nullptr, 16);
}

void processVagOdoFuel(CBuffer* buffer)
{
  // =========================================================================
  // ODOMETER PROCESSING (Passat B8 UDS / Standard OBD2 / Fallback to GPS)
  // Runs once per minute - the odometer value changes slowly, and each run
  // costs several extra CAN protocol AT commands (ATSP6/ATSH/1003 + restore),
  // so there's no benefit to checking as often as the 5s tier-poll cycle.
  // Independent of GPS position transmission, which is unaffected by this.
  // =========================================================================
  static uint32_t lastOdoCheck = 0;
  if (millis() - lastOdoCheck >= 60000) {
    lastOdoCheck = millis();
    int odoVal = 0;
    uint32_t odometerKm = 0;
    char responseBuf[64];

    char rawBuf[20] = "-"; // truncated raw hex response, for the diag line
    int ret1 = 0;
    long parsed1 = -1;
    const char* src = "NONE";
    char sessResp1[24] = "-"; // response to 1003 (extended session request)
    int sessRet1 = 0;
    char sessResp2[24] = "-"; // Alt 1: same request, ATCF/ATCM filter instead of ATCRA
    int sessRet2 = 0;
    char monResp[220] = "";   // Alt 2: passive ATMA capture right after a fresh 1003
    int monLen = 0;
    bool monSawResponseId = false;
    char alt3Resp[64] = "-";  // Alt 3: ATH1 (no ATCRA), skip 1003, direct 2202BD
    int alt3Ret = 0;
    bool alt3Hit = false;
    char chipIdResp[16] = ""; // one-time ATI chip identification

    if (obdLinkUp()) {
      // Odometer confirmed 2026-09-12 via a HexSniff bench capture against
      // a real VCDS session (module "19 - Gateway", its "odo read" menu
      // entry) - full raw capture + byte breakdown in the HexSniff repo at
      // vehicles/VAG/VW/PassatB8/decoded.md. The two hypotheses previously
      // tried here - extended 29-bit addressing straight to the instrument
      // cluster (0x18DA17F1/0x18DAF117) and standard 11-bit 0x714/0x77E -
      // are both confirmed dead ends: on this MQB platform the odometer is
      // only reachable through the CAN Gateway module (0x19) itself, not
      // through the cluster, and DID 0x0505 isn't it anyway.
      char ignore[32];
      obdSendCommand("ATSP6\r", ignore, sizeof(ignore), 200);     // standardne CAN 11-bit/500k
      // Confirmed 2026-09-14: every UDS read on a real drive gets "NO DATA"
      // on the 1003 session-open request itself (SESS=9:NO DATA on 100% of
      // attempts). Two fix attempts based on the HexSniff capture of a real
      // VCDS session on this vehicle - explicit flow control (ATFCSH/FCSD/
      // FCSM) and a functional TesterPresent broadcast (ATSH700 + 3E80 x5)
      // - were both tried and BOTH confirmed to make no difference via
      // server-side telemetry from real drives. Removed rather than left in
      // as dead weight; see git history for the reasoning that ruled each
      // one out. Addressing (0x710/0x77A), timing (real responses in the
      // VCDS capture arrive in 8-13ms, nowhere near our 100-200ms budget),
      // and SFD security-gateway locking (no 0x27/0x29 traffic at all in
      // the successful VCDS capture) have also been checked and ruled out.
      // 2026-09-14: pulled the actual SD event log via /api/events over USB
      // (device on the bench, engine off) and confirmed the raw text really
      // is "NO DATA" (not "CAN ERROR"/"BUS ERROR" as briefly suspected from
      // the telemetry length alone - see PID_ODO_SESS_RAW1..3 below, added
      // specifically so this never has to be inferred from length again).
      // Also confirmed an ELM327 "ATCS" (CAN Status) query - tried here for
      // one drive as a possible bus-error diagnostic - is NOT supported by
      // this adapter: it answered with "?" (ELM327's unrecognized-command
      // response), not status data. Removed; not worth keeping as dead code.
      // Root cause still open.
      obdSendCommand("ATSH710\r", ignore, sizeof(ignore), 100);   // hlavicka poziadavky -> Gateway (0x19)
      obdSendCommand("ATCRA77A\r", ignore, sizeof(ignore), 100);  // filtruj len jeho odpoved
      sessResp1[0] = 0;
      sessRet1 = obdSendCommand("1003\r", sessResp1, sizeof(sessResp1), 200);

      responseBuf[0] = 0;
      ret1 = obdSendCommand("2202BD\r", responseBuf, sizeof(responseBuf), 100);
      strncpy(rawBuf, responseBuf, sizeof(rawBuf) - 1);
      if (ret1 > 0) {
        // Datovy blok je CA <km: 3 bajty BE> 00 00 6A 58 D4 <rolling ctr> -
        // km su bajty 1..3 (0-indexovane) dat, hned za uvodnym statovym
        // bajtom. Posledny bajt (rolling counter) sa meni pri kazdom
        // pollovani a nie je sucastou hodnoty - zamerne sa neparsuje.
        parsed1 = parseUdsByteRange(responseBuf, 0x02BD, 1, 3);
        if (parsed1 > 0) { odometerKm = (uint32_t)parsed1; src = "UDS02BD"; }
      }

      // --- Alt 1: same 1003 request, receive filter set via ATCF+ATCM
      // instead of ATCRA - see PID_ODO_SESS2 comment above for why this is
      // worth testing even though the two are meant to be spec-equivalent.
      {
        obdSendCommand("ATCF77A\r", ignore, sizeof(ignore), 100);
        obdSendCommand("ATCM7FF\r", ignore, sizeof(ignore), 100);
        sessResp2[0] = 0;
        sessRet2 = obdSendCommand("1003\r", sessResp2, sizeof(sessResp2), 200);
      }

      // --- Alt 2 (ATMA monitor-all), RESTORED 2026-09-14 after being pulled
      // earlier the same day over a suspected total-freeze (see git history
      // for that removal's reasoning). Two things changed since then: (1) a
      // real code-level bug was found and fixed - the entire WiFi/cellular
      // connect path in teleclient.cpp never wrote to the SD log at all
      // (Serial-only), which is now the far more likely explanation for
      // that earlier "telemetry went totally silent" symptom than ATMA
      // itself; (2) OBD2UART.cpp (github.com/stanleyhuangyc/Freematics),
      // the actual source for this chip's firmware (self-identifies as
      // "OBD2USART V1.4" via ATI, confirmed 2026-09-14 - NOT a real
      // ELM327/STN1110), lists its full AT command set: ATZ, ATE0, ATH0,
      // ATSP, ATI, ATRV, ATBR1, plus ATACL/ATGYRO/ATMAG/ATTEMP/ATORI. ATMA
      // is NOT in that list - same as ATCS, which we already confirmed
      // unsupported (returned "?", not real data). So ATMA most likely
      // *isn't* a real monitor-mode command on this chip at all and simply
      // gets rejected like ATCS did, making the original "stuck streaming
      // forever" theory unlikely on this specific firmware. Logging the
      // actual captured text this time (monResp, truncated) instead of
      // just length+hit, unlike the first attempt - so whichever of these
      // turns out to be true, we can see it directly instead of inferring.
      {
        obdSendCommand("1003\r", ignore, sizeof(ignore), 50); // fresh trigger, not read here
        monLen = obdSendCommand("ATMA\r", monResp, sizeof(monResp), 300);
        char stopBuf[8];
        obdSendCommand(" \r", stopBuf, sizeof(stopBuf), 200); // stop monitor mode (if supported), drain/resync
      }
      monSawResponseId = strstr(monResp, "77A") != nullptr;

      // --- Alt 3: no ATCRA at all - ATH1 (headers on) + ATS0 (no spaces),
      // straight to 2202BD without any prior 1003. See PID_ODO_ALT3_SESS/HIT
      // comment above for why (excieve/odographe's obd-core session.rs uses
      // exactly this pattern successfully on a different vehicle). Response
      // header is on, so a hit for our module looks like the CAN ID "77A"
      // literally prefixing the data in alt3Resp, e.g. "77A104A6202BD...".
      {
        obdSendCommand("ATH1\r", ignore, sizeof(ignore), 100);
        obdSendCommand("ATS0\r", ignore, sizeof(ignore), 100);
        alt3Resp[0] = 0;
        alt3Ret = obdSendCommand("2202BD\r", alt3Resp, sizeof(alt3Resp), 150);
        // Immediately back to this device's normal defaults (COBD::init()
        // sets ATH0; ATS1/space-on is the ELM327 power-on default) - every
        // other PID read in this firmware assumes headers-off formatting.
        obdSendCommand("ATH0\r", ignore, sizeof(ignore), 100);
        obdSendCommand("ATS1\r", ignore, sizeof(ignore), 100);
      }
      alt3Hit = strstr(alt3Resp, "77A") != nullptr;

      // --- One-time chip identification (ATI) - see PID_CHIP_ID1..3 above.
      // Also sends ATAT1 (adaptive timing, medium) here since it's another
      // genuinely untried thing found while reading excieve/odographe's
      // session.rs line by line - it lets the ELM327 auto-tune its response
      // wait time from observed bus timing instead of a fixed guess, which
      // could matter if real responses are arriving just outside whatever
      // window this chip defaults to. Left on permanently (not restored) -
      // unlike ATH1/ATCRA/ATCF/ATCM above, it only affects internal timing,
      // not response formatting, so it's safe to leave active for every
      // future request including ordinary Mode 1 PID reads. Both guarded so
      // they only ever run once per boot, not once per minute.
      static bool chipIdSent = false;
      if (!chipIdSent) {
        chipIdSent = true;
        obdSendCommand("ATI\r", chipIdResp, sizeof(chipIdResp), 500);
        obdSendCommand("ATAT1\r", ignore, sizeof(ignore), 100);
      }

      // --- Vratit spat predvolene (11-bit) adresovanie na motor, aby
      // pokracovalo bezne citanie Mode 1 PID-ov (RPM, rychlost, ...) v
      // hlavnej tier-poll slucke bez zmeny. Bare ATCRA also clears whatever
      // ATCF/ATCM state Alt 1 above left behind (same underlying mechanism
      // per the ELM327 spec).
      obdSendCommand("ATCRA\r", ignore, sizeof(ignore), 100);   // zrus response filter
      obdSendCommand("ATSH7E0\r", ignore, sizeof(ignore), 100); // spat na motor
      obdSendCommand("ATSP0\r", ignore, sizeof(ignore), 200);   // spat na auto-detekciu protokolu
    }

    // 4. Skusime standardny 8-bitovy OBD2 PID 0xA6 (ak ho auto podporuje)
    if (odometerKm == 0 && obdReadStdPID(0xA6, odoVal) && odoVal > 0) {
      odometerKm = (uint32_t)odoVal;
      src = "PID_A6";
    }

    // 5. Ak OBD/UDS vycitanie zlyha (napr. Peugeot Traveller, alebo VAG s
    // uzamknutym gateway), pouzije sa vzdialenost napocitana z GPS suradnic
    // (pozri gpsOdometerKm() / processGPS). Ide o vzdialenost najazdenu od
    // posledneho restartu zariadenia, nie o skutocny stav tachometra vozidla.
    if (odometerKm == 0) {
      uint32_t gpsKm = gpsOdometerKm();
      if (gpsKm > 0) { odometerKm = gpsKm; src = "GPS"; }
    }

    // Jeden konsolidovany diagnosticky riadok za cyklus (kazdych 5 s) -
    // ide na Serial (zive sledovanie) aj do existujuceho SD/SPIFFS
    // event logu (logNetEvent), takze je citatelny aj bez pripojenia
    // k pocitacu - stacia vytiahnut SD kartu a pozriet si log subor.
    // Format: ODO FW=<firmware verzia> SRC=<zdroj> KM=<hodnota>
    //         SESS=<ret 1003>:<odp. 1003> R1=<ret 0x02BD @710/77A> RAW1=<surova odp. 0x02BD @710/77A>
    {
      char diag[240];
      snprintf(diag, sizeof(diag), "ODO FW=%s SRC=%s KM=%lu SESS=%d:%s R1=%d RAW1=%s",
          FIRMWARE_VERSION, src, (unsigned long)odometerKm, sessRet1, sessResp1, ret1, rawBuf);
      Serial.print("[ODO] "); Serial.println(diag);
      logNetEvent(diag);
    }

    // Send the same diagnosis as regular telemetry fields (see PID_ODO_DIAG
    // comment above) so it's readable from the server even when this run
    // happens on a real drive with no WiFi/USB nearby.
    {
      int32_t diagCode = !obdLinkUp() ? 0 : (ret1 <= 0 ? 1 : (parsed1 <= 0 ? 2 : 3));
      int32_t diagRet = ret1;
      int32_t diagLen = (int32_t)strlen(rawBuf);
      buffer->add(PID_ODO_DIAG, ELEMENT_INT32, &diagCode, sizeof(diagCode));
      buffer->add(PID_ODO_RET, ELEMENT_INT32, &diagRet, sizeof(diagRet));
      buffer->add(PID_ODO_LEN, ELEMENT_INT32, &diagLen, sizeof(diagLen));

      int sessNrc = 0;
      int32_t sessCode = classifySessionResponse(sessRet1, sessResp1, &sessNrc);
      int32_t sessNrc32 = sessNrc;
      buffer->add(PID_ODO_SESS, ELEMENT_INT32, &sessCode, sizeof(sessCode));
      buffer->add(PID_ODO_SESS_NRC, ELEMENT_INT32, &sessNrc32, sizeof(sessNrc32));

      uint32_t sessRaw1 = packChars4(sessResp1, 0);
      uint32_t sessRaw2 = packChars4(sessResp1, 4);
      uint32_t sessRaw3 = packChars4(sessResp1, 8);
      buffer->add(PID_ODO_SESS_RAW1, ELEMENT_UINT32, &sessRaw1, sizeof(sessRaw1));
      buffer->add(PID_ODO_SESS_RAW2, ELEMENT_UINT32, &sessRaw2, sizeof(sessRaw2));
      buffer->add(PID_ODO_SESS_RAW3, ELEMENT_UINT32, &sessRaw3, sizeof(sessRaw3));

      // Alt 1/2 results (see PID_ODO_SESS2/MON_HIT/MON_LEN comment above).
      int sessNrc2 = 0;
      int32_t sessCode2 = classifySessionResponse(sessRet2, sessResp2, &sessNrc2);
      int32_t monHit = monSawResponseId ? 1 : 0;
      int32_t monLen32 = monLen;
      buffer->add(PID_ODO_SESS2, ELEMENT_INT32, &sessCode2, sizeof(sessCode2));
      buffer->add(PID_ODO_MON_HIT, ELEMENT_INT32, &monHit, sizeof(monHit));
      buffer->add(PID_ODO_MON_LEN, ELEMENT_INT32, &monLen32, sizeof(monLen32));

      // Alt 3 result - not a 1003 response, so classifySessionResponse()
      // (which only recognizes SID 0x50/0x7F) doesn't apply here; alt3Hit
      // (did "77A" show up at all) is the actually-meaningful signal.
      // 0 = no link, 1 = no/timeout response, 2 = got a response but no
      // "77A" header in it, 3 = "77A" header present (real hit).
      int32_t alt3Code = !obdLinkUp() ? 0 : (alt3Ret <= 0 ? 1 : (alt3Hit ? 3 : 2));
      int32_t alt3Hit32 = alt3Hit ? 1 : 0;
      buffer->add(PID_ODO_ALT3_SESS, ELEMENT_INT32, &alt3Code, sizeof(alt3Code));
      buffer->add(PID_ODO_ALT3_HIT, ELEMENT_INT32, &alt3Hit32, sizeof(alt3Hit32));

      if (chipIdResp[0]) {
        uint32_t chip1 = packChars4(chipIdResp, 0);
        uint32_t chip2 = packChars4(chipIdResp, 4);
        uint32_t chip3 = packChars4(chipIdResp, 8);
        buffer->add(PID_CHIP_ID1, ELEMENT_UINT32, &chip1, sizeof(chip1));
        buffer->add(PID_CHIP_ID2, ELEMENT_UINT32, &chip2, sizeof(chip2));
        buffer->add(PID_CHIP_ID3, ELEMENT_UINT32, &chip3, sizeof(chip3));
      }
    }

    // Second diag line, alt-approach results only (kept separate from the
    // main ODO line above so neither exceeds FileLogger's ~132 char/line
    // truncation limit). Format: ODO2 SESS2=<ret>:<resp> MON=<len>:<hit 0/1>
    // CHIPID=<ATI response, if queried this boot>
    if (obdLinkUp()) {
      char diag2[200];
      snprintf(diag2, sizeof(diag2), "ODO2 SESS2=%d:%s MON=%d:%d CHIPID=%s",
          sessRet2, sessResp2, monLen, monSawResponseId ? 1 : 0,
          chipIdResp[0] ? chipIdResp : "-");
      Serial.print("[ODO] "); Serial.println(diag2);
      logNetEvent(diag2);
    }

    // Third diag line, Alt 3 only (kept separate - see ODO2 comment above).
    // Format: ODO3 ALT3=<ret>:<resp>
    if (obdLinkUp()) {
      char diag3[100];
      snprintf(diag3, sizeof(diag3), "ODO3 ALT3=%d:%s", alt3Ret, alt3Resp);
      Serial.print("[ODO] "); Serial.println(diag3);
      logNetEvent(diag3);
    }

    // Fourth diag line, Alt 2's ACTUAL raw captured text (not just length/
    // hit as before - see the Alt 2 restoration comment above for why this
    // was the gap that mattered). Truncated to keep this line under
    // FileLogger's ~132 char limit; monResp itself can hold up to 220.
    // Format: ODO4 MONRAW=<first ~110 chars of monResp>
    if (obdLinkUp()) {
      char diag4[128];
      snprintf(diag4, sizeof(diag4), "ODO4 MONRAW=%.110s", monResp);
      Serial.print("[ODO] "); Serial.println(diag4);
      logNetEvent(diag4);
    }

    if (odometerKm > 0) {
      buffer->add(PID_ODOMETER | 0x100, ELEMENT_INT32, &odometerKm, sizeof(odometerKm));
    }
  }
  // =========================================================================

  // =========================================================================
  // FUEL LEVEL PROCESSING (Passat B8 UDS "Calculated volume", module 17)
  // Same once-per-minute cadence as the odometer block above, for the same
  // reason - several extra AT commands per read, value doesn't change fast
  // enough to justify polling on the 5s tier cycle. Independent of the
  // odometer block (different module/addressing: 0x714/0x77E here vs.
  // 0x710/0x77A for odometer's Gateway).
  // =========================================================================
  static uint32_t lastFuelCheck = 0;
  if (millis() - lastFuelCheck >= 60000) {
    lastFuelCheck = millis();
    long fuelParsed = -1;
    int fuelDeciLiters = 0; // liters * 10, e.g. 475 = 47.5 l
    char fuelRawBuf[24] = "-";
    int fuelRet = 0;
    char fuelSessResp[24] = "-"; // response to 1003 (extended session request)
    int fuelSessRet = 0;
    char fuelSessResp2[24] = "-"; // Alt 1: same request, ATCF/ATCM filter instead of ATCRA
    int fuelSessRet2 = 0;
    char fuelMonResp[220] = "";  // Alt 2: passive ATMA capture right after a fresh 1003
    int fuelMonLen = 0;
    bool fuelMonSawResponseId = false;
    char fuelAlt3Resp[64] = "-"; // Alt 3: ATH1 (no ATCRA), skip 1003, direct 2222B0
    int fuelAlt3Ret = 0;
    bool fuelAlt3Hit = false;

    if (obdLinkUp()) {
      // Confirmed 2026-09-12 via a HexSniff bench capture against a real
      // VCDS session (module "17 - Instruments" -> "Advanced Measuring
      // Values" -> "Calculated volume - Fuel level") - see
      // vehicles/VAG/VW/PassatB8/decoded.md in the HexSniff repo for the
      // raw capture. NOT the same DID as the standard Mode 1 fuel-level
      // PID (see the note next to PID_FUEL_LEVEL in telelogger.ino) - that
      // one gets zero responses on this vehicle; this UDS DID is the one
      // that actually works.
      char ignore[32];
      obdSendCommand("ATSP6\r", ignore, sizeof(ignore), 200);     // standardne CAN 11-bit/500k
      // See the ODO block's comment above - flow control and functional
      // TesterPresent were both tried and both ruled out via real-drive
      // telemetry, removed here too. ATCS was also tried and removed - this
      // ELM327 adapter doesn't support that command at all (answers "?"),
      // confirmed via the SD event log, not just inferred.
      obdSendCommand("ATSH714\r", ignore, sizeof(ignore), 100);   // hlavicka poziadavky -> Pristroje (0x17)
      obdSendCommand("ATCRA77E\r", ignore, sizeof(ignore), 100);  // filtruj len jeho odpoved
      fuelSessResp[0] = 0;
      fuelSessRet = obdSendCommand("1003\r", fuelSessResp, sizeof(fuelSessResp), 200);

      char fuelResponseBuf[200]; // response is 56 bytes total, needs room for hex+spacing
      fuelResponseBuf[0] = 0;
      fuelRet = obdSendCommand("2222B0\r", fuelResponseBuf, sizeof(fuelResponseBuf), 150);
      strncpy(fuelRawBuf, fuelResponseBuf, sizeof(fuelRawBuf) - 1);
      if (fuelRet > 0) {
        // Data block is 54 bytes; "Calculated volume - Fuel level" is the
        // big-endian 2-byte field at offset [33:35] - divide by 10 for
        // liters. Rest of the block (other tank sensor readings) isn't
        // decoded/needed here.
        fuelParsed = parseUdsByteRange(fuelResponseBuf, 0x22B0, 33, 2);
        if (fuelParsed >= 0) { fuelDeciLiters = (int)fuelParsed; }
      }

      // --- Alt 1: ATCF/ATCM instead of ATCRA - see the ODO block's
      // identical alt-test comment above.
      {
        obdSendCommand("ATCF77E\r", ignore, sizeof(ignore), 100);
        obdSendCommand("ATCM7FF\r", ignore, sizeof(ignore), 100);
        fuelSessResp2[0] = 0;
        fuelSessRet2 = obdSendCommand("1003\r", fuelSessResp2, sizeof(fuelSessResp2), 200);
      }

      // --- Alt 2 (ATMA) REMOVED 2026-09-14 - see the ODO block's identical
      // comment above (same suspected total-freeze mechanism, same fix).
      // fuelMonResp/fuelMonLen/fuelMonSawResponseId stay at their declared
      // zero/empty/false values.

      // --- Alt 3: no ATCRA at all - see the ODO block's identical alt-test
      // comment above. Hit looks like CAN ID "77E" prefixing the data.
      {
        obdSendCommand("ATH1\r", ignore, sizeof(ignore), 100);
        obdSendCommand("ATS0\r", ignore, sizeof(ignore), 100);
        fuelAlt3Resp[0] = 0;
        fuelAlt3Ret = obdSendCommand("2222B0\r", fuelAlt3Resp, sizeof(fuelAlt3Resp), 150);
        obdSendCommand("ATH0\r", ignore, sizeof(ignore), 100);
        obdSendCommand("ATS1\r", ignore, sizeof(ignore), 100);
      }
      fuelAlt3Hit = strstr(fuelAlt3Resp, "77E") != nullptr;

      // --- Vratit spat predvolene (11-bit) adresovanie na motor. Bare
      // ATCRA also clears whatever ATCF/ATCM state Alt 1 left behind.
      obdSendCommand("ATCRA\r", ignore, sizeof(ignore), 100);
      obdSendCommand("ATSH7E0\r", ignore, sizeof(ignore), 100);
      obdSendCommand("ATSP0\r", ignore, sizeof(ignore), 200);
    }

    // Format: FUEL FW=<firmware verzia> DL=<decilitre, t.j. l*10>
    //         SESS=<ret 1003>:<odp. 1003> R1=<ret> RAW1=<surova odpoved 0x22B0 @714/77E>
    {
      char diag[240];
      snprintf(diag, sizeof(diag), "FUEL FW=%s DL=%d SESS=%d:%s R1=%d RAW1=%s",
          FIRMWARE_VERSION, fuelDeciLiters, fuelSessRet, fuelSessResp, fuelRet, fuelRawBuf);
      Serial.print("[FUEL] "); Serial.println(diag);
      logNetEvent(diag);
    }

    // See PID_ODO_DIAG comment above - same idea, fuel block.
    {
      int32_t diagCode = !obdLinkUp() ? 0 : (fuelRet <= 0 ? 1 : (fuelParsed < 0 ? 2 : 3));
      int32_t diagRet = fuelRet;
      int32_t diagLen = (int32_t)strlen(fuelRawBuf);
      buffer->add(PID_FUEL_DIAG, ELEMENT_INT32, &diagCode, sizeof(diagCode));
      buffer->add(PID_FUEL_RET, ELEMENT_INT32, &diagRet, sizeof(diagRet));
      buffer->add(PID_FUEL_LEN, ELEMENT_INT32, &diagLen, sizeof(diagLen));

      int fuelSessNrc = 0;
      int32_t fuelSessCode = classifySessionResponse(fuelSessRet, fuelSessResp, &fuelSessNrc);
      int32_t fuelSessNrc32 = fuelSessNrc;
      buffer->add(PID_FUEL_SESS, ELEMENT_INT32, &fuelSessCode, sizeof(fuelSessCode));
      buffer->add(PID_FUEL_SESS_NRC, ELEMENT_INT32, &fuelSessNrc32, sizeof(fuelSessNrc32));

      uint32_t fuelSessRaw1 = packChars4(fuelSessResp, 0);
      uint32_t fuelSessRaw2 = packChars4(fuelSessResp, 4);
      uint32_t fuelSessRaw3 = packChars4(fuelSessResp, 8);
      buffer->add(PID_FUEL_SESS_RAW1, ELEMENT_UINT32, &fuelSessRaw1, sizeof(fuelSessRaw1));
      buffer->add(PID_FUEL_SESS_RAW2, ELEMENT_UINT32, &fuelSessRaw2, sizeof(fuelSessRaw2));
      buffer->add(PID_FUEL_SESS_RAW3, ELEMENT_UINT32, &fuelSessRaw3, sizeof(fuelSessRaw3));

      // Alt 1/2 results (see PID_FUEL_SESS2/MON_HIT/MON_LEN comment above).
      int fuelSessNrc2 = 0;
      int32_t fuelSessCode2 = classifySessionResponse(fuelSessRet2, fuelSessResp2, &fuelSessNrc2);
      int32_t fuelMonHit = fuelMonSawResponseId ? 1 : 0;
      int32_t fuelMonLen32 = fuelMonLen;
      buffer->add(PID_FUEL_SESS2, ELEMENT_INT32, &fuelSessCode2, sizeof(fuelSessCode2));
      buffer->add(PID_FUEL_MON_HIT, ELEMENT_INT32, &fuelMonHit, sizeof(fuelMonHit));
      buffer->add(PID_FUEL_MON_LEN, ELEMENT_INT32, &fuelMonLen32, sizeof(fuelMonLen32));

      // Alt 3 result - see the ODO block's identical PID_ODO_ALT3_SESS
      // comment above for why classifySessionResponse() doesn't apply here.
      int32_t fuelAlt3Code = !obdLinkUp() ? 0 : (fuelAlt3Ret <= 0 ? 1 : (fuelAlt3Hit ? 3 : 2));
      int32_t fuelAlt3Hit32 = fuelAlt3Hit ? 1 : 0;
      buffer->add(PID_FUEL_ALT3_SESS, ELEMENT_INT32, &fuelAlt3Code, sizeof(fuelAlt3Code));
      buffer->add(PID_FUEL_ALT3_HIT, ELEMENT_INT32, &fuelAlt3Hit32, sizeof(fuelAlt3Hit32));
    }

    // Second diag line, alt-approach results only - see the ODO block's
    // identical ODO2 line above for why this is kept separate.
    if (obdLinkUp()) {
      char diag2[200];
      snprintf(diag2, sizeof(diag2), "FUEL2 SESS2=%d:%s MON=%d:%d",
          fuelSessRet2, fuelSessResp2, fuelMonLen, fuelMonSawResponseId ? 1 : 0);
      Serial.print("[FUEL] "); Serial.println(diag2);
      logNetEvent(diag2);
    }

    // Third diag line, Alt 3 only - see the ODO block's identical ODO3 line.
    if (obdLinkUp()) {
      char diag3[100];
      snprintf(diag3, sizeof(diag3), "FUEL3 ALT3=%d:%s", fuelAlt3Ret, fuelAlt3Resp);
      Serial.print("[FUEL] "); Serial.println(diag3);
      logNetEvent(diag3);
    }

    if (fuelDeciLiters > 0) {
      buffer->add(PID_FUEL_LEVEL | 0x100, ELEMENT_INT32, &fuelDeciLiters, sizeof(fuelDeciLiters));
    }
  }
  // =========================================================================
}
