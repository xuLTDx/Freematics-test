// Sniff mode (2026-10-11): the former HexSniff firmware as a mode of the
// telelogger, switched on and off by command - no firmware change needed.
//
// While on: the ESP32 CAN controller (the module on the Molex pins, CTX =
// GPIO26, CRX = GPIO34, shared with canodo.cpp) runs in LISTEN-ONLY mode -
// it never transmits and never acknowledges - and every frame goes to SD,
// /SNIFF/S<n>.LOG (n above every file already there, none ever deleted), one line per frame
// in the HexSniff format:
//     <ms>.<us>,<ID hex>,<s|x>,<dlc>,<data hex>[,TX]
// with "# stats" lines (frames, lost before SD, controller missed/overrun,
// bus errors) every minute, so a log is known to be complete or not.
// The bus speed is found automatically (500, then 250, 125 kbit/s).
//
// The telelogger side (telelogger.ino) stops everything that would transmit
// on the bus - OBD requests through the co-processor and the odometer reads -
// and does not go to standby, so a VCDS session with the engine off is
// recorded too. GPS, telemetry and OTA keep running. The mode survives a
// restart and switches itself off after the given minutes (default
// SNIFF_DEFAULT_MIN), so a forgotten sniff cannot keep the box awake on the
// car battery.
//
// Commands: /api/control?cmd=SNIFF=1[,<min>] | SNIFF=0 | SNIFF? |
//   SNIFF_SEND=<id hex>,<data hex> | SNIFF_VEH=<tag>, and the same SNIFF=...
//   as a server command (EV=5 ...,CMD=SNIFF=1,120 - teleclient.cpp).
// Files: /api/sniff/list, /api/sniff/<n>, live view /api/sniff/live.
#pragma once
#include <Arduino.h>

#define SNIFF_DEFAULT_MIN 120
#define SNIFF_MAX_MIN     1440

// setup(), after NVS is open: true when the mode was on before the restart
// (the caller then must not start canodo - sniffTick() takes the controller)
bool sniffInit();
// any task: ask for the mode on (minutes, 0 = default) or off; applied by
// the next sniffTick()
void sniffRequest(bool on, uint16_t minutes);
// the task that owns the CAN controller (process()): applies a request,
// probes the bus speed, counts down the auto-off.
// Returns SNIFF_EV_STARTED / SNIFF_EV_STOPPED on a change, else 0.
#define SNIFF_EV_STARTED 1
#define SNIFF_EV_STOPPED 2
int sniffTick();
// stop at once (standby() on the same task)
void sniffStopNow();
bool sniffActive();
// on, or asked to come on (before the first sniffTick() after a restart):
// initialize() must not start OBD then
bool sniffWanted();
uint16_t sniffMinutesLeft();
uint32_t sniffFileNo();

// web / API (httpd task)
int sniffStatus(char* buf, int size);                          // JSON
int sniffLive(char* buf, int size, uint32_t id, uint32_t mask); // text, last frames
// one frame, single shot, in normal mode for that frame only; the frames of
// the next second are written to out
int sniffSend(uint32_t id, const uint8_t* data, int len, char* out, int cap);
bool sniffSetVehicle(const char* tag);
