/*
 * ble_spp_server_nimble.h
 *
 * ============================================================================
 *  BLE SPP server implementation, built against the h2zero/NimBLE-Arduino
 *  library. The only implementation as of 2026-09-21 - the classic
 *  ESP-IDF Bluedroid implementation (ble_spp_server.c/.h) was removed
 *  (dead weight, superseded by this one). Verified working on real
 *  hardware (Freematics Controller App) 2026-09-21.
 * ============================================================================
 *
 * Reimplemented against the h2zero/NimBLE-Arduino library instead of raw
 * ESP-IDF Bluedroid, for a smaller RAM footprint that coexists with WiFi.
 *
 * External interface preserved exactly (verified against every call site in
 * firmware_v5/telelogger/telelogger.ino and firmware_v5/datalogger/
 * datalogger.ino - both only ever call ble_init(), ble_recv_command(), and
 * ble_send_response(); ble_send() itself is unused by either .ino today but
 * is kept for interface parity since it's part of the public header):
 *
 *   void  ble_init(const char* adv_name);
 *   void  ble_send(int spp_index, void* data, int len);
 *   char* ble_recv_command(int timeout);
 *   void  ble_send_response(void* data, int len, char* ptr_to_free);
 *
 * Ownership/threading contracts are unchanged from the original:
 *  - ble_recv_command(timeout) blocks up to `timeout` ms on a FreeRTOS queue
 *    and returns either NULL or a malloc()'d, NUL-terminated string that the
 *    CALLER owns (telelogger.ino's processBLE() passes it straight into
 *    ble_send_response() as ptr_to_free, which frees it).
 *  - ble_send_response() frees ptr_to_free unconditionally (if non-NULL),
 *    then - if len > 0 - notifies the response bytes out over BLE.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// spp_index values accepted by ble_send(). Only STATUS is actually wired to
// a real characteristic in this implementation (see .cpp header comment for
// why - mirrors the original, where the DATA notify/receive characteristics
// were #if 0'd out of the GATT table and never created either).
enum {
    SPP_IDX_SPP_STATUS_VAL = 0,
    SPP_IDX_SPP_DATA_NTY_VAL = 1,
};

void  ble_init(const char* adv_name);
void  ble_send(int spp_index, void* data, int len);
char* ble_recv_command(int timeout);
void  ble_send_response(void* data, int len, char* ptr_to_free);

// 2026-09-21: BT-controller pause/resume for the WiFi/BT coexistence fix -
// see the comment block above their definitions in ble_spp_server_nimble.cpp
// for the full story (why a full NimBLEDevice::deinit()/init() cycle is used
// rather than a raw esp_bt_controller_disable()/enable(), and how GATT
// re-registration on resume was verified against the actual NimBLE-Arduino
// 1.4.1 source. See the call-site contract (when to call each, and the
// WiFi.setSleep() ordering requirement) at those call sites in
// telelogger.ino/FreematicsNetwork.cpp.
void  ble_pause(void);
void  ble_resume(void);
bool  ble_isPausedTooLong(uint32_t maxPauseMs);

#ifdef __cplusplus
}
#endif
