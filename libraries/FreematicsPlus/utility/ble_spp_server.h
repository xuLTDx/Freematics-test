/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


/*
 * DEFINES
 ****************************************************************************************
 */
//#define SUPPORT_HEARTBEAT
#define SPP_DEBUG_MODE

#define spp_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)
///Attributes State Machine
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

#ifdef SUPPORT_HEARTBEAT
    SPP_IDX_SPP_HEARTBEAT_CHAR,
    SPP_IDX_SPP_HEARTBEAT_VAL,
    SPP_IDX_SPP_HEARTBEAT_CFG,
#endif

    SPP_IDX_NB,
};

void ble_init(const char* adv_name);
void ble_send(int spp_index, void* data, int len);
char* ble_recv_command(int timeout);
void ble_send_response(void* data, int len, char* ptr_to_free);

// 2026-09-21: BT-controller pause/resume for the WiFi/BT coexistence fix -
// see the ble_pause()/ble_resume()/ble_isPausedTooLong() comment block above
// their definitions in ble_spp_server.c (this backend) and
// ble_spp_server_nimble.cpp (the NimBLE backend, same interface) for the
// full story. All three are safe to call unconditionally (no-ops) even if
// ble_init() was never called (BLE compiled out or disabled at runtime).
//
//   ble_pause()   - fully disables the BT controller. Call right before a
//                   WiFi (re)connection attempt (WiFi.begin()) starts.
//   ble_resume()  - re-enables the BT controller and resumes advertising.
//                   Call once the attempt is definitively over (connected,
//                   or given up) - and only AFTER WiFi.setSleep(true) has
//                   already restored modem sleep, never before (the whole
//                   point of pausing BT is to let WiFi.setSleep(false) be
//                   in effect without BT enabled at the same time - see
//                   FreematicsNetwork.cpp's ClientWIFI::begin()/setup()).
//   ble_isPausedTooLong(maxPauseMs) - pure query (no side effect): true if
//                   ble_pause() has been in effect longer than maxPauseMs
//                   without a matching ble_resume(). Safety-net backstop so
//                   a WiFi outage (or a code path that never calls
//                   ble_resume() itself) can't strand BT disabled forever -
//                   the caller (wifiConnect() in telelogger.ino) is
//                   responsible for restoring WiFi.setSleep(true) before
//                   calling ble_resume() when this returns true.
void ble_pause(void);
void ble_resume(void);
bool ble_isPausedTooLong(uint32_t maxPauseMs);
