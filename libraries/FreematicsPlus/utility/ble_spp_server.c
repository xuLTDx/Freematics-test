/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


// Compile-time switch: this classic ESP-IDF Bluedroid implementation is the
// DEFAULT and is compiled unless ENABLE_BLE_NIMBLE selects the NimBLE
// alternative instead (see ble_spp_server_nimble.cpp, config.h,
// platformio.ini). PlatformIO's Library Dependency Finder auto-compiles
// every .c/.cpp under this utility/ directory for the current environment
// regardless of which header gets #included, so both this file and
// ble_spp_server_nimble.cpp are always translation units in every build -
// exactly one of the two must actually define ble_init()/ble_send()/
// ble_recv_command()/ble_send_response(), or the link step would fail with
// duplicate symbols. This mirrors the guard in ble_spp_server_nimble.cpp
// (which compiles to nothing unless ENABLE_BLE_NIMBLE is set).
#ifndef ENABLE_BLE_NIMBLE
#define ENABLE_BLE_NIMBLE 0
#endif

#if !ENABLE_BLE_NIMBLE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "string.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "driver/uart.h"
#include "ble_spp_server.h"
#include "esp32-hal-bt.h"

#define GATTS_TABLE_TAG  "GATTS_SPP_SERVER"

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SAMPLE_DEVICE_NAME          "FreematicsPlus"    //The Device Name Characteristics in GAP
#define SPP_SVC_INST_ID	            0

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xFFE1
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xFFE2
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xFFE3
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xFFE4

#ifdef SUPPORT_HEARTBEAT
#define ESP_GATT_UUID_SPP_HEARTBEAT         0xABF5
#endif

static uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02,0x01,0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,0x03,0xF0,0xAB,
    /* Complete Local Name in advertising */
    0x0F,0x09, 'F', 'r', 'e', 'e', 'm', 'a', 't', 'i', 'c', 's', 'P', 'l', 'u', 's'
};

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
static xQueueHandle cmd_cmd_queue = NULL;

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};

// ble_pause()/ble_resume()/ble_isPausedTooLong() state - declared here
// (rather than next to their functions, further down) so ble_init() can set
// ble_initialized without needing a forward declaration.
static bool       ble_initialized      = false;
static bool       ble_paused           = false;
static TickType_t ble_pause_start_tick = 0;

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_WRITE;

///SPP Service - data receive characteristic, read&write without response
//static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
//static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
//static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
//static const uint8_t  spp_data_notify_val[20] = {0x00};
//static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                      	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

#if 0
    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]             	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},
#endif

    //SPP -  command characteristic Declaration
    [SPP_IDX_SPP_COMMAND_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  command characteristic Value
    [SPP_IDX_SPP_COMMAND_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    //SPP -  status characteristic Declaration
    [SPP_IDX_SPP_STATUS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  status characteristic Value
    [SPP_IDX_SPP_STATUS_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_STATUS_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < SPP_IDX_NB ; i++){
        if(handle == spp_handle_table[i]){
            return i;
        }
    }

    return error;
}

void ble_send(int spp_index, void* data, int len)
{
    // spp_index: SPP_IDX_SPP_STATUS_VAL, SPP_IDX_SPP_DATA_NTY_VAL
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[spp_index], len, (uint8_t*)data, false);
}

char* ble_recv_command(int timeout)
{
    char *cmd = 0;
    if(cmd_cmd_queue && xQueueReceive(cmd_cmd_queue, &cmd, timeout / portTICK_PERIOD_MS)) {
        return cmd;
    }
    return 0;
}

void ble_send_response(void* data, int len, char* ptr_to_free)
{
    if (ptr_to_free) free(ptr_to_free);
    if (len > 0) {
        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_STATUS_VAL], len, (uint8_t*)data, false);
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGD(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n",event);
    switch (event) {
    	case ESP_GATTS_REG_EVT:
    	    ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
            res = find_char_and_desr_index(p_data->read.handle);
            if(res == SPP_IDX_SPP_COMMAND_VAL){
                //TODO:client read the status characteristic
                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_COMMAND_VAL], 2, (uint8_t*)"OK", false);
            }
       	 break;
    	case ESP_GATTS_WRITE_EVT: {
    	    res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false){
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
                if(res == SPP_IDX_SPP_COMMAND_VAL){
                    uint8_t * spp_cmd_buff = (uint8_t *)malloc(p_data->write.len + 1);
                    if(spp_cmd_buff == NULL){
                        ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
                        break;
                    }
                    memcpy(spp_cmd_buff,p_data->write.value,p_data->write.len);
                    spp_cmd_buff[p_data->write.len] = 0;
                    if (xQueueSend(cmd_cmd_queue,&spp_cmd_buff,0) == errQUEUE_FULL) {
                        char *temp;
                        xQueueReceive(cmd_cmd_queue, &temp, 0);
                        xQueueSend(cmd_cmd_queue,&spp_cmd_buff,0);
                    }
                }else if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = false;
                    }
                }
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL){
                    int len = p_data->write.len;
                    esp_log_buffer_char(GATTS_TABLE_TAG,(char *)(p_data->write.value),len);
                    //process_cmd(p_data->write.value, len);
                }else{
                    //TODO:
                }
            }else if((p_data->write.is_prep == true)&&(res == SPP_IDX_SPP_DATA_RECV_VAL)){
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
            }
      	 	break;
    	}
    	case ESP_GATTS_EXEC_WRITE_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
    	    if(p_data->exec_write.exec_write_flag){
    	    }
    	    break;
    	}
    	case ESP_GATTS_MTU_EVT:
    	    spp_mtu_size = p_data->mtu.mtu;
    	    break;
    	case ESP_GATTS_CONF_EVT:
    	    break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
    	case ESP_GATTS_CONNECT_EVT:
    	    spp_conn_id = p_data->connect.conn_id;
    	    spp_gatts_if = gatts_if;
    	    is_connected = true;
    	    memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:
    	    is_connected = false;
    	    enable_data_ntf = false;
    	    esp_ble_gap_start_advertising(&spp_adv_params);
    	    break;
    	case ESP_GATTS_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
    	    break;
    	case ESP_GATTS_CONGEST_EVT:
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x\n",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != SPP_IDX_NB){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
    	    }
    	    else {
    	        memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
    	        esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void ble_init(const char* adv_name)
{
    esp_err_t ret;
    //esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    if (adv_name) strncpy((char*)spp_adv_data + 9, adv_name, 13);

    if(!btStarted() && !btStart()){
      ESP_LOGE(GATTS_TABLE_TAG, "%s BT init failed\n", __func__);
      return;
    }

/*
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
*/

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    cmd_cmd_queue = xQueueCreate(4, sizeof(void*));
    ble_initialized = true;
    return;
}

// ---------------------------------------------------------------------------
// ble_pause() / ble_resume() / ble_isPausedTooLong()
// ---------------------------------------------------------------------------
// 2026-09-21: fixes the WiFi/BT coexistence boot loop documented in
// FreematicsNetwork.cpp's ClientWIFI::begin() - WiFi.setSleep(false) (the
// standard fix for WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT(15)/NO_AP_FOUND(201)
// disconnects) aborts the device (coex_core_enable(): "Should enable WiFi
// modem sleep when both WiFi and Bluetooth are enabled!!!!!!" -> abort(),
// called from esp_bt_controller_enable()) whenever the BT controller is
// enabled AT THE SAME TIME WiFi has modem sleep disabled - confirmed live
// twice (Bluedroid 2026-09-20, NimBLE 2026-09-21, identical abort both
// times). The fix: never let those two conditions overlap. ble_pause() is
// called right before WiFi.setSleep(false)+WiFi.begin() starts a connection
// attempt; ble_resume() is called only after WiFi.setSleep(true) has
// restored modem sleep, once the attempt is over (success or failure/
// timeout) - see FreematicsNetwork.cpp and telelogger.ino's onWifiEvent()/
// wifiConnect() for the exact call sites.
//
// Deliberately pauses at the CONTROLLER level only (btStop()/btStart(), the
// same esp32-hal-bt.c helpers ble_init() above already uses to bring the
// controller up) rather than tearing down the Bluedroid HOST
// (esp_bluedroid_disable()/deinit()) as well: btStop() only touches
// esp_bt_controller_disable()+esp_bt_controller_deinit() (verified by
// reading framework-arduinoespressif32's cores/esp32/esp32-hal-bt.c in this
// project's pinned package - it does NOT call any esp_bluedroid_* function),
// so the already-registered GATT profile/app (spp_gatts_if, spp_handle_table,
// built once at boot by ble_init()'s esp_ble_gatts_app_register() ->
// ESP_GATTS_REG_EVT -> esp_ble_gatts_create_attr_tab() chain) is never torn
// down and does not need to be rebuilt on resume - only advertising needs to
// be explicitly restarted, exactly like the existing ESP_GATTS_DISCONNECT_EVT
// handler above already does after a normal client disconnect. This makes
// the pause/resume cycle cheap and safely repeatable (no re-registration
// race, no heap churn) across many WiFi reconnects per session.
//
// Because the controller (radio) is fully powered off while paused, any BLE
// client (the phone app) connected at pause time is necessarily dropped -
// there is no way around that (the radio itself goes away). This resume
// path explicitly clears is_connected/enable_data_ntf (mirroring
// ESP_GATTS_DISCONNECT_EVT's handler exactly) before restarting advertising,
// since no ESP_GATTS_DISCONNECT_EVT fires on its own here (the link didn't
// close cleanly - the controller just vanished), so that stale state can't
// be left behind for the phone's actual reconnect to trip over.
void ble_pause(void)
{
    if (!ble_initialized || ble_paused) return;
    ESP_LOGI(GATTS_TABLE_TAG, "%s pausing BT controller for WiFi (re)connect\n", __func__);
    if (!btStop()) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s btStop() failed - BT controller may still be enabled\n", __func__);
        // Fall through and mark paused anyway: WiFi.setSleep(false) is about
        // to be requested by the caller regardless, and it is safer to at
        // least attempt the resume path later than to silently forget this
        // pause ever happened.
    }
    ble_pause_start_tick = xTaskGetTickCount();
    ble_paused = true;
}

void ble_resume(void)
{
    if (!ble_initialized || !ble_paused) return;
    // Clear the flag FIRST, before doing any of the actual re-init work.
    // ble_resume() is called from two call sites that can run on different
    // FreeRTOS tasks (ClientWIFI::setup()'s success path on the main loop
    // task, and onWifiEvent()'s ARDUINO_EVENT_WIFI_STA_GOT_IP handler on the
    // WiFi event task) for the SAME connection event - confirmed live
    // 2026-09-21 on the NimBLE sibling of this function that both can pass
    // the `!ble_paused` guard above before either reaches the flag-clear at
    // the end, double-entering the BT controller init and boot-looping the
    // device. If btStart() then fails, re-arm the flag so the stale-pause
    // watchdog/a later call still retries - only the success path needs the
    // flag cleared early to close the race.
    ble_paused = false;
    ESP_LOGI(GATTS_TABLE_TAG, "%s resuming BT controller after WiFi (re)connect\n", __func__);
    if (!btStart()) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s btStart() failed - will retry on the next resume trigger\n", __func__);
        ble_paused = true; // re-arm: leave it paused so a later call (or the
                            // stale-pause watchdog) tries again
        return;
    }
    is_connected = false;
    enable_data_ntf = false;
    esp_ble_gap_start_advertising(&spp_adv_params);
}

bool ble_isPausedTooLong(uint32_t maxPauseMs)
{
    if (!ble_paused) return false;
    return (xTaskGetTickCount() - ble_pause_start_tick) * portTICK_PERIOD_MS > maxPauseMs;
}

#endif // !ENABLE_BLE_NIMBLE
