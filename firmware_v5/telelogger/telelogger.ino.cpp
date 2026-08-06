# 1 "C:\\Users\\DereC\\AppData\\Local\\Temp\\tmpwmmwb869"
#include <Arduino.h>
# 1 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
# 18 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
#include <Arduino.h>
#include <Update.h>
#include <ctype.h>
#include <math.h>
#include <string.h>


extern UpdateClass Update;
#include <FreematicsPlus.h>
#include <httpd.h>
#include <mbedtls/sha256.h>
#include "config.h"
#include "telestore.h"
#include "teleclient.h"
#if BOARD_HAS_PSRAM
#include "esp32/himem.h"
#endif
#include "driver/adc.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_sleep.h"
#if ENABLE_OLED
#include "FreematicsOLED.h"
#endif


#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_GPS_ONLINE 0x20
#define STATE_CELL_CONNECTED 0x40
#define STATE_WIFI_CONNECTED 0x80
#define STATE_WORKING 0x100
#define STATE_STANDBY 0x200

typedef struct {
  byte pid;
  byte tier;
  int value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {

  {PID_SPEED, 1},
  {PID_RPM, 1},
  {PID_THROTTLE, 1},
  {PID_ENGINE_LOAD, 1},

  {PID_FUEL_PRESSURE, 2},
  {PID_TIMING_ADVANCE, 2},
  {PID_INTAKE_MAP, 2},
  {PID_MAF_FLOW, 2},

  {PID_COOLANT_TEMP, 3},
  {PID_INTAKE_TEMP, 3},
  {PID_SHORT_TERM_FUEL_TRIM_1, 3},
  {PID_LONG_TERM_FUEL_TRIM_1, 3},
  {PID_SHORT_TERM_FUEL_TRIM_2, 3},
  {PID_LONG_TERM_FUEL_TRIM_2, 3},
  {PID_RUNTIME, 3},
  {PID_FUEL_LEVEL, 3},
  {PID_BAROMETRIC, 3},
  {PID_CONTROL_MODULE_VOLTAGE, 3},
  {PID_ABSOLUTE_ENGINE_LOAD, 3},
  {PID_RELATIVE_THROTTLE_POS, 3},
  {PID_AMBIENT_TEMP, 3},
  {PID_ACC_PEDAL_POS_D, 3},
  {PID_ACC_PEDAL_POS_E, 3},
  {PID_ENGINE_OIL_TEMP, 3},
  {PID_ETHANOL_FUEL, 3},
  {PID_HYBRID_BATTERY_PERCENTAGE, 3},
  {PID_ENGINE_FUEL_RATE, 3},
  {PID_REL_ACCEL_PEDAL, 3},
# 102 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
  {PID_CATALYST_TEMP_B1S1, 3},
  {PID_CATALYST_TEMP_B2S1, 3},
  {PID_ENGINE_TORQUE_DEMANDED, 3},
  {PID_ENGINE_TORQUE_PERCENTAGE, 3},
  {PID_ENGINE_REF_TORQUE, 3},
};

CBufferManager bufman;
Task subtask;

#if ENABLE_MEMS
float accBias[3] = {0};
float accSum[3] = {0};
float acc[3] = {0};
float gyr[3] = {0};
float mag[3] = {0};
uint8_t accCount = 0;
#endif
int deviceTemp = 0;


char apn[32];
char simPin[16] = SIM_CARD_PIN;
#if ENABLE_WIFI
char wifiSSID[32] = WIFI_SSID;
char wifiPassword[32] = WIFI_PASSWORD;
char wifiSSID2[32] = WIFI_SSID2;
char wifiPassword2[32] = WIFI_PASSWORD2;




uint8_t wifiCurrentIdx = 0;
# 143 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
void wifiConnect();
void wifiReconnectCurrent();
void httpControlStandby(bool enter);
bool httpIsStandby();
void printTimeoutStats();
void beep(int duration);
void processExtInputs(CBuffer* buffer);
int handlerLiveData(UrlHandlerParam* param);
void processOBD(CBuffer* buffer);
bool initGPS();
static double haversineKm(float lat1, float lng1, float lat2, float lng2);
bool waitMotionGPS(int timeout);
void calibrateMEMS();
void printTime();
void initialize();
void showStats();
bool waitMotion(long timeout);
void process();
void telemetry(void* inst);
void standby();
void genDeviceID(char* buf);
void showSysInfo();
void loadConfig();
static bool _applyNvsFromSD();
static String _maskOtaHost(const char* host);
void printOtaStatus();
void setup();
void loop();
#line 143 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
void wifiConnect()
{
  static uint8_t wifiNetIdx = 0;
  const char* ssid = "";
  const char* pass = "";
  for (int tries = 0; tries < 2; tries++) {
    uint8_t thisIdx = wifiNetIdx;
    ssid = thisIdx == 0 ? wifiSSID : wifiSSID2;
    pass = thisIdx == 0 ? wifiPassword : wifiPassword2;
    wifiNetIdx ^= 1;
    if (ssid[0]) { wifiCurrentIdx = thisIdx; break; }
  }
  if (!ssid[0]) return;
  Serial.print("WIFI:");
  Serial.println(ssid);
  teleClient.wifi.begin(ssid, pass);
}






void wifiReconnectCurrent()
{
  const char* ssid = wifiCurrentIdx == 0 ? wifiSSID : wifiSSID2;
  const char* pass = wifiCurrentIdx == 0 ? wifiPassword : wifiPassword2;
  if (!ssid[0]) return;
  Serial.print("WIFI:");
  Serial.println(ssid);
  teleClient.wifi.begin(ssid, pass);
}
#endif



char serverHost[128] = SERVER_HOST;
uint16_t serverPort = SERVER_PORT;


char webhookPath[256] = "";
# 195 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
char cellServerHost[128] = "";
uint16_t cellServerPort = 443;
char cellWebhookPath[256] = "";




uint8_t enableHttpd = ENABLE_HTTPD;




uint8_t enableBle = 1;
nvs_handle_t nvs;




char nvsVersion[64] = "";
# 223 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
char otaToken[68] = "";



char otaHost[128] = "";
uint16_t otaPort = 443;

uint16_t otaCheckIntervalS = 0;


String netop;
String ip;
int16_t rssi = 0;
int16_t rssiLast = 0;
char vin[18] = {0};
uint16_t dtc[6] = {0};
float batteryVoltage = 0;
GPS_DATA* gd = 0;

char devid[12] = {0};
char isoTime[32] = {0};


uint32_t lastMotionTime = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;
uint32_t lastStatsTime = 0;

int32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;
int32_t dataInterval = 1000;

#if STORAGE != STORAGE_NONE
int fileid = 0;
uint16_t lastSizeKB = 0;
#endif

byte ledMode = 0;




bool enableLedRed = true;
bool enableLedWhite = true;
bool enableBeep = true;





bool enableObd = true;
bool enableCan = false;




bool enableDeepStandby = false;



char vehicleMake[32] = "";
char vehicleModel[32] = "";
char vehicleYear[8] = "";



char vehiclePidsStr[128] = "";

#define MAX_VEHICLE_PIDS 16
PID_POLLING_INFO vehicleObdData[MAX_VEHICLE_PIDS];
int vehicleObdDataCount = 0;





uint16_t nvsStandbyTimeS = 0;
# 307 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
#define CAN_DATA_LIST_MAX 32



char s_canFrameList[CAN_DATA_LIST_MAX][20];
int s_canFrameCount = 0;
uint32_t s_canFrameTotal = 0;
portMUX_TYPE s_canBufMux = portMUX_INITIALIZER_UNLOCKED;


static bool s_canSniffActive = false;



volatile bool s_ota_active = false;






#define OTA_PENDING_PATH "/ota_fw.bin"
#define OTA_META_PATH "/ota_meta.txt"
#define OTA_NVS_PATH "/ota_nvs.bin"




static volatile bool s_ota_pending = false;
# 346 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
static int8_t s_lastLedWhite = -1;
static int8_t s_lastBeep = -1;
static int8_t s_lastConnType = -1;
static int8_t s_lastObd = -1;
static int8_t s_lastCan = -1;
static int16_t s_lastStandbyTime = -1;
static int8_t s_lastDeepStandby = -1;
# 363 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
static volatile bool s_send_state_pids = false;




static uint32_t s_cachedSdTotalMb = 0;
static uint32_t s_cachedSdFreeMb = 0;






#define PULL_OTA_MIN_FW_SIZE 65536U

#define PULL_OTA_CHUNK_SIZE 4096U

#define PULL_OTA_CHUNK_TIMEOUT_MS 30000U


#define OTA_TELEMETRY_YIELD_DELAY_MS 1000U



static uint8_t s_otaChunkBuf[PULL_OTA_CHUNK_SIZE];

bool serverSetup(IPAddress& ip);
void serverProcess(int timeout);
void processMEMS(CBuffer* buffer);
bool processGPS(CBuffer* buffer);
void processBLE(int timeout);


bool performPullOtaCheck();
#if STORAGE == STORAGE_SD

static bool performPullOtaFlash();
#endif

class State {
public:
  bool check(uint16_t flags) { return (m_state & flags) == flags; }
  void set(uint16_t flags) { m_state |= flags; }
  void clear(uint16_t flags) { m_state &= ~flags; }
  uint16_t m_state = 0;
};

FreematicsESP32 sys;

class OBD : public COBD
{
protected:
  void idleTasks()
  {

#if ENABLE_MEMS
    processMEMS(0);
#endif
    processBLE(0);
  }
};

OBD obd;

MEMS_I2C* mems = 0;

#if STORAGE == STORAGE_SPIFFS
SPIFFSLogger logger;
#elif STORAGE == STORAGE_SD
SDLogger logger;
#endif

#if SERVER_PROTOCOL == PROTOCOL_UDP
TeleClientUDP teleClient;
#else
TeleClientHTTP teleClient;
#endif

#if ENABLE_OLED
OLED_SH1106 oled;
#endif

State state;
# 454 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
static volatile bool s_http_standby_enter = false;
static volatile bool s_http_standby_exit = false;



void httpControlStandby(bool enter) {
    if (enter) {
        s_http_standby_enter = true;
        s_http_standby_exit = false;
    } else {
        s_http_standby_exit = true;
        s_http_standby_enter = false;
    }
}



bool httpIsStandby() {
    return state.check(STATE_STANDBY) || s_http_standby_enter;
}

void printTimeoutStats()
{
  Serial.print("Timeouts: OBD:");
  Serial.print(timeoutsOBD);
  Serial.print(" Network:");
  Serial.println(timeoutsNet);
}

void beep(int duration)
{

    sys.buzzer(2000);
    delay(duration);

    sys.buzzer(0);
}

#if LOG_EXT_SENSORS
void processExtInputs(CBuffer* buffer)
{
#if LOG_EXT_SENSORS == 1
  uint8_t levels[2] = {(uint8_t)digitalRead(PIN_SENSOR1), (uint8_t)digitalRead(PIN_SENSOR2)};
  buffer->add(PID_EXT_SENSORS, ELEMENT_UINT8, levels, sizeof(levels), 2);
#elif LOG_EXT_SENSORS == 2
  uint16_t reading[] = {adc1_get_raw(ADC1_CHANNEL_0), adc1_get_raw(ADC1_CHANNEL_1)};
  Serial.print("GPIO0:");
  Serial.print((float)reading[0] * 3.15 / 4095 - 0.01);
  Serial.print(" GPIO1:");
  Serial.println((float)reading[1] * 3.15 / 4095 - 0.01);
  buffer->add(PID_EXT_SENSORS, ELEMENT_UINT16, reading, sizeof(reading), 2);
#endif
}
#endif




#if ENABLE_HTTPD
int handlerLiveData(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int n = snprintf(buf, bufsize, "{\"obd\":{\"vin\":\"%s\",\"battery\":%.1f,\"pid\":[", vin, batteryVoltage);
    uint32_t t = millis();
    for (int i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | obdData[i].pid, obdData[i].value, (unsigned int)(t - obdData[i].ts));
    }
    n--;
    n += snprintf(buf + n, bufsize - n, "]}");
#if ENABLE_MEMS
    if (accCount) {
      n += snprintf(buf + n, bufsize - n, ",\"mems\":{\"acc\":[%d,%d,%d],\"stationary\":%u}",
          (int)((accSum[0] / accCount - accBias[0]) * 100), (int)((accSum[1] / accCount - accBias[1]) * 100), (int)((accSum[2] / accCount - accBias[2]) * 100),
          (unsigned int)(millis() - lastMotionTime));
    }
#endif
    if (gd && gd->ts) {
      n += snprintf(buf + n, bufsize - n, ",\"gps\":{\"utc\":\"%s\",\"lat\":%f,\"lng\":%f,\"alt\":%f,\"speed\":%f,\"sat\":%d,\"age\":%u}",
          isoTime, gd->lat, gd->lng, gd->alt, gd->speed, (int)gd->sat, (unsigned int)(millis() - gd->ts));
    }
    buf[n++] = '}';
    param->contentLength = n;
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}
#endif




#if ENABLE_OBD



static long parseUdsHexValue(const char* resp, uint16_t did);




uint32_t gpsOdometerKm();

void processOBD(CBuffer* buffer)
{
  static int idx[2] = {0, 0};
  int tier = 1;
  for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
    if (obdData[i].tier > tier) {

        idx[tier - 2] = 0;

        tier = obdData[i].tier;

        i += idx[tier - 2]++;

        if (obdData[i].tier != tier) {
            idx[tier - 2]= 0;
            i--;
            continue;
        }
    }
    byte pid = obdData[i].pid;
    if (!obd.isValidPID(pid)) continue;
    int value;
    if (obd.readPID(pid, value)) {
        obdData[i].ts = millis();
        obdData[i].value = value;
        buffer->add((uint16_t)pid | 0x100, ELEMENT_INT32, &value, sizeof(value));
    } else {
        timeoutsOBD++;
        printTimeoutStats();
        break;
    }
    if (tier > 1) break;
  }



  if (vehicleObdDataCount > 0) {
    static int vehiclePidIdx = 0;
    if (vehiclePidIdx >= vehicleObdDataCount) vehiclePidIdx = 0;
    byte vpid = vehicleObdData[vehiclePidIdx].pid;
    if (obd.isValidPID(vpid)) {
      int vval;
      if (obd.readPID(vpid, vval)) {
        vehicleObdData[vehiclePidIdx].ts = millis();
        vehicleObdData[vehiclePidIdx].value = vval;
        buffer->add((uint16_t)vpid | 0x100, ELEMENT_INT32, &vval, sizeof(vval));
      }
    }
    vehiclePidIdx++;
  }




  static uint32_t lastOdoCheck = 0;
  if (millis() - lastOdoCheck >= 5000) {
    lastOdoCheck = millis();
    int odoVal = 0;
    uint32_t odometerKm = 0;
    char responseBuf[64];


    char rawBuf[2][20] = { "-", "-" };
    int ret1 = 0, ret2 = 0;
    long parsed1 = -1, parsed2 = -1;
    const char* src = "NONE";

    if (obd.link) {
# 634 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
      char ignore[32];
      obd.link->sendCommand("ATSP7\r", ignore, sizeof(ignore), 200);
      obd.link->sendCommand("ATSH18DA17F1\r", ignore, sizeof(ignore), 100);
      obd.link->sendCommand("ATCRA18DAF117\r", ignore, sizeof(ignore), 100);


      obd.link->sendCommand("1003\r", ignore, sizeof(ignore), 200);


      responseBuf[0] = 0;
      ret1 = obd.link->sendCommand("220505\r", responseBuf, sizeof(responseBuf), 100);
      strncpy(rawBuf[0], responseBuf, sizeof(rawBuf[0]) - 1);
      if (ret1 > 0) {
        parsed1 = parseUdsHexValue(responseBuf, 0x0505);
        if (parsed1 > 0) { odometerKm = (uint32_t)parsed1; src = "UDS0505"; }
      }


      if (odometerKm == 0) {
        responseBuf[0] = 0;
        ret2 = obd.link->sendCommand("222BDC\r", responseBuf, sizeof(responseBuf), 100);
        strncpy(rawBuf[1], responseBuf, sizeof(rawBuf[1]) - 1);
        if (ret2 > 0) {
          parsed2 = parseUdsHexValue(responseBuf, 0x2BDC);
          if (parsed2 > 0) { odometerKm = (uint32_t)(parsed2 / 10); src = "UDS2BDC"; }
        }
      }




      obd.link->sendCommand("ATCRA\r", ignore, sizeof(ignore), 100);
      obd.link->sendCommand("ATSH7E0\r", ignore, sizeof(ignore), 100);
      obd.link->sendCommand("ATSP0\r", ignore, sizeof(ignore), 200);
    }


    if (odometerKm == 0 && obd.readPID(0xA6, odoVal) && odoVal > 0) {
      odometerKm = (uint32_t)odoVal;
      src = "PID_A6";
    }





    if (odometerKm == 0) {
      uint32_t gpsKm = gpsOdometerKm();
      if (gpsKm > 0) { odometerKm = gpsKm; src = "GPS"; }
    }
# 692 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
    {
      char diag[176];
      snprintf(diag, sizeof(diag), "ODO FW=%s SRC=%s KM=%lu R1=%d R2=%d RAW1=%s RAW2=%s",
          FIRMWARE_VERSION, src, (unsigned long)odometerKm, ret1, ret2, rawBuf[0], rawBuf[1]);
      Serial.print("[ODO] "); Serial.println(diag);
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) {
        logger.logEvent(diag);
      }
#endif
    }

    if (odometerKm > 0) {
      buffer->add(PID_ODOMETER | 0x100, ELEMENT_INT32, &odometerKm, sizeof(odometerKm));
    }
  }


  int kph = obdData[0].value;
  if (kph >= 2) lastMotionTime = millis();
}
#endif
# 724 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
static long parseUdsHexValue(const char* resp, uint16_t did)
{
  if (!resp) return -1;
  char hex[32];
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

  if (n <= 6) return -1;
  return strtol(hex + 6, nullptr, 16);
}

bool initGPS()
{

  if (sys.gpsBeginExt()) {
    Serial.println("GNSS:OK(E)");
  } else if (sys.gpsBegin()) {
    Serial.println("GNSS:OK(I)");
  } else {
    Serial.println("GNSS:NO");
    return false;
  }
  return true;
}







static double s_gpsOdometerKm = 0.0;

uint32_t gpsOdometerKm()
{
  return (uint32_t)s_gpsOdometerKm;
}

static double haversineKm(float lat1, float lng1, float lat2, float lng2)
{
  const double R = 6371.0;
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLng = (lng2 - lng1) * DEG_TO_RAD;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
             sin(dLng / 2) * sin(dLng / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

bool processGPS(CBuffer* buffer)
{
  static uint32_t lastGPStime = 0;
  static float lastGPSLat = 0;
  static float lastGPSLng = 0;

  if (!gd) {
    lastGPStime = 0;
    lastGPSLat = 0;
    lastGPSLng = 0;
  }
#if GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {

    if (!sys.gpsGetData(&gd)) {
      return false;
    }
  }
#else
    if (!teleClient.cell.getLocation(&gd)) {
      return false;
    }
#endif
  if (!gd || lastGPStime == gd->time) return false;
  if (gd->date) {

    char *p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
        (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
        (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100);
    unsigned char tenth = (gd->time % 100) / 10;
    if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
    *p = 'Z';
    *(p + 1) = 0;
  }
  if (gd->lng == 0 && gd->lat == 0) {



    if (buffer) {
      if (gd->sat) buffer->add(PID_GPS_SAT_COUNT, ELEMENT_UINT8, &gd->sat, sizeof(uint8_t));
      if (gd->hdop) buffer->add(PID_GPS_HDOP, ELEMENT_UINT8, &gd->hdop, sizeof(uint8_t));
    }
    if (gd->date) {
      Serial.print("[GNSS] ");
      Serial.print(isoTime);
      Serial.print(" SATS:");
      Serial.println(gd->sat);
    }
    return false;
  }
  if ((lastGPSLat || lastGPSLng) && (abs(gd->lat - lastGPSLat) > 0.001 || abs(gd->lng - lastGPSLng) > 0.001)) {

    lastGPSLat = 0;
    lastGPSLng = 0;
    return false;
  }
  if (lastGPSLat || lastGPSLng) {
    s_gpsOdometerKm += haversineKm(lastGPSLat, lastGPSLng, gd->lat, gd->lng);
  }
  lastGPSLat = gd->lat;
  lastGPSLng = gd->lng;

  float kph = gd->speed * 1.852f;
  if (kph >= 2) lastMotionTime = millis();

  if (buffer) {
    buffer->add(PID_GPS_TIME, ELEMENT_UINT32, &gd->time, sizeof(uint32_t));
    buffer->add(PID_GPS_LATITUDE, ELEMENT_FLOAT, &gd->lat, sizeof(float));
    buffer->add(PID_GPS_LONGITUDE, ELEMENT_FLOAT, &gd->lng, sizeof(float));
    buffer->add(PID_GPS_ALTITUDE, ELEMENT_FLOAT_D1, &gd->alt, sizeof(float));
    buffer->add(PID_GPS_SPEED, ELEMENT_FLOAT_D1, &kph, sizeof(kph));
    buffer->add(PID_GPS_HEADING, ELEMENT_UINT16, &gd->heading, sizeof(uint16_t));
    if (gd->sat) buffer->add(PID_GPS_SAT_COUNT, ELEMENT_UINT8, &gd->sat, sizeof(uint8_t));
    if (gd->hdop) buffer->add(PID_GPS_HDOP, ELEMENT_UINT8, &gd->hdop, sizeof(uint8_t));
  }

  Serial.print("[GNSS] ");
  Serial.print(gd->lat, 6);
  Serial.print(' ');
  Serial.print(gd->lng, 6);
  Serial.print(' ');
  Serial.print((int)kph);
  Serial.print("km/h");
  Serial.print(" SATS:");
  Serial.print(gd->sat);
  Serial.print(" HDOP:");
  Serial.print(gd->hdop);
  Serial.print(" Course:");
  Serial.println(gd->heading);

  lastGPStime = gd->time;
  return true;
}

bool waitMotionGPS(int timeout)
{
  unsigned long t = millis();
  lastMotionTime = 0;
  do {
      serverProcess(100);
    if (!processGPS(0)) continue;
    if (lastMotionTime) return true;
  } while (millis() - t < timeout);
  return false;
}

#if ENABLE_MEMS
void processMEMS(CBuffer* buffer)
{
  if (!state.check(STATE_MEMS_READY)) return;


  float temp;
#if ENABLE_ORIENTATION
  ORIENTATION ori;
  if (!mems->read(acc, gyr, mag, &temp, &ori)) return;
#else
  if (!mems->read(acc, gyr, mag, &temp)) return;
#endif
  deviceTemp = (int)temp;

  accSum[0] += acc[0];
  accSum[1] += acc[1];
  accSum[2] += acc[2];
  accCount++;
# 919 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
  {
    float motion = 0;
    for (byte i = 0; i < 3; i++) {
      float m = acc[i] - accBias[i];
      motion += m * m;
    }
    if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
      lastMotionTime = millis();
    }
  }

  if (buffer) {
    if (accCount) {
      float value[3];
      value[0] = accSum[0] / accCount - accBias[0];
      value[1] = accSum[1] / accCount - accBias[1];
      value[2] = accSum[2] / accCount - accBias[2];
      buffer->add(PID_ACC, ELEMENT_FLOAT_D2, value, sizeof(value), 3);
#if ENABLE_ORIENTATION
      value[0] = ori.yaw;
      value[1] = ori.pitch;
      value[2] = ori.roll;
      buffer->add(PID_ORIENTATION, ELEMENT_FLOAT_D2, value, sizeof(value), 3);
#endif
    }
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accCount = 0;
  }
}

void calibrateMEMS()
{
  if (state.check(STATE_MEMS_READY)) {
    accBias[0] = 0;
    accBias[1] = 0;
    accBias[2] = 0;
    int n;
    unsigned long t = millis();
    for (n = 0; millis() - t < 1000; n++) {
      float acc[3];
      if (!mems->read(acc)) continue;
      accBias[0] += acc[0];
      accBias[1] += acc[1];
      accBias[2] += acc[2];
      delay(10);
    }
    accBias[0] /= n;
    accBias[1] /= n;
    accBias[2] /= n;
    Serial.print("ACC BIAS:");
    Serial.print(accBias[0]);
    Serial.print('/');
    Serial.print(accBias[1]);
    Serial.print('/');
    Serial.println(accBias[2]);
  }
}
#endif

void printTime()
{
  time_t utc;
  time(&utc);
  struct tm *btm = gmtime(&utc);
  if (btm->tm_year > 100) {

    char buf[64];
    sprintf(buf, "%04u-%02u-%02u %02u:%02u:%02u",
      1900 + btm->tm_year, btm->tm_mon + 1, btm->tm_mday, btm->tm_hour, btm->tm_min, btm->tm_sec);
    Serial.print("UTC:");
    Serial.println(buf);
  }
}




void initialize()
{

  bufman.purge();
# 1011 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
  s_lastLedWhite = -1;
  s_lastBeep = -1;
  s_lastConnType = -1;
  s_lastObd = -1;
  s_lastCan = -1;
  s_lastStandbyTime = -1;


  s_send_state_pids = true;

#if ENABLE_MEMS
  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
  }
#endif

#if GNSS == GNSS_STANDALONE
  if (!state.check(STATE_GPS_READY)) {
    if (initGPS()) {
      state.set(STATE_GPS_READY);
    }
  }
#endif

#if ENABLE_OBD

  if (enableObd && !state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
# 1048 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
    if (obd.link) {
      char wbuf[64];
      obd.link->sendCommand("ATZ\r", wbuf, sizeof(wbuf), 1000);
      obd.link->sendCommand("ATE0\r", wbuf, sizeof(wbuf), 500);
      obd.link->sendCommand("ATH0\r", wbuf, sizeof(wbuf), 500);
      obd.link->sendCommand("010C\r", wbuf, sizeof(wbuf), 500);
      obd.link->sendCommand("0105\r", wbuf, sizeof(wbuf), 500);
      Serial.println("OBD:pre-wake sent");
      delay(100);
    }
    if (obd.init()) {
      Serial.println("OBD:OK");
      state.set(STATE_OBD_READY);
#if ENABLE_OLED
      oled.println("OBD OK");
#endif
    } else {
      Serial.println("OBD:NO");


    }
  }




  if (enableCan != s_canSniffActive) {
    obd.sniff(enableCan);
    s_canSniffActive = enableCan;
    Serial.println(enableCan ? "CAN:sniff on" : "CAN:sniff off");
    if (enableCan) {



      byte wakeFrame[] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      char rxbuf[32];
      obd.setCANID(0x7DF);
      obd.sendCANMessage(wakeFrame, 8, rxbuf, sizeof(rxbuf));
      Serial.println("CAN:wake-up sent");
    }
  }
#endif

#if STORAGE != STORAGE_NONE
  if (!state.check(STATE_STORAGE_READY)) {

    if (logger.init()) {
      state.set(STATE_STORAGE_READY);
    }
  }
  if (state.check(STATE_STORAGE_READY)) {
    fileid = logger.begin();
    if (fileid) {



      char diag[128];
      logger.timestamp(millis());
      snprintf(diag, sizeof(diag), "BOOT FW=%s ID=%s", FIRMWARE_VERSION, devid);
      logger.logEvent(diag);
      snprintf(diag, sizeof(diag), "STATE OBD=%c GPS=%c MEMS=%c",
          state.check(STATE_OBD_READY) ? '1' : '0',
          state.check(STATE_GPS_READY) ? '1' : '0',
          state.check(STATE_MEMS_READY) ? '1' : '0');
      logger.logEvent(diag);
#if ENABLE_WIFI
      snprintf(diag, sizeof(diag), "WIFI SSID1=%s SSID2=%s",
          wifiSSID[0] ? wifiSSID : "-", wifiSSID2[0] ? wifiSSID2 : "-");
      logger.logEvent(diag);
#endif
      logger.flush();
    }
  }
#endif

#if STORAGE == STORAGE_SD





  if (state.check(STATE_STORAGE_READY)) {
    if (SD.exists(OTA_META_PATH)) {
      unsigned long expectedSize = 0;
      {
        File mf = SD.open(OTA_META_PATH, FILE_READ);
        if (mf) {
          char buf[16] = {0};
          mf.readBytesUntil('\n', buf, sizeof(buf) - 1);
          mf.close();
          expectedSize = strtoul(buf, nullptr, 10);
        }
      }
      bool stagingValid = false;
      if (expectedSize >= PULL_OTA_MIN_FW_SIZE && SD.exists(OTA_PENDING_PATH)) {
        File ff = SD.open(OTA_PENDING_PATH, FILE_READ);
        unsigned long actual = ff ? (unsigned long)ff.size() : 0UL;
        if (ff) ff.close();
        stagingValid = (actual == expectedSize);
      }
      if (stagingValid) {
        Serial.println("[OTA-PULL] Staged firmware found on SD — flashing at boot");
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL BOOT_FLASH");




        if (performPullOtaFlash()) {

          while (true) delay(1000);
        }


      } else {
        SD.remove(OTA_PENDING_PATH);
        SD.remove(OTA_META_PATH);
        SD.remove(OTA_NVS_PATH);
        Serial.println("[OTA-PULL] Stale/incomplete SD staging files removed");
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL STALE_REMOVED");
      }
    } else if (SD.exists(OTA_PENDING_PATH)) {

      SD.remove(OTA_PENDING_PATH);
      SD.remove(OTA_NVS_PATH);
    } else {

      Serial.println("[OTA] SD:none");
    }
  }
#endif


#if ENABLE_OBD
  if (state.check(STATE_OBD_READY)) {
    char buf[128];
    if (obd.getVIN(buf, sizeof(buf))) {
      memcpy(vin, buf, sizeof(vin) - 1);
      Serial.print("VIN:");
      Serial.println(vin);
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) {
        char diag[32];
        snprintf(diag, sizeof(diag), "VIN=%s", vin);
        logger.logEvent(diag);
      }
#endif
    }
    int dtcCount = obd.readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      Serial.print("DTC:");
      Serial.println(dtcCount);
    }
#if ENABLE_OLED
    oled.print("VIN:");
    oled.println(vin);
#endif
  }
#endif


  printTime();

  lastMotionTime = millis();
  state.set(STATE_WORKING);

#if ENABLE_OLED
  delay(1000);
  oled.clear();
  oled.print("DEVICE ID: ");
  oled.println(devid);
  oled.setCursor(0, 7);
  oled.print("Packets");
  oled.setCursor(80, 7);
  oled.print("KB Sent");
  oled.setFontSize(FONT_SIZE_MEDIUM);
#endif
}

void showStats()
{
  uint32_t t = millis() - teleClient.startTime;
  char buf[32];
  sprintf(buf, "%02u:%02u.%c ", t / 60000, (t % 60000) / 1000, (t % 1000) / 100 + '0');
  Serial.print("[NET] ");
  Serial.print(buf);
  Serial.print("| Packet #");
  Serial.print(teleClient.txCount);
  Serial.print(" | Out: ");
  Serial.print(teleClient.txBytes >> 10);
  Serial.print(" KB | In: ");
  Serial.print(teleClient.rxBytes);
  Serial.print(" bytes | ");
  Serial.print((unsigned int)((uint64_t)(teleClient.txBytes + teleClient.rxBytes) * 3600 / (millis() - teleClient.startTime)));
  Serial.print(" KB/h");

  Serial.println();
#if ENABLE_OLED
  oled.setCursor(0, 2);
  oled.println(timestr);
  oled.setCursor(0, 5);
  oled.printInt(teleClient.txCount, 2);
  oled.setCursor(80, 5);
  oled.printInt(teleClient.txBytes >> 10, 3);
#endif
}

bool waitMotion(long timeout)
{
#if ENABLE_MEMS
  unsigned long t = millis();
  if (state.check(STATE_MEMS_READY)) {
    do {

      float motion = 0;
      float acc[3];
      if (!mems->read(acc)) continue;
      if (accCount == 10) {
        accCount = 0;
        accSum[0] = 0;
        accSum[1] = 0;
        accSum[2] = 0;
      }
      accSum[0] += acc[0];
      accSum[1] += acc[1];
      accSum[2] += acc[2];
      accCount++;
      for (byte i = 0; i < 3; i++) {
        float m = (acc[i] - accBias[i]);
        motion += m * m;
      }
#if ENABLE_HTTPD
      serverProcess(100);
#endif
      processBLE(100);

      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {

        Serial.println(motion);
        return true;
      }
    } while (state.check(STATE_STANDBY) && ((long)(millis() - t) < timeout || timeout == -1));
    return false;
  }
#endif
  serverProcess(timeout);
  return false;
}




void process()
{
  static uint32_t lastGPStick = 0;
  uint32_t startTime = millis();

  CBuffer* buffer = bufman.getFree();
  buffer->state = BUFFER_STATE_FILLING;

#if ENABLE_OBD

  if (state.check(STATE_OBD_READY)) {
    processOBD(buffer);
    if (obd.errors >= MAX_OBD_ERRORS) {
      if (!obd.init()) {
        Serial.println("[OBD] ECU OFF");
#if STORAGE != STORAGE_NONE
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OBD ECU_OFF");
#endif
        state.clear(STATE_OBD_READY | STATE_WORKING);
        return;
      }
    }
  } else {
# 1332 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
    static uint32_t lastOBDReinit = 0;
    if (millis() - lastOBDReinit >= 30000) {
      lastOBDReinit = millis();
      if (obd.init(PROTO_AUTO, false)) {
        state.set(STATE_OBD_READY);
        Serial.println("[OBD] ECU ON");
      }
    }
  }
#endif

#if ENABLE_OBD





  if (enableCan) {
    byte rxbuf[32];
    int rxbytes;
    while ((rxbytes = obd.receiveData(rxbuf, sizeof(rxbuf))) > 0) {

      char hexEntry[sizeof(rxbuf) * 2 + 1];
      int hexLen = 0;
      for (int i = 0; i < rxbytes && hexLen < (int)sizeof(hexEntry) - 2; i++) {
        hexLen += snprintf(hexEntry + hexLen, sizeof(hexEntry) - hexLen, "%02X", rxbuf[i]);
      }
      hexEntry[hexLen] = 0;
      portENTER_CRITICAL(&s_canBufMux);
      if (s_canFrameCount < CAN_DATA_LIST_MAX) {

        strncpy(s_canFrameList[s_canFrameCount], hexEntry, sizeof(s_canFrameList[0]) - 1);
        s_canFrameList[s_canFrameCount][sizeof(s_canFrameList[0]) - 1] = 0;
        s_canFrameCount++;
      } else {

        memmove(s_canFrameList[0], s_canFrameList[1],
                (CAN_DATA_LIST_MAX - 1) * sizeof(s_canFrameList[0]));
        strncpy(s_canFrameList[CAN_DATA_LIST_MAX - 1], hexEntry, sizeof(s_canFrameList[0]) - 1);
        s_canFrameList[CAN_DATA_LIST_MAX - 1][sizeof(s_canFrameList[0]) - 1] = 0;
      }
      s_canFrameTotal++;
      portEXIT_CRITICAL(&s_canBufMux);
    }
  }
#endif
  if (rssi != rssiLast) {
    int val = (rssiLast = rssi);
    buffer->add(PID_CSQ, ELEMENT_INT32, &val, sizeof(val));
  }
#if ENABLE_OBD
  if (sys.devType > 12) {
    batteryVoltage = (float)(analogRead(A0) * 45) / 4095;
  } else {
    batteryVoltage = obd.getVoltage();
  }
  if (batteryVoltage) {
    uint16_t v = batteryVoltage * 100;
    buffer->add(PID_BATTERY_VOLTAGE, ELEMENT_UINT16, &v, sizeof(v));
  }
#endif

#if LOG_EXT_SENSORS
  processExtInputs(buffer);
#endif

#if ENABLE_MEMS
  processMEMS(buffer);
#endif

  bool success = processGPS(buffer);
#if GNSS_RESET_TIMEOUT
  if (success) {
    lastGPStick = millis();
    state.set(STATE_GPS_ONLINE);
  } else {
    if (millis() - lastGPStick > GNSS_RESET_TIMEOUT * 1000) {
      sys.gpsEnd();
      state.clear(STATE_GPS_ONLINE | STATE_GPS_READY);
      delay(20);
      if (initGPS()) state.set(STATE_GPS_READY);
      lastGPStick = millis();
    }
  }
#endif

  if (!state.check(STATE_MEMS_READY)) {
    deviceTemp = readChipTemperature();
  }
  buffer->add(PID_DEVICE_TEMP, ELEMENT_INT32, &deviceTemp, sizeof(deviceTemp));
# 1430 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
  {
    uint8_t lwv = enableLedWhite ? 1 : 0;
    uint8_t bv = enableBeep ? 1 : 0;
    if ((int8_t)lwv != s_lastLedWhite) {
      s_lastLedWhite = (int8_t)lwv;
      buffer->add(PID_LED_WHITE_STATE, ELEMENT_UINT8, &lwv, sizeof(lwv));
    }
    if ((int8_t)bv != s_lastBeep) {
      s_lastBeep = (int8_t)bv;
      buffer->add(PID_BEEP_STATE, ELEMENT_UINT8, &bv, sizeof(bv));
    }
  }
# 1450 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
  if (state.check(STATE_NET_READY)) {

    uint8_t ctv = state.check(STATE_WIFI_CONNECTED) ? 1 : 2;
    if ((int8_t)ctv != s_lastConnType) {
      s_lastConnType = (int8_t)ctv;
      buffer->add(PID_CONN_TYPE, ELEMENT_UINT8, &ctv, sizeof(ctv));
    }
  }



  {
    uint8_t ov = enableObd ? 1 : 0;
    uint8_t cv = enableCan ? 1 : 0;
    if ((int8_t)ov != s_lastObd) {
      s_lastObd = (int8_t)ov;
      buffer->add(PID_OBD_STATE, ELEMENT_UINT8, &ov, sizeof(ov));
    }
    if ((int8_t)cv != s_lastCan) {
      s_lastCan = (int8_t)cv;
      buffer->add(PID_CAN_STATE, ELEMENT_UINT8, &cv, sizeof(cv));
    }
  }


  {
    int16_t sv = (int16_t)nvsStandbyTimeS;
    if (sv != s_lastStandbyTime) {
      s_lastStandbyTime = sv;
      buffer->add(PID_STANDBY_TIME, ELEMENT_UINT16, &nvsStandbyTimeS, sizeof(nvsStandbyTimeS));
    }
  }


  {
    uint8_t dv = enableDeepStandby ? 1 : 0;
    if ((int8_t)dv != s_lastDeepStandby) {
      s_lastDeepStandby = (int8_t)dv;
      buffer->add(PID_DEEP_STANDBY, ELEMENT_UINT8, &dv, sizeof(dv));
    }
  }

#if STORAGE == STORAGE_SD




  {
    static uint32_t lastSdReportMs = 0;
    uint32_t nowMs = millis();
    if (lastSdReportMs == 0 || nowMs - lastSdReportMs >= 60000UL) {
      lastSdReportMs = nowMs;
      uint32_t sdTotalMb = 0;
      uint32_t sdFreeMb = 0;
      if (state.check(STATE_STORAGE_READY)) {

        if (logger.purgeOldFiles()) {
          logger.logEvent("SD:PURGE");
        }
        uint64_t tot = SD.totalBytes();
        uint64_t used = SD.usedBytes();
        sdTotalMb = (uint32_t)(tot >> 20);
        sdFreeMb = (uint32_t)((tot > used ? tot - used : 0) >> 20);
      }



      s_cachedSdTotalMb = sdTotalMb;
      s_cachedSdFreeMb = sdFreeMb;
      buffer->add(PID_SD_TOTAL_MB, ELEMENT_UINT32, &sdTotalMb, sizeof(sdTotalMb));
      buffer->add(PID_SD_FREE_MB, ELEMENT_UINT32, &sdFreeMb, sizeof(sdFreeMb));
    }
  }
#endif

  buffer->timestamp = millis();
  buffer->state = BUFFER_STATE_FILLED;


  if (startTime - lastStatsTime >= 3000) {
    bufman.printStats();
    lastStatsTime = startTime;
  }

#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {






    logger.timestamp(buffer->timestamp);
    buffer->serialize(logger);
    uint16_t sizeKB = (uint16_t)(logger.size() >> 10);
    if (sizeKB != lastSizeKB) {
      logger.flush();
      lastSizeKB = sizeKB;
      Serial.print("[FILE] ");
      Serial.print(sizeKB);
      Serial.println("KB");
    }



    {
      static uint16_t prevDiagState = 0xFFFF;
      const uint16_t DIAG_MASK = STATE_OBD_READY | STATE_GPS_READY | STATE_GPS_ONLINE
                                | STATE_WIFI_CONNECTED | STATE_CELL_CONNECTED;
      uint16_t cur = state.m_state & DIAG_MASK;
      if (cur != prevDiagState) {
        char diag[128];
        snprintf(diag, sizeof(diag),
            "STATUS OBD=%c GPS=%c FIX=%c WIFI=%c CELL=%c t=%lu",
            state.check(STATE_OBD_READY) ? '1' : '0',
            state.check(STATE_GPS_READY) ? '1' : '0',
            state.check(STATE_GPS_ONLINE) ? '1' : '0',
            state.check(STATE_WIFI_CONNECTED) ? '1' : '0',
            state.check(STATE_CELL_CONNECTED) ? '1' : '0',
            millis() / 1000);
        logger.logEvent(diag);
        prevDiagState = cur;
      }
    }
  }
#endif

  const int dataIntervals[] = DATA_INTERVAL_TABLE;
#if ENABLE_OBD || ENABLE_MEMS



  const uint16_t stationaryTimeDefaults[] = STATIONARY_TIME_TABLE;
  const byte stationaryCount = sizeof(stationaryTimeDefaults) / sizeof(stationaryTimeDefaults[0]);
  uint16_t stationaryTime[stationaryCount];
  for (byte i = 0; i < stationaryCount; i++) {
    stationaryTime[i] = stationaryTimeDefaults[i];
  }
  if (nvsStandbyTimeS >= 5) {
    stationaryTime[stationaryCount - 1] = nvsStandbyTimeS;
  }
  unsigned int motionless = (millis() - lastMotionTime) / 1000;
  bool stationary = true;
  for (byte i = 0; i < stationaryCount; i++) {
    dataInterval = dataIntervals[i];
    if (motionless < stationaryTime[i] || stationaryTime[i] == 0) {
      stationary = false;
      break;
    }
  }
# 1608 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
#if ENABLE_OBD
  {
    static bool s_obdAliveChecked = false;
    const bool inStationaryPhase = (motionless >= stationaryTime[0]);
    if (!inStationaryPhase) {

      s_obdAliveChecked = false;
    } else if (enableObd && state.check(STATE_OBD_READY) && !s_obdAliveChecked) {
      s_obdAliveChecked = true;




      char abuf[32] = {};
      const int ret = obd.link ? obd.link->sendCommand("0100\r", abuf, sizeof(abuf), 1000) : 0;
      if (ret <= 0) {


        Serial.println("OBD:ECU offline at standby-timer start - entering standby immediately");
        state.clear(STATE_WORKING);
        return;
      }
      Serial.println("OBD:ECU alive - standby countdown running");
    }
  }
#endif

  if (stationary) {

    Serial.print("Stationary for ");
    Serial.print(motionless);
    Serial.println(" secs");

    state.clear(STATE_WORKING);
    return;
  }
#else
  dataInterval = dataIntervals[0];
#endif






  do {
    long t = dataInterval - (millis() - startTime);
    long slice = (t > 100) ? 100 : (t > 0 ? t : 0);
    processBLE(slice);
#if ENABLE_HTTPD
    if (enableHttpd) serverProcess(0);
#endif
  } while (millis() - startTime < dataInterval);
}

bool initCell(bool quick = false)
{
  Serial.println("[CELL] Activating...");

  if (!teleClient.cell.begin(&sys)) {
    Serial.println("[CELL] No supported module");
#if ENABLE_OLED
    oled.println("No Cell Module");
#endif
    return false;
  }
  if (quick) return true;
#if ENABLE_OLED
    oled.print(teleClient.cell.deviceName());
    oled.println(" OK\r");
    oled.print("IMEI:");
    oled.println(teleClient.cell.IMEI);
#endif
  Serial.print("CELL:");
  Serial.println(teleClient.cell.deviceName());

  {
    bool simReady = false;
    for (byte simRetry = 0; simRetry < 3 && !simReady; simRetry++) {
      if (teleClient.cell.checkSIM(simPin)) {
        simReady = true;
      } else if (simRetry < 2) {
        delay(2000);
      }
    }
    if (!simReady) {
      Serial.println("NO SIM CARD");
    }
  }
  Serial.print("IMEI:");
  Serial.println(teleClient.cell.IMEI);
  Serial.println("[CELL] Searching...");
  if (*apn) {
    Serial.print("APN:");
    Serial.println(apn);
  }
  if (teleClient.cell.setup(apn, APN_USERNAME, APN_PASSWORD)) {
    netop = teleClient.cell.getOperatorName();
    if (netop.length()) {
      Serial.print("Operator:");
      Serial.println(netop);
#if ENABLE_OLED
      oled.println(op);
#endif
    }

#if GNSS == GNSS_CELLULAR
    if (teleClient.cell.setGPS(true)) {
      Serial.println("CELL GNSS:OK");
    }
#endif

    ip = teleClient.cell.getIP();
    if (ip.length()) {
      Serial.print("[CELL] IP:");
      Serial.println(ip);
#if ENABLE_OLED
      oled.print("IP:");
      oled.println(ip);
#endif
    }
    state.set(STATE_CELL_CONNECTED);
  } else {
    char *p = strstr(teleClient.cell.getBuffer(), "+CPSI:");
    if (p) {
      char *q = strchr(p, '\r');
      if (q) *q = 0;
      Serial.print("[CELL] ");
      Serial.println(p + 7);
#if ENABLE_OLED
      oled.println(p + 7);
#endif
    } else {
      Serial.print(teleClient.cell.getBuffer());
    }
  }
  timeoutsNet = 0;
  return state.check(STATE_CELL_CONNECTED);
}




void telemetry(void* inst)
{
  uint32_t lastRssiTime = 0;
  uint8_t connErrors = 0;
  CStorageRAM store;
  store.init(
#if BOARD_HAS_PSRAM
    (char*)heap_caps_malloc(SERIALIZE_BUFFER_SIZE, MALLOC_CAP_SPIRAM),
#else
    (char*)malloc(SERIALIZE_BUFFER_SIZE),
#endif
    SERIALIZE_BUFFER_SIZE
  );
  teleClient.reset();

  for (;;) {



    if (s_ota_active) {
      delay(500);
      continue;
    }




    if (s_http_standby_enter) {
      s_http_standby_enter = false;
      state.set(STATE_STANDBY);
      state.clear(STATE_WORKING);
      Serial.println("[HTTP] Telemetry paused via /api/control?cmd=OFF");
    }
    if (s_http_standby_exit) {
      s_http_standby_exit = false;
      state.clear(STATE_STANDBY);
      Serial.println("[HTTP] Telemetry resumed via /api/control?cmd=ON");
    }

    if (state.check(STATE_STANDBY)) {
      if (state.check(STATE_CELL_CONNECTED) || state.check(STATE_WIFI_CONNECTED)) {
        teleClient.shutdown();
        netop = "";
        ip = "";
        rssi = 0;
      }
      state.clear(STATE_NET_READY | STATE_CELL_CONNECTED | STATE_WIFI_CONNECTED);
      teleClient.reset();
      bufman.purge();





      s_lastLedWhite = -1;
      s_lastBeep = -1;
      s_lastConnType = -1;
      s_lastObd = -1;
      s_lastCan = -1;
      s_lastStandbyTime = -1;
      s_send_state_pids = true;

      uint32_t t = millis();
      do {
        delay(1000);
      } while (state.check(STATE_STANDBY) && millis() - t < 1000L * PING_BACK_INTERVAL);
      if (state.check(STATE_STANDBY)) {

#if ENABLE_WIFI
        if (wifiSSID[0] || wifiSSID2[0]) {
          wifiConnect();
        }
        if (teleClient.wifi.setup()) {
          Serial.println("[WIFI] Ping...");
          teleClient.ping();
        }
        else
#endif
        {
          if (initCell()) {
            Serial.println("[CELL] Ping...");
            teleClient.ping();
          }
        }
        teleClient.shutdown();
        state.clear(STATE_CELL_CONNECTED | STATE_WIFI_CONNECTED);
      }
      continue;
    }

#if ENABLE_WIFI
    if ((wifiSSID[0] || wifiSSID2[0]) && !state.check(STATE_WIFI_CONNECTED)) {
      if (!teleClient.wifi.connected()) {
        wifiConnect();
      }
      teleClient.wifi.setup(WIFI_JOIN_TIMEOUT);
    }
#endif

    while (state.check(STATE_WORKING)) {






      if (s_ota_active || s_http_standby_enter) break;

#if ENABLE_WIFI
      if (wifiSSID[0]) {
        if (!state.check(STATE_WIFI_CONNECTED) && teleClient.wifi.connected()) {
          ip = teleClient.wifi.getIP();
          if (ip.length()) {
            Serial.print("[WIFI] IP:");
            Serial.println(ip);
          }
          connErrors = 0;
          if (teleClient.connect()) {
            state.set(STATE_WIFI_CONNECTED | STATE_NET_READY);
            if (enableBeep) beep(50);




            s_lastLedWhite = -1;
            s_lastBeep = -1;
            s_lastConnType = -1;
            s_lastObd = -1;
            s_lastCan = -1;
            s_lastStandbyTime = -1;
            s_send_state_pids = true;

            if (state.check(STATE_CELL_CONNECTED)) {
              teleClient.cell.end();
              state.clear(STATE_CELL_CONNECTED);
              Serial.println("[CELL] Deactivated");
            }
          }
        } else if (state.check(STATE_WIFI_CONNECTED) && !teleClient.wifi.connected()) {
          Serial.println("[WIFI] Disconnected");
          state.clear(STATE_WIFI_CONNECTED);
        }
      }
#endif
      if (!state.check(STATE_WIFI_CONNECTED) && !state.check(STATE_CELL_CONNECTED)) {
        connErrors = 0;
        if (!initCell() || !teleClient.connect()) {
          teleClient.cell.end();
          state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
          Serial.println("[CELL] Deactivated");
#if ENABLE_WIFI
          if (wifiSSID[0] || wifiSSID2[0]) {

            if (!teleClient.wifi.connected()) {
              wifiConnect();
            }
            if (teleClient.wifi.setup(WIFI_JOIN_TIMEOUT)) {
              break;
            }
          }
#endif

          delay(60000 * 3);
          break;
        }
        Serial.println("[CELL] In service");
        state.set(STATE_NET_READY);
        if (enableBeep) beep(50);




        s_lastLedWhite = -1;
        s_lastBeep = -1;
        s_lastConnType = -1;
        s_lastObd = -1;
        s_lastCan = -1;
        s_lastStandbyTime = -1;
        s_send_state_pids = true;
      }

      if (millis() - lastRssiTime > SIGNAL_CHECK_INTERVAL * 1000) {
#if ENABLE_WIFI
        if (state.check(STATE_WIFI_CONNECTED))
        {
          rssi = teleClient.wifi.RSSI();
        }
        else
#endif
        {
          rssi = teleClient.cell.RSSI();
        }
        if (rssi) {
          Serial.print("RSSI:");
          Serial.print(rssi);
          Serial.println("dBm");
        }
        lastRssiTime = millis();

#if ENABLE_WIFI
        if ((wifiSSID[0] || wifiSSID2[0]) && !state.check(STATE_WIFI_CONNECTED) && !teleClient.wifi.connected()) {
          wifiConnect();
        }
#endif
      }
# 1969 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
      if (otaToken[0] && otaCheckIntervalS > 0 &&
          state.check(STATE_WIFI_CONNECTED)) {
        static uint32_t lastOtaCheckMs = 0;
        uint32_t nowMs = millis();
        if (lastOtaCheckMs == 0 || nowMs - lastOtaCheckMs >= (uint32_t)otaCheckIntervalS * 1000UL) {
          lastOtaCheckMs = nowMs;
          Serial.println("[OTA-PULL] Checking for firmware update...");
# 1994 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
          if (performPullOtaCheck()) {


            while (true) delay(1000);
          }
#if STORAGE == STORAGE_SD
# 2009 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
          if (s_ota_pending) {
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            state.clear(STATE_NET_READY | STATE_WIFI_CONNECTED);
            break;
          }
#endif




#if ENABLE_WIFI
          if (state.check(STATE_WIFI_CONNECTED) &&
              ESP.getMaxAllocHeap() < TLS_MIN_FREE_HEAP) {
            Serial.printf("[WIFI] Low heap (%u bytes max block) after OTA TLS fail, restarting WiFi\n",
                          (unsigned)ESP.getMaxAllocHeap());
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            state.clear(STATE_NET_READY | STATE_WIFI_CONNECTED);
            break;
          }
#endif
        }
      }


      CBuffer* buffer = bufman.getNewest();
      if (!buffer) {
        delay(50);
        continue;
      }
#if SERVER_PROTOCOL == PROTOCOL_UDP
      store.header(devid);
#endif
      store.timestamp(buffer->timestamp);
      buffer->serialize(store);
      bufman.free(buffer);
# 2058 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
      if (s_send_state_pids) {
        s_send_state_pids = false;
        {
          uint8_t v = enableLedWhite ? 1 : 0;
          store.log(PID_LED_WHITE_STATE, &v, 1);
          s_lastLedWhite = (int8_t)v;
        }
        {
          uint8_t v = enableBeep ? 1 : 0;
          store.log(PID_BEEP_STATE, &v, 1);
          s_lastBeep = (int8_t)v;
        }
        if (state.check(STATE_NET_READY)) {
          uint8_t v = state.check(STATE_WIFI_CONNECTED) ? 1 : 2;
          store.log(PID_CONN_TYPE, &v, 1);
          s_lastConnType = (int8_t)v;
        }
        {
          uint8_t ov = enableObd ? 1 : 0;
          store.log(PID_OBD_STATE, &ov, 1);
          s_lastObd = (int8_t)ov;
        }
        {
          uint8_t cv = enableCan ? 1 : 0;
          store.log(PID_CAN_STATE, &cv, 1);
          s_lastCan = (int8_t)cv;
        }
        {
          store.log(PID_STANDBY_TIME, &nvsStandbyTimeS, 1);
          s_lastStandbyTime = (int16_t)nvsStandbyTimeS;
        }
        {
          uint8_t dv = enableDeepStandby ? 1 : 0;
          store.log(PID_DEEP_STANDBY, &dv, 1);
          s_lastDeepStandby = (int8_t)dv;
        }
#if STORAGE == STORAGE_SD


        store.log(PID_SD_TOTAL_MB, &s_cachedSdTotalMb, 1);
        store.log(PID_SD_FREE_MB, &s_cachedSdFreeMb, 1);
#endif
      }
      store.tailer();
      Serial.print("[DAT] ");
      Serial.println(store.buffer());
# 2112 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
#ifdef PIN_LED
      const bool ledWhiteFlash = enableLedWhite;
      if (ledWhiteFlash) digitalWrite(PIN_LED, HIGH);
#endif

      if (teleClient.transmit(store.buffer(), store.length())) {

        connErrors = 0;
        showStats();
      } else {
        timeoutsNet++;
        connErrors++;
        printTimeoutStats();
        if (connErrors < MAX_CONN_ERRORS_RECONNECT) {

          if (!teleClient.connect(true)) {
# 2142 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
#if ENABLE_WIFI
            if (state.check(STATE_WIFI_CONNECTED) &&
                ESP.getMaxAllocHeap() < TLS_MIN_FREE_HEAP) {
              Serial.printf("[WIFI] Low heap (%u bytes max block) after TLS fail, restarting WiFi\n",
                            (unsigned)ESP.getMaxAllocHeap());
              teleClient.wifi.end();
              state.clear(STATE_NET_READY | STATE_WIFI_CONNECTED);
              break;
            }
#endif
          }
        }
      }
#ifdef PIN_LED
      if (ledWhiteFlash) digitalWrite(PIN_LED, LOW);
#endif
      store.purge();

      teleClient.inbound();

      if (state.check(STATE_CELL_CONNECTED) && !teleClient.cell.check(1000)) {
        Serial.println("[CELL] Not in service");
        state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
        break;
      }

      if (syncInterval > 10000 && millis() - teleClient.lastSyncTime > syncInterval) {
        Serial.println("[NET] Poor connection");
        timeoutsNet++;
        if (!teleClient.connect()) {
          connErrors++;
        }
      }

      if (connErrors >= MAX_CONN_ERRORS_RECONNECT) {
#if ENABLE_WIFI
        if (state.check(STATE_WIFI_CONNECTED)) {
          teleClient.wifi.end();
          state.clear(STATE_NET_READY | STATE_WIFI_CONNECTED);
          break;
        }
#endif
        if (state.check(STATE_CELL_CONNECTED)) {
          teleClient.cell.end();
          state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
          break;
        }
      }

      if (deviceTemp >= COOLING_DOWN_TEMP) {

        Serial.print("HIGH DEVICE TEMP: ");
        Serial.println(deviceTemp);
        bufman.purge();
      }

    }
  }
}




void standby()
{
  state.set(STATE_STANDBY);

#if STORAGE == STORAGE_SD




  if (s_ota_pending && state.check(STATE_STORAGE_READY)) {
    s_ota_active = true;
    delay(OTA_TELEMETRY_YIELD_DELAY_MS);
    if (performPullOtaFlash()) {

      while (true) delay(1000);
    }

    s_ota_active = false;
    s_ota_pending = false;
  }
#endif

#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    logger.end();
  }
#endif

#if !GNSS_ALWAYS_ON && GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    Serial.println("[GNSS] OFF");
    sys.gpsEnd(true);
    state.clear(STATE_GPS_READY | STATE_GPS_ONLINE);
    gd = 0;
  }
#endif

  state.clear(STATE_WORKING | STATE_OBD_READY | STATE_STORAGE_READY);

#if ENABLE_OLED
  oled.print("STANDBY");
  delay(1000);
  oled.clear();
#endif
  Serial.println("STANDBY");
  obd.enterLowPowerMode();




  if (enableDeepStandby) {
    uint64_t sleep_us = ((nvsStandbyTimeS >= 5) ? (uint64_t)nvsStandbyTimeS : 180ULL) * 1000000ULL;
    Serial.print("DEEP_SLEEP ");
    Serial.print((unsigned)(sleep_us / 1000000ULL));
    Serial.println("s");
    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();

  }

#if ENABLE_MEMS
  calibrateMEMS();
  waitMotion(-1);
#elif ENABLE_OBD
  do {
    delay(5000);
  } while (obd.getVoltage() < JUMPSTART_VOLTAGE);
#else
  delay(5000);
#endif
  Serial.println("WAKEUP");
  sys.resetLink();
#if RESET_AFTER_WAKEUP
#if ENABLE_MEMS
  if (mems) mems->end();
#endif
  ESP.restart();
#endif
  state.clear(STATE_STANDBY);
}




void genDeviceID(char* buf)
{
    uint64_t seed = ESP.getEfuseMac() >> 8;
    for (int i = 0; i < 8; i++, seed >>= 5) {
      byte x = (byte)seed & 0x1f;
      if (x >= 10) {
        x = x - 10 + 'A';
        switch (x) {
          case 'B': x = 'W'; break;
          case 'D': x = 'X'; break;
          case 'I': x = 'Y'; break;
          case 'O': x = 'Z'; break;
        }
      } else {
        x += '0';
      }
      buf[i] = x;
    }
    buf[8] = 0;
}

void showSysInfo()
{
  Serial.print("CPU:");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.print("MHz FLASH:");
  Serial.print(ESP.getFlashChipSize() >> 20);
  Serial.println("MB");






  Serial.print("RAM:");
  Serial.print(ESP.getHeapSize() >> 10);
  Serial.print("KB");
  if (psramInit()) {
    Serial.print(" PSRAM:");
    Serial.print(esp_spiram_get_size() >> 20);
    Serial.print("MB");
  }
  Serial.println();

  int rtc = rtc_clk_slow_freq_get();
  if (rtc) {
    Serial.print("RTC:");
    Serial.println(rtc);
  }

#if ENABLE_OLED
  oled.clear();
  oled.print("CPU:");
  oled.print(ESP.getCpuFreqMHz());
  oled.print("Mhz ");
  oled.print(getFlashSize() >> 10);
  oled.println("MB Flash");
#endif

  Serial.print("DEVICE ID:");
  Serial.println(devid);
#if ENABLE_OLED
  oled.print("DEVICE ID:");
  oled.println(devid);
#endif
  Serial.print("FW:");
  Serial.print(FIRMWARE_VERSION);
  Serial.print(" Built:");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  if (nvsVersion[0]) {
    Serial.print("NVS:");
    Serial.println(nvsVersion);
  }
}

void loadConfig()
{
  size_t len;
  len = sizeof(apn);
  apn[0] = 0;
  nvs_get_str(nvs, "CELL_APN", apn, &len);
  if (!apn[0]) {
    strcpy(apn, CELL_APN);
  }

  len = sizeof(simPin);
  simPin[0] = 0;
  nvs_get_str(nvs, "SIM_PIN", simPin, &len);
  if (!simPin[0]) {
    strncpy(simPin, SIM_CARD_PIN, sizeof(simPin) - 1);
    simPin[sizeof(simPin) - 1] = 0;
  }

#if ENABLE_WIFI
  len = sizeof(wifiSSID);
  nvs_get_str(nvs, "WIFI_SSID", wifiSSID, &len);
  len = sizeof(wifiPassword);
  nvs_get_str(nvs, "WIFI_PWD", wifiPassword, &len);
  len = sizeof(wifiSSID2);
  nvs_get_str(nvs, "WIFI_SSID2", wifiSSID2, &len);
  len = sizeof(wifiPassword2);
  nvs_get_str(nvs, "WIFI_PWD2", wifiPassword2, &len);
#endif



  len = sizeof(serverHost);
  serverHost[0] = 0;
  nvs_get_str(nvs, "SERVER_HOST", serverHost, &len);
  if (!serverHost[0]) {
    strncpy(serverHost, SERVER_HOST, sizeof(serverHost) - 1);
    serverHost[sizeof(serverHost) - 1] = 0;
  }
  uint16_t nvsPort = 0;
  nvs_get_u16(nvs, "SERVER_PORT", &nvsPort);
  if (nvsPort) serverPort = nvsPort;

  len = sizeof(webhookPath);
  webhookPath[0] = 0;
  nvs_get_str(nvs, "WEBHOOK_PATH", webhookPath, &len);







  len = sizeof(cellServerHost);
  cellServerHost[0] = 0;
  nvs_get_str(nvs, "CELL_HOST", cellServerHost, &len);
  uint16_t nvsCellPort = 0;
  nvs_get_u16(nvs, "CELL_PORT", &nvsCellPort);
  if (nvsCellPort) cellServerPort = nvsCellPort;
  len = sizeof(cellWebhookPath);
  cellWebhookPath[0] = 0;
  nvs_get_str(nvs, "CELL_PATH", cellWebhookPath, &len);



  uint8_t nvsHttpd = 0;
  if (nvs_get_u8(nvs, "ENABLE_HTTPD", &nvsHttpd) == ESP_OK) {
    enableHttpd = nvsHttpd;
  }

#if ENABLE_BLE





  uint8_t nvsBle = 1;
  if (nvs_get_u8(nvs, "ENABLE_BLE", &nvsBle) == ESP_OK) {
    enableBle = nvsBle;
  } else if (webhookPath[0]) {




    enableBle = 0;
  }
#endif





  uint8_t nvsCellDebug = 0;
  if (nvs_get_u8(nvs, "CELL_DEBUG", &nvsCellDebug) == ESP_OK) {
    cellNetDebug = nvsCellDebug;
  }
# 2469 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
  uint8_t nvsLedRedEn = 1;
  if (nvs_get_u8(nvs, "LED_RED_EN", &nvsLedRedEn) == ESP_OK) {
    enableLedRed = nvsLedRedEn != 0;
  }
  uint8_t nvsLedWhiteEn = 1;
  if (nvs_get_u8(nvs, "LED_WHITE_EN", &nvsLedWhiteEn) == ESP_OK) {
    enableLedWhite = nvsLedWhiteEn != 0;
  }
  uint8_t nvsBeepEn = 1;
  if (nvs_get_u8(nvs, "BEEP_EN", &nvsBeepEn) == ESP_OK) {
    enableBeep = nvsBeepEn != 0;
  }



  uint8_t nvsObdEn = 1;
  if (nvs_get_u8(nvs, "OBD_EN", &nvsObdEn) == ESP_OK) {
    enableObd = nvsObdEn != 0;
  }



  uint8_t nvsCanEn = 0;
  if (nvs_get_u8(nvs, "CAN_EN", &nvsCanEn) == ESP_OK) {
    enableCan = nvsCanEn != 0;
  }



  uint8_t nvsDeepStandby = 0;
  if (nvs_get_u8(nvs, "DEEP_STANDBY", &nvsDeepStandby) == ESP_OK) {
    enableDeepStandby = nvsDeepStandby != 0;
  }




  uint16_t nvsStby = 0;
  if (nvs_get_u16(nvs, "STANDBY_TIME", &nvsStby) == ESP_OK) {
    nvsStandbyTimeS = (nvsStby >= 5) ? nvsStby : 0;
  }



  uint16_t nvsDataInterval = 0;
  if (nvs_get_u16(nvs, "DATA_INTERVAL", &nvsDataInterval) == ESP_OK && nvsDataInterval >= 500) {
    dataInterval = nvsDataInterval;
  }


  uint16_t nvsSyncInterval = 0;
  if (nvs_get_u16(nvs, "SYNC_INTERVAL", &nvsSyncInterval) == ESP_OK && nvsSyncInterval > 0) {
    syncInterval = (int32_t)nvsSyncInterval * 1000;
  }



  len = sizeof(otaToken);
  otaToken[0] = 0;
  nvs_get_str(nvs, "OTA_TOKEN", otaToken, &len);


  len = sizeof(otaHost);
  otaHost[0] = 0;
  nvs_get_str(nvs, "OTA_HOST", otaHost, &len);
  if (!otaHost[0] && otaToken[0]) {

    strncpy(otaHost, serverHost, sizeof(otaHost) - 1);
    otaHost[sizeof(otaHost) - 1] = 0;
  }

  uint16_t nvsOtaPort = 0;
  nvs_get_u16(nvs, "OTA_PORT", &nvsOtaPort);
  if (nvsOtaPort) otaPort = nvsOtaPort;

  uint16_t nvsOtaInterval = 0;
  nvs_get_u16(nvs, "OTA_INTERVAL", &nvsOtaInterval);
  otaCheckIntervalS = nvsOtaInterval;





  len = sizeof(nvsVersion);
  nvsVersion[0] = 0;
  nvs_get_str(nvs, "NVS_VER", nvsVersion, &len);



  size_t vlen = sizeof(vehicleMake);
  nvs_get_str(nvs, "VEHICLE_MAKE", vehicleMake, &vlen);
  vlen = sizeof(vehicleModel);
  nvs_get_str(nvs, "VEHICLE_MODEL", vehicleModel, &vlen);
  vlen = sizeof(vehicleYear);
  nvs_get_str(nvs, "VEHICLE_YEAR", vehicleYear, &vlen);




  vlen = sizeof(vehiclePidsStr);
  vehiclePidsStr[0] = 0;
  nvs_get_str(nvs, "VEHICLE_PIDS", vehiclePidsStr, &vlen);
  vehicleObdDataCount = 0;
  if (vehiclePidsStr[0]) {
    char tmp[sizeof(vehiclePidsStr)];
    strncpy(tmp, vehiclePidsStr, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = 0;
    char *tok = strtok(tmp, ",");
    while (tok && vehicleObdDataCount < MAX_VEHICLE_PIDS) {
      byte pid = (byte)strtol(tok, nullptr, 16);


      if (pid > 0) {
        vehicleObdData[vehicleObdDataCount].pid = pid;
        vehicleObdData[vehicleObdDataCount].tier = 3;
        vehicleObdData[vehicleObdDataCount].value = 0;
        vehicleObdData[vehicleObdDataCount].ts = 0;
        vehicleObdDataCount++;
      }
      tok = strtok(nullptr, ",");
    }
    if (vehicleObdDataCount > 0) {
      Serial.printf("VEHICLE:%s %s %s pids=%d\n",
                    vehicleMake, vehicleModel, vehicleYear, vehicleObdDataCount);
    }
  }
}
# 2620 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
#if STORAGE == STORAGE_SD
# 2637 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
static bool _applyNvsFromSD()
{
  if (!SD.exists(OTA_NVS_PATH)) {
    return true;
  }

  File nvsFile = SD.open(OTA_NVS_PATH, FILE_READ);
  if (!nvsFile) {
    Serial.println("[OTA-PULL] Cannot open NVS staging file");
    SD.remove(OTA_NVS_PATH);
    return false;
  }
  size_t nvsSize = nvsFile.size();



  if (nvsSize < 4096 || nvsSize > 0x5000) {
    nvsFile.close();
    Serial.printf("[OTA-PULL] NVS staging file has invalid size: %u — skipping\n",
                  (unsigned)nvsSize);
    SD.remove(OTA_NVS_PATH);
    return false;
  }
# 2672 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
  size_t nvsBufSize = (nvsSize + 3) & ~3UL;
  uint8_t* nvsBuf = (uint8_t*)malloc(nvsBufSize);
  if (!nvsBuf) {
    nvsFile.close();
    Serial.println("[OTA-PULL] NVS: not enough RAM to buffer image — skipping NVS update");
    SD.remove(OTA_NVS_PATH);
    return false;
  }

  if (nvsBufSize > nvsSize) {
    memset(nvsBuf + nvsSize, 0xFF, nvsBufSize - nvsSize);
  }

  size_t readTotal = 0;
  bool readOk = true;
  while (readTotal < nvsSize) {
    int toRead = (int)(nvsSize - readTotal);
    if (toRead > (int)PULL_OTA_CHUNK_SIZE) toRead = (int)PULL_OTA_CHUNK_SIZE;
    int n = nvsFile.read(nvsBuf + readTotal, toRead);
    if (n <= 0) {
      Serial.printf("[OTA-PULL] NVS SD read error at offset %u\n", (unsigned)readTotal);
      readOk = false;
      break;
    }
    readTotal += (size_t)n;
  }
  nvsFile.close();
  SD.remove(OTA_NVS_PATH);

  if (!readOk || readTotal != nvsSize) {
    free(nvsBuf);
    Serial.println("[OTA-PULL] NVS staging file read incomplete — NVS unchanged");
    return false;
  }

  const esp_partition_t* nvsPart = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "nvs");
  if (!nvsPart) {
    free(nvsBuf);
    Serial.println("[OTA-PULL] NVS partition not found in partition table");
    return false;
  }



  nvs_flash_deinit();

  esp_err_t err = esp_partition_erase_range(nvsPart, 0, nvsPart->size);
  if (err != ESP_OK) {
    free(nvsBuf);
    Serial.printf("[OTA-PULL] NVS partition erase failed: %d\n", (int)err);
    return false;
  }





  size_t written = 0;
  bool writeOk = true;
  while (written < nvsSize) {
    size_t toWrite = nvsSize - written;
    if (toWrite > PULL_OTA_CHUNK_SIZE) toWrite = PULL_OTA_CHUNK_SIZE;

    size_t aligned = (toWrite + 3) & ~3UL;
    err = esp_partition_write(nvsPart, written, nvsBuf + written, aligned);
    if (err != ESP_OK) {
      Serial.printf("[OTA-PULL] NVS partition write failed at offset %u: %d\n",
                    (unsigned)written, (int)err);
      writeOk = false;
      break;
    }
    written += toWrite;
  }
  free(nvsBuf);

  if (!writeOk) {
    return false;
  }

  Serial.printf("[OTA-PULL] NVS settings applied: %u bytes written\n", (unsigned)written);
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL NVS OK");
#endif
  return true;
}
# 2776 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
static bool performPullOtaFlash()
{

  if (!SD.exists(OTA_PENDING_PATH)) {
    Serial.println("[OTA-PULL] Staged firmware not found on SD");
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_NOT_FOUND");
#endif
    SD.remove(OTA_META_PATH);
    return false;
  }


  unsigned long expectedSize = 0;
  {
    File mf = SD.open(OTA_META_PATH, FILE_READ);
    if (mf) {
      char buf[16] = {0};
      mf.readBytesUntil('\n', buf, sizeof(buf) - 1);
      mf.close();
      expectedSize = strtoul(buf, nullptr, 10);
    }
  }
  {
    File ff = SD.open(OTA_PENDING_PATH, FILE_READ);
    unsigned long actual = ff ? (unsigned long)ff.size() : 0UL;
    if (ff) ff.close();
    if (expectedSize < PULL_OTA_MIN_FW_SIZE || actual != expectedSize) {
      Serial.printf("[OTA-PULL] SD staging size mismatch: file=%lu expected=%lu — removing\n",
                    actual, expectedSize);
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_SIZE");
#endif
      SD.remove(OTA_PENDING_PATH);
      SD.remove(OTA_META_PATH);
      return false;
    }
  }

  size_t fwSize = (size_t)expectedSize;
  Serial.printf("[OTA-PULL] Flashing staged firmware from SD: %u bytes\n", (unsigned)fwSize);
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    char diag[48];
    snprintf(diag, sizeof(diag), "OTA-PULL FLASH START SIZE=%u", (unsigned)fwSize);
    logger.logEvent(diag);
  }
#endif

  if (!Update.begin(fwSize)) {
    Serial.printf("[OTA-PULL] Update.begin failed: %s\n", Update.errorString());
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=UPD_BEGIN");
#endif
    return false;
  }

  File fwFile = SD.open(OTA_PENDING_PATH, FILE_READ);
  if (!fwFile) {
    Serial.println("[OTA-PULL] Cannot open staged firmware for reading");
    Update.abort();
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_OPEN_READ");
#endif
    return false;
  }

  size_t written = 0;
  uint32_t t0 = millis();
  size_t lastLogAt = 0;

  while (written < fwSize) {
    int toRead = (int)(fwSize - written);
    if (toRead > (int)PULL_OTA_CHUNK_SIZE) toRead = (int)PULL_OTA_CHUNK_SIZE;
    int n = fwFile.read(s_otaChunkBuf, toRead);
    if (n <= 0) {
      Serial.printf("[OTA-PULL] SD read error at offset %u\n", (unsigned)written);
      Update.abort();
      fwFile.close();
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_READ");
#endif
      return false;
    }
    size_t w = Update.write(s_otaChunkBuf, (size_t)n);
    if (w != (size_t)n) {
      Serial.printf("[OTA-PULL] Flash write error at offset %u\n", (unsigned)written);
      Update.abort();
      fwFile.close();
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FLASH_WRITE");
#endif
      return false;
    }
    written += (size_t)n;


    if (written - lastLogAt >= (fwSize / 10 ? fwSize / 10 : 1)) {
      lastLogAt = written;
      Serial.printf("[OTA-PULL] Flash %u / %u bytes (%.0f%%) in %u ms\n",
                    (unsigned)written, (unsigned)fwSize,
                    100.0f * written / fwSize, (unsigned)(millis() - t0));
    }
  }
  fwFile.close();

  if (!Update.end()) {
    Serial.printf("[OTA-PULL] Update.end failed: %s\n", Update.errorString());
    SD.remove(OTA_PENDING_PATH);
    SD.remove(OTA_META_PATH);
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=UPD_END");
#endif
    return false;
  }


  SD.remove(OTA_PENDING_PATH);
  SD.remove(OTA_META_PATH);

  Serial.printf("[OTA-PULL] Flash from SD successful: %u bytes in %u ms\n",
                (unsigned)written, (unsigned)(millis() - t0));







  {
    bool _nvsStaged = SD.exists(OTA_NVS_PATH);
    if (_applyNvsFromSD()) {
      if (_nvsStaged) {
        Serial.println("[OTA-PULL] Settings (NVS) updated successfully");
      } else {
        Serial.println("[OTA-PULL] No NVS settings staged — rebooting with firmware only");
      }
    } else {
      Serial.println("[OTA-PULL] Settings (NVS) update failed — rebooting with old NVS");
    }
  }

#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    char diag[64];
    snprintf(diag, sizeof(diag), "OTA-PULL FLASH OK FW=%s SIZE=%u",
             FIRMWARE_VERSION, (unsigned)written);
    logger.logEvent(diag);
    logger.flush();
  }
#endif


  static esp_timer_handle_t s_ota_flash_timer = NULL;
  if (!s_ota_flash_timer) {
    esp_timer_create_args_t args = {};
    args.callback = [](void*) { esp_restart(); };
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "ota_flash_reboot";
    esp_timer_create(&args, &s_ota_flash_timer);
  } else {
    esp_timer_stop(s_ota_flash_timer);
  }
  esp_timer_start_once(s_ota_flash_timer, 1500000);
  return true;
}
#endif
# 2952 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
static String _maskOtaHost(const char* host) {
  if (!host || !host[0]) return String("(none)");
  size_t n = strlen(host);
  if (n > 12) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.8s...", host);
    return String(buf);
  }
  return String(host);
}
# 2985 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
bool performPullOtaCheck()
{
  if (!otaToken[0] || !otaHost[0]) return false;




#if ENABLE_WIFI
  if (!WiFi.isConnected()) return false;
#else
  return false;
#endif

#if STORAGE == STORAGE_SD



  if (s_ota_pending) {
    Serial.println("[OTA-PULL] Firmware already staged on SD, restarting to flash");
    esp_restart();
    return false;
  }
#endif



  char metaPath[384];
  snprintf(metaPath, sizeof(metaPath),
           "/api/freematics/ota_pull/%s/meta.json", otaToken);

  Serial.printf("[OTA-PULL] URL: https://%s:%u/api/freematics/ota_pull/%.8s.../meta.json\n",
                _maskOtaHost(otaHost).c_str(), (unsigned)otaPort, otaToken);

  char metaBuf[2048];
  int metaBytes = 0;
  char* metaBody = nullptr;

#if ENABLE_WIFI




  if (!teleClient.wifi.open(otaHost, otaPort)) {
    Serial.printf("[OTA-PULL] Cannot connect to %s:%u\n", _maskOtaHost(otaHost).c_str(), (unsigned)otaPort);
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=CONNECT");
#endif
    return false;
  }

  if (!teleClient.wifi.send(METHOD_GET, metaPath)) {
    Serial.println("[OTA-PULL] META send failed");
    teleClient.wifi.close();
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=META_SEND");
#endif
    return false;
  }

  metaBody = teleClient.wifi.receive(metaBuf, sizeof(metaBuf) - 1, &metaBytes);
  if (!metaBody || teleClient.wifi.code() != 200) {
    Serial.printf("[OTA-PULL] META HTTP %u\n", (unsigned)teleClient.wifi.code());
    teleClient.wifi.close();
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) {
      char _ota_diag[48];
      snprintf(_ota_diag, sizeof(_ota_diag), "OTA-PULL ERR=META_HTTP%d", (int)teleClient.wifi.code());
      logger.logEvent(_ota_diag);
    }
#endif
    return false;
  }
  metaBuf[metaBytes < (int)sizeof(metaBuf) - 1 ? metaBytes : (int)sizeof(metaBuf) - 1] = '\0';
# 3066 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
#endif




  if (!strstr(metaBody, "\"available\":true") && !strstr(metaBody, "\"available\": true")) {
    Serial.println("[OTA-PULL] No update available");
    return false;
  }
# 3084 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
  bool nvsOnly = (strstr(metaBody, "\"nvs_only\":true") != nullptr ||
                  strstr(metaBody, "\"nvs_only\": true") != nullptr);


  char nvsPath[256] = "";
  {
    char* nvsField = strstr(metaBody, "\"nvs_url\":");
    if (nvsField) {
      char* start = strchr(nvsField + 10, '"');
      if (start) {
        start++;
        char* end = strchr(start, '"');
        if (end && (size_t)(end - start) < sizeof(nvsPath) - 1) {
          memcpy(nvsPath, start, end - start);
          nvsPath[end - start] = '\0';
        }
      }
    }
  }





#if STORAGE == STORAGE_SD
  if (nvsOnly) {
    if (!state.check(STATE_STORAGE_READY)) {
      Serial.println("[OTA-PULL] NVS-only: SD not ready, skipping");
      return false;
    }
    Serial.println("[OTA-PULL] NVS-only update: downloading settings binary");
    if (SD.exists(OTA_NVS_PATH)) SD.remove(OTA_NVS_PATH);
    if (nvsPath[0]) {
#if ENABLE_WIFI
      if (teleClient.wifi.open(otaHost, otaPort) &&
          teleClient.wifi.send(METHOD_GET, nvsPath)) {
        int _nvsCL = 0;
        int _nvsHC = teleClient.wifi.receiveHeaders(&_nvsCL);
        if (_nvsHC == 200 && _nvsCL > 0) {
          File _nvsFile = SD.open(OTA_NVS_PATH, FILE_WRITE);
          if (_nvsFile) {
            WiFiClientSecure& _nvsRaw = teleClient.wifi.rawClient();
            size_t _nvsWr = 0, _nvsExp = (size_t)_nvsCL;
            bool _nvsOk = true;
            while (_nvsWr < _nvsExp) {
              uint32_t _t1 = millis();
              while (!_nvsRaw.available() && millis() - _t1 < PULL_OTA_CHUNK_TIMEOUT_MS) delay(1);
              if (!_nvsRaw.available()) { _nvsOk = false; break; }
              int _nr = (int)(_nvsExp - _nvsWr);
              if (_nr > (int)PULL_OTA_CHUNK_SIZE) _nr = (int)PULL_OTA_CHUNK_SIZE;
              int _n = _nvsRaw.read(s_otaChunkBuf, _nr);
              if (_n <= 0) { _nvsOk = false; break; }
              if ((size_t)_nvsFile.write(s_otaChunkBuf, (size_t)_n) != (size_t)_n) { _nvsOk = false; break; }
              _nvsWr += (size_t)_n;
            }
            _nvsFile.close();
            if (!_nvsOk || _nvsWr != _nvsExp) {
              Serial.println("[OTA-PULL] NVS download incomplete — settings unchanged");
              SD.remove(OTA_NVS_PATH);
            } else {
              Serial.printf("[OTA-PULL] NVS staged: %u bytes\n", (unsigned)_nvsWr);
            }
          }
        } else {
          Serial.printf("[OTA-PULL] NVS HTTP %d — settings unchanged\n", _nvsHC);
        }
        teleClient.wifi.close();
      } else {
        Serial.println("[OTA-PULL] NVS connect/send failed — settings unchanged");
        teleClient.wifi.close();
      }
#endif
    }


    if (!SD.exists(OTA_NVS_PATH)) {
      Serial.println("[OTA-PULL] NVS-only: download failed, will retry on next check");
      return false;
    }
    if (_applyNvsFromSD()) {
      Serial.println("[OTA-PULL] Settings (NVS) applied — restarting");
      esp_restart();
      return false;
    } else {



      Serial.println("[OTA-PULL] Settings (NVS) apply failed — will retry on next check");
      return false;
    }
  }
#endif
  if (nvsOnly) {

    Serial.println("[OTA-PULL] NVS-only update skipped (no SD storage)");
    return false;
  }


  size_t fwSize = 0;
  char* sizeField = strstr(metaBody, "\"size\":");
  if (!sizeField) {
    Serial.println("[OTA-PULL] META: missing size field");
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=META_NOSIZE");
#endif
    return false;
  }
  fwSize = (size_t)atol(sizeField + 7);
  if (fwSize < PULL_OTA_MIN_FW_SIZE) {
    Serial.printf("[OTA-PULL] META: implausible size %u\n", (unsigned)fwSize);
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=META_SIZE");
#endif
    return false;
  }

  Serial.printf("[OTA-PULL] Update available: %u bytes\n", (unsigned)fwSize);
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    char _ota_diag[48];
    snprintf(_ota_diag, sizeof(_ota_diag), "OTA-PULL START SIZE=%u", (unsigned)fwSize);
    logger.logEvent(_ota_diag);
  }
#endif







  char fwSha256Hex[65] = "";
  {
    char* sf = strstr(metaBody, "\"sha256\":");
    if (sf) {
      char* s = strchr(sf + 9, '"');
      if (s) {
        s++;
        char* e = strchr(s, '"');
        if (e && (size_t)(e - s) == 64) {
          memcpy(fwSha256Hex, s, 64);
          fwSha256Hex[64] = '\0';


          for (int _i = 0; _i < 64; _i++) {
            char _c = fwSha256Hex[_i];
            if (!((_c >= '0' && _c <= '9') || (_c >= 'a' && _c <= 'f') ||
                  (_c >= 'A' && _c <= 'F'))) {
              fwSha256Hex[0] = '\0';
              break;
            }
          }
        }
      }
    }
  }


  char fwPath[384];
  snprintf(fwPath, sizeof(fwPath),
           "/api/freematics/ota_pull/%s/firmware.bin", otaToken);

#if STORAGE == STORAGE_SD


  if (state.check(STATE_STORAGE_READY)) {

    if (SD.exists(OTA_PENDING_PATH)) SD.remove(OTA_PENDING_PATH);
    if (SD.exists(OTA_META_PATH)) SD.remove(OTA_META_PATH);

    File fwFile = SD.open(OTA_PENDING_PATH, FILE_WRITE);
    if (!fwFile) {
      Serial.println("[OTA-PULL] Cannot create SD staging file");
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_OPEN");
#endif
      return false;
    }

    size_t written = 0;
    uint32_t dlStart = millis();
    size_t lastLogAt = 0;
    bool dlOk = true;




    mbedtls_sha256_context sha256Ctx;
    bool doSha256 = (fwSha256Hex[0] != '\0');
    if (doSha256) {
      mbedtls_sha256_init(&sha256Ctx);
      mbedtls_sha256_starts_ret(&sha256Ctx, 0);
    }


#if ENABLE_WIFI
    if (!teleClient.wifi.open(otaHost, otaPort)) {
      Serial.printf("[OTA-PULL] FW connect failed to %s:%u\n", _maskOtaHost(otaHost).c_str(), (unsigned)otaPort);
      fwFile.close();
      SD.remove(OTA_PENDING_PATH);
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FW_CONNECT");
#endif
      return false;
    }

    if (!teleClient.wifi.send(METHOD_GET, fwPath)) {
      Serial.println("[OTA-PULL] FW send failed");
      teleClient.wifi.close();
      fwFile.close();
      SD.remove(OTA_PENDING_PATH);
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FW_SEND");
#endif
      return false;
    }

    int contentLength = 0;
    int httpCode = teleClient.wifi.receiveHeaders(&contentLength);
    if (httpCode != 200) {
      Serial.printf("[OTA-PULL] FW HTTP %d\n", httpCode);
      teleClient.wifi.close();
      fwFile.close();
      SD.remove(OTA_PENDING_PATH);
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) {
        char _ota_diag[48];
        snprintf(_ota_diag, sizeof(_ota_diag), "OTA-PULL ERR=FW_HTTP%d", httpCode);
        logger.logEvent(_ota_diag);
      }
#endif
      return false;
    }
    if (contentLength > 0 && (size_t)contentLength != fwSize) {
      Serial.printf("[OTA-PULL] FW size mismatch: meta=%u header=%d\n",
                    (unsigned)fwSize, contentLength);
      fwSize = (size_t)contentLength;
    }

    WiFiClientSecure& rawSock = teleClient.wifi.rawClient();

    while (written < fwSize) {
      uint32_t chunkStart = millis();
      while (!rawSock.available() && millis() - chunkStart < PULL_OTA_CHUNK_TIMEOUT_MS) delay(1);
      if (!rawSock.available()) {
        Serial.printf("[OTA-PULL] Recv timeout at offset %u\n", (unsigned)written);
#if STORAGE != STORAGE_NONE
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=RECV_TIMEOUT");
#endif
        dlOk = false;
        break;
      }
      int toRead = (int)(fwSize - written);
      if (toRead > (int)PULL_OTA_CHUNK_SIZE) toRead = (int)PULL_OTA_CHUNK_SIZE;
      int n = rawSock.read(s_otaChunkBuf, toRead);
      if (n <= 0) {
        Serial.printf("[OTA-PULL] Read error at offset %u\n", (unsigned)written);
#if STORAGE != STORAGE_NONE
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=RECV_READ");
#endif
        dlOk = false;
        break;
      }
      size_t nw = fwFile.write(s_otaChunkBuf, (size_t)n);
      if (nw != (size_t)n) {

        Serial.printf("[OTA-PULL] SD write retry at offset %u (got %u/%u)\n",
                      (unsigned)written, (unsigned)nw, (unsigned)n);
        delay(50);
        if (nw < (size_t)n) {
          size_t nw2 = fwFile.write(s_otaChunkBuf + nw, (size_t)n - nw);
          nw += nw2;
        }
        if (nw != (size_t)n) {
          Serial.printf("[OTA-PULL] SD write error at offset %u\n", (unsigned)written);
#if STORAGE != STORAGE_NONE
          if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SD_WRITE");
#endif
          dlOk = false;
          break;
        }
        Serial.printf("[OTA-PULL] SD write retry succeeded at offset %u\n", (unsigned)written);
      }
      written += (size_t)n;
      if (doSha256) mbedtls_sha256_update_ret(&sha256Ctx, s_otaChunkBuf, (size_t)n);
      if (written - lastLogAt >= (fwSize / 10 ? fwSize / 10 : 1)) {
        lastLogAt = written;
        Serial.printf("[OTA-PULL] %u / %u bytes (%.0f%%) in %u ms\n",
                      (unsigned)written, (unsigned)fwSize,
                      100.0f * written / fwSize, (unsigned)(millis() - dlStart));
      }
    }
    fwFile.close();
    teleClient.wifi.close();
#endif

    if (!dlOk || written != fwSize) {
      Serial.printf("[OTA-PULL] Download incomplete (%u / %u bytes) — staging file removed\n",
                    (unsigned)written, (unsigned)fwSize);
      SD.remove(OTA_PENDING_PATH);
      if (doSha256) mbedtls_sha256_free(&sha256Ctx);
      return false;
    }






    if (doSha256) {
      static const int SHA256_DIGEST_BYTES = 32;
      uint8_t digest[SHA256_DIGEST_BYTES];
      mbedtls_sha256_finish_ret(&sha256Ctx, digest);
      mbedtls_sha256_free(&sha256Ctx);
      char actualHex[SHA256_DIGEST_BYTES * 2 + 1];
      for (int i = 0; i < SHA256_DIGEST_BYTES; i++) snprintf(actualHex + i * 2, 3, "%02x", digest[i]);
      actualHex[SHA256_DIGEST_BYTES * 2] = '\0';
      if (strncmp(actualHex, fwSha256Hex, SHA256_DIGEST_BYTES * 2) != 0) {
        Serial.printf("[OTA-PULL] SHA256 mismatch — staging file removed (retry at next interval)\n"
                      "[OTA-PULL]   expected: %s\n"
                      "[OTA-PULL]   actual:   %s\n",
                      fwSha256Hex, actualHex);
#if STORAGE != STORAGE_NONE
        if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=SHA256");
#endif
        SD.remove(OTA_PENDING_PATH);
        return false;
      }
      Serial.println("[OTA-PULL] SHA256 OK");
    }






#if ENABLE_WIFI




    if (ESP.getMaxAllocHeap() < TLS_MIN_FREE_HEAP) {
      Serial.printf("[OTA-PULL] Heap fragmented (%u bytes), restarting WiFi\n",
                    (unsigned)ESP.getMaxAllocHeap());
      teleClient.wifi.end();
      wifiReconnectCurrent();
      if (!teleClient.wifi.setup(WIFI_JOIN_TIMEOUT)) {
        Serial.println("[OTA-PULL] WiFi reconnect timeout after heap recovery");
      }
    }
    {
      char _confirmPath[128];
      snprintf(_confirmPath, sizeof(_confirmPath),
               "/api/freematics/ota_pull/%s/ota_confirm", otaToken);
      if (teleClient.wifi.open(otaHost, otaPort) &&
          teleClient.wifi.send(METHOD_GET, _confirmPath)) {
        int _confirmCL = 0;
        int _confirmCode = teleClient.wifi.receiveHeaders(&_confirmCL);
        Serial.printf("[OTA-PULL] Confirm %s (HTTP %d)\n",
                      _confirmCode == 200 ? "OK" : "FAILED", _confirmCode);
      } else {
        Serial.println("[OTA-PULL] Confirm request failed (non-fatal)");
      }
      teleClient.wifi.close();
    }
#endif


    {
      File metaFile = SD.open(OTA_META_PATH, FILE_WRITE);
      if (metaFile) {
        char metaBufOut[16];
        snprintf(metaBufOut, sizeof(metaBufOut), "%u\n", (unsigned)fwSize);
        metaFile.print(metaBufOut);
        metaFile.close();
      }
    }


    if (SD.exists(OTA_NVS_PATH)) SD.remove(OTA_NVS_PATH);
    if (nvsPath[0]) {
      Serial.printf("[OTA-PULL] Downloading NVS settings from %s\n", nvsPath);
#if ENABLE_WIFI
      if (teleClient.wifi.open(otaHost, otaPort) &&
          teleClient.wifi.send(METHOD_GET, nvsPath)) {
        int nvsContentLen = 0;
        int nvsHttpCode = teleClient.wifi.receiveHeaders(&nvsContentLen);
        if (nvsHttpCode == 200 && nvsContentLen > 0) {
          File nvsFile = SD.open(OTA_NVS_PATH, FILE_WRITE);
          if (nvsFile) {
            WiFiClientSecure& rawSock2 = teleClient.wifi.rawClient();
            size_t nvsWritten = 0;
            size_t nvsExpected = (size_t)nvsContentLen;
            bool nvsOk = true;
            while (nvsWritten < nvsExpected) {
              uint32_t t1 = millis();
              while (!rawSock2.available() && millis() - t1 < PULL_OTA_CHUNK_TIMEOUT_MS) delay(1);
              if (!rawSock2.available()) { nvsOk = false; break; }
              int toRead = (int)(nvsExpected - nvsWritten);
              if (toRead > (int)PULL_OTA_CHUNK_SIZE) toRead = (int)PULL_OTA_CHUNK_SIZE;
              int nr = rawSock2.read(s_otaChunkBuf, toRead);
              if (nr <= 0) { nvsOk = false; break; }
              if ((size_t)nvsFile.write(s_otaChunkBuf, (size_t)nr) != (size_t)nr) { nvsOk = false; break; }
              nvsWritten += (size_t)nr;
            }
            nvsFile.close();
            if (nvsOk && nvsWritten == nvsExpected) {
              Serial.printf("[OTA-PULL] NVS staged: %u bytes\n", (unsigned)nvsWritten);
#if STORAGE != STORAGE_NONE
              if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL NVS DL OK");
#endif
            } else {
              Serial.println("[OTA-PULL] NVS download incomplete — skipping settings update");
              SD.remove(OTA_NVS_PATH);
            }
          }
        } else {
          Serial.printf("[OTA-PULL] NVS HTTP %d — skipping settings update\n", nvsHttpCode);
        }
        teleClient.wifi.close();
      } else {
        Serial.println("[OTA-PULL] NVS connect/send failed — skipping settings update");
        teleClient.wifi.close();
      }
#endif
    }

    s_ota_pending = true;
    Serial.printf("[OTA-PULL] Download complete: %u bytes in %u ms\n"
                  "[OTA-PULL] Firmware staged on SD (%s) — restarting to flash\n",
                  (unsigned)written, (unsigned)(millis() - dlStart), OTA_PENDING_PATH);
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) {
      char _ota_diag[64];
      snprintf(_ota_diag, sizeof(_ota_diag), "OTA-PULL DL OK SIZE=%u", (unsigned)fwSize);
      logger.logEvent(_ota_diag);
    }
#endif






    esp_restart();
    return false;
  }
#endif


#if ENABLE_WIFI

  s_ota_active = true;
  delay(1500);

  if (!teleClient.wifi.open(otaHost, otaPort)) {
    Serial.printf("[OTA-PULL] FW connect failed to %s:%u\n", _maskOtaHost(otaHost).c_str(), (unsigned)otaPort);
    s_ota_active = false;
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FW_CONNECT");
#endif
    return false;
  }

  if (!teleClient.wifi.send(METHOD_GET, fwPath)) {
    Serial.println("[OTA-PULL] FW send failed");
    teleClient.wifi.close();
    s_ota_active = false;
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FW_SEND");
#endif
    return false;
  }

  int contentLength = 0;
  int httpCode = teleClient.wifi.receiveHeaders(&contentLength);
  if (httpCode != 200) {
    Serial.printf("[OTA-PULL] FW HTTP %d\n", httpCode);
    teleClient.wifi.close();
    s_ota_active = false;
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) {
      char _ota_diag[48];
      snprintf(_ota_diag, sizeof(_ota_diag), "OTA-PULL ERR=FW_HTTP%d", httpCode);
      logger.logEvent(_ota_diag);
    }
#endif
    return false;
  }
  if (contentLength > 0 && (size_t)contentLength != fwSize) {
    Serial.printf("[OTA-PULL] FW size mismatch: meta=%u header=%d\n",
                  (unsigned)fwSize, contentLength);
    fwSize = (size_t)contentLength;
  }

  if (!Update.begin(fwSize)) {
    Serial.printf("[OTA-PULL] Update.begin failed: %s\n", Update.errorString());
    teleClient.wifi.close();
    s_ota_active = false;
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=UPD_BEGIN");
#endif
    return false;
  }

  WiFiClientSecure& rawSock = teleClient.wifi.rawClient();
  size_t written = 0;
  uint32_t dlStart = millis();
  size_t lastLogAt = 0;

  while (written < fwSize) {
    uint32_t chunkStart = millis();
    while (!rawSock.available() && millis() - chunkStart < PULL_OTA_CHUNK_TIMEOUT_MS) delay(1);
    if (!rawSock.available()) {
      Serial.printf("[OTA-PULL] Recv timeout at offset %u\n", (unsigned)written);
      Update.abort();
      teleClient.wifi.close();
      s_ota_active = false;
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=RECV_TIMEOUT");
#endif
      return false;
    }

    int toRead = (int)(fwSize - written);
    if (toRead > (int)PULL_OTA_CHUNK_SIZE) toRead = (int)PULL_OTA_CHUNK_SIZE;
    int n = rawSock.read(s_otaChunkBuf, toRead);
    if (n <= 0) {
      Serial.printf("[OTA-PULL] Read error at offset %u\n", (unsigned)written);
      Update.abort();
      teleClient.wifi.close();
      s_ota_active = false;
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=RECV_READ");
#endif
      return false;
    }

    size_t w = Update.write(s_otaChunkBuf, (size_t)n);
    if (w != (size_t)n) {
      Serial.printf("[OTA-PULL] Flash write error at offset %u\n", (unsigned)written);
      Update.abort();
      teleClient.wifi.close();
      s_ota_active = false;
#if STORAGE != STORAGE_NONE
      if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=FLASH_WRITE");
#endif
      return false;
    }
    written += (size_t)n;

    if (written - lastLogAt >= (fwSize / 10 ? fwSize / 10 : 1)) {
      lastLogAt = written;
      Serial.printf("[OTA-PULL] %u / %u bytes (%.0f%%) in %u ms\n",
                    (unsigned)written, (unsigned)fwSize,
                    100.0f * written / fwSize, (unsigned)(millis() - dlStart));
    }
  }

  teleClient.wifi.close();
  Serial.printf("[OTA-PULL] Download complete: %u bytes in %u ms\n",
                (unsigned)written, (unsigned)(millis() - dlStart));

  if (!Update.end()) {
    Serial.printf("[OTA-PULL] Update.end failed: %s\n", Update.errorString());
    s_ota_active = false;
#if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) logger.logEvent("OTA-PULL ERR=UPD_END");
#endif
    return false;
  }

  Serial.println("[OTA-PULL] Flash successful, rebooting in 1.5 s");
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    char _ota_diag[64];
    snprintf(_ota_diag, sizeof(_ota_diag),
             "OTA-PULL OK FW=%s SIZE=%u", FIRMWARE_VERSION, (unsigned)written);
    logger.logEvent(_ota_diag);
  }
#endif



  {
    char _confirmPath[128];
    snprintf(_confirmPath, sizeof(_confirmPath),
             "/api/freematics/ota_pull/%s/ota_confirm", otaToken);
    if (teleClient.wifi.open(otaHost, otaPort) &&
        teleClient.wifi.send(METHOD_GET, _confirmPath)) {
      int _confirmCL = 0;
      int _confirmCode = teleClient.wifi.receiveHeaders(&_confirmCL);
      Serial.printf("[OTA-PULL] Confirm %s (HTTP %d)\n",
                    _confirmCode == 200 ? "OK" : "FAILED", _confirmCode);
    } else {
      Serial.println("[OTA-PULL] Confirm request failed (non-fatal)");
    }
    teleClient.wifi.close();
  }

  static esp_timer_handle_t s_pull_ota_timer = NULL;
  if (!s_pull_ota_timer) {
    esp_timer_create_args_t args = {};
    args.callback = [](void*) { esp_restart(); };
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "pull_ota_restart";
    esp_timer_create(&args, &s_pull_ota_timer);
  } else {
    esp_timer_stop(s_pull_ota_timer);
  }
  esp_timer_start_once(s_pull_ota_timer, 1500000);
  return true;
#endif
  return false;
}

void processBLE(int timeout)
{
#if ENABLE_BLE
  if (!enableBle) {
    if (timeout) delay(timeout);
    return;
  }
  static byte echo = 0;
  char* cmd;
  if (!(cmd = ble_recv_command(timeout))) {
    return;
  }

  char *p = strchr(cmd, '\r');
  if (p) *p = 0;
  char buf[48];
  int bufsize = sizeof(buf);
  int n = 0;
  if (echo) n += snprintf(buf + n, bufsize - n, "%s\r", cmd);
  Serial.print("[BLE] ");
  Serial.print(cmd);
  if (!strcmp(cmd, "UPTIME") || !strcmp(cmd, "TICK")) {
    n += snprintf(buf + n, bufsize - n, "%lu", millis());
  } else if (!strcmp(cmd, "BATT")) {
    n += snprintf(buf + n, bufsize - n, "%.2f", (float)(analogRead(A0) * 42) / 4095);
  } else if (!strcmp(cmd, "RESET")) {
#if STORAGE
    logger.end();
#endif
    ESP.restart();

  } else if (!strcmp(cmd, "OFF")) {
    state.set(STATE_STANDBY);
    state.clear(STATE_WORKING);
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ON")) {
    state.clear(STATE_STANDBY);
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ON?")) {
    n += snprintf(buf + n, bufsize - n, "%u", state.check(STATE_STANDBY) ? 0 : 1);
  } else if (!strcmp(cmd, "APN?")) {
    n += snprintf(buf + n, bufsize - n, "%s", *apn ? apn : "DEFAULT");
  } else if (!strncmp(cmd, "APN=", 4)) {
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "CELL_APN", strcmp(cmd + 4, "DEFAULT") ? cmd + 4 : "") == ESP_OK
        && nvs_commit(nvs) == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strcmp(cmd, "PIN?")) {
    n += snprintf(buf + n, bufsize - n, "%s", *simPin ? "SET" : "NONE");
  } else if (!strncmp(cmd, "PIN=", 4)) {
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "SIM_PIN", strcmp(cmd + 4, "CLEAR") ? cmd + 4 : "") == ESP_OK
        && nvs_commit(nvs) == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strcmp(cmd, "NET_OP")) {
    if (state.check(STATE_WIFI_CONNECTED)) {
#if ENABLE_WIFI
      n += snprintf(buf + n, bufsize - n, "%s", wifiSSID[0] ? wifiSSID : "-");
#endif
    } else {
      snprintf(buf + n, bufsize - n, "%s", netop.length() ? netop.c_str() : "-");
      char *p = strchr(buf + n, ' ');
      if (p) *p = 0;
      n += strlen(buf + n);
    }
  } else if (!strcmp(cmd, "NET_IP")) {
    n += snprintf(buf + n, bufsize - n, "%s", ip.length() ? ip.c_str() : "-");
  } else if (!strcmp(cmd, "NET_PACKET")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.txCount);
  } else if (!strcmp(cmd, "NET_DATA")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.txBytes);
  } else if (!strcmp(cmd, "NET_RATE")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.startTime ? (unsigned int)((uint64_t)(teleClient.txBytes + teleClient.rxBytes) * 3600 / (millis() - teleClient.startTime)) : 0);
  } else if (!strcmp(cmd, "RSSI")) {
    n += snprintf(buf + n, bufsize - n, "%d", rssi);
#if ENABLE_WIFI
  } else if (!strcmp(cmd, "SSID?")) {
    n += snprintf(buf + n, bufsize - n, "%s", wifiSSID[0] ? wifiSSID : "-");
  } else if (!strncmp(cmd, "SSID=", 5)) {
    const char* p = cmd + 5;
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "WIFI_SSID", strcmp(p, "-") ? p : "") == ESP_OK
        && nvs_commit(nvs) == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strcmp(cmd, "WPWD?")) {
    n += snprintf(buf + n, bufsize - n, "%s", wifiPassword[0] ? wifiPassword : "-");
  } else if (!strncmp(cmd, "WPWD=", 5)) {
    const char* p = cmd + 5;
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "WIFI_PWD", strcmp(p, "-") ? p : "") == ESP_OK
        && nvs_commit(nvs) == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strncmp(cmd, "OTA_TOKEN=", 10)) {




    const char* p = cmd + 10;
    const bool clr = (p[0] == '\0' || (p[0] == '-' && p[1] == '\0'));
    esp_err_t e = nvs_set_str(nvs, "OTA_TOKEN", clr ? "" : p);
    if (e == ESP_OK) e = nvs_commit(nvs);
    if (e == ESP_OK) {
      size_t tlen = sizeof(otaToken);
      otaToken[0] = 0;
      nvs_get_str(nvs, "OTA_TOKEN", otaToken, &tlen);


      if (!otaHost[0] && otaToken[0]) {
        strncpy(otaHost, serverHost, sizeof(otaHost) - 1);
        otaHost[sizeof(otaHost) - 1] = 0;
      }
    }
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
  } else if (!strncmp(cmd, "OTA_HOST=", 9)) {



    const char* p = cmd + 9;
    const bool clr = (p[0] == '-' && p[1] == '\0');
    esp_err_t e = nvs_set_str(nvs, "OTA_HOST", clr ? "" : p);
    if (e == ESP_OK) e = nvs_commit(nvs);
    if (e == ESP_OK) {
      size_t hlen = sizeof(otaHost);
      otaHost[0] = 0;
      nvs_get_str(nvs, "OTA_HOST", otaHost, &hlen);

      if (!otaHost[0] && otaToken[0]) {
        strncpy(otaHost, serverHost, sizeof(otaHost) - 1);
        otaHost[sizeof(otaHost) - 1] = 0;
      }
    }
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
  } else if (!strncmp(cmd, "OTA_INTERVAL=", 13)) {



    uint16_t interval = (uint16_t)atoi(cmd + 13);
    esp_err_t e = nvs_set_u16(nvs, "OTA_INTERVAL", interval);
    if (e == ESP_OK) e = nvs_commit(nvs);
    if (e == ESP_OK) otaCheckIntervalS = interval;
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
#else
  } else if (!strcmp(cmd, "SSID?") || !strcmp(cmd, "WPWD?")) {
    n += snprintf(buf + n, bufsize - n, "-");
#endif
#if ENABLE_MEMS
  } else if (!strcmp(cmd, "TEMP")) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)deviceTemp);
  } else if (!strcmp(cmd, "ACC")) {
    n += snprintf(buf + n, bufsize - n, "%.1f/%.1f/%.1f", acc[0], acc[1], acc[2]);
  } else if (!strcmp(cmd, "GYRO")) {
    n += snprintf(buf + n, bufsize - n, "%.1f/%.1f/%.1f", gyr[0], gyr[1], gyr[2]);
  } else if (!strcmp(cmd, "GF")) {
    n += snprintf(buf + n, bufsize - n, "%f", (float)sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]));
#endif
  } else if (!strcmp(cmd, "ATE0")) {
    echo = 0;
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ATE1")) {
    echo = 1;
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "FS")) {
    n += snprintf(buf + n, bufsize - n, "%u",
#if STORAGE == STORAGE_NONE
    0
#else
    logger.size()
#endif
      );
  } else if (!memcmp(cmd, "01", 2)) {
    byte pid = hex2uint8(cmd + 2);
    for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
      if (obdData[i].pid == pid) {
        n += snprintf(buf + n, bufsize - n, "%d", obdData[i].value);
        pid = 0;
        break;
      }
    }
    if (pid) {
      int value;
      if (obd.readPID(pid, value)) {
        n += snprintf(buf + n, bufsize - n, "%d", value);
      } else {
        n += snprintf(buf + n, bufsize - n, "N/A");
      }
    }
  } else if (!strcmp(cmd, "VIN")) {
    n += snprintf(buf + n, bufsize - n, "%s", vin[0] ? vin : "N/A");
  } else if (!strcmp(cmd, "LAT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%f", gd->lat);
  } else if (!strcmp(cmd, "LNG") && gd) {
    n += snprintf(buf + n, bufsize - n, "%f", gd->lng);
  } else if (!strcmp(cmd, "ALT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)gd->alt);
  } else if (!strcmp(cmd, "SAT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%u", (unsigned int)gd->sat);
  } else if (!strcmp(cmd, "SPD") && gd) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)(gd->speed * 1852 / 1000));
  } else if (!strcmp(cmd, "CRS") && gd) {
    n += snprintf(buf + n, bufsize - n, "%u", (unsigned int)gd->heading);
  } else if (!strncmp(cmd, "LED_WHITE=", 10)) {

    uint8_t v = (uint8_t)atoi(cmd + 10);
    enableLedWhite = (v != 0);
    esp_err_t e = nvs_set_u8(nvs, "LED_WHITE_EN", v);
    if (e == ESP_OK) e = nvs_commit(nvs);
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
  } else if (!strncmp(cmd, "LED_RED=", 8)) {

    uint8_t v = (uint8_t)atoi(cmd + 8);
    enableLedRed = (v != 0);
    esp_err_t e = nvs_set_u8(nvs, "LED_RED_EN", v);
    if (e == ESP_OK) e = nvs_commit(nvs);
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
  } else if (!strncmp(cmd, "BEEP=", 5)) {

    uint8_t v = (uint8_t)atoi(cmd + 5);
    enableBeep = (v != 0);
    esp_err_t e = nvs_set_u8(nvs, "BEEP_EN", v);
    if (e == ESP_OK) e = nvs_commit(nvs);
    n += snprintf(buf + n, bufsize - n, e == ESP_OK ? "OK" : "ERR");
  } else {
    n += snprintf(buf + n, bufsize - n, "ERROR");
  }
  Serial.print(" -> ");
  Serial.println((p = strchr(buf, '\r')) ? p + 1 : buf);
  if (n < bufsize - 1) {
    buf[n++] = '\r';
  } else {
    n = bufsize - 1;
  }
  buf[n] = 0;
  ble_send_response(buf, n, cmd);
#else
  if (timeout) delay(timeout);
#endif
}
# 3951 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
void printOtaStatus()
{
  if (otaToken[0]) {


    char _tok8[9];
    strncpy(_tok8, otaToken, 8);
    _tok8[8] = '\0';
    Serial.printf("OTA:TOKEN=%s... HOST=%s PORT=%u INTERVAL=%us%s\n",
                  _tok8,
                  otaHost[0] ? _maskOtaHost(otaHost).c_str() : "(server fallback)",
                  (unsigned)otaPort,
                  (unsigned)otaCheckIntervalS,
                  otaCheckIntervalS == 0 ? " (checks disabled)" : "");
  } else {
    Serial.println("OTA:disabled");
  }
}

void setup()
{






#ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
#endif

  delay(500);







  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    if (nvs_flash_erase() == ESP_OK) {
      err = nvs_flash_init();
    }
  }
  if (err == ESP_OK && nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK) {
    loadConfig();
  }

#if ENABLE_OLED
  oled.begin();
  oled.setFontSize(FONT_SIZE_SMALL);
#endif

  Serial.begin(115200);




#ifdef PIN_LED
  digitalWrite(PIN_LED, enableLedRed ? HIGH : LOW);
#endif


  genDeviceID(devid);

#if CONFIG_MODE_TIMEOUT
  configMode();
#endif

#if LOG_EXT_SENSORS == 1
  pinMode(PIN_SENSOR1, INPUT);
  pinMode(PIN_SENSOR2, INPUT);
#elif LOG_EXT_SENSORS == 2
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
#endif


  showSysInfo();

  bufman.init();




#if ENABLE_OBD
  if (sys.begin()) {
    Serial.print("TYPE:");
    Serial.println(sys.devType);
    obd.begin(sys.link);
  }
#else
  sys.begin(false, true);
#endif

#if ENABLE_MEMS
if (!state.check(STATE_MEMS_READY)) do {
  Serial.print("MEMS:");
  mems = new ICM_42627;
  byte ret = mems->begin();
  if (ret) {
    state.set(STATE_MEMS_READY);
    Serial.println("ICM-42627");
    break;
  }
  delete mems;
  mems = new ICM_20948_I2C;
  ret = mems->begin();
  if (ret) {
    state.set(STATE_MEMS_READY);
    Serial.println("ICM-20948");
    break;
  }
  delete mems;
# 4077 "C:/Users/DereC/Desktop/Freematics-master/northpower25/Freematics-master/firmware_v5/telelogger/telelogger.ino"
  mems = 0;
  Serial.println("NO");
} while (0);
#endif

#if ENABLE_HTTPD
  if (enableHttpd) {
    IPAddress ip;
    if (serverSetup(ip)) {
      Serial.print("HTTPD:");
      Serial.println(ip);
#if ENABLE_OLED
      oled.println(ip);
#endif
    } else {
      Serial.println("HTTPD:NO");
    }
  }
#endif

  state.set(STATE_WORKING);

#if ENABLE_BLE
  if (enableBle) {

    ble_init("FreematicsPlus");
  }
#endif






  printOtaStatus();


  initialize();




  subtask.create(telemetry, "telemetry", 2, 16384);

#ifdef PIN_LED
  digitalWrite(PIN_LED, LOW);
#endif
}

void loop()
{

  if (!state.check(STATE_WORKING)) {
    standby();
#ifdef PIN_LED
    if (enableLedRed) digitalWrite(PIN_LED, HIGH);
#endif
    initialize();
#ifdef PIN_LED
    digitalWrite(PIN_LED, LOW);
#endif
    return;
  }


  process();
}