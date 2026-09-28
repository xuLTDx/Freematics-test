#include "canodo.h"
#include "driver/twai.h"

#define CANODO_TX_PIN     GPIO_NUM_26
#define CANODO_RX_PIN     GPIO_NUM_34
#define CANODO_GW_REQ     0x710   // tester -> Gateway (module 19)
#define CANODO_GW_RESP    0x77A   // Gateway -> tester
#define CANODO_IC_REQ     0x714   // tester -> Instruments (module 17)
#define CANODO_IC_RESP    0x77E   // Instruments -> tester
#define CANODO_PAD        0x55
#define CANODO_TIMEOUT_MS 300     // per wait for the next frame
#define CANODO_MAX_REPLY  64      // fuel reply: 3 + 53 bytes

static bool s_ready = false;
static uint32_t s_framesAll = 0;
static uint32_t s_frames77A = 0;
static uint32_t s_lastId = 0;

bool canOdoBegin()
{
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CANODO_TX_PIN, CANODO_RX_PIN, TWAI_MODE_NORMAL);
  g.tx_queue_len = 2;
  g.rx_queue_len = 32;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  // all frames accepted: the count of any traffic seen tells whether the
  // module receives from the bus at all (the box's own co-processor queries
  // the engine on the same wires), independent of the gateway
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t e = twai_driver_install(&g, &t, &f);
  if (e == ESP_OK) e = twai_start();
  s_ready = (e == ESP_OK);
  Serial.printf("[CAN] %s (err=0x%x)\n", s_ready ? "ready" : "FAILED", (unsigned)e);
  // Wiring check without a multimeter: CRX mirrors the bus through the
  // transceiver. 1 = bus recessive (free); stuck at 0 = something (e.g. CTX)
  // holds the bus dominant, or CRX/3V3 is not connected - not for the car.
  int ones = 0;
  for (int i = 0; i < 20; i++) {
    ones += gpio_get_level(CANODO_RX_PIN);
    delay(5);
  }
  Serial.printf("[CAN] CRX high %d/20\n", ones);
  return s_ready;
}

// Back to a usable state after errors (no ACK on the bench, bus-off).
static void recover()
{
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return;
  if (st.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
  } else if (st.state == TWAI_STATE_STOPPED) {
    twai_start();
  }
}

static void fillStatus(CanOdoDiag* d)
{
  d->framesAll = s_framesAll;
  d->frames77A = s_frames77A;
  d->lastId = s_lastId;
  twai_status_info_t st;
  if (s_ready && twai_get_status_info(&st) == ESP_OK) {
    d->state = (uint8_t)st.state;
    d->txErr = st.tx_error_counter;
    d->rxErr = st.rx_error_counter;
    d->busErr = st.bus_error_count;
    d->txFailed = st.tx_failed_count;
  }
}

static bool sendFrame(uint32_t id, const uint8_t* data)
{
  twai_message_t m = {};
  m.identifier = id;
  m.ss = 1;                 // single shot: never retransmitted
  m.data_length_code = 8;
  memcpy(m.data, data, 8);
  return twai_transmit(&m, pdMS_TO_TICKS(20)) == ESP_OK;
}

static void countFrame(const twai_message_t& m)
{
  s_framesAll++;
  s_lastId = m.identifier;
  if (m.identifier == CANODO_GW_RESP && !m.extd) s_frames77A++;
}

// Drains the receive queue into the counters (frames between reads).
static void countPending()
{
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) countFrame(m);
}

static bool waitFrame(uint32_t respId, twai_message_t* m)
{
  uint32_t start = millis();
  while (millis() - start < CANODO_TIMEOUT_MS) {
    if (twai_receive(m, pdMS_TO_TICKS(20)) != ESP_OK) continue;
    countFrame(*m);
    if (m->identifier == respId && !m->extd && m->data_length_code >= 2) return true;
  }
  return false;
}

static void setRaw(CanOdoDiag* d, const uint8_t* p, size_t n)
{
  d->rawLen = n < sizeof(d->raw) ? n : sizeof(d->raw);
  memcpy(d->raw, p, d->rawLen);
}

static bool fail(CanOdoDiag* d, uint8_t result)
{
  d->result = result;
  fillStatus(d);
  return false;
}

// UDS 22 <did> to `reqId`, reply from `respId` (ISO-TP, flow control by us).
// The whole positive reply (62 <did> <data>) goes into `payload`.
static bool udsRead(uint32_t reqId, uint32_t respId, uint16_t did,
                    uint8_t* payload, size_t cap, size_t* len, CanOdoDiag* d)
{
  memset(d, 0, sizeof(*d));
  if (!s_ready) return fail(d, CANODO_NOT_READY);
  recover();
  countPending();

  const uint8_t req[8] = {0x03, 0x22, (uint8_t)(did >> 8), (uint8_t)did,
                          CANODO_PAD, CANODO_PAD, CANODO_PAD, CANODO_PAD};
  if (!sendFrame(reqId, req)) return fail(d, CANODO_TX_FAILED);

  size_t want = 0, got = 0;
  twai_message_t m;
  for (;;) {
    if (!waitFrame(respId, &m)) return fail(d, CANODO_NO_REPLY);
    uint8_t pci = m.data[0] >> 4;
    if (pci == 0) {                        // single frame (negative response or short reply)
      size_t n = m.data[0] & 0x0F;
      if (n >= 3 && m.data[1] == 0x7F && m.data[3] == 0x78) continue;  // response pending
      if (n >= 3 && m.data[1] == 0x7F) {
        setRaw(d, m.data + 1, n > 7 ? 7 : n);
        d->nrc = m.data[3];
        return fail(d, CANODO_NRC);
      }
      want = n > cap ? cap : n;
      memcpy(payload, m.data + 1, want);
      got = want;
      break;
    }
    if (pci == 1) {                        // first frame: send flow control, collect the rest
      want = ((size_t)(m.data[0] & 0x0F) << 8) | m.data[1];
      if (want > cap) want = cap;
      got = want < 6 ? want : 6;
      memcpy(payload, m.data + 2, got);
      const uint8_t fc[8] = {0x30, 0x00, 0x00, CANODO_PAD, CANODO_PAD, CANODO_PAD, CANODO_PAD, CANODO_PAD};
      if (!sendFrame(reqId, fc)) return fail(d, CANODO_FC_FAILED);
      uint8_t seq = 1;
      while (got < want) {
        if (!waitFrame(respId, &m)) { setRaw(d, payload, got); return fail(d, CANODO_CF_TIMEOUT); }
        if ((m.data[0] >> 4) != 2 || (m.data[0] & 0x0F) != (seq & 0x0F)) return fail(d, CANODO_CF_ORDER);
        size_t n = want - got < 7 ? want - got : 7;
        memcpy(payload + got, m.data + 1, n);
        got += n;
        seq++;
      }
      break;
    }
    // flow control or anything else from the ECU: not expected here
  }
  setRaw(d, payload, got);
  if (got < 3 || payload[0] != 0x62 || payload[1] != (uint8_t)(did >> 8) || payload[2] != (uint8_t)did) {
    return fail(d, CANODO_BAD_REPLY);
  }
  *len = got;
  d->result = CANODO_OK;
  fillStatus(d);
  return true;
}

bool canOdoRead(uint32_t* km, CanOdoDiag* d)
{
  uint8_t payload[32];
  size_t len = 0;
  if (!udsRead(CANODO_GW_REQ, CANODO_GW_RESP, 0x02BD, payload, sizeof(payload), &len, d)) return false;
  // 62 02 BD <status> <km hi> <km mid> <km lo> ...
  if (len < 7) return fail(d, CANODO_BAD_REPLY);
  uint32_t v = ((uint32_t)payload[4] << 16) | ((uint32_t)payload[5] << 8) | payload[6];
  if (v == 0 || v > 2000000) return fail(d, CANODO_IMPLAUSIBLE);
  *km = v;
  return true;
}

bool canFuelRead(uint8_t* data, size_t cap, size_t* len, CanOdoDiag* d)
{
  uint8_t payload[CANODO_MAX_REPLY];
  size_t n = 0;
  if (!udsRead(CANODO_IC_REQ, CANODO_IC_RESP, 0x22B0, payload, sizeof(payload), &n, d)) return false;
  n -= 3;                                  // drop 62 22 B0
  *len = n < cap ? n : cap;
  memcpy(data, payload + 3, *len);
  return true;
}
