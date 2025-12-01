/*
  ESP32 MASTER BRIDGE
  -------------------
  - Recibe frames binarios por Serial2 desde el RP2040:
      UART_SYNC(0x55) + LEN + PAYLOAD + CHK
  - PAYLOAD[0] = tipo (0x01, 0x10, 0x11, 0x12, 0x20)
  - Reenvía por ESP-NOW hacia el SLAVE:
      * TYPE_CMD_FRAME (0x01) -> SLAVE (XF551)
  - Recibe desde SLAVE por ESP-NOW:
      * TYPE_HELLO (0x20)
      * TYPE_SECTOR_CHUNK (0x10)
      * TYPE_ACK (0x11)
      * TYPE_NAK (0x12)
    y los reenvía por UART al RP2040 con el mismo framing.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ===== Protocol =====
#define TYPE_CMD_FRAME    0x01
#define TYPE_SECTOR_CHUNK 0x10
#define TYPE_ACK          0x11
#define TYPE_NAK          0x12
#define TYPE_HELLO        0x20

#define UART_SYNC 0x55

// Device id que esperamos (D1=0x31)
const uint8_t DEVICE_ID = 0x31;

// Broadcast MAC
const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF};

// Último SLAVE conocido
uint8_t g_lastSlave[6] = {0};
bool    g_haveSlave    = false;

// UART2 (con RP2040)
const int PIN_RP_RX = 16; // RX2
const int PIN_RP_TX = 17; // TX2

// ===== Utils =====
uint8_t calcChecksum(const uint8_t* buf, int len) {
  uint16_t s = 0;
  for (int i = 0; i < len; i++) s += buf[i];
  return (uint8_t)s;
}

void ensurePeer(const uint8_t* mac) {
  if (!mac) return;
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  esp_now_add_peer(&p);
}

const uint8_t* slaveMac() {
  return g_haveSlave ? g_lastSlave : BCAST_MAC;
}

bool sendEspNow(const uint8_t* mac, const uint8_t* data, int len) {
  ensurePeer(mac);
  esp_err_t r = esp_now_send(mac, data, len);
  return (r == ESP_OK);
}

bool sendEspToSlave(const uint8_t* data, int len) {
  return sendEspNow(slaveMac(), data, len);
}

// ===== UART TX -> RP2040 =====
void sendUartFrameToRP(const uint8_t* payload, uint8_t len) {
  if (len == 0) return;
  uint8_t chk = calcChecksum(payload, len);

  Serial2.write(UART_SYNC);
  Serial2.write(len);
  Serial2.write(payload, len);
  Serial2.write(chk);
  Serial2.flush();

  Serial.print(F("[MASTER] UART->RP len="));
  Serial.print(len);
  Serial.print(F(" tipo=0x"));
  if (payload[0] < 0x10) Serial.print('0');
  Serial.println(payload[0], HEX);
}

// ===== UART RX <- RP2040 FSM =====
static uint8_t  uartState    = 0;
static uint8_t  uartLen      = 0;
static uint8_t  uartIdx      = 0;
static uint32_t uartLastTime = 0;
static uint8_t  uartBuf[255];

void pollUartFromRP() {
  while (Serial2.available() > 0) {
    uint8_t b = (uint8_t)Serial2.read();
    uartLastTime = millis();

    Serial.print(F("[MASTER UART FSM] state="));
    Serial.print(uartState);
    Serial.print(F(" byte=0x"));
    if (b < 0x10) Serial.print('0');
    Serial.println(b, HEX);

    switch (uartState) {
      case 0:
        if (b == UART_SYNC) uartState = 1;
        break;
      case 1:
        uartLen = b;
        if (uartLen == 0) uartState = 0;
        else {
          uartIdx   = 0;
          uartState = 2;
        }
        break;
      case 2:
        uartBuf[uartIdx++] = b;
        if (uartIdx >= uartLen) uartState = 3;
        break;
      case 3: {
        uint8_t chk = b;
        uint8_t sum = calcChecksum(uartBuf, uartLen);
        if (chk != sum) {
          Serial.println(F("[MASTER] UART: checksum inválido, descartando frame."));
          uartState = 0;
        } else {
          uint8_t type = uartBuf[0];
          Serial.print(F("[MASTER] Frame UART válido recibido, len="));
          Serial.print(uartLen);
          Serial.print(F(" tipo=0x"));
          if (type < 0x10) Serial.print('0');
          Serial.println(type, HEX);

          if (type == TYPE_CMD_FRAME) {
            // Comando desde RP -> reenviar al SLAVE
            sendEspToSlave(uartBuf, uartLen);
            Serial.print(F("[MASTER] Enviado payload a SLAVE tipo=0x"));
            if (type < 0x10) Serial.print('0');
            Serial.print(type, HEX);
            Serial.print(F(" len="));
            Serial.println(uartLen);
          } else {
            // Otros tipos desde RP (no esperados por ahora)
            Serial.println(F("[MASTER] Tipo desconocido desde RP (no reenviado)."));
          }

          uartState = 0;
        }
        break;
      }
    }
  }

  if (uartState != 0 && (millis() - uartLastTime > 50)) {
    Serial.println(F("[MASTER] UART: timeout mid-frame, reseteando FSM."));
    uartState = 0;
    uartIdx   = 0;
    uartLen   = 0;
  }
}

// ===== ESP-NOW callbacks =====
#if ESP_IDF_VERSION_MAJOR >= 5

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t* data, int len) {
  if (!data || len <= 0) return;
  const uint8_t* src = info ? info->src_addr : nullptr;

  if (src) {
    bool isBcast = true;
    for (int i = 0; i < 6; i++) {
      if (src[i] != 0xFF) { isBcast = false; break; }
    }
    if (!isBcast) {
      memcpy(g_lastSlave, src, 6);
      g_haveSlave = true;
      ensurePeer(g_lastSlave);
    }
  }

  uint8_t type = data[0];

  Serial.print(F("[MASTER] onDataRecv type=0x"));
  if (type < 0x10) Serial.print('0');
  Serial.print(type, HEX);
  Serial.print(F(" len="));
  Serial.println(len);

  if (type == TYPE_HELLO && len >= 3) {
    uint8_t dev = data[1];
    uint8_t dd  = data[2];
    Serial.print(F("[MASTER] HELLO de SLAVE DEV=0x"));
    if (dev < 0x10) Serial.print('0');
    Serial.println(dev, HEX);

    // Reenviar HELLO al RP
    sendUartFrameToRP(data, len);
  }
  else if (type == TYPE_SECTOR_CHUNK || type == TYPE_ACK || type == TYPE_NAK) {
    // Reenviar tal cual al RP
    sendUartFrameToRP(data, len);
  }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t s) {
  (void)info;
  if (s == ESP_NOW_SEND_SUCCESS)
    Serial.println(F("[MASTER] ESP-NOW envío OK"));
  else
    Serial.println(F("[MASTER] ESP-NOW fallo en envío"));
}

#else

void onDataRecv(const uint8_t* src, const uint8_t* data, int len) {
  if (!data || len <= 0) return;

  if (src) {
    bool isBcast = true;
    for (int i = 0; i < 6; i++) {
      if (src[i] != 0xFF) { isBcast = false; break; }
    }
    if (!isBcast) {
      memcpy(g_lastSlave, src, 6);
      g_haveSlave = true;
      ensurePeer(g_lastSlave);
    }
  }

  uint8_t type = data[0];

  Serial.print(F("[MASTER] onDataRecv type=0x"));
  if (type < 0x10) Serial.print('0');
  Serial.print(type, HEX);
  Serial.print(F(" len="));
  Serial.println(len);

  if (type == TYPE_HELLO && len >= 3) {
    uint8_t dev = data[1];
    uint8_t dd  = data[2];
    Serial.print(F("[MASTER] HELLO de SLAVE DEV=0x"));
    if (dev < 0x10) Serial.print('0');
    Serial.println(dev, HEX);

    sendUartFrameToRP(data, len);
  }
  else if (type == TYPE_SECTOR_CHUNK || type == TYPE_ACK || type == TYPE_NAK) {
    sendUartFrameToRP(data, len);
  }
}

void onDataSent(const uint8_t* mac, esp_now_send_status_t s) {
  (void)mac;
  if (s == ESP_NOW_SEND_SUCCESS)
    Serial.println(F("[MASTER] ESP-NOW envío OK"));
  else
    Serial.println(F("[MASTER] ESP-NOW fallo en envío"));
}

#endif

// ===== SETUP / LOOP =====
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(F("\n=== ESP32 MASTER BRIDGE (RP2040 <-> SLAVE XF551) ==="));

  Serial2.begin(115200, SERIAL_8N1, PIN_RP_RX, PIN_RP_TX);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[MASTER] ERROR: esp_now_init fallo"));
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  ensurePeer(BCAST_MAC);
}

void loop() {
  pollUartFromRP();
  delay(2);
}
