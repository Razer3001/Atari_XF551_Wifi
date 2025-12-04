/*
  ESP32 MASTER BRIDGE + WEB UI
  ----------------------------
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

  - Web UI (HTTP en puerto 80, AP "XF551_MASTER" / "xf551wifi").
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <stdarg.h>

// ===== Protocol =====
#define TYPE_CMD_FRAME    0x01
#define TYPE_SECTOR_CHUNK 0x10
#define TYPE_ACK          0x11
#define TYPE_NAK          0x12
#define TYPE_HELLO        0x20

#define UART_SYNC 0x55

// D1..D4
#define DEV_MIN 0x31
#define DEV_MAX 0x34

// Prefetch máximo
#define MAX_PREFETCH_SECTORS 4

// Broadcast MAC
const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Último SLAVE conocido
uint8_t g_lastSlave[6] = {0};
bool    g_haveSlave    = false;

// UART2 (con RP2040)
const int PIN_RP_RX = 16; // RX2
const int PIN_RP_TX = 17; // TX2

// ========= Estado de SLAVES =========
struct SlaveInfo {
  bool present;
  bool supports256;
  uint8_t mac[6];
  unsigned long lastSeen;
};

SlaveInfo slaves[4]; // D1..D4

// Prefetch configurado por unidad
uint8_t prefetchCfg[4] = {1, 1, 1, 1};

// ========= Timings SIO (µs) – solo config (para futuro uso con RP) =========
// Valores por defecto
uint16_t T_ACK_TO_COMPLETE   = 600;
uint16_t T_COMPLETE_TO_DATA  = 400;
uint16_t T_DATA_TO_CHK       = 80;
uint16_t T_CHUNK_DELAY       = 600;

// ========= WebServer & NVS =========
WebServer server(80);
Preferences prefs;
const uint32_t CFG_MAGIC = 0xCAFEBABE;

// ========= HTML UI (igual al que tienes) =========

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
 <!-- … (HTML idéntico al que ya tienes) … -->
</head>
<body>
 <!-- … (contenido HTML y JS de tu panel) … -->
</body>
</html>
)rawliteral";

// ========= Utils =========

void logf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
}

void logHex(const char* prefix, const uint8_t* data, int len, int maxBytes = 16) {
  Serial.print(prefix);
  int n = (len < maxBytes) ? len : maxBytes;
  for (int i = 0; i < n; i++) {
    Serial.print(' ');
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
  }
  Serial.println();
}

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
  esp_err_t e = esp_now_add_peer(&p);
  if (e != ESP_OK) {
    logf("[ESPNOW] esp_now_add_peer error=%d", (int)e);
  }
}

int devIndex(uint8_t dev){
  if (dev < DEV_MIN || dev > DEV_MAX) return -1;
  return dev - DEV_MIN; // 0..3
}

const char* devName(uint8_t dev){
  static const char* names[] = {"D1", "D2", "D3", "D4"};
  int idx = devIndex(dev);
  return (idx >= 0) ? names[idx] : "UNK";
}

uint8_t prefetchForDev(uint8_t dev){
  int idx = devIndex(dev);
  if (idx < 0) return 0;
  return prefetchCfg[idx];
}

String formatMac(const uint8_t mac[6]){
  char buf[32];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
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

// ========= Pendiente de WRITE desde el RP =========

struct PendingWriteFromRP {
  bool    active;
  uint8_t dev;
  uint16_t sec;
};

PendingWriteFromRP g_pendingWriteRP = { false, 0, 0 };

// ========= NVS: cargar/guardar =========

void saveTimingConfigToNvs(){
  if (!prefs.begin("xf551cfg", false)) {
    logf("[NVS] Error al abrir NVS para tiempos");
    return;
  }
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putUShort("t_ack",  T_ACK_TO_COMPLETE);
  prefs.putUShort("t_comp", T_COMPLETE_TO_DATA);
  prefs.putUShort("t_chk",  T_DATA_TO_CHK);
  prefs.putUShort("t_chd",  T_CHUNK_DELAY);
  prefs.end();
  logf("[NVS] Tiempos guardados");
}

void savePrefetchConfigToNvs(){
  if (!prefs.begin("xf551cfg", false)) {
    logf("[NVS] Error al abrir NVS para prefetch");
    return;
  }
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putUChar("pf1", prefetchCfg[0]);
  prefs.putUChar("pf2", prefetchCfg[1]);
  prefs.putUChar("pf3", prefetchCfg[2]);
  prefs.putUChar("pf4", prefetchCfg[3]);
  prefs.end();
  logf("[NVS] Prefetch guardado");
}

void loadConfigFromNvs(){
  if (!prefs.begin("xf551cfg", false)) {
    logf("[NVS] No se pudo abrir NVS, uso defaults");
    return;
  }

  uint32_t magic = prefs.getUInt("magic", 0);
  if (magic != CFG_MAGIC) {
    prefs.putUInt("magic", CFG_MAGIC);
    prefs.putUShort("t_ack",  T_ACK_TO_COMPLETE);
    prefs.putUShort("t_comp", T_COMPLETE_TO_DATA);
    prefs.putUShort("t_chk",  T_DATA_TO_CHK);
    prefs.putUShort("t_chd",  T_CHUNK_DELAY);
    prefs.putUChar("pf1", prefetchCfg[0]);
    prefs.putUChar("pf2", prefetchCfg[1]);
    prefs.putUChar("pf3", prefetchCfg[2]);
    prefs.putUChar("pf4", prefetchCfg[3]);
    prefs.end();
    logf("[NVS] Config inicial grabada (defaults)");
    return;
  }

  T_ACK_TO_COMPLETE   = prefs.getUShort("t_ack",  T_ACK_TO_COMPLETE);
  T_COMPLETE_TO_DATA  = prefs.getUShort("t_comp", T_COMPLETE_TO_DATA);
  T_DATA_TO_CHK       = prefs.getUShort("t_chk",  T_DATA_TO_CHK);
  T_CHUNK_DELAY       = prefs.getUShort("t_chd",  T_CHUNK_DELAY);

  prefetchCfg[0] = prefs.getUChar("pf1", prefetchCfg[0]);
  prefetchCfg[1] = prefs.getUChar("pf2", prefetchCfg[1]);
  prefetchCfg[2] = prefs.getUChar("pf3", prefetchCfg[2]);
  prefetchCfg[3] = prefs.getUChar("pf4", prefetchCfg[3]);

  prefs.end();

  logf("[NVS] Config cargada:");
  logf(" T_ACK_TO_COMPLETE=%u",  T_ACK_TO_COMPLETE);
  logf(" T_COMPLETE_TO_DATA=%u", T_COMPLETE_TO_DATA);
  logf(" T_DATA_TO_CHK=%u",      T_DATA_TO_CHK);
  logf(" T_CHUNK_DELAY=%u",      T_CHUNK_DELAY);
  logf(" Prefetch D1..D4=%u,%u,%u,%u",
       prefetchCfg[0], prefetchCfg[1], prefetchCfg[2], prefetchCfg[3]);
}

// ========= Web Handlers (idénticos a los que ya usas) =========

void handleRoot(){
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleApiStatus(){
  // … igual que tu versión actual …
}

void handleSetTiming(){
  // … igual que tu versión actual …
}

void handleSetPrefetch(){
  // … igual que tu versión actual …
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

          switch (type) {
            case TYPE_CMD_FRAME: {
              // Ajuste explícito de FORMAT / WRITE
              if (uartLen >= 7) {
                uint8_t cmd  = uartBuf[1];
                uint8_t dev  = uartBuf[2];
                uint8_t aux1 = uartBuf[3];
                uint8_t aux2 = uartBuf[4];
                uint8_t dens = uartBuf[5];
                uint8_t pf   = uartBuf[6];

                // Prefetch override
                uint8_t pfCfg = prefetchForDev(dev);
                if (pfCfg > MAX_PREFETCH_SECTORS) pfCfg = MAX_PREFETCH_SECTORS;
                if (pfCfg > 0) {
                  uartBuf[6] = pfCfg;
                  logf("[MASTER] PREFETCH %s = %u sectores",
                       devName(dev), pfCfg);
                }

                bool isWriteSector = (cmd == 0x50 || cmd == 0x57);
                bool isWritePercom = (cmd == 0x4F);
                bool isFormatSD    = (cmd == 0x21);
                bool isFormatED    = (cmd == 0x22);

                if (isWriteSector || isWritePercom) {
                  uint16_t sec;
                  if (isWritePercom) {
                    sec = 0xFFFF;
                  } else {
                    sec = (uint16_t)aux1 | ((uint16_t)aux2 << 8);
                  }

                  g_pendingWriteRP.active = true;
                  g_pendingWriteRP.dev    = dev;
                  g_pendingWriteRP.sec    = sec;
                  logf("[MASTER] WRITE pendiente dev=%s sec=%u cmd=0x%02X",
                       devName(dev), (unsigned)sec, (unsigned)cmd);
                }

                if (isFormatSD || isFormatED) {
                  logf("[MASTER] FORMAT %s para %s aux1=0x%02X aux2=0x%02X dens=%u",
                       isFormatED ? "ED(0x22)" : "SD(0x21)",
                       devName(dev), aux1, aux2, dens);
                }
              }

              // Comando desde RP -> reenviar al SLAVE
              sendEspToSlave(uartBuf, uartLen);
              Serial.print(F("[MASTER] Enviado payload a SLAVE tipo=0x"));
              if (uartBuf[0] < 0x10) Serial.print('0');
              Serial.print(uartBuf[0], HEX);
              Serial.print(F(" len="));
              Serial.println(uartLen);
            } break;

            case TYPE_SECTOR_CHUNK: {
              if (uartLen < 6) {
                Serial.println(F("[MASTER] SECTOR_CHUNK desde RP demasiado corto"));
                break;
              }

              uint8_t dev    = uartBuf[1];
              uint16_t sec   = (uint16_t)uartBuf[2] | ((uint16_t)uartBuf[3] << 8);
              uint8_t idx    = uartBuf[4];
              uint8_t count  = uartBuf[5];
              int     dlen   = uartLen - 6;

              Serial.print(F("[MASTER] SECTOR_CHUNK desde RP dev=0x"));
              if (dev < 0x10) Serial.print('0');
              Serial.print(dev, HEX);
              Serial.print(F(" sec="));
              Serial.print(sec);
              Serial.print(F(" idx="));
              Serial.print(idx);
              Serial.print(F("/"));
              Serial.print(count);
              Serial.print(F(" dataLen="));
              Serial.println(dlen);

              bool isPercom = (sec == 0xFFFF);

              if (isPercom) {
                sendEspToSlave(uartBuf, uartLen);
                Serial.println(F("[MASTER] SECTOR_CHUNK PERCOM reenviado al SLAVE (WRITE PERCOM)"));
                g_pendingWriteRP.active = false;
                break;
              }

              if (!g_pendingWriteRP.active) {
                Serial.println(F("[MASTER] SECTOR_CHUNK desde RP sin WRITE pendiente, se ignora"));
                break;
              }
              if (dev != g_pendingWriteRP.dev || sec != g_pendingWriteRP.sec) {
                Serial.println(F("[MASTER] SECTOR_CHUNK dev/sec no coincide con WRITE pendiente, se ignora"));
                break;
              }

              sendEspToSlave(uartBuf, uartLen);
              Serial.println(F("[MASTER] SECTOR_CHUNK reenviado al SLAVE (WRITE DATA)"));

              g_pendingWriteRP.active = false;
            } break;

            default:
              Serial.println(F("[MASTER] Tipo desconocido desde RP (no reenviado)."));
              break;
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

    int idx = devIndex(dev);
    if (idx >= 0 && src) {
      slaves[idx].present     = true;
      slaves[idx].supports256 = (dd != 0);
      memcpy(slaves[idx].mac, src, 6);
      slaves[idx].lastSeen = millis();
      ensurePeer(slaves[idx].mac);
      logf("[HELLO] %s presente DD=%u MAC=%s",
           devName(dev), dd ? 1 : 0, formatMac(slaves[idx].mac).c_str());
    }

    sendUartFrameToRP(data, len);
  }
  else if (type == TYPE_SECTOR_CHUNK || type == TYPE_ACK || type == TYPE_NAK) {
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

// ===== SETUP / LOOP =====

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(F("\n=== ESP32 MASTER BRIDGE (RP2040 <-> SLAVE XF551) + WEB ==="));

  Serial2.begin(115200, SERIAL_8N1, PIN_RP_RX, PIN_RP_TX);

  for (int i = 0; i < 4; i++){
    slaves[i].present     = false;
    slaves[i].supports256 = false;
    memset(slaves[i].mac, 0, 6);
    slaves[i].lastSeen = 0;
  }

  loadConfigFromNvs();

  WiFi.mode(WIFI_AP_STA);
  const char* apSsid = "XF551_MASTER";
  const char* apPass = "xf551wifi";
  WiFi.softAP(apSsid, apPass);

  Serial.print("[WIFI] AP SSID: ");
  Serial.println(apSsid);
  Serial.print("[WIFI] AP IP: ");
  Serial.println(WiFi.softAPIP());

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[MASTER] ERROR: esp_now_init fallo"));
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  ensurePeer(BCAST_MAC);

  server.on("/",           handleRoot);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/set_timing", HTTP_GET, handleSetTiming);
  server.on("/set_prefetch", HTTP_GET, handleSetPrefetch);
  server.begin();
  Serial.println("[WEB] Servidor HTTP iniciado en puerto 80");
}

void loop() {
  server.handleClient();

  unsigned long now = millis();
  for (int i = 0; i < 4; i++){
    if (slaves[i].present && (now - slaves[i].lastSeen > 60000)){
      slaves[i].present = false;
    }
  }

  pollUartFromRP();

  delay(2);
}
