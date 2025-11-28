/*******************************************************
 * MASTER – STATUS + READ (D1..D4) + WEB UI + PREFETCH
 * - Emula D1..D4 (0x31..0x34) frente al Atari
 * - Soporta STATUS (0x53) y READ SECTOR (0x52)
 * - Usa ESP-NOW para delegar en SLAVES con XF551 real
 * - Protocolo:
 *    • Cada SLAVE manda HELLO (TYPE_HELLO) con dev y soporte DD
 *    • MASTER manda TYPE_CMD_FRAME a D1..D4:
 *        [0]=TYPE_CMD_FRAME
 *        [1]=cmd
 *        [2]=dev
 *        [3]=secL
 *        [4]=secH
 *        [5]=dens (!=0=DD)
 *        [6]=prefetchCount (#sectores adelantados)
 *    • SLAVE responde con TYPE_SECTOR_CHUNK
 * - Web UI (HTTP en puerto 80, AP "XF551_MASTER" / "xf551wifi"):
 *    • Ver estado de disqueteras (SLAVES)
 *    • Ajustar tiempos SIO
 *    • Ajustar prefetch por unidad (0..4 sectores)
 *******************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HardwareSerial.h>
#include <WebServer.h>
#include <stdarg.h>

// ========= Config SIO MASTER (lado Atari) =========
#define SIO_DATA_IN   16
#define SIO_DATA_OUT  17
#define SIO_COMMAND   18

HardwareSerial SerialSIO(2);

// Códigos SIO
#define SIO_ACK      0x41
#define SIO_NAK      0x4E
#define SIO_COMPLETE 0x43
#define SIO_ERROR    0x45

// D1..D4
#define DEV_MIN      0x31
#define DEV_MAX      0x34

// Tamaños
#define SECTOR_128        128
#define SECTOR_256        256
#define MAX_SECTOR_BYTES  256

// ========= Protocolo ESP-NOW =========
#define TYPE_HELLO        0x20
#define TYPE_CMD_FRAME    0x01
#define TYPE_SECTOR_CHUNK 0x10
#define TYPE_ACK          0x11
#define TYPE_NAK          0x12

#define CHUNK_PAYLOAD     240

// Prefetch máximo que vamos a permitir configurar
#define MAX_PREFETCH_SECTORS 4

// ========= Timings SIO (µs) – AJUSTABLES POR WEB =========
uint16_t T_ACK_TO_COMPLETE   = 2000;
uint16_t T_COMPLETE_TO_DATA  = 1600;
uint16_t T_DATA_TO_CHK       = 400;
uint16_t T_CHUNK_DELAY       = 1600;

// ========= Estado de SLAVES (uno por unidad) =========
struct SlaveInfo {
  bool     present;
  bool     supports256;
  uint8_t  mac[6];
  unsigned long lastSeen;
};

SlaveInfo slaves[4]; // D1..D4

// Prefetch configurado por unidad (0..MAX_PREFETCH_SECTORS)
uint8_t prefetchCfg[4] = {1, 1, 1, 1};

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

// ========= Estado de operación remota =========
enum OpType : uint8_t {
  OP_NONE   = 0,
  OP_READ   = 1,
  OP_STATUS = 2
};

volatile OpType  currentOpType = OP_NONE;
volatile uint8_t currentOpDev  = 0;
volatile uint16_t currentOpSec = 0;

// ========= Buffers de respuesta (desde SLAVES) =========
volatile bool     replyReady       = false;
volatile uint16_t replySec         = 0;
volatile uint16_t replyLen         = 0;
volatile uint8_t  expectedChunks   = 0;
volatile uint8_t  receivedChunks   = 0;
uint8_t           replyBuf[MAX_SECTOR_BYTES];

// ========= Otros =========
const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ========= WebServer =========
WebServer server(80);

// ========= Utilidades varias =========

uint8_t sioChecksum(const uint8_t* d, int len){
  uint16_t s = 0;
  for (int i = 0; i < len; i++){
    s += d[i];
    if (s > 0xFF) s = (s & 0xFF) + 1;
  }
  return s & 0xFF;
}

void logf(const char* fmt, ...){
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);  // <-- acá va fmt, no ap
  va_end(ap);
  Serial.println(buf);
}


void logHex(const char* prefix, const uint8_t* data, int len, int maxBytes = 16){
  Serial.print(prefix);
  int n = (len < maxBytes) ? len : maxBytes;
  for (int i = 0; i < n; i++){
    Serial.print(' ');
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
  }
  Serial.println();
}

void ensurePeer(const uint8_t* mac){
  if (!mac) return;
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  esp_err_t e = esp_now_add_peer(&p);
  if (e != ESP_OK){
    logf("[ESPNOW] esp_now_add_peer error=%d", (int)e);
  }
}

const uint8_t* macForDev(uint8_t dev){
  int idx = devIndex(dev);
  if (idx < 0) return BCAST_MAC;
  if (!slaves[idx].present) return BCAST_MAC;
  return slaves[idx].mac;
}

String formatMac(const uint8_t mac[6]){
  char buf[32];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// ========= Web UI Handlers =========

void handleRoot(){
  String html;
  html.reserve(6000);

  html += F(
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<title>XF551 WIFI MASTER</title>"
    "<style>"
    "body{font-family:Arial,Helvetica,sans-serif;background:#202124;color:#e8eaed;margin:0;padding:0;}"
    "h1,h2{margin:16px;}"
    "section{margin:16px;padding:16px;background:#303134;border-radius:8px;}"
    "table{border-collapse:collapse;width:100%;}"
    "th,td{border:1px solid #555;padding:6px 8px;font-size:13px;text-align:left;}"
    "th{background:#3c4043;}"
    "input[type='number']{width:90px;}"
    "input[type='submit']{margin-top:8px;padding:6px 12px;border:none;border-radius:4px;background:#1a73e8;color:#fff;cursor:pointer;}"
    "input[type='submit']:hover{background:#1664c4;}"
    "label{margin-right:8px;display:inline-block;margin-top:4px;}"
    "</style></head><body>"
  );

  html += F("<h1>XF551 WIFI MASTER - Panel</h1>");

  // ====== SECCIÓN ESTADO DE DISQUETERAS ======
  html += F("<section><h2>Estado de disqueteras (SLAVES)</h2>");
  html += F("<table><tr><th>Unidad</th><th>Presente</th><th>MAC</th><th>DD</th><th>Último HELLO (ms)</th><th>Prefetch</th></tr>");

  unsigned long now = millis();
  for (uint8_t dev = DEV_MIN; dev <= DEV_MAX; dev++){
    int idx = devIndex(dev);
    if (idx < 0) continue;

    html += F("<tr><td>");
    html += devName(dev);
    html += F("</td>");

    bool present = slaves[idx].present;
    html += F("<td>");
    html += present ? F("SI") : F("NO");
    html += F("</td>");

    html += F("<td>");
    if (present){
      html += formatMac(slaves[idx].mac);
    } else {
      html += F("-");
    }
    html += F("</td>");

    html += F("<td>");
    html += (slaves[idx].supports256 ? F("DD") : F("SD"));
    html += F("</td>");

    html += F("<td>");
    if (present){
      unsigned long age = now - slaves[idx].lastSeen;
      html += String(age);
    } else {
      html += F("-");
    }
    html += F("</td>");

    html += F("<td>");
    html += String(prefetchCfg[idx]);
    html += F("</td></tr>");
  }
  html += F("</table>");
  html += F("<p>Refresca la página para actualizar el estado.</p>");
  html += F("</section>");

  // ====== SECCIÓN CONFIG TIMINGS ======
  html += F("<section><h2>Configuración de tiempos SIO (μs)</h2>");
  html += F("<form method='GET' action='/set_timing'>");

  html += F("<label>ACK → COMPLETE: <input type='number' name='t_ack' min='0' max='10000' value='");
  html += String(T_ACK_TO_COMPLETE);
  html += F("'></label><br>");

  html += F("<label>COMPLETE → DATA: <input type='number' name='t_comp' min='0' max='10000' value='");
  html += String(T_COMPLETE_TO_DATA);
  html += F("'></label><br>");

  html += F("<label>DATA → CHK: <input type='number' name='t_chk' min='0' max='5000' value='");
  html += String(T_DATA_TO_CHK);
  html += F("'></label><br>");

  html += F("<label>CHUNK DELAY: <input type='number' name='t_chunk' min='0' max='10000' value='");
  html += String(T_CHUNK_DELAY);
  html += F("'></label><br>");

  html += F("<input type='submit' value='Guardar tiempos'>");
  html += F("</form>");
  html += F("</section>");

  // ====== SECCIÓN PREFETCH POR UNIDAD ======
  html += F("<section><h2>Prefetch por unidad (sectores adelantados)</h2>");
  html += F("<form method='GET' action='/set_prefetch'>");

  for (uint8_t dev = DEV_MIN; dev <= DEV_MAX; dev++){
    int idx = devIndex(dev);
    if (idx < 0) continue;
    html += F("<label>");
    html += devName(dev);
    html += F(" prefetch: <input type='number' name='pf");
    html += String(idx + 1);
    html += F("' min='0' max='");
    html += String(MAX_PREFETCH_SECTORS);
    html += F("' value='");
    html += String(prefetchCfg[idx]);
    html += F("'></label><br>");
  }

  html += F("<input type='submit' value='Guardar prefetch'>");
  html += F("</form>");
  html += F("<p><small>0 = sin prefetch, 1..");
  html += String(MAX_PREFETCH_SECTORS);
  html += F(" = cantidad de sectores que el SLAVE leerá por adelantado.</small></p>");
  html += F("</section>");

  html += F("</body></html>");

  server.send(200, "text/html", html);
}

void handleSetTiming(){
  if (server.hasArg("t_ack")){
    T_ACK_TO_COMPLETE = (uint16_t)server.arg("t_ack").toInt();
  }
  if (server.hasArg("t_comp")){
    T_COMPLETE_TO_DATA = (uint16_t)server.arg("t_comp").toInt();
  }
  if (server.hasArg("t_chk")){
    T_DATA_TO_CHK = (uint16_t)server.arg("t_chk").toInt();
  }
  if (server.hasArg("t_chunk")){
    T_CHUNK_DELAY = (uint16_t)server.arg("t_chunk").toInt();
  }

  logf("[WEB] Tiempos: ACK->COMP=%u, COMP->DATA=%u, DATA->CHK=%u, CHUNK=%u",
       T_ACK_TO_COMPLETE, T_COMPLETE_TO_DATA, T_DATA_TO_CHK, T_CHUNK_DELAY);

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "OK");
}

void handleSetPrefetch(){
  for (int idx = 0; idx < 4; idx++){
    String argName = "pf" + String(idx + 1);
    if (server.hasArg(argName)){
      int v = server.arg(argName).toInt();
      if (v < 0) v = 0;
      if (v > MAX_PREFETCH_SECTORS) v = MAX_PREFETCH_SECTORS;
      prefetchCfg[idx] = (uint8_t)v;
    }
  }

  logf("[WEB] Prefetch: D1=%u, D2=%u, D3=%u, D4=%u",
       prefetchCfg[0], prefetchCfg[1], prefetchCfg[2], prefetchCfg[3]);

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "OK");
}

// ========= Lógica común de recepción/envío ESP-NOW =========

static void handleEspNowRecvCommon(const uint8_t* mac, const uint8_t* data, int len){
  if (!data || len <= 0) return;
  uint8_t type = data[0];

  // HELLO: [0]=TYPE_HELLO,[1]=dev,[2]=supports256
  if (type == TYPE_HELLO && len >= 3){
    uint8_t dev = data[1];
    bool s256   = (data[2] != 0);

    int idx = devIndex(dev);
    if (idx >= 0 && mac){
      slaves[idx].present     = true;
      slaves[idx].supports256 = s256;
      memcpy(slaves[idx].mac, mac, 6);
      slaves[idx].lastSeen    = millis();
      ensurePeer(slaves[idx].mac);
      logf("[HELLO] %s presente DD=%u MAC=%s",
           devName(dev), s256, formatMac(slaves[idx].mac).c_str());
    }
    return;
  }

  // SECTOR_CHUNK: [0]=TYPE_SECTOR_CHUNK,[1]=dev,[2]=secL,[3]=secH,[4]=ci,[5]=cc,[6..]=data
  if (type == TYPE_SECTOR_CHUNK && len >= 6){
    uint8_t dev  = data[1];
    uint16_t sec = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    uint8_t ci   = data[4];
    uint8_t cc   = data[5];
    int payload  = len - 6;

    if (dev != currentOpDev || currentOpType == OP_NONE) return;

    if (ci == 0){
      memset(replyBuf, 0, sizeof(replyBuf));
      replySec       = sec;
      expectedChunks = cc;
      receivedChunks = 0;
      replyLen       = 0;
      replyReady     = false;
    }

    int off = ci * CHUNK_PAYLOAD;
    int copyLen = payload;
    if (off + copyLen > MAX_SECTOR_BYTES)
      copyLen = MAX_SECTOR_BYTES - off;

    if (copyLen > 0){
      memcpy(replyBuf + off, data + 6, copyLen);
      replyLen += copyLen;
    }

    receivedChunks++;
    logf("[NET] TYPE_SECTOR_CHUNK dev=%s sec=%u ci=%u/%u payload=%d",
         devName(dev), sec, ci, cc, payload);

    if (receivedChunks >= expectedChunks){
      replyReady = true;
      logf("[NET] Respuesta completa dev=%s sec=%u len=%u",
           devName(dev), sec, replyLen);
    }
    return;
  }

  if (type == TYPE_ACK){
    logf("[ESPNOW] ACK recibido");
  } else if (type == TYPE_NAK){
    logf("[ESPNOW] NAK recibido");
  }
}

static void handleEspNowSentCommon(esp_now_send_status_t status){
  if (status == ESP_NOW_SEND_SUCCESS)
    Serial.println("[ESPNOW] Envío OK");
  else
    Serial.println("[ESPNOW] Falla en envío");
}

// ========= Callbacks ESP-NOW dependientes de versión =========

#if ESP_IDF_VERSION_MAJOR >= 5

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  const uint8_t *mac = info ? info->src_addr : nullptr;
  handleEspNowRecvCommon(mac, data, len);
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status){
  (void)info;
  handleEspNowSentCommon(status);
}

#else  // IDF < 5

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){
  handleEspNowRecvCommon(mac, data, len);
}

void onDataSent(const uint8_t* mac, esp_now_send_status_t status){
  (void)mac;
  handleEspNowSentCommon(status);
}

#endif

// ========= Envío de comandos al SLAVE =========

// [0]=TYPE_CMD_FRAME,[1]=cmd,[2]=dev,[3]=secL,[4]=secH,[5]=dens,[6]=prefetchCount
bool sendCmd(uint8_t dev, uint8_t cmd, uint16_t sec, bool dd){
  uint8_t p[7];
  p[0] = TYPE_CMD_FRAME;
  p[1] = cmd;
  p[2] = dev;
  p[3] = (uint8_t)(sec & 0xFF);
  p[4] = (uint8_t)(sec >> 8);
  p[5] = dd ? 1 : 0;
  uint8_t pf = prefetchForDev(dev);
  if (pf > MAX_PREFETCH_SECTORS) pf = MAX_PREFETCH_SECTORS;
  p[6] = pf;

  const uint8_t* dest = macForDev(dev);

  logf("[ESPNOW][TX] CMD_FRAME dev=%s cmd=0x%02X sec=%u dd=%s pf=%u",
       devName(dev), cmd, sec, dd ? "DD" : "SD", pf);
  logHex("[ESPNOW][TX] bytes:", p, sizeof(p));

  esp_err_t e = esp_now_send(dest, p, sizeof(p));
  if (e != ESP_OK){
    logf("[ESPNOW] esp_now_send error=%d", (int)e);
  }
  return (e == ESP_OK);
}

// Lee sector remoto (128/256) para dev dado
int readRemoteSector(uint8_t dev, uint16_t sec, bool dd, uint8_t* out){
  int idx = devIndex(dev);
  if (idx < 0 || !slaves[idx].present){
    logf("[READ] %s no presente", devName(dev));
    return 0;
  }

  replyReady = false;
  replySec   = 0;
  replyLen   = 0;
  expectedChunks = receivedChunks = 0;

  currentOpType = OP_READ;
  currentOpDev  = dev;
  currentOpSec  = sec;

  logf("[READ] Solicitando sec=%u dd=%s a %s",
       sec, dd ? "DD" : "SD", devName(dev));

  if (!sendCmd(dev, 0x52, sec, dd)){
    logf("[READ] sendCmd falló para %s", devName(dev));
    currentOpType = OP_NONE;
    return 0;
  }

  unsigned long t0 = millis();
  int expectedLen = dd ? SECTOR_256 : SECTOR_128;

  while (millis() - t0 < 8000){
    if (replyReady && replySec == sec) break;
    delay(2);
  }

  currentOpType = OP_NONE;

  if (!(replyReady && replySec == sec)){
    logf("[READ] Timeout esperando sec=%u de %s", sec, devName(dev));
    return 0;
  }

  int n = (replyLen > expectedLen) ? expectedLen : replyLen;
  memcpy(out, replyBuf, n);

  logf("[READ] OK %s sec=%u len=%d", devName(dev), sec, n);
  logHex("[READ] Primeros bytes:", out, n);

  return n;
}

// Lee STATUS remoto (4 bytes) para dev dado
int readRemoteStatus(uint8_t dev, bool dd, uint8_t* out4){
  (void)dd; // no usamos densidad para STATUS de momento

  int idx = devIndex(dev);
  if (idx < 0 || !slaves[idx].present){
    logf("[STATUS] %s no presente", devName(dev));
    return 0;
  }

  replyReady = false;
  replySec   = 0;
  replyLen   = 0;
  expectedChunks = receivedChunks = 0;

  currentOpType = OP_STATUS;
  currentOpDev  = dev;
  currentOpSec  = 0;

  logf("[STATUS] Solicitando STATUS a %s", devName(dev));

  // Para STATUS ignoramos prefetch, mandamos pf=0 internamente en el SLAVE
  if (!sendCmd(dev, 0x53, 0, false)){
    logf("[STATUS] sendCmd falló para %s", devName(dev));
    currentOpType = OP_NONE;
    return 0;
  }

  unsigned long t0 = millis();
  while (millis() - t0 < 5000){
    if (replyReady) break;
    delay(2);
  }

  currentOpType = OP_NONE;

  if (!replyReady){
    logf("[STATUS] Timeout esperando STATUS de %s", devName(dev));
    return 0;
  }

  if (replyLen < 4){
    logf("[STATUS] Respuesta corta (%u bytes) de %s", replyLen, devName(dev));
    return 0;
  }

  memcpy(out4, replyBuf, 4);
  logHex("[STATUS] bytes:", out4, 4);
  return 4;
}

// ========= SIO: envío de DATA+CHK al Atari =========

void sendAtariData(const uint8_t* buf, int len){
  // COMPLETE
  delayMicroseconds(T_ACK_TO_COMPLETE);
  SerialSIO.write(SIO_COMPLETE);
  SerialSIO.flush();
  logf("[TX->Atari] COMPLETE");

  // DATA
  delayMicroseconds(T_COMPLETE_TO_DATA);
  SerialSIO.write(buf, len);
  SerialSIO.flush();
  logf("[TX->Atari] DATA len=%d", len);
  logHex("[TX->Atari] DATA bytes:", buf, len);

  // CHK
  delayMicroseconds(T_DATA_TO_CHK);
  uint8_t chk = sioChecksum(buf, len);
  SerialSIO.write(chk);
  SerialSIO.flush();
  logf("[TX->Atari] CHK=%02X", chk);

  delayMicroseconds(T_CHUNK_DELAY);
}

// ========= Procesar cabecera SIO del Atari =========

void handleSioHeader(uint8_t f[5]){
  uint8_t dev     = f[0];
  uint8_t cmd     = f[1];
  uint8_t aux1    = f[2];
  uint8_t aux2    = f[3];
  uint8_t recvChk = f[4];

  uint8_t calcChk = sioChecksum(f, 4);

  logf("[HDR] %02X %02X %02X %02X %02X", f[0], f[1], f[2], f[3], f[4]);

  // 1) Validamos checksum
  if (recvChk != calcChk){
    delayMicroseconds(800);
    SerialSIO.write(SIO_NAK);
    SerialSIO.flush();
    logf("[SIO] Checksum inválido (recv=%02X calc=%02X)", recvChk, calcChk);
    return;
  }

  // 2) Filtramos dispositivos que no sean D1..D4
  if (dev < DEV_MIN || dev > DEV_MAX){
    return;
  }

  bool dd      = (cmd & 0x80) != 0;
  uint8_t base = cmd & 0x7F;
  uint16_t sec = (uint16_t)aux1 | ((uint16_t)aux2 << 8);

  // ===== READ SECTOR =====
  if (base == 0x52){
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    logf("[TX->Atari] ACK (READ) %s sec=%u", devName(dev), sec);
    delayMicroseconds(800);

    uint8_t buf[MAX_SECTOR_BYTES];
    int n = readRemoteSector(dev, sec, dd, buf);
    if (n <= 0){
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      logf("[SIO] READ remoto falló %s sec=%u", devName(dev), sec);
      return;
    }

    sendAtariData(buf, n);
    return;
  }

  // ===== STATUS =====
  if (base == 0x53){
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    logf("[TX->Atari] ACK (STATUS) %s", devName(dev));
    delayMicroseconds(800);

    uint8_t st[4];
    int n = readRemoteStatus(dev, dd, st);
    if (n != 4){
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      logf("[SIO] STATUS remoto falló %s", devName(dev));
      return;
    }

    sendAtariData(st, 4);
    return;
  }

  // ===== Otros comandos =====
  delayMicroseconds(800);
  SerialSIO.write(SIO_NAK);
  SerialSIO.flush();
  logf("[SIO] Cmd no soportado 0x%02X para %s => NAK", base, devName(dev));
}

// ========= Máquina de estados para COMMAND =========

bool     inCommand      = false;
uint8_t  hdrBuf[5];
uint8_t  hdrCount       = 0;
uint32_t cmdStartMicros = 0;

// ========= SETUP / LOOP =========

void setup(){
  Serial.begin(115200);
  Serial.println("\n[MASTER] STATUS+READ + WEB UI + PREFETCH D1..D4");

  pinMode(SIO_COMMAND, INPUT_PULLUP);
  SerialSIO.begin(19200, SERIAL_8N1, SIO_DATA_IN, SIO_DATA_OUT, false);

  // Init tabla slaves
  for (int i = 0; i < 4; i++){
    slaves[i].present     = false;
    slaves[i].supports256 = false;
    memset(slaves[i].mac, 0, 6);
    slaves[i].lastSeen    = 0;
  }

  // WiFi AP+STA para Web UI + ESP-NOW
  WiFi.mode(WIFI_AP_STA);
  const char* apSsid = "XF551_MASTER";
  const char* apPass = "xf551wifi";
  WiFi.softAP(apSsid, apPass);

  Serial.print("[WIFI] AP SSID: ");
  Serial.println(apSsid);
  Serial.print("[WIFI] AP IP:   ");
  Serial.println(WiFi.softAPIP());

  // ESP-NOW
  if (esp_now_init() != ESP_OK){
    Serial.println("[ERR] esp_now_init falló");
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  ensurePeer(BCAST_MAC);

  // WebServer rutas
  server.on("/", handleRoot);
  server.on("/set_timing", handleSetTiming);
  server.on("/set_prefetch", handleSetPrefetch);
  server.begin();
  Serial.println("[WEB] Servidor HTTP iniciado en puerto 80");
}

void loop(){
  // Atender Web UI
  server.handleClient();

  // Limpiar esclavos inactivos (>60s sin HELLO)
  unsigned long now = millis();
  for (int i = 0; i < 4; i++){
    if (slaves[i].present && (now - slaves[i].lastSeen > 60000)){
      slaves[i].present = false;
    }
  }

  int cmdLevel = digitalRead(SIO_COMMAND);

  // Flanco de bajada -> inicio de posible comando
  if (!inCommand && cmdLevel == LOW){
    inCommand      = true;
    hdrCount       = 0;
    cmdStartMicros = micros();
  }

  if (inCommand){
    // Acumular bytes de cabecera
    while (SerialSIO.available() && hdrCount < 5){
      hdrBuf[hdrCount++] = SerialSIO.read();
    }

    // Si ya tenemos 5 bytes, procesamos
    if (hdrCount == 5){
      handleSioHeader(hdrBuf);
      inCommand = false;
      while (SerialSIO.available()) SerialSIO.read();
    }

    // Si COMMAND volvió a HIGH y no alcanzamos cabecera completa, descartamos
    if (cmdLevel == HIGH && hdrCount < 5 && (micros() - cmdStartMicros) > 3000){
      inCommand = false;
      while (SerialSIO.available()) SerialSIO.read();
    }
  }

  delay(1);
}
