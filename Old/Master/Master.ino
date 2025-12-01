/*******************************************************
 * MASTER – ATARI SIO BRIDGE + ESPNOW + WebUI Prefetch
 *
 * - Va al lado del ATARI (puerto SIO)
 * - Emula hasta 4 unidades: D1..D4 (0x31..0x34)
 * - Comandos soportados desde Atari:
 *     • READ SECTOR  (0x52 / 0xD2)
 *     • STATUS       (0x53 / 0xD3)
 *     • WRITE SECTOR (0x57 / 0xD7)
 *     • FORMAT       (0x21 / 0xA1)
 * - READ/STATUS/WRITE/FORMAT se delegan a SLAVES XF551
 *   vía ESP-NOW (TYPE_CMD_FRAME + chunks).
 *
 * WEB UI (http://192.168.4.1):
 *   - Lista D1..D4 (online/offline + DD)
 *   - Prefetch ON/OFF por unidad
 *   - RESET por unidad (TYPE_RESET)
 *   - REBOOT del MASTER
 *
 * Firmas ESPNOW: compatibles con IDF v5.x
 *
 * Eduardo Quintana – 2025
 *******************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <stdarg.h>

// ==================== Debug ====================
#define DEBUG 1

void logf(const char* fmt, ...) {
#if DEBUG
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
#endif
}

// ==================== SIO (Atari) ====================
#define SIO_DATA_IN   18   // Atari → ESP32
#define SIO_DATA_OUT  19   // ESP32 → Atari
#define SIO_COMMAND   21   // Línea COMMAND (entrada)
#define LED_STATUS     2

HardwareSerial SerialSIO(2);

// Códigos SIO
#define SIO_ACK      0x41
#define SIO_NAK      0x4E
#define SIO_COMPLETE 0x43
#define SIO_ERROR    0x45

// Tamaños típicos
#define SECTOR_128   128
#define SECTOR_256   256
#define MAX_SECTOR_BYTES 256

// ==================== ESP-NOW protocolo ====================
#define TYPE_HELLO        0x20
#define TYPE_CMD_FRAME    0x01
#define TYPE_RESET        0x02
#define TYPE_SECTOR_CHUNK 0x10
#define TYPE_ACK          0x11
#define TYPE_NAK          0x12
#define TYPE_CFG_PREFETCH 0x30
#define TYPE_WRITE_CHUNK  0x21
#define TYPE_FORMAT_BAD   0x22

#define CHUNK_PAYLOAD     240

// ==================== Estado de SLAVES ====================
struct SlaveInfo {
  bool present;
  bool supports256;
  uint8_t mac[6];
  unsigned long lastSeen;
};

SlaveInfo slaves[4]; // D1..D4

int devIndex(uint8_t dev) {
  if (dev < 0x31 || dev > 0x34) return -1;
  return dev - 0x31;   // 0..3
}

const char* devName(uint8_t dev){
  switch (dev) {
    case 0x31: return "D1";
    case 0x32: return "D2";
    case 0x33: return "D3";
    case 0x34: return "D4";
    default:   return "UNK";
  }
}

bool isSlavePresent(uint8_t dev) {
  int idx = devIndex(dev);
  if (idx < 0) return false;
  return slaves[idx].present;
}

// Prefetch habilitado por unidad (D1..D4)
bool g_prefetchEnabled[4] = { true, true, true, true };

// ==================== WiFi / Web ====================
WebServer web(80);
String AP_SSID = "ATARI_BRIDGE";
String AP_PSK  = "atari123";

const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ==================== Utilidades comunes ====================
uint8_t sioChecksum(const uint8_t* d, int len){
  uint16_t s = 0;
  for (int i=0; i<len; i++){
    s += d[i];
    if (s > 0xFF) s = (s & 0xFF) + 1;
  }
  return s & 0xFF;
}

void ensurePeer(const uint8_t* mac){
  if (!mac) return;
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  esp_now_add_peer(&p);
}

bool send_now_to(const uint8_t* mac, const uint8_t* data, int len){
  ensurePeer(mac);
  return (esp_now_send(mac, data, len) == ESP_OK);
}

// ==================== Buffers de respuesta remota (READ/STATUS) ====================
volatile bool     g_replyReady       = false;
volatile uint8_t  g_replyDev         = 0;
volatile uint16_t g_replySec         = 0;
volatile uint8_t  g_expectedChunks   = 0;
volatile uint8_t  g_receivedChunks   = 0;
volatile uint16_t g_replyLen         = 0;
uint8_t           g_replyBuf[MAX_SECTOR_BYTES];

// ==================== Resultado remoto de WRITE ====================
volatile bool    g_writeResultReady = false;
volatile bool    g_writeResultOk    = false;
volatile uint8_t g_writeDev         = 0;

// ==================== Resultado de FORMAT remoto ====================
volatile bool    g_formatReady      = false;
volatile uint8_t g_formatDev        = 0;
uint8_t          g_formatMap[128];

// ==================== ESPNOW helpers de configuración ====================

bool sendPrefetchConfig(uint8_t dev, bool enable) {
  uint8_t p[3] = { TYPE_CFG_PREFETCH, dev, (uint8_t)(enable ? 1 : 0) };
  return send_now_to(BCAST_MAC, p, sizeof(p));
}

bool sendResetTo(uint8_t dev) {
  uint8_t p[2] = { TYPE_RESET, dev };
  return send_now_to(BCAST_MAC, p, sizeof(p));
}

// ==================== Prototipos de callbacks / handlers ====================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incoming, int len);
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status);
void handleRoot();
void handlePrefetch();
void handleReset();
void handleReboot();

// ==================== ESPNOW callbacks ====================

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incoming, int len){
  if (!info || !incoming || len <= 0) return;
  const uint8_t *mac = info->src_addr;
  (void)mac;

  uint8_t type = incoming[0];

  // ---- HELLO de SLAVE ----
  if (type == TYPE_HELLO && len >= 3) {
    uint8_t dev = incoming[1];
    bool dd     = (incoming[2] != 0);

    int idx = devIndex(dev);
    if (idx >= 0) {
      slaves[idx].present     = true;
      slaves[idx].supports256 = dd;
      memcpy(slaves[idx].mac, mac, 6);
      slaves[idx].lastSeen = millis();
      ensurePeer(slaves[idx].mac);

      logf("[HELLO] %s presente. DD=%u MAC=%02X:%02X:%02X:%02X:%02X:%02X",
           devName(dev), dd,
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

      // Sincronizar modo Prefetch hacia este SLAVE
      bool en = g_prefetchEnabled[idx];
      sendPrefetchConfig(dev, en);
      logf("[CFG] Prefetch %s enviado a %s",
           en ? "ON" : "OFF", devName(dev));
    }
    return;
  }

  // ---- Mapa de FORMAT ----
  if (type == TYPE_FORMAT_BAD && len >= 6) {
    uint8_t dev = incoming[1];
    int payload = len - 6;
    if (payload > 128) payload = 128;

    memcpy(g_formatMap, incoming + 6, payload);
    if (payload < 128) {
      memset(g_formatMap + payload, 0xFF, 128 - payload);
    }

    g_formatDev   = dev;
    g_formatReady = true;
    logf("[FORMAT] BAD MAP recibido de %s (%d bytes)",
         devName(dev), payload);
    return;
  }

  // ---- Datos de sector (READ/STATUS) ----
  if (type == TYPE_SECTOR_CHUNK && len >= 6) {
    uint8_t dev   = incoming[1];
    uint16_t sec  = (uint16_t)incoming[2] | ((uint16_t)incoming[3] << 8);
    uint8_t ci    = incoming[4];
    uint8_t cc    = incoming[5];
    int payload    = len - 6;

    if (ci == 0) {
      memset(g_replyBuf, 0, sizeof(g_replyBuf));
      g_expectedChunks = cc;
      g_receivedChunks = 0;
      g_replyLen       = 0;
      g_replyDev       = dev;
      g_replySec       = sec;
    }

    int offset   = ci * CHUNK_PAYLOAD;
    int copyLen  = payload;
    if (offset + copyLen > MAX_SECTOR_BYTES)
      copyLen = MAX_SECTOR_BYTES - offset;
    if (copyLen > 0) {
      memcpy(g_replyBuf + offset, incoming + 6, copyLen);
      uint16_t newLen = offset + copyLen;
      if (newLen > g_replyLen) g_replyLen = newLen;
    }

    g_receivedChunks++;
    if (g_receivedChunks >= g_expectedChunks) {
      g_replyReady = true;
    }
    return;
  }

  // ---- ACK / NAK ----
  if ((type == TYPE_ACK || type == TYPE_NAK) && len >= 2) {
    uint8_t dev = incoming[1];
    logf("[%s] desde %s",
         (type == TYPE_ACK ? "ACK":"NAK"),
         devName(dev));

    // Marca resultado para WRITE
    g_writeResultReady = true;
    g_writeResultOk    = (type == TYPE_ACK);
    g_writeDev         = dev;
    return;
  }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status){
  (void)info;
#if DEBUG
  if (status == ESP_NOW_SEND_SUCCESS)
    Serial.println("[ESPNOW] Envío OK");
  else
    Serial.println("[ESPNOW] Falla en envío");
#endif
}

// ==================== Lecturas remotas (READ / STATUS) ====================

int remoteReadBlock(uint8_t dev, uint8_t cmdBase, uint16_t sec,
                    bool dd, int expectedLen, uint8_t* out) {
  if (!isSlavePresent(dev)) {
    logf("[REMOTE] %s no presente, salto ESPNOW", devName(dev));
    return 0;
  }

  g_replyReady       = false;
  g_expectedChunks   = 0;
  g_receivedChunks   = 0;
  g_replyLen         = 0;
  g_replyDev         = 0;
  g_replySec         = 0;

  uint8_t pkt[6];
  pkt[0] = TYPE_CMD_FRAME;
  pkt[1] = cmdBase | (dd ? 0x80 : 0);
  pkt[2] = dev;
  pkt[3] = (uint8_t)(sec & 0xFF);
  pkt[4] = (uint8_t)(sec >> 8);
  pkt[5] = dd ? 1 : 0;

  send_now_to(BCAST_MAC, pkt, sizeof(pkt));

  unsigned long t0 = millis();
  const unsigned long TIMEOUT = 6000;

  while (millis() - t0 < TIMEOUT) {
    if (g_replyReady && g_replyDev == dev) {
      if (cmdBase == 0x52) {
        if (g_replySec == sec) break;
      } else {
        break;
      }
    }
    delay(1);
  }

  if (!g_replyReady || g_replyLen == 0) {
    logf("[REMOTE] Timeout dev=%s cmd=0x%02X sec=%u",
         devName(dev), cmdBase, sec);
    return 0;
  }

  int n = min(expectedLen, (int)g_replyLen);
  memcpy(out, g_replyBuf, n);
  return n;
}

int remoteReadSector(uint8_t dev, uint16_t sec, bool dd, uint8_t* out) {
  int expected = dd ? SECTOR_256 : SECTOR_128;
  int n = remoteReadBlock(dev, 0x52, sec, dd, expected, out);
#if DEBUG
  if (n > 0) {
    logf("[REMOTE] %s READ sec=%u len=%d", devName(dev), sec, n);
  }
#endif
  return n;
}

int remoteReadStatus(uint8_t dev, uint8_t* out) {
  int n = remoteReadBlock(dev, 0x53, 0, false, 4, out);
#if DEBUG
  if (n > 0) {
    logf("[REMOTE] %s STATUS OK", devName(dev));
  }
#endif
  return n;
}

// ==================== WRITE remoto ====================

bool remoteWriteSector(uint8_t dev, uint16_t sec, bool dd,
                       const uint8_t* buf, int len) {
  if (!isSlavePresent(dev)) {
    logf("[REMOTE] WRITE %s no presente", devName(dev));
    return false;
  }

  g_writeResultReady = false;
  g_writeResultOk    = false;
  g_writeDev         = 0;

  // 1) Comando WRITE (informa sector y densidad)
  uint8_t cmdPkt[6];
  cmdPkt[0] = TYPE_CMD_FRAME;
  cmdPkt[1] = 0x57 | (dd ? 0x80 : 0);
  cmdPkt[2] = dev;
  cmdPkt[3] = (uint8_t)(sec & 0xFF);
  cmdPkt[4] = (uint8_t)(sec >> 8);
  cmdPkt[5] = dd ? 1 : 0;
  send_now_to(BCAST_MAC, cmdPkt, sizeof(cmdPkt));

  // 2) Enviar DATA en chunks
  int cc = (len + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD;
  for (int i=0; i<cc; i++){
    int off = i * CHUNK_PAYLOAD;
    int n   = min(CHUNK_PAYLOAD, len - off);
    uint8_t pkt[6 + CHUNK_PAYLOAD];
    pkt[0] = TYPE_WRITE_CHUNK;
    pkt[1] = dev;
    pkt[2] = (uint8_t)(sec & 0xFF);
    pkt[3] = (uint8_t)(sec >> 8);
    pkt[4] = (uint8_t)i;
    pkt[5] = (uint8_t)cc;
    memcpy(pkt+6, buf+off, n);
    send_now_to(BCAST_MAC, pkt, 6 + n);
    delay(2);
  }

  // 3) Esperar ACK/NAK final del SLAVE
  unsigned long t0 = millis();
  const unsigned long TIMEOUT = 8000;
  while (millis() - t0 < TIMEOUT) {
    if (g_writeResultReady && g_writeDev == dev) break;
    delay(1);
  }

  if (!(g_writeResultReady && g_writeDev == dev)) {
    logf("[REMOTE] WRITE timeout dev=%s sec=%u", devName(dev), sec);
    return false;
  }

  return g_writeResultOk;
}

// ==================== FORMAT remoto ====================

bool remoteFormatAndSendToAtari(uint8_t dev, bool dd){
  (void)dd; // por si luego queremos usar densidad en el SLAVE

  if (!isSlavePresent(dev)) {
    logf("[FORMAT] %s no presente", devName(dev));
    SerialSIO.write(SIO_ERROR);
    SerialSIO.flush();
    return false;
  }

  g_formatReady = false;
  g_formatDev   = 0;

  uint8_t pkt[6];
  pkt[0] = TYPE_CMD_FRAME;
  pkt[1] = 0x21 | (dd ? 0x80 : 0);
  pkt[2] = dev;
  pkt[3] = 0x00;
  pkt[4] = 0x00;
  pkt[5] = dd ? 1 : 0;
  send_now_to(BCAST_MAC, pkt, sizeof(pkt));

  // Esperar BAD MAP desde el SLAVE (hasta 90s)
  unsigned long t0 = millis();
  const unsigned long TIMEOUT = 90000;
  while (millis() - t0 < TIMEOUT) {
    if (g_formatReady && g_formatDev == dev) break;
    delay(5);
  }

  if (!(g_formatReady && g_formatDev == dev)) {
    logf("[FORMAT] Timeout BAD MAP dev=%s", devName(dev));
    SerialSIO.write(SIO_ERROR);
    SerialSIO.flush();
    return false;
  }

  // Si todo 0x00, generar FF FF (sin sectores malos)
  bool empty = true;
  for (int i=0; i<128; i++){
    if (g_formatMap[i] != 0x00){
      empty = false;
      break;
    }
  }
  if (empty) {
    memset(g_formatMap, 0x00, 128);
    g_formatMap[0] = 0xFF;
    g_formatMap[1] = 0xFF;
    logf("[FORMAT] BAD MAP vacío → FF FF (sin sectores malos)");
  }

  // Cálculo "estilo XF551" con FF FF extra (como simulación que ya probaste)
  uint16_t sum = 0;
  for (int i=0; i<128; i++){
    sum += g_formatMap[i];
    if (sum > 0xFF) sum = (sum & 0xFF) + 1;
  }
  sum += 0xFF; if (sum > 0xFF) sum = (sum & 0xFF) + 1;
  sum += 0xFF; if (sum > 0xFF) sum = (sum & 0xFF) + 1;
  uint8_t fchk = sum & 0xFF;

  if (fchk != 0xFF) {
    uint8_t diff = 0xFF - fchk;
    g_formatMap[127] = (uint8_t)((g_formatMap[127] + diff) & 0xFF);
    fchk = 0xFF;
  }

  // *** Ojo: FORMAT sigue con la misma secuencia que ya tenías ***
  delayMicroseconds(1500);
  SerialSIO.write(SIO_COMPLETE);
  SerialSIO.flush();

  delayMicroseconds(1700);
  for (int i=0; i<128; i++){
    SerialSIO.write(g_formatMap[i]);
    delayMicroseconds(8);
  }
  SerialSIO.write(0xFF);
  delayMicroseconds(8);
  SerialSIO.write(0xFF);
  delayMicroseconds(300);
  SerialSIO.write(fchk);
  SerialSIO.flush();
  delayMicroseconds(1700);

  logf("[FORMAT] %s BAD MAP enviado (CHK=%02X)", devName(dev), fchk);

  g_formatReady = false;
  return true;
}

// ==================== SIO (Atari) – envío de datos simples ====================

// <<< ÚNICA FUNCIÓN MODIFICADA >>>
void sendAtariData(const uint8_t *buf, int len) {
  // En este punto el Atari YA recibió el ACK.
  // Secuencia que espera el Atari: DATA -> CHECKSUM -> COMPLETE.

  // Pausa entre ACK y primer byte de datos (~1.7 ms)
  delayMicroseconds(1700);

  // Enviar datos
  for (int i = 0; i < len; i++) {
    SerialSIO.write(buf[i]);
    // pequeño respiro, pero muy corto para no matar el throughput
    delayMicroseconds(6);
  }
  SerialSIO.flush();

  // Checksum estilo SIO (end-around carry)
  uint8_t c = sioChecksum(buf, len);

  // Pausa corta antes del checksum (~0.28 ms)
  delayMicroseconds(280);
  SerialSIO.write(c);
  SerialSIO.flush();

  // Pausa y luego COMPLETE para cerrar la operación
  delayMicroseconds(1700);
  SerialSIO.write(SIO_COMPLETE);
  SerialSIO.flush();

#if DEBUG
  logf("[SIO] DATA+CHK+COMPLETE enviados (len=%d)", len);
#endif
}

// ==================== Manejo de frames SIO ====================

void handleSioFrame(){
  uint8_t f[5];

  for (int i=0; i<5; i++) {
    unsigned long t0 = micros();
    while (!SerialSIO.available()) {
      if (micros() - t0 > 3000) {
        return; // timeout leyendo el frame
      }
    }
    f[i] = SerialSIO.read();
  }

  uint8_t dev     = f[0];
  uint8_t cmd     = f[1];
  uint16_t sec    = (uint16_t)f[3] << 8 | (uint16_t)f[2];
  uint8_t recvChk = f[4];
  uint8_t calcChk = sioChecksum(f, 4);

#if DEBUG
  Serial.print("[FRAME] ");
  for (int i=0; i<5; i++) Serial.printf("%02X ", f[i]);
  Serial.println();
#endif

  // 1) Si no es D1..D4 => ignorar silenciosamente
  if (dev < 0x31 || dev > 0x34) {
#if DEBUG
    static uint8_t  lastDevLogged = 0x00;
    static uint32_t lastLogMs     = 0;
    uint32_t now = millis();
    if (dev != lastDevLogged || (now - lastLogMs) > 1000) {
      logf("[SIO] Frame para dev=0x%02X ignorado (no es D1..D4)", dev);
      lastDevLogged = dev;
      lastLogMs     = now;
    }
#endif
    return;
  }

  // 2) Checksum inválido => NAK
  if (recvChk != calcChk) {
    delayMicroseconds(1000);
    SerialSIO.write(SIO_NAK);
    SerialSIO.flush();
    logf("[SIO] Checksum inválido (recv=%02X calc=%02X)", recvChk, calcChk);
    return;
  }

  bool dd      = (cmd & 0x80) != 0;
  uint8_t base = cmd & 0x7F;

  // 3) Si no hay SLAVE, devolver ERROR rápido (sin ESPNOW)
  if (!isSlavePresent(dev)) {
    delayMicroseconds(1000);
    SerialSIO.write(SIO_ERROR);
    SerialSIO.flush();
    logf("[SIO] %s no presente, ERROR inmediato", devName(dev));
    return;
  }

  // === READ SECTOR ===
  if (base == 0x52) {
    delayMicroseconds(1000);
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();

    uint8_t buf[MAX_SECTOR_BYTES];
    int n = remoteReadSector(dev, sec, dd, buf);
    if (n <= 0) {
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      logf("[SIO] READ remoto falló %s sec=%u", devName(dev), sec);
      return;
    }

    sendAtariData(buf, n);
    return;
  }

  // === STATUS ===
  if (base == 0x53) {
    delayMicroseconds(1000);
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();

    uint8_t st[4];
    int n = remoteReadStatus(dev, st);
    if (n != 4) {
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      logf("[SIO] STATUS remoto falló %s", devName(dev));
      return;
    }

    sendAtariData(st, 4);
    return;
  }

  // === WRITE SECTOR ===
  if (base == 0x57) {
    delayMicroseconds(1000);
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();

    int expected = dd ? SECTOR_256 : SECTOR_128;
    uint8_t buf[MAX_SECTOR_BYTES];

    // Leer DATA desde el Atari
    int idx = 0;
    unsigned long t0 = millis();
    while (idx < expected && millis() - t0 < 6000) {
      if (SerialSIO.available()) {
        buf[idx++] = SerialSIO.read();
      } else {
        delay(1);
      }
    }
    if (idx != expected) {
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      logf("[SIO] WRITE: no llegaron %d bytes (llegaron %d)",
           expected, idx);
      return;
    }

    // Leer CHECKSUM
    uint8_t rchk = 0;
    t0 = millis();
    while (millis() - t0 < 2000) {
      if (SerialSIO.available()) {
        rchk = SerialSIO.read();
        break;
      }
      delay(1);
    }
    uint8_t calc = sioChecksum(buf, expected);
    if (rchk != calc) {
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      logf("[SIO] WRITE: checksum inválido (recv=%02X calc=%02X)",
           rchk, calc);
      return;
    }

    bool ok = remoteWriteSector(dev, sec, dd, buf, expected);
    if (!ok) {
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      logf("[SIO] WRITE remoto falló %s sec=%u", devName(dev), sec);
      return;
    }

    // COMPLETE simple (sin DATA) para cerrar
    delayMicroseconds(1500);
    SerialSIO.write(SIO_COMPLETE);
    SerialSIO.flush();
    logf("[SIO] WRITE %s sec=%u OK", devName(dev), sec);
    return;
  }

  // === FORMAT ===
  if (base == 0x21) {
    delayMicroseconds(1000);
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();

    if (!remoteFormatAndSendToAtari(dev, dd)) {
      return;
    }
    return;
  }

  // Comando no soportado
  delayMicroseconds(1000);
  SerialSIO.write(SIO_ERROR);
  SerialSIO.flush();
  logf("[SIO] Comando 0x%02X no soportado en %s", cmd, devName(dev));
}

// ==================== WEB UI ====================

String htmlHeader(){
  return F("<!doctype html><html lang='es'><head><meta charset='utf-8'>"
           "<meta name='viewport' content='width=device-width,initial-scale=1'>"
           "<title>ATARI Bridge – Prefetch</title>"
           "<style>"
           "body{font-family:system-ui,Arial,sans-serif;margin:0;padding:16px;background:#f5f5f5;color:#333}"
           "h1{margin-top:0;font-size:24px}"
           ".box{background:#fff;border-radius:8px;padding:16px;margin-bottom:16px;box-shadow:0 1px 3px rgba(0,0,0,0.1)}"
           "table{width:100%;border-collapse:collapse;font-size:14px}"
           "th,td{border:1px solid #ddd;padding:8px;text-align:left}"
           "th{background:#f0f0f0}"
           "button{padding:6px 10px;border:none;border-radius:4px;cursor:pointer;font-size:13px}"
           ".btn-on{background:#28a745;color:#fff}"
           ".btn-off{background:#6c757d;color:#fff}"
           ".btn-reset{background:#dc3545;color:#fff}"
           ".btn-reboot{background:#007bff;color:#fff;margin-top:8px}"
           ".status-on{color:#28a745;font-weight:600}"
           ".status-off{color:#dc3545;font-weight:600}"
           "</style></head><body>");
}

String htmlFooter(){
  return F("</body></html>");
}

void handleRoot(){
  String s = htmlHeader();
  s += "<h1>🎮 ATARI Bridge – XF551</h1>";
  s += "<div class='box'><h2>Unidades</h2>";
  s += "<table><tr><th>ID</th><th>Estado</th><th>MAC</th><th>DD</th><th>Prefetch</th><th>Acciones</th></tr>";

  for (int i=0; i<4; i++){
    uint8_t dev = 0x31 + i;
    char macbuf[24];

    if (slaves[i].present) {
      sprintf(macbuf,"%02X:%02X:%02X:%02X:%02X:%02X",
              slaves[i].mac[0],slaves[i].mac[1],slaves[i].mac[2],
              slaves[i].mac[3],slaves[i].mac[4],slaves[i].mac[5]);
    } else {
      strcpy(macbuf,"—");
    }

    String estado = slaves[i].present
                    ? "<span class='status-on'>🟢 Online</span>"
                    : "<span class='status-off'>🔴 Offline</span>";

    String ddInfo = slaves[i].supports256 ? "✅ Sí" : "❌ No";
    bool pfEn = g_prefetchEnabled[i];

    s += "<tr>";
    s += "<td><strong>D"+String(i+1)+"</strong></td>";
    s += "<td>"+estado+"</td>";
    s += "<td style='font-family:monospace;font-size:12px;'>"+String(macbuf)+"</td>";
    s += "<td>"+ddInfo+"</td>";

    // Prefetch
    s += "<td>";
    s += "<form method='post' action='/prefetch' style='display:inline'>";
    s += "<input type='hidden' name='d' value='"+String(i+1)+"'>";
    if (pfEn) {
      s += "<input type='hidden' name='mode' value='0'>";
      s += "<button class='btn-on' type='submit'>ON</button>";
    } else {
      s += "<input type='hidden' name='mode' value='1'>";
      s += "<button class='btn-off' type='submit'>OFF</button>";
    }
    s += "</form>";
    s += "</td>";

    // Reset
    s += "<td>";
    s += "<form method='post' action='/reset' style='display:inline'>";
    s += "<input type='hidden' name='d' value='"+String(i+1)+"'>";
    s += "<button class='btn-reset' type='submit'>🔄 Reset</button>";
    s += "</form>";
    s += "</td>";

    s += "</tr>";
  }

  s += "</table></div>";

  s += "<div class='box'><h2>MASTER</h2>";
  s += "<p>AP SSID: <strong>"+AP_SSID+"</strong><br>IP: <code>"+WiFi.softAPIP().toString()+"</code></p>";
  s += "<form method='post' action='/reboot'>";
  s += "<button class='btn-reboot' type='submit'>🔁 Reiniciar MASTER</button>";
  s += "</form>";
  s += "</div>";

  s += htmlFooter();
  web.send(200, "text/html", s);
}

// Cambiar modo Prefetch para una unidad
void handlePrefetch(){
  if (!web.hasArg("d") || !web.hasArg("mode")) {
    web.send(400,"text/plain","Faltan parámetros d/mode");
    return;
  }
  int dn = web.arg("d").toInt();  // 1..4
  if (dn < 1 || dn > 4) {
    web.send(400,"text/plain","Unidad inválida");
    return;
  }
  int idx = dn - 1;
  bool en = (web.arg("mode") == "1");

  g_prefetchEnabled[idx] = en;
  uint8_t dev = 0x30 + dn; // 0x31..0x34
  sendPrefetchConfig(dev, en);

  web.sendHeader("Location","/");
  web.send(302);
}

// Reset de SLAVE
void handleReset(){
  if (!web.hasArg("d")) {
    web.send(400,"text/plain","Falta parámetro d");
    return;
  }
  int dn = web.arg("d").toInt();
  if (dn < 1 || dn > 4) {
    web.send(400,"text/plain","Unidad inválida");
    return;
  }
  uint8_t dev = 0x30 + dn;
  sendResetTo(dev);
  web.sendHeader("Location","/");
  web.send(302);
}

// Reboot del MASTER
void handleReboot(){
  web.send(200,"text/plain","Reiniciando MASTER...");
  delay(500);
  ESP.restart();
}

void startAP(){
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID.c_str(), AP_PSK.c_str());
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  logf("[AP] SSID=%s IP=%s",
       AP_SSID.c_str(),
       WiFi.softAPIP().toString().c_str());
}

void startWeb(){
  web.on("/", HTTP_GET,  handleRoot);
  web.on("/prefetch", HTTP_POST, handlePrefetch);
  web.on("/reset",    HTTP_POST, handleReset);
  web.on("/reboot",   HTTP_POST, handleReboot);
  web.begin();
  logf("[WEB] UI en http://%s", WiFi.softAPIP().toString().c_str());
}

// ==================== SETUP / LOOP ====================

void setup(){
  setCpuFrequencyMhz(240);  // máxima frecuencia para bajar latencia
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[MASTER] ATARI Bridge – XF551");

  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, HIGH);

  SerialSIO.begin(19200, SERIAL_8N1, SIO_DATA_IN, SIO_DATA_OUT);
  pinMode(SIO_COMMAND, INPUT_PULLUP);

  // Estado inicial de slaves
  for (int i=0; i<4; i++) {
    slaves[i].present = false;
    slaves[i].supports256 = false;
    memset(slaves[i].mac, 0, 6);
    slaves[i].lastSeen = 0;
    g_prefetchEnabled[i] = true;
  }

  startAP();
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] esp_now_init falló");
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
  ensurePeer(BCAST_MAC);

  startWeb();

  digitalWrite(LED_STATUS, LOW);
}

void loop(){
  web.handleClient();

  // Marcar SLAVES inactivos
  unsigned long now = millis();
  for (int i=0; i<4; i++){
    if (slaves[i].present && (now - slaves[i].lastSeen > 90000)) {
      slaves[i].present = false;
    }
  }

  // Gestión del bus SIO (Atari)
  if (digitalRead(SIO_COMMAND) == LOW) {
    // pequeña espera para evitar falsos positivos
    delayMicroseconds(800);
    if (digitalRead(SIO_COMMAND) == LOW && SerialSIO.available() >= 5) {
      handleSioFrame();
    }
  }

  delay(1);
}
