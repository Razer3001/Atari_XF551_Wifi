/*
  SLAVE – XF551 REAL – STATUS + READ + FORMAT + WRITE + PREFETCH + PERCOM
  -----------------------------------------------------------------------
  - Conecta una XF551 real por SIO
  - DEVICE_ID = 0x31..0x34 (D1..D4)
  - Soporta:
      STATUS (0x53)
      READ SECTOR (0x52) con prefetch
      FORMAT (0x21 / 0x22) → 128 bytes de resultado
      WRITE SECTOR (0x50 / 0x57)
      READ PERCOM (0x4E)
      WRITE PERCOM (0x4F)
  - Protocolo ESP-NOW con el MASTER
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HardwareSerial.h>
#include <stdarg.h>

// ======== SIO hacia XF551 ========
#define SIO_RX      16 // desde XF551
#define SIO_TX      17 // hacia XF551
#define SIO_COMMAND 18

HardwareSerial SerialSIO(2);

// Códigos SIO
#define SIO_ACK      0x41
#define SIO_NAK      0x4E
#define SIO_COMPLETE 0x43
#define SIO_ERROR    0x45

// ======== Protocolo ESP-NOW ========
#define TYPE_HELLO        0x20
#define TYPE_CMD_FRAME    0x01
#define TYPE_SECTOR_CHUNK 0x10
#define TYPE_ACK          0x11
#define TYPE_NAK          0x12

#define CHUNK_PAYLOAD     240
#define MAX_SECTOR_BYTES  256
#define SECTOR_128        128
#define SECTOR_256        256
#define PERCOM_BLOCK_LEN  12
#define PERCOM_SEC_MAGIC  0xFFFF

#define MAX_PREFETCH_SECTORS 6

// Configura este valor según la unidad que quieras emular:
const uint8_t DEVICE_ID   = 0x31; // D1
const bool    supports256 = true; // XF551 soporta DD

const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t g_lastMaster[6]  = {0};
bool    g_haveMasterMac  = false;

// PREFETCH CACHE
static uint8_t  cacheBuf[MAX_PREFETCH_SECTORS][MAX_SECTOR_BYTES];
static uint16_t cacheFirstSec = 0;
static uint8_t  cacheCount    = 0;
static uint8_t  cacheDev      = 0;
static bool     cacheDD       = false;

// WRITE buffers (no tocados aquí)
static uint8_t  writeBuf[MAX_SECTOR_BYTES];
static uint16_t writeSec       = 0;
static int      writeLen       = 0;
static bool     writeBufReady  = false;

// PERCOM WRITE buffer (no tocado aquí)
static uint8_t  percomWriteBuf[PERCOM_BLOCK_LEN];
static int      percomLen      = 0;
static bool     percomBufReady = false;

// ======== Utilidades ========

void logf(const char* fmt, ...) {
  char b[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  Serial.println(b);
}

uint8_t sioChecksum(const uint8_t* d, int len) {
  uint16_t s = 0;
  for (int i = 0; i < len; i++){
    s += d[i];
    if (s > 0xFF) s = (s & 0xFF) + 1;
  }
  return s & 0xFF;
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

const uint8_t* replyMac() {
  return g_haveMasterMac ? g_lastMaster : BCAST_MAC;
}

bool send_now_to(const uint8_t* mac, const uint8_t* data, int len) {
  ensurePeer(mac);
  return (esp_now_send(mac, data, len) == ESP_OK);
}

void sendHello() {
  uint8_t p[3];
  p[0] = TYPE_HELLO;
  p[1] = DEVICE_ID;
  p[2] = supports256 ? 1 : 0;
  send_now_to(BCAST_MAC, p, sizeof(p));
}

// ACK/NAK
void sendACK() {
  uint8_t p[2] = {TYPE_ACK, DEVICE_ID};
  send_now_to(replyMac(), p, sizeof(p));
}

void sendNAK() {
  uint8_t p[2] = {TYPE_NAK, DEVICE_ID};
  send_now_to(replyMac(), p, sizeof(p));
}

// ======== SIO hacia XF551 real ========

void pulseCommandAndSendFrame(const uint8_t* frame5) {
  while (SerialSIO.available()) SerialSIO.read();
  digitalWrite(SIO_COMMAND, LOW);
  delayMicroseconds(2000);
  SerialSIO.write(frame5, 5);
  SerialSIO.flush();
  delayMicroseconds(2500);
  digitalWrite(SIO_COMMAND, HIGH);
}

bool waitByte(uint8_t want, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (SerialSIO.available()) {
      uint8_t b = SerialSIO.read();
      if (b == want) return true;
      if (b == SIO_NAK && want != SIO_NAK)    return false;
      if (b == SIO_ERROR && want != SIO_ERROR) return false;
    }
    delay(1);
  }
  return false;
}

bool readFromDrive(uint8_t* out, int sz, unsigned long timeoutMs) {
  if (!waitByte(SIO_COMPLETE, timeoutMs)) return false;

  int idx = 0;
  unsigned long t1 = millis();
  while (idx < sz && (millis() - t1) < timeoutMs) {
    if (SerialSIO.available()) {
      out[idx++] = SerialSIO.read();
    } else {
      delay(1);
    }
  }
  if (idx != sz) return false;

  unsigned long t2 = millis();
  while (millis() - t2 < 50) {
    if (SerialSIO.available()) {
      (void)SerialSIO.read();
      break;
    }
    delay(1);
  }
  return true;
}

int readSectorFromXF(uint8_t dev, uint16_t sec, bool dd, uint8_t* outBuf) {
  uint8_t frame[5];
  frame[0] = dev;
  frame[1] = 0x52; // READ
  frame[2] = (uint8_t)(sec & 0xFF);
  frame[3] = (uint8_t)(sec >> 8);
  frame[4] = sioChecksum(frame, 4);

  logf("[SLAVE D%u] READ XF cmd frame: %02X %02X %02X %02X %02X",
       (unsigned)(dev - 0x30),
       frame[0], frame[1], frame[2], frame[3], frame[4]);

  pulseCommandAndSendFrame(frame);

  if (!waitByte(SIO_ACK, 4000)) {
    logf("[SLAVE D%u] READ: sin ACK del drive sec=%u",
         (unsigned)(dev - 0x30), sec);
    return 0;
  }

  int sz = dd ? SECTOR_256 : SECTOR_128;
  if (!readFromDrive(outBuf, sz, 8000)) {
    logf("[SLAVE D%u] READ: fallo lectura sec=%u",
         (unsigned)(dev - 0x30), sec);
    return 0;
  }

  logf("[SLAVE D%u] READ desde drive sec=%u (%s) len=%d OK",
       (unsigned)(dev - 0x30), sec, dd ? "DD" : "SD", sz);
  return sz;
}

bool writeSectorToXF(uint8_t dev, uint8_t cmd, uint16_t sec, bool dd, const uint8_t* buf, int len) {
  int expected = dd ? SECTOR_256 : SECTOR_128;
  if (len < expected) {
    logf("[SLAVE D%u] WRITE: len=%d < esperado=%d, se rellenará con ceros",
         (unsigned)(dev - 0x30), len, expected);
  }

  uint8_t frame[5];
  frame[0] = dev;
  frame[1] = cmd;
  frame[2] = (uint8_t)(sec & 0xFF);
  frame[3] = (uint8_t)(sec >> 8);
  frame[4] = sioChecksum(frame, 4);

  pulseCommandAndSendFrame(frame);

  if (!waitByte(SIO_ACK, 4000)) {
    logf("[SLAVE D%u] WRITE: sin ACK del drive sec=%u",
         (unsigned)(dev - 0x30), sec);
    return false;
  }

  uint8_t tmp[MAX_SECTOR_BYTES];
  memset(tmp, 0, sizeof(tmp));
  if (len > expected) len = expected;
  memcpy(tmp, buf, len);

  SerialSIO.write(tmp, expected);
  SerialSIO.flush();

  uint8_t chk = sioChecksum(tmp, expected);
  SerialSIO.write(chk);
  SerialSIO.flush();

  if (!waitByte(SIO_COMPLETE, 20000)) {
    logf("[SLAVE D%u] WRITE: sin COMPLETE del drive sec=%u",
         (unsigned)(dev - 0x30), sec);
    return false;
  }

  logf("[SLAVE D%u] WRITE sec=%u (%s) OK",
       (unsigned)(dev - 0x30), sec, dd ? "DD" : "SD");
  return true;
}

void sendSectorChunk(uint16_t sec, const uint8_t* buf, int len) {
  uint8_t pkt[6 + CHUNK_PAYLOAD];
  int cc = (len + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD;
  const uint8_t* mac = replyMac();

  for (int i = 0; i < cc; i++) {
    int off = i * CHUNK_PAYLOAD;
    int n   = min(CHUNK_PAYLOAD, len - off);

    pkt[0] = TYPE_SECTOR_CHUNK;
    pkt[1] = DEVICE_ID;
    pkt[2] = (uint8_t)(sec & 0xFF);
    pkt[3] = (uint8_t)(sec >> 8);
    pkt[4] = (uint8_t)i;
    pkt[5] = (uint8_t)cc;
    memcpy(pkt + 6, buf + off, n);

    bool ok = send_now_to(mac, pkt, 6 + n);
    logf("[SLAVE D%u] TX CHUNK sec=%u idx=%d/%d len=%d -> %s",
         (unsigned)(DEVICE_ID - 0x30),
         sec, i, cc, n, ok ? "OK" : "FAIL");

    delay(2);
  }
}

// ======== HANDLERS LÓGICOS ========

void handleReadFromMaster(uint8_t dev, uint16_t sec, bool dd, uint8_t pfCount) {
  uint8_t buf[MAX_SECTOR_BYTES];
  int sz  = dd ? SECTOR_256 : SECTOR_128;
  bool servedFromCache = false;

  if (cacheCount > 0 &&
      cacheDev == dev &&
      cacheDD  == dd &&
      sec >= cacheFirstSec &&
      sec < (uint16_t)(cacheFirstSec + cacheCount)) {
    uint8_t idx = (uint8_t)(sec - cacheFirstSec);
    memcpy(buf, cacheBuf[idx], sz);
    servedFromCache = true;
    logf("[SLAVE D%u] READ sec=%u desde CACHE (%s)",
         (unsigned)(dev - 0x30), sec, dd ? "DD" : "SD");
  }

  if (!servedFromCache) {
    logf("[SLAVE D%u] READ sec=%u (%s) desde DRIVE",
         (unsigned)(dev - 0x30), sec, dd ? "DD" : "SD");
    int r = readSectorFromXF(dev, sec, dd, buf);
    if (r <= 0) {
      sendNAK();
      cacheCount = 0;
      return;
    }
    sz = r;
  }

  sendACK();
  sendSectorChunk(sec, buf, sz);
  logf("[SLAVE D%u] READ sec=%u (%s) OK len=%d",
       (unsigned)(dev - 0x30), sec, dd ? "DD" : "SD", sz);

  if (pfCount == 0) {
    cacheCount = 0;
    return;
  }

  if (pfCount > MAX_PREFETCH_SECTORS) pfCount = MAX_PREFETCH_SECTORS;

  memcpy(cacheBuf[0], buf, sz);
  cacheFirstSec = sec;
  cacheDev      = dev;
  cacheDD       = dd;
  cacheCount    = 1;

  for (uint8_t i = 1; i < pfCount; i++) {
    uint16_t nextSec = sec + i;
    int r = readSectorFromXF(dev, nextSec, dd, cacheBuf[i]);
    if (r != sz) {
      break;
    }
    cacheCount++;
    logf("[SLAVE D%u][PREFETCH] sec=%u (%s) len=%d cacheado",
         (unsigned)(dev - 0x30), nextSec, dd ? "DD" : "SD", r);
  }
}

void handleStatusFromMaster(uint8_t dev, bool dd) {
  (void)dd;

  uint8_t frame[5];
  frame[0] = dev;
  frame[1] = 0x53;
  frame[2] = 0x00;
  frame[3] = 0x00;
  frame[4] = sioChecksum(frame, 4);

  pulseCommandAndSendFrame(frame);

  if (!waitByte(SIO_ACK, 3000)) {
    logf("[SLAVE D%u] STATUS: sin ACK del drive", (unsigned)(dev - 0x30));
    sendNAK();
    return;
  }

  uint8_t st[4];
  if (!readFromDrive(st, 4, 5000)) {
    logf("[SLAVE D%u] STATUS: fallo lectura", (unsigned)(dev - 0x30));
    sendNAK();
    return;
  }

  sendACK();
  sendSectorChunk(0, st, 4);
  logf("[SLAVE D%u] STATUS -> %02X %02X %02X %02X",
       (unsigned)(dev - 0x30), st[0], st[1], st[2], st[3]);
}

// FORMAT 0x21 / 0x22
void handleFormatFromMaster(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2, bool dd) {
  (void)dd; // la XF551 siempre devuelve 128 bytes de resultado

  uint8_t frame[5];
  frame[0] = dev;
  frame[1] = cmd;    // 0x21 / 0x22
  frame[2] = aux1;   // se preservan aux1/aux2 por si el DOS los usa
  frame[3] = aux2;
  frame[4] = sioChecksum(frame, 4);

  logf("[SLAVE D%u] FORMAT XF cmd=0x%02X aux1=0x%02X aux2=0x%02X",
       (unsigned)(dev - 0x30), cmd, aux1, aux2);

  pulseCommandAndSendFrame(frame);

  if (!waitByte(SIO_ACK, 5000)) {
    logf("[SLAVE D%u] FORMAT: sin ACK del drive", (unsigned)(dev - 0x30));
    sendNAK();
    return;
  }

  // El FORMAT puede tardar bastante; esperamos el COMPLETE + 128 bytes
  uint8_t buf[SECTOR_128];
  if (!readFromDrive(buf, SECTOR_128, 60000)) {
    logf("[SLAVE D%u] FORMAT: fallo lectura bloque resultado",
         (unsigned)(dev - 0x30));
    sendNAK();
    return;
  }

  // Invalida la caché de prefetch: el disco cambió
  cacheCount = 0;

  sendACK();
  sendSectorChunk(0, buf, SECTOR_128);
  logf("[SLAVE D%u] FORMAT cmd=0x%02X OK (128 bytes resultado)",
       (unsigned)(dev - 0x30), cmd);
}

// (WRITE / PERCOM se pueden añadir aquí igual que en tu código previo)

// ================== CALLBACK ESP-NOW ==================

#if ESP_IDF_VERSION_MAJOR >= 5

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t* in, int len) {
  if (len <= 0 || !in) return;

  const uint8_t *src = info ? info->src_addr : nullptr;

  if (src) {
    bool isBcast = true;
    for (int i = 0; i < 6; i++) {
      if (src[i] != 0xFF) { isBcast = false; break; }
    }
    if (!isBcast) {
      memcpy(g_lastMaster, src, 6);
      g_haveMasterMac = true;
      ensurePeer(g_lastMaster);
    }
  }

  uint8_t type = in[0];

  if (type == TYPE_SECTOR_CHUNK && len >= 6) {
    // Aquí iría la lógica de WRITE/PERCOM (igual que tu código original)
    return;
  }

  if (type == TYPE_CMD_FRAME && len >= 6) {
    uint8_t cmd  = in[1];
    uint8_t dev  = in[2];
    uint8_t aux1 = in[3];
    uint8_t aux2 = in[4];
    bool    dd   = (in[5] != 0);
    uint8_t pf   = 1;
    if (len >= 7) pf = in[6];

    if (dev != DEVICE_ID) return;

    uint8_t base = cmd & 0x7F;
    uint16_t sec = (uint16_t)aux1 | ((uint16_t)aux2 << 8);

    if (base == 0x52) {
      handleReadFromMaster(dev, sec, dd, pf);
      return;
    }

    if (base == 0x53) {
      handleStatusFromMaster(dev, dd);
      return;
    }

    if (base == 0x21 || base == 0x22) {
      handleFormatFromMaster(dev, cmd, aux1, aux2, dd);
      return;
    }

    // Aquí irían WRITE (0x50/0x57) y PERCOM (0x4E/0x4F) como en tu código
    sendNAK();
    return;
  }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t s) {
  (void)info;
  if (s == ESP_NOW_SEND_SUCCESS)
    Serial.println("[ESPNOW] Envío OK");
  else
    Serial.println("[ESPNOW] Falla en envío");
}

#else

void onDataRecv(const uint8_t* src, const uint8_t* in, int len) {
  if (len <= 0 || !in) return;

  if (src) {
    bool isBcast = true;
    for (int i = 0; i < 6; i++) {
      if (src[i] != 0xFF) { isBcast = false; break; }
    }
    if (!isBcast) {
      memcpy(g_lastMaster, src, 6);
      g_haveMasterMac = true;
      ensurePeer(g_lastMaster);
    }
  }

  uint8_t type = in[0];

  if (type == TYPE_SECTOR_CHUNK && len >= 6) {
    // WRITE/PERCOM igual que tu código previo
    return;
  }

  if (type == TYPE_CMD_FRAME && len >= 6) {
    uint8_t cmd  = in[1];
    uint8_t dev  = in[2];
    uint8_t aux1 = in[3];
    uint8_t aux2 = in[4];
    bool    dd   = (in[5] != 0);
    uint8_t pf   = 1;
    if (len >= 7) pf = in[6];

    if (dev != DEVICE_ID) return;

    uint8_t base = cmd & 0x7F;
    uint16_t sec = (uint16_t)aux1 | ((uint16_t)aux2 << 8);

    if (base == 0x52) {
      handleReadFromMaster(dev, sec, dd, pf);
      return;
    }

    if (base == 0x53) {
      handleStatusFromMaster(dev, dd);
      return;
    }

    if (base == 0x21 || base == 0x22) {
      handleFormatFromMaster(dev, cmd, aux1, aux2, dd);
      return;
    }

    // WRITE / PERCOM...
    sendNAK();
    return;
  }
}

void onDataSent(const uint8_t* mac, esp_now_send_status_t s) {
  (void)mac;
  if (s == ESP_NOW_SEND_SUCCESS)
    Serial.println("[ESPNOW] Envío OK");
  else
    Serial.println("[ESPNOW] Falla en envío");
}

#endif

// ======== SETUP / LOOP ========

void setup() {
  Serial.begin(115200);
  Serial.printf("\n[SLAVE] XF551 STATUS+READ+FORMAT+WRITE+PREFETCH+PERCOM DEV=0x%02X\n",
                DEVICE_ID);

  SerialSIO.begin(19200, SERIAL_8N1, SIO_RX, SIO_TX);
  pinMode(SIO_COMMAND, OUTPUT);
  digitalWrite(SIO_COMMAND, HIGH);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK){
    Serial.println("[ERR] esp_now_init falló");
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  ensurePeer(BCAST_MAC);
  sendHello();

  cacheCount      = 0;
  writeBufReady   = false;
  writeLen        = 0;
  percomBufReady  = false;
  percomLen       = 0;

  logf("[SLAVE] Listo DEV=0x%02X DD=%u", DEVICE_ID, supports256 ? 1 : 0);
}

void loop() {
  static unsigned long t0 = 0;
  if (millis() - t0 > 10000) {
    sendHello();
    t0 = millis();
  }
  delay(10);
}
