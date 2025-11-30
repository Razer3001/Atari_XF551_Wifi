#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <stdarg.h>

// ====================== CONFIG ESPNOW / SIO LÓGICO ======================

// D1..D4
#define DEV_MIN 0x31
#define DEV_MAX 0x34

// Tipos de mensaje ESPNOW
#define TYPE_HELLO        0x20
#define TYPE_CMD_FRAME    0x01
#define TYPE_SECTOR_CHUNK 0x10
#define TYPE_ACK          0x11
#define TYPE_NAK          0x12

#define CHUNK_PAYLOAD     240
#define MAX_SECTOR_BYTES  256
#define SECTOR_128        128
#define SECTOR_256        256

// Prefetch
#define MAX_PREFETCH_SECTORS 4

// ESPNOW broadcast
const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ====================== ESTADO DE SLAVES (D1..D4) =======================

struct SlaveInfo {
  bool present;
  bool supports256;
  uint8_t mac[6];
  unsigned long lastSeen;
};

SlaveInfo slaves[4]; // D1..D4

// ====================== ESTADO DE OPERACIÓN REMOTA ======================

enum OpType : uint8_t {
  OP_NONE   = 0,
  OP_READ   = 1,
  OP_STATUS = 2
};

volatile OpType   currentOpType = OP_NONE;
volatile uint8_t  currentOpDev  = 0;
volatile uint16_t currentOpSec  = 0;

volatile bool     replyReady     = false;
volatile uint16_t replySec       = 0;
volatile uint16_t replyLen       = 0;
volatile uint8_t  expectedChunks = 0;
volatile uint8_t  receivedChunks = 0;
uint8_t           replyBuf[MAX_SECTOR_BYTES];

volatile bool lastAckDone = false;
volatile bool lastAckOk   = false;

// ====================== UART hacia RP2040 ===============================
// Usaremos UART1: RX=4, TX=5 (ajusta si usas otros pines)
HardwareSerial LinkSerial(1);

// ====================== UTILIDADES =====================================

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
  for (int i = 0; i < len; i++) {
    s += d[i];
    if (s > 0xFF) s = (s & 0xFF) + 1;
  }
  return (uint8_t)(s & 0xFF);
}

int devIndex(uint8_t dev) {
  if (dev < DEV_MIN || dev > DEV_MAX) return -1;
  return dev - DEV_MIN; // 0..3
}

const char* devName(uint8_t dev) {
  static const char* names[] = {"D1", "D2", "D3", "D4"};
  int idx = devIndex(dev);
  return (idx >= 0) ? names[idx] : "UNK";
}

uint8_t prefetchForDev(uint8_t dev) {
  (void)dev;
  return 1; // por ahora fijo
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

const uint8_t* macForDev(uint8_t dev) {
  int idx = devIndex(dev);
  if (idx < 0) return BCAST_MAC;
  if (!slaves[idx].present) return BCAST_MAC;
  return slaves[idx].mac;
}

// ====================== ESPNOW CALLBACKS ================================

static void handleEspNowRecvCommon(const uint8_t* mac, const uint8_t* data, int len) {
  if (!data || len <= 0) return;
  uint8_t type = data[0];

  if (type == TYPE_HELLO && len >= 3) {
    uint8_t dev = data[1];
    bool s256   = (data[2] != 0);

    int idx = devIndex(dev);
    if (idx >= 0 && mac) {
      slaves[idx].present     = true;
      slaves[idx].supports256 = s256;
      memcpy(slaves[idx].mac, mac, 6);
      slaves[idx].lastSeen = millis();
      ensurePeer(slaves[idx].mac);
      logf("[HELLO] %s presente DD=%u",
           devName(dev), s256 ? 1 : 0);
    }
    return;
  }

  // Respuestas de sectores (STATUS = 4 bytes, READ = 128/256)
  if (type == TYPE_SECTOR_CHUNK && len >= 6) {
    uint8_t dev = data[1];
    uint16_t sec = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    uint8_t ci = data[4];
    uint8_t cc = data[5];
    int payload = len - 6;

    if (dev != currentOpDev || currentOpType == OP_NONE) return;

    if (ci == 0) {
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

    if (copyLen > 0) {
      memcpy(replyBuf + off, data + 6, copyLen);
      replyLen += copyLen;
    }

    receivedChunks++;
    logf("[NET] TYPE_SECTOR_CHUNK dev=%s sec=%u ci=%u/%u payload=%d",
         devName(dev), sec, ci, cc, payload);

    if (receivedChunks >= expectedChunks) {
      replyReady = true;
      logf("[NET] Respuesta completa dev=%s sec=%u len=%u",
           devName(dev), sec, replyLen);
    }
    return;
  }

  if (type == TYPE_ACK) {
    logf("[ESPNOW] ACK recibido");
    lastAckOk   = true;
    lastAckDone = true;
    return;
  } else if (type == TYPE_NAK) {
    logf("[ESPNOW] NAK recibido");
    lastAckOk   = false;
    lastAckDone = true;
    return;
  }
}

static void handleEspNowSentCommon(esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS)
    Serial.println("[ESPNOW] Envío OK");
  else
    Serial.println("[ESPNOW] Falla en envío");
}

#if ESP_IDF_VERSION_MAJOR >= 5

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info ? info->src_addr : nullptr;
  handleEspNowRecvCommon(mac, data, len);
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  handleEspNowSentCommon(status);
}

#else

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  handleEspNowRecvCommon(mac, data, len);
}

void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  (void)mac;
  handleEspNowSentCommon(status);
}

#endif

// ====================== ENVÍO DE CMD A SLAVE ============================
// [0]=TYPE_CMD_FRAME,[1]=cmd,[2]=dev,[3]=secL,[4]=secH,[5]=dens,[6]=prefetchCount
bool sendCmd(uint8_t dev, uint8_t cmd, uint16_t sec, bool dd) {
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
  esp_err_t e = esp_now_send(dest, p, sizeof(p));
  if (e != ESP_OK) {
    logf("[ESPNOW] esp_now_send error=%d", (int)e);
  }
  return (e == ESP_OK);
}

// ====================== OPERACIONES REMOTAS =============================

// STATUS remoto – 4 bytes
int readRemoteStatus(uint8_t dev, bool dd, uint8_t* out4) {
  (void)dd;

  int idx = devIndex(dev);
  if (idx < 0 || !slaves[idx].present) {
    logf("[STATUS] %s no presente", devName(dev));
    return 0;
  }

  replyReady     = false;
  replySec       = 0;
  replyLen       = 0;
  expectedChunks = receivedChunks = 0;

  currentOpType = OP_STATUS;
  currentOpDev  = dev;
  currentOpSec  = 0;

  logf("[STATUS] Solicitando STATUS a %s", devName(dev));
  if (!sendCmd(dev, 0x53, 0, false)) {
    logf("[STATUS] sendCmd falló para %s", devName(dev));
    currentOpType = OP_NONE;
    return 0;
  }

  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    if (replyReady) break;
    delay(2);
  }

  currentOpType = OP_NONE;

  if (!replyReady) {
    logf("[STATUS] Timeout esperando STATUS de %s", devName(dev));
    return 0;
  }

  if (replyLen < 4) {
    logf("[STATUS] Respuesta corta (%u bytes) de %s", replyLen, devName(dev));
    return 0;
  }

  memcpy(out4, replyBuf, 4);
  return 4;
}

// READ remoto – sector 128/256
int readRemoteSector(uint8_t dev, uint16_t sec, bool dd, uint8_t* out) {
  int idx = devIndex(dev);
  if (idx < 0 || !slaves[idx].present) {
    logf("[READ] %s no presente", devName(dev));
    return 0;
  }

  replyReady     = false;
  replySec       = 0;
  replyLen       = 0;
  expectedChunks = receivedChunks = 0;

  currentOpType = OP_READ;
  currentOpDev  = dev;
  currentOpSec  = sec;

  logf("[READ] Solicitando sec=%u dd=%s a %s",
       sec, dd ? "DD" : "SD", devName(dev));

  if (!sendCmd(dev, 0x52, sec, dd)) {
    logf("[READ] sendCmd falló para %s", devName(dev));
    currentOpType = OP_NONE;
    return 0;
  }

  unsigned long t0 = millis();
  int expectedLen = dd ? SECTOR_256 : SECTOR_128;

  while (millis() - t0 < 8000) {
    if (replyReady && replySec == sec) break;
    delay(2);
  }

  currentOpType = OP_NONE;

  if (!(replyReady && replySec == sec)) {
    logf("[READ] Timeout esperando sec=%u de %s", sec, devName(dev));
    return 0;
  }

  int n = (replyLen > expectedLen) ? expectedLen : replyLen;
  memcpy(out, replyBuf, n);

  logf("[READ] OK %s sec=%u len=%d", devName(dev), sec, n);
  return n;
}

// ====================== RESPUESTAS AL RP2040 ============================

// RESP_STATUS: 0xA5 0x81 status(0=OK,!0=error) [4 bytes]
void sendRespStatus(uint8_t status, const uint8_t* st4) {
  LinkSerial.write(0xA5);
  LinkSerial.write((uint8_t)0x81);
  LinkSerial.write(status);
  if (status == 0 && st4) {
    LinkSerial.write(st4, 4);
  }
  LinkSerial.flush();
}

// RESP_READ: 0xA5 0x82 status(0=OK,!0=error) len data[len]
void sendRespRead(uint8_t status, const uint8_t* data, uint8_t len) {
  LinkSerial.write(0xA5);
  LinkSerial.write((uint8_t)0x82);
  LinkSerial.write(status);
  if (status == 0 && data && len > 0) {
    LinkSerial.write(len);
    LinkSerial.write(data, len);
  } else {
    LinkSerial.write((uint8_t)0); // len=0 en error
  }
  LinkSerial.flush();
}

// ====================== PROCESAR FRAMES DESDE RP2040 ====================

void handleSioCommandFromRp2040(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2, uint8_t chk) {
  (void)chk;

  uint8_t base = cmd & 0x7F;
  bool dd      = (cmd & 0x80) != 0;
  uint16_t sec = (uint16_t)aux1 | ((uint16_t)aux2 << 8);

  // STATUS D1..D4
  if (base == 0x53 && dev >= DEV_MIN && dev <= DEV_MAX) {
    uint8_t st[4];
    int n = readRemoteStatus(dev, dd, st);
    if (n == 4) {
      sendRespStatus(0x00, st); // OK
    } else {
      sendRespStatus(0x01, nullptr); // error
    }
    return;
  }

  // READ SECTOR D1..D4
  if (base == 0x52 && dev >= DEV_MIN && dev <= DEV_MAX) {
    uint8_t buf[MAX_SECTOR_BYTES];
    int n = readRemoteSector(dev, sec, dd, buf);
    if (n > 0 && n <= 255) {
      sendRespRead(0x00, buf, (uint8_t)n);
    } else {
      sendRespRead(0x01, nullptr, 0);
    }
    return;
  }

  logf("[MASTER] Cmd no implementado 0x%02X para %s (aún en headless)",
       base, devName(dev));
}

void processLinkSerial() {
  static bool    inFrame = false;
  static uint8_t buf[6]; // tipo + 5 bytes SIO
  static int     pos = 0;

  while (LinkSerial.available()) {
    uint8_t b = (uint8_t)LinkSerial.read();

    if (!inFrame) {
      if (b == 0xA5) {
        inFrame = true;
        pos = 0;
      }
    } else {
      buf[pos++] = b;

      if (pos == 1) {
        if (buf[0] != 0x01) {
          logf("[MASTER] Tipo de frame desconocido desde RP2040: 0x%02X", buf[0]);
          inFrame = false;
          pos = 0;
        }
      } else if (pos == 6) {
        inFrame = false;

        uint8_t tipo = buf[0];
        uint8_t dev  = buf[1];
        uint8_t cmd  = buf[2];
        uint8_t aux1 = buf[3];
        uint8_t aux2 = buf[4];
        uint8_t chk  = buf[5];

        if (tipo != 0x01) {
          logf("[MASTER] Tipo inválido (0x%02X) en frame completo", tipo);
          continue;
        }

        logf("[MASTER] SIO_COMMAND desde RP2040: DEV=0x%02X CMD=0x%02X AUX1=0x%02X AUX2=0x%02X CHK=0x%02X",
             dev, cmd, aux1, aux2, chk);

        handleSioCommandFromRp2040(dev, cmd, aux1, aux2, chk);
      }
    }
  }
}

// ====================== SETUP / LOOP ====================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[MASTER] Headless ESPNOW + XF551 (STATUS + READ) para RP2040");

  // Estado inicial de SLAVES
  for (int i = 0; i < 4; i++) {
    slaves[i].present     = false;
    slaves[i].supports256 = false;
    memset(slaves[i].mac, 0, 6);
    slaves[i].lastSeen = 0;
  }

  // WiFi STA + ESPNOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] esp_now_init falló");
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  ensurePeer(BCAST_MAC);

  // UART hacia RP2040: RX=4, TX=5
  LinkSerial.begin(115200, SERIAL_8N1, 4, 5);

  Serial.println("[MASTER] Listo, esperando comandos desde RP2040 y HELLO desde SLAVES");
}

void loop() {
  // Actualizar presencia de SLAVES (timeout 60s)
  unsigned long now = millis();
  for (int i = 0; i < 4; i++) {
    if (slaves[i].present && (now - slaves[i].lastSeen > 60000)) {
      slaves[i].present = false;
    }
  }

  // Procesar frames desde RP2040
  processLinkSerial();

  delay(2);
}
