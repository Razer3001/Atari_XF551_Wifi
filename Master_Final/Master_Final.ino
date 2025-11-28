/*******************************************************
 * MASTER MINIMO – STATUS + READ (D1..D4)
 * - Emula D1..D4 (0x31..0x34) frente al Atari
 * - Soporta STATUS (0x53) y READ SECTOR (0x52)
 * - Usa ESP-NOW para delegar en SLAVES con XF551 real
 * - Protocolo:
 *    • Cada SLAVE manda HELLO (TYPE_HELLO) con dev y soporte DD
 *    • MASTER manda TYPE_CMD_FRAME a D1..D4
 *    • SLAVE responde con TYPE_SECTOR_CHUNK
 *******************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HardwareSerial.h>

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

// ========= Timings simples SIO (µs) =========
const uint16_t T_ACK_TO_COMPLETE   = 2000;
const uint16_t T_COMPLETE_TO_DATA  = 1600;
const uint16_t T_DATA_TO_CHK       = 400;
const uint16_t T_CHUNK_DELAY       = 1600;

// ========= Estado de SLAVES (uno por unidad) =========
struct SlaveInfo {
  bool     present;
  bool     supports256;
  uint8_t  mac[6];
  unsigned long lastSeen;
};

SlaveInfo slaves[4]; // D1..D4

int devIndex(uint8_t dev){
  if (dev < DEV_MIN || dev > DEV_MAX) return -1;
  return dev - DEV_MIN; // 0..3
}

const char* devName(uint8_t dev){
  static const char* names[] = {"D1", "D2", "D3", "D4"};
  int idx = devIndex(dev);
  return (idx >= 0) ? names[idx] : "UNK";
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
const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF};

// ========= Pequeñas utilidades de log =========

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
  vsnprintf(buf, sizeof(buf), fmt, ap);
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
  esp_now_add_peer(&p);
}

const uint8_t* macForDev(uint8_t dev){
  int idx = devIndex(dev);
  if (idx < 0) return BCAST_MAC;
  if (!slaves[idx].present) return BCAST_MAC;
  return slaves[idx].mac;
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
      logf("[HELLO] %s presente DD=%u MAC=%02X:%02X:%02X:%02X:%02X:%02X",
           devName(dev), s256,
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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

    // Solo aceptamos si coincide con la operación en curso
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

#else  // IDF < 5 (firmware antiguos)

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){
  handleEspNowRecvCommon(mac, data, len);
}

void onDataSent(const uint8_t* mac, esp_now_send_status_t status){
  (void)mac;
  handleEspNowSentCommon(status);
}

#endif

// ========= Envío de comandos al SLAVE =========

// [0]=TYPE_CMD_FRAME,[1]=cmd,[2]=dev,[3]=secL,[4]=secH,[5]=dens
bool sendCmd(uint8_t dev, uint8_t cmd, uint16_t sec, bool dd){
  uint8_t p[6];
  p[0] = TYPE_CMD_FRAME;
  p[1] = cmd;
  p[2] = dev;
  p[3] = (uint8_t)(sec & 0xFF);
  p[4] = (uint8_t)(sec >> 8);
  p[5] = dd ? 1 : 0;

  const uint8_t* dest = macForDev(dev);

  logf("[ESPNOW][TX] CMD_FRAME dev=%s cmd=0x%02X sec=%u dd=%s",
       devName(dev), cmd, sec, dd ? "DD" : "SD");
  logHex("[ESPNOW][TX] bytes:", p, sizeof(p));

  return (esp_now_send(dest, p, sizeof(p)) == ESP_OK);
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

  logf("[STATUS] Solicitando STATUS dd=%s a %s",
       dd ? "DD" : "SD", devName(dev));

  if (!sendCmd(dev, 0x53, 0, dd)){
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
    // Ignoramos totalmente (para evitar "tanda basura")
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
  Serial.println("\n[MASTER] MINIMAL STATUS+READ D1..D4");

  pinMode(SIO_COMMAND, INPUT_PULLUP);
  SerialSIO.begin(19200, SERIAL_8N1, SIO_DATA_IN, SIO_DATA_OUT, false);

  // Init tabla slaves
  for (int i = 0; i < 4; i++){
    slaves[i].present     = false;
    slaves[i].supports256 = false;
    memset(slaves[i].mac, 0, 6);
    slaves[i].lastSeen    = 0;
  }

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK){
    Serial.println("[ERR] esp_now_init falló");
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  ensurePeer(BCAST_MAC);
}

void loop(){
  // Limpiar esclavos inactivos (por ejemplo > 60s sin HELLO)
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
      // Limpiar residuos de este comando
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
