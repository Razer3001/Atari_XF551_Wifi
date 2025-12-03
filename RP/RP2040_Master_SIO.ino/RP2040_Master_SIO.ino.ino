#include <Arduino.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>

// ================== PINES ==================

// SIO Atari
const int PIN_CMD    = 2;   // CMD SIO (desde Atari, con divisor)
const int PIN_SIO_TX = 0;   // TX hacia Atari (DATA IN Atari)
const int PIN_SIO_RX = 1;   // RX desde Atari (DATA OUT Atari)

// UART hacia ESP32 MASTER (usaremos uart1)
const int PIN_ESP_TX = 4;   // RP2040 -> ESP32 (RX del ESP)
const int PIN_ESP_RX = 5;   // ESP32 -> RP2040 (TX del ESP)

// LED de estado
const int LED_STATUS = LED_BUILTIN;   // RP2 Nano

// Alias para el puerto SIO
#define SerialSIO Serial1

// UART de enlace con el MASTER
#define UART_ESP uart1
#define UART_ESP_BAUD 115200

// ================== Protocolo UART RP <-> MASTER ==================

#define TYPE_CMD_FRAME    0x01
#define TYPE_SECTOR_CHUNK 0x10
#define TYPE_ACK          0x11
#define TYPE_NAK          0x12
#define TYPE_HELLO        0x20

// Códigos SIO
#define SIO_ACK      0x41
#define SIO_NAK      0x4E
#define SIO_COMPLETE 0x43
#define SIO_ERROR    0x45

// Solo D1
const uint8_t SIO_DEV_D1 = 0x31;

// PERCOM
#define PERCOM_BLOCK_LEN 12
#define PERCOM_SEC_MAGIC 0xFFFF

// Timings (los mismos que tu MASTER original)
uint16_t T_ACK_TO_COMPLETE   = 600;   // µs
uint16_t T_COMPLETE_TO_DATA  = 400;   // µs
uint16_t T_DATA_TO_CHK       = 80;    // µs
uint16_t T_CHUNK_DELAY       = 600;   // µs

// Estado de la línea CMD
bool lastCmdState = HIGH;

// Buffer de comando SIO
uint8_t cmdBuf[5];

// ================== FSM UART desde MASTER ==================

uint8_t  uartState = 0;
uint8_t  uartLen   = 0;        // LEN = TYPE + PAYLOAD
uint8_t  uartIdx   = 0;
uint8_t  uartBuf[260];         // TYPE + PAYLOAD

// Operaciones en curso
enum CurrentOp : uint8_t {
  OP_NONE = 0,
  OP_STATUS,
  OP_READ,
  OP_FORMAT,
  OP_PERCOM,   // NUEVO
  OP_WRITE     // NUEVO
};

CurrentOp g_currentOp = OP_NONE;

// STATUS remoto
bool     g_statusDone    = false;
bool     g_statusSuccess = false;
uint8_t  g_statusData[4];

// READ remoto
bool     g_readDone        = false;
bool     g_readSuccess     = false;
uint8_t  g_readBuf[256];
int      g_readLen         = 0;
uint16_t g_readSec         = 0;
uint8_t  g_readChunkCount  = 0;
uint8_t  g_readChunksSeen  = 0;

// FORMAT remoto (bloque de 128 bytes)
bool     g_formatDone      = false;
bool     g_formatSuccess   = false;
uint8_t  g_formatBuf[128];
int      g_formatLen       = 0;

// PERCOM remoto (12 bytes)
bool     g_percomDone      = false;
bool     g_percomSuccess   = false;
uint8_t  g_percomBuf[PERCOM_BLOCK_LEN];

// WRITE remoto (solo ACK/NAK)
bool     g_writeDone       = false;
bool     g_writeSuccess    = false;
uint16_t g_writeSec        = 0;

// ================== Utilidades ==================

uint8_t calcChecksumSIO(const uint8_t *buf, int len) {
  uint16_t s = 0;
  for (int i = 0; i < len; i++) {
    s += buf[i];
    if (s > 0xFF) s = (s & 0xFF) + 1;   // con acarreo
  }
  return (uint8_t)(s & 0xFF);
}

bool readByteWithTimeoutSIO(uint8_t &outByte, uint16_t timeoutMs) {
  unsigned long t0 = millis();
  while (!SerialSIO.available()) {
    if (millis() - t0 >= timeoutMs) return false;
  }
  outByte = (uint8_t)SerialSIO.read();
  return true;
}

void dumpCommandFrame(const uint8_t *buf) {
  uint8_t dev  = buf[0];
  uint8_t cmd  = buf[1];
  uint8_t aux1 = buf[2];
  uint8_t aux2 = buf[3];
  uint8_t chk  = buf[4];
  uint8_t calc = calcChecksumSIO(buf, 4);

  Serial.print(F("[SIO] Frame CMD: DEV=0x"));
  if (dev < 0x10) Serial.print('0');
  Serial.print(dev, HEX);

  Serial.print(F(" CMD=0x"));
  if (cmd < 0x10) Serial.print('0');
  Serial.print(cmd, HEX);

  Serial.print(F(" AUX1=0x"));
  if (aux1 < 0x10) Serial.print('0');
  Serial.print(aux1, HEX);

  Serial.print(F(" AUX2=0x"));
  if (aux2 < 0x10) Serial.print('0');
  Serial.print(aux2, HEX);

  Serial.print(F(" CHK=0x"));
  if (chk < 0x10) Serial.print('0');
  Serial.print(chk, HEX);

  Serial.print(F(" (calc=0x"));
  if (calc < 0x10) Serial.print('0');
  Serial.print(calc, HEX);
  Serial.println(F(")"));

  if (dev == SIO_DEV_D1) {
    Serial.println(F("[SIO] → Comando dirigido a D1: (0x31)"));
  } else {
    Serial.print(F("[SIO] → Dispositivo distinto de D1: 0x"));
    if (dev < 0x10) Serial.print('0');
    Serial.println(dev, HEX);
  }
}

// ================== UART → MASTER ==================

void uartSendByte(uint8_t b) {
  uart_putc_raw(UART_ESP, b);
}

void uartSendFrame(uint8_t type, const uint8_t *payload, uint8_t payloadLen) {
  uint8_t len = 1 + payloadLen; // TYPE + PAYLOAD

  uint8_t sum = type;
  for (uint8_t i = 0; i < payloadLen; i++) {
    sum += payload[i];
  }

  uartSendByte(0x55);
  uartSendByte(len);
  uartSendByte(type);
  if (payloadLen > 0) {
    uart_write_blocking(UART_ESP, payload, payloadLen);
  }
  uartSendByte(sum);

  Serial.print(F("[RP2040] UART->ESP len="));
  Serial.print(len);
  Serial.print(F(" tipo=0x"));
  if (type < 0x10) Serial.print('0');
  Serial.println(type, HEX);
}

// ================== Declaración de funciones remotas ==================

bool doRemoteStatus(uint8_t dev);
bool doRemoteRead(uint8_t dev, uint16_t sec);
bool doRemoteFormat(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2);
bool doRemotePercomRead(uint8_t dev);
bool doRemoteWrite(uint8_t dev, uint16_t sec, uint8_t cmd, bool dd, const uint8_t *data, int len);

// ================== Procesar frame recibido del MASTER ==================

void onMasterFrame(uint8_t type, const uint8_t *data, uint8_t len) {
  switch (type) {
    case TYPE_HELLO: {
      if (len >= 2) {
        uint8_t devId  = data[0];
        uint8_t sup256 = data[1];
        Serial.print(F("[RP2040] HELLO SLAVE DEV=0x"));
        if (devId < 0x10) Serial.print('0');
        Serial.print(devId, HEX);
        Serial.print(F(" DD="));
        Serial.println(sup256 ? 1 : 0);
      }
    } break;

    case TYPE_ACK: {
      Serial.println(F("[RP2040] ACK desde SLAVE (MASTER)"));
      // Para WRITE, el ACK indica que la escritura fue correcta
      if (g_currentOp == OP_WRITE) {
        g_writeDone    = true;
        g_writeSuccess = true;
        g_currentOp    = OP_NONE;
      }
    } break;

    case TYPE_NAK: {
      Serial.println(F("[RP2040] NAK desde SLAVE (MASTER)"));
      if (g_currentOp == OP_STATUS) {
        g_statusDone    = true;
        g_statusSuccess = false;
      } else if (g_currentOp == OP_READ) {
        g_readDone    = true;
        g_readSuccess = false;
      } else if (g_currentOp == OP_FORMAT) {
        g_formatDone    = true;
        g_formatSuccess = false;
      } else if (g_currentOp == OP_PERCOM) {
        g_percomDone    = true;
        g_percomSuccess = false;
      } else if (g_currentOp == OP_WRITE) {
        g_writeDone    = true;
        g_writeSuccess = false;
      }
      g_currentOp = OP_NONE;
    } break;

    case TYPE_SECTOR_CHUNK: {
      if (len < 5) {
        Serial.println(F("[RP2040] SECTOR_CHUNK demasiado corto"));
        return;
      }

      uint8_t dev    = data[0];
      uint16_t sec   = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
      uint8_t idx    = data[3];
      uint8_t count  = data[4];
      int     dlen   = len - 5;
      const uint8_t *payload = data + 5;

      Serial.print(F("[RP2040] SECTOR_CHUNK dev=0x"));
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

      // STATUS: sec=0, 4 bytes
      if (g_currentOp == OP_STATUS && sec == 0) {
        int copyLen = (dlen > 4) ? 4 : dlen;
        memcpy(g_statusData, payload, copyLen);
        g_statusDone    = true;
        g_statusSuccess = (copyLen == 4);
        g_currentOp     = OP_NONE;
        if (!g_statusSuccess) {
          Serial.println(F("[RP2040] STATUS: longitud distinta de 4"));
        }
        return;
      }

      // READ: sec == g_readSec
      if (g_currentOp == OP_READ && sec == g_readSec) {
        if (idx == 0) {
          g_readLen        = 0;
          g_readChunkCount = count;
          g_readChunksSeen = 0;
        }

        if (g_readLen + dlen > (int)sizeof(g_readBuf)) {
          dlen = sizeof(g_readBuf) - g_readLen;
        }
        if (dlen > 0) {
          memcpy(g_readBuf + g_readLen, payload, dlen);
          g_readLen += dlen;
        }
        g_readChunksSeen++;

        if (g_readChunksSeen >= g_readChunkCount) {
          g_readDone    = true;
          g_readSuccess = true;
          g_currentOp   = OP_NONE;
          Serial.print(F("[RP2040] READ completo sec="));
          Serial.print(sec);
          Serial.print(F(" len="));
          Serial.println(g_readLen);
        }
        return;
      }

      // FORMAT: sec=0, 128 bytes
      if (g_currentOp == OP_FORMAT && sec == 0) {
        int copyLen = (dlen > (int)sizeof(g_formatBuf)) ? sizeof(g_formatBuf) : dlen;
        memcpy(g_formatBuf, payload, copyLen);
        g_formatLen      = copyLen;
        g_formatDone     = true;
        g_formatSuccess  = (copyLen > 0);
        g_currentOp      = OP_NONE;

        if (!g_formatSuccess) {
          Serial.println(F("[RP2040] FORMAT: longitud 0"));
        } else {
          Serial.print(F("[RP2040] FORMAT resultado len="));
          Serial.println(g_formatLen);
        }
        return;
      }

      // PERCOM: sector mágico 0xFFFF, 12 bytes
      if (g_currentOp == OP_PERCOM && sec == PERCOM_SEC_MAGIC) {
        int copyLen = (dlen > PERCOM_BLOCK_LEN) ? PERCOM_BLOCK_LEN : dlen;
        memcpy(g_percomBuf, payload, copyLen);
        g_percomDone     = true;
        g_percomSuccess  = (copyLen == PERCOM_BLOCK_LEN);
        g_currentOp      = OP_NONE;

        if (!g_percomSuccess) {
          Serial.println(F("[RP2040] PERCOM: longitud incorrecta"));
        } else {
          Serial.print(F("[RP2040] PERCOM resultado len="));
          Serial.println(copyLen);
        }
        return;
      }

    } break;

    default:
      Serial.print(F("[RP2040] UART RX tipo desconocido 0x"));
      if (type < 0x10) Serial.print('0');
      Serial.println(type, HEX);
      break;
  }
}

// ================== FSM UART desde MASTER ==================

void serviceUartFromMaster() {
  while (uart_is_readable(UART_ESP)) {
    uint8_t b = (uint8_t)uart_getc(UART_ESP);

    switch (uartState) {
      case 0: // Esperar 0x55
        if (b == 0x55) {
          uartState = 1;
        }
        break;

      case 1: // LEN
        uartLen = b;
        uartIdx = 0;
        uartState = 2;
        break;

      case 2: // TYPE + PAYLOAD
        uartBuf[uartIdx++] = b;
        if (uartIdx >= uartLen) {
          uartState = 3;  // siguiente byte = CHK
        }
        break;

      case 3: { // CHK
        uint8_t chk = b;

        uint8_t sum = 0;
        for (uint8_t i = 0; i < uartLen; i++) {
          sum += uartBuf[i];
        }

        if (sum == chk) {
          uint8_t type  = uartBuf[0];
          uint8_t *data = &uartBuf[1];
          uint8_t plen  = uartLen - 1;

          Serial.print(F("[RP UART FSM] frame válido len="));
          Serial.print(uartLen);
          Serial.print(F(" tipo=0x"));
          if (type < 0x10) Serial.print('0');
          Serial.println(type, HEX);

          onMasterFrame(type, data, plen);
        } else {
          Serial.print(F("[RP UART FSM] checksum inválido (chk=0x"));
          if (chk < 0x10) Serial.print('0');
          Serial.print(chk, HEX);
          Serial.print(F(", sum=0x"));
          if (sum < 0x10) Serial.print('0');
          Serial.print(sum, HEX);
          Serial.println(F("), descartando frame."));
        }

        uartState = 0;
      } break;

      default:
        uartState = 0;
        break;
    }
  }
}

// ================== SIO → Atari ==================

void sendAtariData(const uint8_t *buf, int len) {
  // COMPLETE
  delayMicroseconds(T_ACK_TO_COMPLETE);
  SerialSIO.write(SIO_COMPLETE);
  SerialSIO.flush();
  Serial.println(F("[SIO] COMPLETE enviado"));

  // DATA
  delayMicroseconds(T_COMPLETE_TO_DATA);
  SerialSIO.write(buf, len);
  SerialSIO.flush();
  Serial.print(F("[SIO] DATA enviado len="));
  Serial.println(len);

  // CHK
  delayMicroseconds(T_DATA_TO_CHK);
  uint8_t chk = calcChecksumSIO(buf, len);
  SerialSIO.write(chk);
  SerialSIO.flush();
  Serial.print(F("[SIO] CHK=0x"));
  if (chk < 0x10) Serial.print('0');
  Serial.println(chk, HEX);

  delayMicroseconds(T_CHUNK_DELAY);
}

// ================== Operaciones remotas ==================

bool doRemoteStatus(uint8_t dev) {
  uint8_t payload[6];
  payload[0] = 0x53;      // CMD STATUS
  payload[1] = dev;
  payload[2] = 0;
  payload[3] = 0;
  payload[4] = 0;         // densFlag
  payload[5] = 0;         // prefetch

  g_currentOp     = OP_STATUS;
  g_statusDone    = false;
  g_statusSuccess = false;

  Serial.println(F("[RP2040] Enviando CMD_FRAME STATUS a ESP32..."));
  uartSendFrame(TYPE_CMD_FRAME, payload, 6);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 5000;
  while (!g_statusDone && (millis() - t0) < TIMEOUT_MS) {
    serviceUartFromMaster();
  }
  g_currentOp = OP_NONE;

  if (!g_statusDone || !g_statusSuccess) {
    Serial.println(F("[RP2040] STATUS remoto FALLÓ, no se envía data."));
    return false;
  }

  Serial.println(F("[RP2040] STATUS remoto OK, reenviando al Atari."));
  sendAtariData(g_statusData, 4);
  return true;
}

bool doRemoteRead(uint8_t dev, uint16_t sec) {
  uint8_t payload[6];
  payload[0] = 0x52;                 // CMD READ
  payload[1] = dev;
  payload[2] = (uint8_t)(sec & 0xFF);
  payload[3] = (uint8_t)(sec >> 8);
  payload[4] = 0;                    // densFlag (por ahora SD)
  payload[5] = 1;                    // prefetch básico

  g_currentOp      = OP_READ;
  g_readDone       = false;
  g_readSuccess    = false;
  g_readSec        = sec;
  g_readLen        = 0;
  g_readChunkCount = 0;
  g_readChunksSeen = 0;

  Serial.print(F("[RP2040] Enviando CMD_FRAME READ sec="));
  Serial.print(sec);
  Serial.println(F(" a ESP32..."));
  uartSendFrame(TYPE_CMD_FRAME, payload, 6);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 8000;
  while (!g_readDone && (millis() - t0) < TIMEOUT_MS) {
    serviceUartFromMaster();
  }
  g_currentOp = OP_NONE;

  if (!g_readDone || !g_readSuccess || g_readLen <= 0) {
    Serial.println(F("[RP2040] READ remoto FALLÓ."));
    return false;
  }

  Serial.print(F("[RP2040] READ remoto OK, len="));
  Serial.println(g_readLen);
  sendAtariData(g_readBuf, g_readLen);
  return true;
}

// FORMAT remoto (0x21 / 0x22)
bool doRemoteFormat(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2) {
  uint8_t payload[6];
  payload[0] = cmd;       // 0x21 / 0x22 (y posibles variantes con bit 7)
  payload[1] = dev;
  payload[2] = aux1;      // se preservan aux1/aux2 tal como llegan del Atari
  payload[3] = aux2;

  uint8_t base = (cmd & 0x7F);
  uint8_t densFlag = 0;
  if (base == 0x22) {
    // si quieres marcar DD para la XF551 real
    densFlag = 1;
  }
  payload[4] = densFlag;
  payload[5] = 0;         // prefetch no aplica para FORMAT

  g_currentOp       = OP_FORMAT;
  g_formatDone      = false;
  g_formatSuccess   = false;
  g_formatLen       = 0;

  Serial.print(F("[RP2040] Enviando CMD_FRAME FORMAT cmd=0x"));
  if (cmd < 0x10) Serial.print('0');
  Serial.print(cmd, HEX);
  Serial.print(F(" dev=0x"));
  if (dev < 0x10) Serial.print('0');
  Serial.print(dev, HEX);
  Serial.print(F(" aux1=0x"));
  if (aux1 < 0x10) Serial.print('0');
  Serial.print(aux1, HEX);
  Serial.print(F(" aux2=0x"));
  if (aux2 < 0x10) Serial.print('0');
  Serial.print(aux2, HEX);
  Serial.println(F(" a ESP32..."));

  uartSendFrame(TYPE_CMD_FRAME, payload, 6);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 60000UL; // FORMAT puede tardar
  while (!g_formatDone && (millis() - t0) < TIMEOUT_MS) {
    serviceUartFromMaster();
  }
  g_currentOp = OP_NONE;

  if (!g_formatDone || !g_formatSuccess || g_formatLen <= 0) {
    Serial.println(F("[RP2040] FORMAT remoto FALLÓ."));
    return false;
  }

  Serial.print(F("[RP2040] FORMAT remoto OK, len="));
  Serial.println(g_formatLen);
  sendAtariData(g_formatBuf, g_formatLen);
  return true;
}

// READ PERCOM (0x4E)
bool doRemotePercomRead(uint8_t dev) {
  uint8_t payload[6];
  payload[0] = 0x4E;                        // CMD READ PERCOM
  payload[1] = dev;
  payload[2] = (uint8_t)(PERCOM_SEC_MAGIC & 0xFF);
  payload[3] = (uint8_t)(PERCOM_SEC_MAGIC >> 8);
  payload[4] = 0;                           // densFlag (no aplica)
  payload[5] = 0;                           // prefetch no aplica

  g_currentOp      = OP_PERCOM;
  g_percomDone     = false;
  g_percomSuccess  = false;

  Serial.println(F("[RP2040] Enviando CMD_FRAME READ PERCOM a ESP32..."));
  uartSendFrame(TYPE_CMD_FRAME, payload, 6);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 5000;
  while (!g_percomDone && (millis() - t0) < TIMEOUT_MS) {
    serviceUartFromMaster();
  }
  g_currentOp = OP_NONE;

  if (!g_percomDone || !g_percomSuccess) {
    Serial.println(F("[RP2040] READ PERCOM remoto FALLÓ."));
    return false;
  }

  Serial.println(F("[RP2040] READ PERCOM remoto OK, reenviando al Atari."));
  sendAtariData(g_percomBuf, PERCOM_BLOCK_LEN);
  return true;
}

// WRITE SECTOR (0x50 / 0x57)
bool doRemoteWrite(uint8_t dev, uint16_t sec, uint8_t cmd, bool dd, const uint8_t *data, int len) {
  uint8_t payload[6];
  payload[0] = cmd;                         // 0x50 / 0x57 (y variantes con bit 7)
  payload[1] = dev;
  payload[2] = (uint8_t)(sec & 0xFF);
  payload[3] = (uint8_t)(sec >> 8);
  payload[4] = dd ? 1 : 0;                  // densFlag
  payload[5] = 0;                           // prefetch no aplica

  g_currentOp      = OP_WRITE;
  g_writeDone      = false;
  g_writeSuccess   = false;
  g_writeSec       = sec;

  Serial.print(F("[RP2040] Enviando CMD_FRAME WRITE cmd=0x"));
  if (cmd < 0x10) Serial.print('0');
  Serial.print(cmd, HEX);
  Serial.print(F(" dev=0x"));
  if (dev < 0x10) Serial.print('0');
  Serial.print(dev, HEX);
  Serial.print(F(" sec="));
  Serial.println(sec);

  // 1) Comando WRITE
  uartSendFrame(TYPE_CMD_FRAME, payload, 6);

  // 2) Enviar el bloque de datos como SECTOR_CHUNK idx=0, count=1
  uint8_t chunk[5 + 256];
  chunk[0] = dev;
  chunk[1] = (uint8_t)(sec & 0xFF);
  chunk[2] = (uint8_t)(sec >> 8);
  chunk[3] = 0;        // idx
  chunk[4] = 1;        // count
  memcpy(chunk + 5, data, len);
  uartSendFrame(TYPE_SECTOR_CHUNK, chunk, 5 + (uint8_t)len);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 8000;
  while (!g_writeDone && (millis() - t0) < TIMEOUT_MS) {
    serviceUartFromMaster();
  }
  g_currentOp = OP_NONE;

  if (!g_writeDone || !g_writeSuccess) {
    Serial.println(F("[RP2040] WRITE remoto FALLÓ."));
    return false;
  }

  Serial.println(F("[RP2040] WRITE remoto OK."));
  return true;
}

// ================== SIO: lectura del frame ==================

bool readSioCommandFrame(uint8_t buf[5]) {
  for (int i = 0; i < 5; i++) {
    if (!readByteWithTimeoutSIO(buf[i], 5)) {
      Serial.print(F("[SIO] Timeout leyendo byte "));
      Serial.println(i);
      return false;
    }
  }
  dumpCommandFrame(buf);
  return true;
}

// ================== Manejo del comando SIO ==================

void handleSioCommand() {
  if (!readSioCommandFrame(cmdBuf)) {
    return;
  }

  uint8_t dev  = cmdBuf[0];
  uint8_t cmd  = cmdBuf[1];
  uint8_t aux1 = cmdBuf[2];
  uint8_t aux2 = cmdBuf[3];

  if (dev != SIO_DEV_D1) {
    Serial.print(F("[RP2040] DEV distinto de D1: 0x"));
    if (dev < 0x10) Serial.print('0');
    Serial.println(dev, HEX);
    return;
  }

  uint8_t base = cmd & 0x7F;
  uint16_t sec = (uint16_t)aux1 | ((uint16_t)aux2 << 8);

  // STATUS
  if (base == 0x53) {
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.println(F("[SIO] ACK enviado (STATUS)"));
    delayMicroseconds(800);
    doRemoteStatus(dev);
    return;
  }

  // READ PERCOM (0x4E)
  if (base == 0x4E) {
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.println(F("[SIO] ACK enviado (READ PERCOM)"));
    delayMicroseconds(800);
    doRemotePercomRead(dev);
    return;
  }

  // READ SECTOR
  if (base == 0x52) {
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.print(F("[SIO] ACK enviado (READ) sec="));
    Serial.println(sec);
    delayMicroseconds(800);
    doRemoteRead(dev, sec);
    return;
  }

  // FORMAT (0x21 / 0x22)
  if (base == 0x21 || base == 0x22) {
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.print(F("[SIO] ACK enviado (FORMAT cmd=0x"));
    if (cmd < 0x10) Serial.print('0');
    Serial.print(cmd, HEX);
    Serial.print(F(") aux1=0x"));
    if (aux1 < 0x10) Serial.print('0');
    Serial.print(aux1, HEX);
    Serial.print(F(" aux2=0x"));
    if (aux2 < 0x10) Serial.print('0');
    Serial.println(aux2, HEX);

    // El Atari ahora esperará el bloque de resultado (128 bytes)
    delayMicroseconds(800);
    doRemoteFormat(dev, cmd, aux1, aux2);
    return;
  }

  // WRITE SECTOR (0x50 / 0x57)
  if (base == 0x50 || base == 0x57) {
    bool dd = (base == 0x57);
    int expectedLen = dd ? 256 : 128;
    uint8_t dataBuf[256];

    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    Serial.print(F("[SIO] ACK enviado (WRITE cmd=0x"));
    if (cmd < 0x10) Serial.print('0');
    Serial.print(cmd, HEX);
    Serial.print(F(") sec="));
    Serial.println(sec);

    // Pequeña pausa antes de recibir los datos
    delayMicroseconds(800);

    // Leer los datos desde el Atari
    for (int i = 0; i < expectedLen; i++) {
      if (!readByteWithTimeoutSIO(dataBuf[i], 100)) {
        Serial.print(F("[SIO] Timeout leyendo DATA de WRITE en byte "));
        Serial.println(i);
        SerialSIO.write(SIO_ERROR);
        SerialSIO.flush();
        Serial.println(F("[SIO] ERROR enviado (timeout DATA WRITE)"));
        return;
      }
    }

    // Leer checksum
    uint8_t chkRecv;
    if (!readByteWithTimeoutSIO(chkRecv, 100)) {
      Serial.println(F("[SIO] Timeout leyendo CHK de WRITE"));
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      Serial.println(F("[SIO] ERROR enviado (timeout CHK WRITE)"));
      return;
    }

    uint8_t chkCalc = calcChecksumSIO(dataBuf, expectedLen);
    Serial.print(F("[SIO] WRITE CHK recv=0x"));
    if (chkRecv < 0x10) Serial.print('0');
    Serial.print(chkRecv, HEX);
    Serial.print(F(" calc=0x"));
    if (chkCalc < 0x10) Serial.print('0');
    Serial.println(chkCalc, HEX);

    if (chkRecv != chkCalc) {
      Serial.println(F("[SIO] Checksum WRITE inválido, enviando ERROR"));
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      return;
    }

    // Enviar WRITE remoto a MASTER/ESCLAVO
    if (doRemoteWrite(dev, sec, cmd, dd, dataBuf, expectedLen)) {
      // Si todo OK, devolvemos COMPLETE al Atari
      delayMicroseconds(T_ACK_TO_COMPLETE);
      SerialSIO.write(SIO_COMPLETE);
      SerialSIO.flush();
      Serial.println(F("[SIO] COMPLETE enviado (WRITE OK)"));
    } else {
      Serial.println(F("[SIO] WRITE remoto FALLÓ, enviando ERROR"));
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
    }

    return;
  }

  // Otros comandos aún no implementados – NAK
  delayMicroseconds(800);
  SerialSIO.write(SIO_NAK);
  SerialSIO.flush();
  Serial.print(F("[SIO] CMD no soportado base=0x"));
  if (base < 0x10) Serial.print('0');
  Serial.println(base, HEX);
}

// ================== SETUP / LOOP ==================

void setup() {
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println(F("\n[RP2040] Frente SIO + UART MASTER listo (RP2 Nano)"));

  // SIO a 19200 8N1 en Serial1 (pines 0/1)
  SerialSIO.begin(19200);

  pinMode(PIN_CMD, INPUT_PULLUP);

  // UART1 hacia ESP32 MASTER en pines 4/5
  uart_init(UART_ESP, UART_ESP_BAUD);
  gpio_set_function(PIN_ESP_TX, GPIO_FUNC_UART);
  gpio_set_function(PIN_ESP_RX, GPIO_FUNC_UART);

  lastCmdState = digitalRead(PIN_CMD);

  digitalWrite(LED_STATUS, HIGH);
}

void loop() {
  // 1) Procesar frames que llegan del MASTER
  serviceUartFromMaster();

  // 2) Detectar flanco CMD ↓ desde el Atari
  int cmdState = digitalRead(PIN_CMD);
  if (lastCmdState == HIGH && cmdState == LOW) {
    Serial.println(F("[SIO] CMD ↓ detectado, leyendo frame de comando..."));
    handleSioCommand();
  }
  lastCmdState = cmdState;

  // 3) Drenar bytes sueltos del SIO (opcional)
  while (SerialSIO.available() > 0) {
    uint8_t b = (uint8_t)SerialSIO.read();
    (void)b;
  }

  delay(1);
}
