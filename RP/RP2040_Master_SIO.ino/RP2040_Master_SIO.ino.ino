/*
  RP2040 MASTER SIO + ESP32 bridge (STATUS + READ SD/DD)
  ------------------------------------------------------
  - Emula D1 ante el Atari:
      * STATUS (0x53)
      * READ SECTOR (0x52)
  - Para cada comando:
      Atari <-> RP2040 <-> UART <-> ESP32 MASTER <-> ESP-NOW <-> SLAVE <-> XF551

  Densidad:
    - USE_DD = false → responde 128 bytes (SD)
    - USE_DD = true  → responde 256 bytes (DD)

  Cableado:
    SIO Atari <-> RP2040:
      CMD SIO       -> divisor -> GPIO2 (PIN_CMD)
      TX SIO (DATA OUT Atari)  -> divisor -> GPIO1 (RX Serial1)
      RX SIO (DATA IN Atari)   <- GPIO0 (TX Serial1) con resistencia ~1k
      GND común

    RP2040 <-> ESP32 MASTER:
      RP GPIO4 (TX Serial2) -> ESP32 GPIO16 (RX2)
      RP GPIO5 (RX Serial2) <- ESP32 GPIO17 (TX2)
*/

#include <Arduino.h>

// ===== Config densidad =====
const bool USE_DD = true;   // false = 128 bytes (SD), true = 256 bytes (DD)

// ===== SIO pins (RP2040) =====
const int PIN_CMD    = 2;    // CMD SIO
const int PIN_SIO_TX = 0;    // TX hacia Atari
const int PIN_SIO_RX = 1;    // RX desde Atari

// UART hacia ESP32
const int PIN_ESP_TX = 4;    // RP TX -> ESP RX2
const int PIN_ESP_RX = 5;    // RP RX <- ESP TX2

// LED de estado
const int LED_STATUS = 13;   // o 25 según placa

// ===== SIO / estado CMD =====
bool    lastCmdState = HIGH;
uint8_t cmdBuf[5];

// ===== Protocolo UART/ESPNOW =====
#define TYPE_CMD_FRAME    0x01
#define TYPE_SECTOR_CHUNK 0x10
#define TYPE_ACK          0x11
#define TYPE_NAK          0x12
#define TYPE_HELLO        0x20

#define CHUNK_PAYLOAD     240
#define MAX_SECTOR_BYTES  256
#define SECTOR_128        128
#define SECTOR_256        256

// UART framing
#define UART_SYNC 0x55

// ===== Buffers para sector =====
static uint8_t sectorBuf[MAX_SECTOR_BYTES];

// ===== Utilidades =====
uint8_t calcChecksum(const uint8_t* buf, int len) {
  uint16_t sum = 0;
  for (int i = 0; i < len; i++) sum += buf[i];
  return (uint8_t)sum;
}

// ===== UART TX -> ESP32 =====
bool sendUartFrameToEsp(const uint8_t* payload, uint8_t len) {
  if (len == 0) return false;
  uint8_t chk = calcChecksum(payload, len);

  Serial2.write(UART_SYNC);
  Serial2.write(len);
  Serial2.write(payload, len);
  Serial2.write(chk);
  Serial2.flush();

  Serial.print(F("[RP2040] UART->ESP len="));
  Serial.print(len);
  Serial.print(F(" tipo=0x"));
  if (payload[0] < 0x10) Serial.print('0');
  Serial.println(payload[0], HEX);

  return true;
}

// ===== UART RX <- ESP32: FSM persistente =====
static uint8_t  uartState    = 0;   // 0=SYNC, 1=LEN, 2=PAYLOAD, 3=CHK
static uint8_t  uartLen      = 0;
static uint8_t  uartIdx      = 0;
static uint32_t uartLastTime = 0;
static uint8_t  uartRxBuf[255];

bool pollUartFrameFromEsp(uint8_t* payload, uint8_t &lenOut, uint16_t frameTimeoutMs) {
  bool frameReady = false;

  while (Serial2.available() > 0) {
    uint8_t b = (uint8_t)Serial2.read();
    uartLastTime = millis();

    Serial.print(F("[RP UART FSM] state="));
    Serial.print(uartState);
    Serial.print(F(" byte=0x"));
    if (b < 0x10) Serial.print('0');
    Serial.println(b, HEX);

    switch (uartState) {
      case 0: // Esperando SYNC
        if (b == UART_SYNC) {
          uartState = 1;
        }
        break;

      case 1: // LEN
        uartLen = b;
        if (uartLen == 0) {
          uartState = 0;
        } else {
          uartIdx   = 0;
          uartState = 2;
        }
        break;

      case 2: // PAYLOAD
        uartRxBuf[uartIdx++] = b;
        if (uartIdx >= uartLen) {
          uartState = 3;
        }
        break;

      case 3: { // CHK
        uint8_t chk = b;
        uint8_t sum = calcChecksum(uartRxBuf, uartLen);
        if (chk != sum) {
          Serial.print(F("[RP2040] UART: checksum inválido (chk=0x"));
          if (chk < 0x10) Serial.print('0');
          Serial.print(chk, HEX);
          Serial.print(F(", sum=0x"));
          if (sum < 0x10) Serial.print('0');
          Serial.print(sum, HEX);
          Serial.println(F("), descartando frame."));
          uartState = 0;
        } else {
          lenOut = uartLen;
          memcpy(payload, uartRxBuf, uartLen);
          Serial.print(F("[RP2040] UART RX frame válido len="));
          Serial.print(uartLen);
          Serial.print(F(" tipo=0x"));
          if (payload[0] < 0x10) Serial.print('0');
          Serial.println(payload[0], HEX);
          uartState = 0;
          frameReady = true;
        }
        break;
      }
    }
  }

  // Timeout entre bytes
  if (uartState != 0 && (millis() - uartLastTime > frameTimeoutMs)) {
    Serial.println(F("[RP2040] UART: timeout mid-frame, reseteando FSM."));
    uartState = 0;
    uartIdx   = 0;
    uartLen   = 0;
  }

  return frameReady;
}

// ===== Debug SIO =====
void dumpCommandFrame(const uint8_t *buf) {
  uint8_t dev  = buf[0];
  uint8_t cmd  = buf[1];
  uint8_t aux1 = buf[2];
  uint8_t aux2 = buf[3];
  uint8_t chk  = buf[4];
  uint8_t calc = calcChecksum(buf, 4);

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

  if (dev == 0x31) {
    Serial.println(F("[SIO] → Comando dirigido a D1: (0x31)"));
  } else {
    Serial.print(F("[SIO] → Dispositivo distinto de D1: 0x"));
    if (dev < 0x10) Serial.print('0');
    Serial.println(dev, HEX);
  }
}

// ===== Lee comando SIO con realineación =====
bool readCommandFrame() {
  Serial.println(F("[SIO] CMD ↓ detectado, leyendo frame de comando..."));

  const int FRAME_LEN = 5;
  int count = 0;
  while (count < FRAME_LEN) {
    unsigned long start = millis();
    while (!Serial1.available() && (millis() - start < 10)) {}
    if (!Serial1.available()) {
      Serial.print(F("[SIO] Timeout leyendo byte inicial "));
      Serial.println(count);
      return false;
    }
    cmdBuf[count++] = (uint8_t)Serial1.read();
  }

  for (int attempt = 0; attempt < 4; attempt++) {
    uint8_t dev  = cmdBuf[0];
    uint8_t cmd  = cmdBuf[1];
    uint8_t aux1 = cmdBuf[2];
    uint8_t aux2 = cmdBuf[3];
    uint8_t chk  = cmdBuf[4];
    uint8_t calc = calcChecksum(cmdBuf, 4);

    Serial.print(F("[SIO] Intento "));
    Serial.print(attempt);
    Serial.print(F(" -> DEV=0x"));
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

    if (chk == calc) {
      Serial.println(F("[SIO] Frame CMD alineado correctamente."));
      dumpCommandFrame(cmdBuf);
      return true;
    }

    if (attempt < 3) {
      Serial.println(F("[SIO] Checksum inválido, intentando realinear frame..."));
      cmdBuf[0] = cmdBuf[1];
      cmdBuf[1] = cmdBuf[2];
      cmdBuf[2] = cmdBuf[3];
      cmdBuf[3] = cmdBuf[4];

      unsigned long start = millis();
      while (!Serial1.available() && (millis() - start < 10)) {}
      if (!Serial1.available()) {
        Serial.println(F("[SIO] Timeout leyendo byte extra para realineación."));
        return false;
      }
      cmdBuf[4] = (uint8_t)Serial1.read();
    }
  }

  Serial.println(F("[SIO] No se encontró frame con checksum válido tras realinear."));
  return false;
}

// ===== Envía al Atari ACK + DATA + CHK + COMPLETE =====
void sioSendDataFrame(const uint8_t* data, int len) {
  uint8_t chk = calcChecksum(data, len);

  delayMicroseconds(1500);
  Serial1.write(0x41); // ACK

  delayMicroseconds(1500);
  Serial1.write(data, len);
  Serial1.write(chk);

  delayMicroseconds(1500);
  Serial1.write(0x43); // COMPLETE

  Serial.print(F("[SIO] DATA enviado len="));
  Serial.print(len);
  Serial.print(F(" CHK=0x"));
  if (chk < 0x10) Serial.print('0');
  Serial.println(chk, HEX);
}

// ===== Lógica remota STATUS =====
bool remoteStatus(uint8_t dev, uint8_t *outStatus4) {
  uint8_t tx[7];
  tx[0] = TYPE_CMD_FRAME;
  tx[1] = 0x53;     // STATUS
  tx[2] = dev;
  tx[3] = 0x00;
  tx[4] = 0x00;
  tx[5] = 0;
  tx[6] = 1;

  Serial.println(F("[RP2040] Enviando CMD_FRAME STATUS a ESP32..."));
  sendUartFrameToEsp(tx, 7);

  bool gotChunk = false;
  unsigned long t0 = millis();

  while (millis() - t0 < 5000) {
    uint8_t buf[255];
    uint8_t len;
    if (!pollUartFrameFromEsp(buf, len, 50)) {
      delay(5);
      continue;
    }

    uint8_t type = buf[0];

    if (type == TYPE_ACK && len >= 2) {
      if (buf[1] == dev)
        Serial.println(F("[RP2040] STATUS: ACK recibido desde SLAVE (vía ESP)."));
    }
    else if (type == TYPE_NAK && len >= 2) {
      if (buf[1] == dev) {
        Serial.println(F("[RP2040] STATUS: NAK recibido desde SLAVE."));
        return false;
      }
    }
    else if (type == TYPE_SECTOR_CHUNK && len >= 6) {
      uint8_t rdev = buf[1];
      if (rdev != dev) continue;

      uint16_t sec = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
      if (sec != 0) continue;

      uint8_t chunkIdx   = buf[4];
      uint8_t chunkCount = buf[5];
      int dataLen        = len - 6;

      Serial.print(F("[RP2040] STATUS: chunk "));
      Serial.print(chunkIdx);
      Serial.print(F("/"));
      Serial.print(chunkCount);
      Serial.print(F(" dataLen="));
      Serial.println(dataLen);

      if (chunkIdx == 0 && dataLen >= 4) {
        memcpy(outStatus4, buf + 6, 4);
        gotChunk = true;
        break;
      }
    }
    else {
      Serial.print(F("[RP2040] UART RX tipo desconocido 0x"));
      if (type < 0x10) Serial.print('0');
      Serial.println(type, HEX);
    }
  }

  if (!gotChunk) {
    Serial.println(F("[RP2040] STATUS: no se recibió bloque de 4 bytes."));
    return false;
  }
  return true;
}

// ===== Lógica remota READ SD / DD =====
bool remoteRead(uint8_t dev, uint16_t sector, bool dd, uint8_t *outBuf, int expectedLen) {
  uint8_t tx[7];
  tx[0] = TYPE_CMD_FRAME;
  tx[1] = 0x52; // READ
  tx[2] = dev;
  tx[3] = (uint8_t)(sector & 0xFF);
  tx[4] = (uint8_t)(sector >> 8);
  tx[5] = dd ? 1 : 0;
  tx[6] = 1;

  Serial.print(F("[RP2040] Enviando CMD_FRAME READ sec="));
  Serial.print(sector);
  Serial.print(F(" ("));
  Serial.print(dd ? F("DD 256") : F("SD 128"));
  Serial.println(F(")"));
  sendUartFrameToEsp(tx, 7);

  bool ok = false;
  uint8_t tmp[MAX_SECTOR_BYTES];
  int finalLen        = 0;
  uint8_t chunkCount  = 0;
  uint8_t chunksGot   = 0;

  unsigned long t0 = millis();

  while (millis() - t0 < 8000) {
    uint8_t buf[255];
    uint8_t len;
    if (!pollUartFrameFromEsp(buf, len, 50)) {
      delay(5);
      continue;
    }

    uint8_t type = buf[0];

    if (type == TYPE_ACK && len >= 2) {
      if (buf[1] == dev)
        Serial.println(F("[RP2040] READ: ACK recibido desde SLAVE."));
    }
    else if (type == TYPE_NAK && len >= 2) {
      if (buf[1] == dev) {
        Serial.println(F("[RP2040] READ: NAK recibido desde SLAVE."));
        return false;
      }
    }
    else if (type == TYPE_SECTOR_CHUNK && len >= 6) {
      uint8_t rdev = buf[1];
      if (rdev != dev) continue;

      uint16_t rsec = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
      if (rsec != sector) continue;

      uint8_t idx = buf[4];
      uint8_t cc  = buf[5];
      int dataLen = len - 6;

      if (idx == 0) {
        memset(tmp, 0, MAX_SECTOR_BYTES);
        finalLen   = 0;
        chunkCount = cc;
        chunksGot  = 0;
      }

      int off = idx * CHUNK_PAYLOAD;
      if (off + dataLen > MAX_SECTOR_BYTES) {
        dataLen = MAX_SECTOR_BYTES - off;
      }
      memcpy(tmp + off, buf + 6, dataLen);

      chunksGot++;
      finalLen = off + dataLen;

      Serial.print(F("[RP2040] READ: chunk "));
      Serial.print(idx);
      Serial.print(F("/"));
      Serial.print(cc);
      Serial.print(F(" dataLen="));
      Serial.print(dataLen);
      Serial.print(F(" finalLen="));
      Serial.println(finalLen);

      if (chunksGot >= cc) {
        ok = true;
        break;
      }
    }
    else {
      Serial.print(F("[RP2040] UART RX tipo desconocido 0x"));
      if (type < 0x10) Serial.print('0');
      Serial.println(type, HEX);
    }
  }

  if (!ok) {
    Serial.println(F("[RP2040] READ: no se recibieron todos los chunks."));
    return false;
  }

  if (finalLen > expectedLen) finalLen = expectedLen;
  memcpy(outBuf, tmp, finalLen);
  return true;
}

// ===== SETUP & LOOP =====

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("\n=== RP2040 MASTER SIO + ESP32 bridge (STATUS + READ SD/DD) ==="));
  Serial.print(F("[CONF] USE_DD = "));
  Serial.println(USE_DD ? F("true (256 bytes)") : F("false (128 bytes)"));

  pinMode(PIN_CMD, INPUT_PULLUP);
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  Serial1.setRX(PIN_SIO_RX);
  Serial1.setTX(PIN_SIO_TX);
  Serial1.begin(19200);

  Serial2.setTX(PIN_ESP_TX);
  Serial2.setRX(PIN_ESP_RX);
  Serial2.begin(115200);

  lastCmdState = digitalRead(PIN_CMD);
}

void loop() {
  bool cmdState = digitalRead(PIN_CMD);

  if (lastCmdState == HIGH && cmdState == LOW) {
    digitalWrite(LED_STATUS, HIGH);

    if (readCommandFrame()) {
      uint8_t dev  = cmdBuf[0];
      uint8_t cmd  = cmdBuf[1];
      uint8_t aux1 = cmdBuf[2];
      uint8_t aux2 = cmdBuf[3];

      if (dev == 0x31) { // D1
        if (cmd == 0x53) {
          uint8_t st[4];
          if (remoteStatus(dev, st)) {
            Serial.println(F("[RP2040] STATUS remoto OK, reenviando al Atari."));
            sioSendDataFrame(st, 4);
          } else {
            Serial.println(F("[RP2040] STATUS remoto FALLÓ, no se envía data."));
          }
        }
        else if (cmd == 0x52) { // READ
          uint16_t sector = (uint16_t)aux1 | ((uint16_t)aux2 << 8);
          if (USE_DD) {
            if (remoteRead(dev, sector, true, sectorBuf, SECTOR_256)) {
              Serial.println(F("[RP2040] READ remoto DD OK, reenviando 256 bytes al Atari."));
              sioSendDataFrame(sectorBuf, SECTOR_256);
            } else {
              Serial.println(F("[RP2040] READ remoto DD FALLÓ."));
            }
          } else {
            if (remoteRead(dev, sector, false, sectorBuf, SECTOR_128)) {
              Serial.println(F("[RP2040] READ remoto SD OK, reenviando 128 bytes al Atari."));
              sioSendDataFrame(sectorBuf, SECTOR_128);
            } else {
              Serial.println(F("[RP2040] READ remoto SD FALLÓ."));
            }
          }
        }
        else {
          Serial.print(F("[RP2040] CMD no manejado para D1: 0x"));
          if (cmd < 0x10) Serial.print('0');
          Serial.println(cmd, HEX);
        }
      } else {
        Serial.print(F("[RP2040] DEV distinto de D1: 0x"));
        if (dev < 0x10) Serial.print('0');
        Serial.println(dev, HEX);
      }
    }

    digitalWrite(LED_STATUS, LOW);
  }

  lastCmdState = cmdState;
}
