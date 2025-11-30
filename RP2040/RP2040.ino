// =======================================================
// RP2040 → ATARI SIO BRIDGE → ESP32 MASTER (ESPNOW)
// - Lee comandos SIO del Atari (CMD + DATA)
// - Normaliza frames DEV,CMD,AUX1,AUX2,CHK
// - Para D1..D4 y comandos STATUS (0x53) / READ (0x52):
//     reenvía al ESP32 MASTER por UART
//     recibe STATUS/READ y se lo entrega al Atari con
//     ACK + COMPLETE + DATA + CHK
// =======================================================

#include <Arduino.h>

// ---------- Compatibilidad IRAM_ATTR fuera de ESP32 ----------
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ---------- Pines RP2040 ----------
const int SIO_RX_PIN  = 1;   // Atari DATA OUT → RP2040 RX (Serial1)
const int SIO_TX_PIN  = 2;   // RP2040 TX (Serial1) → Atari DATA IN
const int SIO_CMD_PIN = 3;   // Atari COMMAND → entrada digital

const int LINK_RX_PIN = 8;   // RP2040 RX (Serial2) ← ESP32
const int LINK_TX_PIN = 9;   // RP2040 TX (Serial2) → ESP32

// ---------- UARTS ----------
#define SioSerial   Serial1   // SIO ↔ ATARI
#define LinkSerial  Serial2   // Link ↔ ESP32

// ---------- Protocolo SIO ----------
#define SIO_ACK      0x41
#define SIO_NAK      0x4E
#define SIO_COMPLETE 0x43
#define SIO_ERROR    0x45

// ---------- Rango dispositivos D1..D4 ----------
#define DEV_MIN 0x31
#define DEV_MAX 0x34

// ---------- Protocolo RP2040 ↔ ESP32 ----------
#define FRAME_TYPE_SIO_COMMAND  0x01   // hacia ESP32
#define FRAME_TYPE_RESP_STATUS  0x81   // desde ESP32
#define FRAME_TYPE_RESP_READ    0x82   // desde ESP32

// ---------- Buffers de comando SIO ----------
static volatile bool cmdEdgeLow  = false;
static volatile bool cmdEdgeHigh = false;

static uint8_t cmdBuf[5];
static int     cmdLen = 0;

// ---------- Utils ----------

uint8_t sioChecksum(const uint8_t *d, int len) {
  uint16_t s = 0;
  for (int i = 0; i < len; i++) {
    s += d[i];
    if (s > 0xFF) s = (s & 0xFF) + 1;
  }
  return (uint8_t)(s & 0xFF);
}

void printHexBuf(const char *prefix, const uint8_t *buf, int len) {
  Serial.print(prefix);
  for (int i = 0; i < len; i++) {
    if (i > 0) Serial.print(' ');
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
  }
  Serial.println();
}

// ---------- Normalizar frame SIO (rotando 5 bytes) ----------
// Se reciben cosas como: 00 31 53 00 00  /  84 31 53 00 00
// Queremos encontrar DEV,CMD,AUX1,AUX2,CHK válidos (chk correcto)
bool normalizeSioFrame(const uint8_t in[5], uint8_t out[5]) {
  for (int shift = 0; shift < 5; shift++) {
    for (int i = 0; i < 5; i++) {
      out[i] = in[(i + shift) % 5];
    }
    uint8_t chkCalc = sioChecksum(out, 4);
    uint8_t dev     = out[0];
    if (chkCalc == out[4] && dev >= DEV_MIN && dev <= DEV_MAX) {
      return true;
    }
  }
  return false;
}

// ---------- ISR en cambio de CMD ----------
void IRAM_ATTR onCmdChange() {
  int level = digitalRead(SIO_CMD_PIN);
  if (level == LOW) {
    cmdEdgeLow = true;
  } else {
    cmdEdgeHigh = true;
  }
}

// ---------- Captura de frame SIO en subida de CMD ----------
void captureSioCommand() {
  cmdLen = 0;

  // Pequeño rezago para que terminen de llegar los bytes
  delayMicroseconds(300);

  unsigned long t0 = micros();
  const unsigned long windowUs = 6000; // ventana para leer los 5 bytes

  while (micros() - t0 < windowUs && cmdLen < 5) {
    if (SioSerial.available()) {
      cmdBuf[cmdLen++] = (uint8_t)SioSerial.read();
    }
  }

  if (cmdLen == 5) {
    printHexBuf("cmdBuf (5 bytes): ", cmdBuf, 5);
  } else if (cmdLen > 0) {
    Serial.print("[RP] cmdLen incompleto = ");
    Serial.println(cmdLen);
    printHexBuf("cmdBuf parcial: ", cmdBuf, cmdLen);
  } else {
    Serial.println("[RP] No se recibieron bytes en este comando");
  }
}

// ---------- Enviar comando SIO al ESP32 (MASTER) ----------
void sendSioCommandToMaster(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2, uint8_t chk) {
  LinkSerial.write(0xA5);
  LinkSerial.write((uint8_t)FRAME_TYPE_SIO_COMMAND);
  LinkSerial.write(dev);
  LinkSerial.write(cmd);
  LinkSerial.write(aux1);
  LinkSerial.write(aux2);
  LinkSerial.write(chk);
  LinkSerial.flush();

  uint8_t tmp[4] = { dev, cmd, aux1, aux2 };
  uint8_t calc = sioChecksum(tmp, 4);

  Serial.print("[RP] Enviando comando SIO al ESP32: DEV=0x");
  Serial.print(dev, HEX);
  Serial.print(" CMD=0x");
  Serial.print(cmd, HEX);
  Serial.print(" AUX1=0x");
  Serial.print(aux1, HEX);
  Serial.print(" AUX2=0x");
  Serial.print(aux2, HEX);
  Serial.print(" CHK=0x");
  Serial.print(chk, HEX);
  Serial.print(" (calc=0x");
  Serial.print(calc, HEX);
  Serial.println(")");
}

// ---------- Esperar STATUS desde ESP32 ----------
// Formato: A5 81 status st0 st1 st2 st3
bool waitStatusFromMaster(uint8_t out4[4], uint32_t timeoutMs = 1000) {
  unsigned long t0 = millis();
  enum State { WAIT_A5, WAIT_TYPE, WAIT_STATUS, WAIT_DATA };
  State st = WAIT_A5;

  uint8_t type = 0;
  uint8_t statusByte = 0;
  int idx = 0;

  while (millis() - t0 < timeoutMs) {
    if (!LinkSerial.available()) {
      delay(1);
      continue;
    }
    uint8_t b = (uint8_t)LinkSerial.read();

    switch (st) {
      case WAIT_A5:
        if (b == 0xA5) st = WAIT_TYPE;
        break;

      case WAIT_TYPE:
        type = b;
        if (type == FRAME_TYPE_RESP_STATUS) {
          st = WAIT_STATUS;
        } else {
          // otro tipo → descartamos y volvemos a buscar 0xA5
          st = WAIT_A5;
        }
        break;

      case WAIT_STATUS:
        statusByte = b;
        if (statusByte != 0x00) {
          Serial.print("[RP] STATUS con error desde ESP32: 0x");
          Serial.println(statusByte, HEX);
          return false;
        }
        st = WAIT_DATA;
        idx = 0;
        break;

      case WAIT_DATA:
        out4[idx++] = b;
        if (idx >= 4) {
          return true;
        }
        break;
    }
  }

  Serial.println("[RP] Timeout esperando STATUS desde ESP32");
  return false;
}

// ---------- Esperar READ desde ESP32 ----------
// Formato: A5 82 status len data[len]
int waitReadFromMaster(uint8_t *out, int maxLen, uint32_t timeoutMs = 8000) {
  unsigned long t0 = millis();
  enum State { WAIT_A5, WAIT_TYPE, WAIT_STATUS, WAIT_LEN, WAIT_DATA };
  State st = WAIT_A5;

  uint8_t type = 0;
  uint8_t statusByte = 0;
  uint8_t lenRecv = 0;
  int idx = 0;

  while (millis() - t0 < timeoutMs) {
    if (!LinkSerial.available()) {
      delay(1);
      continue;
    }
    uint8_t b = (uint8_t)LinkSerial.read();

    switch (st) {
      case WAIT_A5:
        if (b == 0xA5) st = WAIT_TYPE;
        break;

      case WAIT_TYPE:
        type = b;
        if (type == FRAME_TYPE_RESP_READ) {
          st = WAIT_STATUS;
        } else {
          // otro tipo → ignorar y seguir
          st = WAIT_A5;
        }
        break;

      case WAIT_STATUS:
        statusByte = b;
        if (statusByte != 0x00) {
          Serial.print("[RP] READ con error desde ESP32: 0x");
          Serial.println(statusByte, HEX);
          return 0;
        }
        st = WAIT_LEN;
        break;

      case WAIT_LEN:
        lenRecv = b;
        if (lenRecv == 0) {
          Serial.println("[RP] READ desde ESP32 con len=0");
          return 0;
        }
        if (lenRecv > maxLen) {
          Serial.print("[RP] READ len=");
          Serial.print(lenRecv);
          Serial.print(" > max=");
          Serial.print(maxLen);
          Serial.println(", se recorta");
        }
        st = WAIT_DATA;
        idx = 0;
        break;

      case WAIT_DATA:
        if (idx < maxLen) {
          out[idx++] = b;
        }
        if (idx >= lenRecv) {
          return idx;
        }
        break;
    }
  }

  Serial.println("[RP] Timeout esperando READ desde ESP32");
  return 0;
}

// ---------- Enviar ACK + COMPLETE + DATA + CHK al Atari ----------
void sendAckCompleteAndDataToAtari(const uint8_t *data, int len, const char *tag) {
  // ACK
  SioSerial.write((uint8_t)SIO_ACK);
  SioSerial.flush();
  delayMicroseconds(300);

  // COMPLETE
  SioSerial.write((uint8_t)SIO_COMPLETE);
  SioSerial.flush();
  delayMicroseconds(300);

  // DATA + CHK
  if (len > 0 && data != nullptr) {
    SioSerial.write(data, len);
    uint8_t chk = sioChecksum(data, len);
    SioSerial.write(chk);
    SioSerial.flush();

    if (tag) {
      Serial.print("[RP] DATA enviada al Atari (");
      Serial.print(tag);
      Serial.print("), chk=0x");
      Serial.println(chk, HEX);
    }
  }
}

// ---------- Handler del comando SIO normalizado ----------
void handleSioCommandNormalized(uint8_t dev, uint8_t cmd, uint8_t aux1, uint8_t aux2, uint8_t chk) {
  uint8_t base = cmd & 0x7F;
  bool dd      = (cmd & 0x80) != 0;
  (void)dd; // por ahora no lo usamos aquí
  uint16_t sec = (uint16_t)aux1 | ((uint16_t)aux2 << 8);

  uint8_t tmp[4] = { dev, cmd, aux1, aux2 };
  uint8_t calcChk = sioChecksum(tmp, 4);

  Serial.print("[RP] Comando SIO normalizado: ");
  Serial.print(dev, HEX);  Serial.print(' ');
  Serial.print(cmd, HEX);  Serial.print(' ');
  Serial.print(aux1, HEX); Serial.print(' ');
  Serial.print(aux2, HEX); Serial.print(' ');
  Serial.print(chk, HEX);
  Serial.print("  | chk calc=0x");
  Serial.print(calcChk, HEX);
  Serial.print(" recv=0x");
  Serial.println(chk, HEX);

  if (calcChk != chk) {
    Serial.println("[RP] Checksum inválido tras normalizar, ignorando");
    return;
  }

  if (dev < DEV_MIN || dev > DEV_MAX) {
    Serial.print("[RP] DEV fuera de rango (0x");
    Serial.print(dev, HEX);
    Serial.println("), ignorando");
    return;
  }

  // STATUS D1..D4
  if (base == 0x53) {
    Serial.println("[RP] STATUS D1..D4 detectado, reenviando a ESP32...");
    sendSioCommandToMaster(dev, cmd, aux1, aux2, chk);

    uint8_t st4[4];
    if (waitStatusFromMaster(st4)) {
      Serial.print("[RP] STATUS desde ESP32: ");
      printHexBuf("", st4, 4);
      Serial.println("[RP] Enviando ACK + STATUS al Atari len=4");
      sendAckCompleteAndDataToAtari(st4, 4, "STATUS");
    } else {
      Serial.println("[RP] Error/timeout STATUS desde ESP32, enviando ERROR al Atari");
      SioSerial.write((uint8_t)SIO_ERROR);
      SioSerial.flush();
    }
    return;
  }

  // READ SECTOR D1..D4
  if (base == 0x52) {
    Serial.println("[RP] READ SECTOR D1 detectado, reenviando a ESP32...");
    sendSioCommandToMaster(dev, cmd, aux1, aux2, chk);

    uint8_t buf[256];
    int expectedLen = dd ? 256 : 128;
    int n = waitReadFromMaster(buf, expectedLen);
    if (n > 0) {
      Serial.print("[RP] READ desde ESP32 len=");
      Serial.println(n);

      Serial.print("[RP] Primeros bytes: ");
      for (int i = 0; i < 16 && i < n; i++) {
        if (i > 0) Serial.print(' ');
        if (buf[i] < 0x10) Serial.print('0');
        Serial.print(buf[i], HEX);
      }
      Serial.println();

      Serial.print("[RP] Enviando ACK + READ al Atari len=");
      Serial.println(n);
      sendAckCompleteAndDataToAtari(buf, n, "READ");
    } else {
      Serial.println("[RP] Error/timeout esperando READ desde ESP32, enviando ERROR al Atari");
      SioSerial.write((uint8_t)SIO_ERROR);
      SioSerial.flush();
    }
    return;
  }

  // Otros comandos por ahora solo se loguean
  Serial.print("[RP] Comando no manejado: DEV=0x");
  Serial.print(dev, HEX);
  Serial.print(" CMD=0x");
  Serial.print(cmd, HEX);
  Serial.print(" (base=0x");
  Serial.print(base, HEX);
  Serial.println(")");
}

// ---------- Procesar frame SIO crudo de 5 bytes ----------
void processRawSioCommand(const uint8_t *raw, int len) {
  if (len != 5) {
    Serial.println("[RP] Frame CMD con len != 5, ignorando");
    return;
  }

  printHexBuf("[RP] Comando SIO crudo: ", raw, 5);

  uint8_t norm[5];
  if (!normalizeSioFrame(raw, norm)) {
    Serial.println("[RP] No se pudo normalizar frame, se ignora.");
    return;
  }

  printHexBuf("[RP] Frame normalizado: ", norm, 5);

  uint8_t dev  = norm[0];
  uint8_t cmd  = norm[1];
  uint8_t aux1 = norm[2];
  uint8_t aux2 = norm[3];
  uint8_t chk  = norm[4];

  handleSioCommandNormalized(dev, cmd, aux1, aux2, chk);
}

// =======================================================
// SETUP / LOOP
// =======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("[RP] RP2040 SIO↔ESP32 bridge (STATUS + READ)");

  // UART SIO hacia Atari
  SioSerial.setRX(SIO_RX_PIN);
  SioSerial.setTX(SIO_TX_PIN);
  SioSerial.begin(19200);

  // UART link hacia ESP32
  LinkSerial.setRX(LINK_RX_PIN);
  LinkSerial.setTX(LINK_TX_PIN);
  LinkSerial.begin(115200);

  pinMode(SIO_CMD_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(SIO_CMD_PIN), onCmdChange, CHANGE);

  Serial.print("[RP] SIO_RX=");
  Serial.print(SIO_RX_PIN);
  Serial.print(" SIO_TX=");
  Serial.print(SIO_TX_PIN);
  Serial.print(" SIO_CMD=");
  Serial.print(SIO_CMD_PIN);
  Serial.print("  | LINK_RX=");
  Serial.print(LINK_RX_PIN);
  Serial.print(" LINK_TX=");
  Serial.println(LINK_TX_PIN);

  Serial.println("[RP] Listo, esperando comandos SIO...");
}

void loop() {
  // Detectar flanco de CMD BAJO
  if (cmdEdgeLow) {
    cmdEdgeLow = false;
    Serial.println();
    Serial.println("=== CMD BAJO: comienzo de comando SIO ===");

    // Aquí limpiamos el UART del SIO para eliminar residuos del comando anterior
    cmdLen = 0;
    while (SioSerial.available()) {
      (void)SioSerial.read();
    }
  }

  // Detectar flanco de CMD ALTO
  if (cmdEdgeHigh) {
    cmdEdgeHigh = false;
    Serial.println();
    Serial.println("=== CMD ALTO: fin de comando SIO (llenando rezagos) ===");
    captureSioCommand();
    if (cmdLen == 5) {
      processRawSioCommand(cmdBuf, cmdLen);
    }
  }

  // No hacemos nada más en loop; todo se dispara por CMD
  delay(1);
}
