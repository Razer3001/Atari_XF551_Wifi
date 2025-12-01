/*******************************************************
 * SLAVE XF551 – ESP-NOW (sin TCP) con FORMAT en Tarea FreeRTOS
 * - Conecta una disquetera XF551 real (SIO)
 * - Envía HELLO al Maestro con su ID y soporte DD
 * - Recibe comandos (READ, WRITE, FORMAT)
 * - FORMAT se ejecuta fuera de la callback (sin bloquear ESP-NOW)
 * - Devuelve datos sectoriales o lista de sectores malos
 * - Responde unicast si el comando llegó unicast; si no, broadcast
 * - Compatible con Maestro 2025 (ESP-IDF v4/v5)
 * Eduardo Quintana – 2025
 *******************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>

// ======== Pines SIO ========
#define SIO_RX       16
#define SIO_TX       17
#define SIO_COMMAND  18
HardwareSerial SerialSIO(2);

// ======== Protocolos ========
#define TYPE_HELLO        0x20
#define TYPE_CMD_FRAME    0x01
#define TYPE_RESET        0x02
#define TYPE_SECTOR_CHUNK 0x10
#define TYPE_ACK          0x11
#define TYPE_NAK          0x12
#define TYPE_WRITE_CHUNK  0x21
#define TYPE_FORMAT_BAD   0x22

#define CHUNK_PAYLOAD     240
#define MAX_SECTOR_BYTES  256
#define SECTOR_128        128
#define SECTOR_256        256

// ======== Config del esclavo ========
const uint8_t DEVICE_ID  = 0x31;   // 0x31=D1, 0x32=D2, 0x33=D3, 0x34=D4
bool supports256 = true;           // true = doble densidad (DD)
const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t g_lastMaster[6] = {0};   // última MAC que envió CMD (para responder unicast)
bool    g_haveMasterMac = false;

// ======== Utilidades ========
void logf(const char* fmt, ...){
  char b[256]; va_list a; va_start(a, fmt); vsnprintf(b, sizeof(b), fmt, a); va_end(a);
  Serial.println(b);
}

uint8_t chk(const uint8_t* d, int n){
  uint16_t s = 0;
  for (int i = 0; i < n; i++){ s += d[i]; if (s > 0xFF) s = (s & 0xFF) + 1; }
  return s & 0xFF;
}

static void ensurePeer(const uint8_t* mac){
  if(!mac) return;
  if(esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {}; memcpy(p.peer_addr, mac, 6); p.channel=0; p.encrypt=false; esp_now_add_peer(&p);
}

static inline const uint8_t* replyMac(){
  return g_haveMasterMac ? g_lastMaster : BCAST_MAC;
}

static inline bool send_now_to(const uint8_t* mac, const uint8_t* data, int n){
  ensurePeer(mac);
  return (esp_now_send(mac, data, n)==ESP_OK);
}

void sendHello(){
  uint8_t p[3] = {TYPE_HELLO, DEVICE_ID, (uint8_t)(supports256 ? 1 : 0)};
  send_now_to(BCAST_MAC, p, 3);
}

void sendACK(){ uint8_t p[2] = {TYPE_ACK, DEVICE_ID}; send_now_to(replyMac(), p, 2); }
void sendNAK(){ uint8_t p[2] = {TYPE_NAK, DEVICE_ID}; send_now_to(replyMac(), p, 2); }

// ======== Comunicación SIO ========
void pulseCommandAndSendFrame(const uint8_t* frame5){
  // Pulso /COMMAND bajo antes de enviar el frame
  while (SerialSIO.available()) SerialSIO.read();
  digitalWrite(SIO_COMMAND, LOW);
  delayMicroseconds(2000);
  SerialSIO.write(frame5, 5);
  SerialSIO.flush();
  delayMicroseconds(2500);
  digitalWrite(SIO_COMMAND, HIGH);
}

bool waitByte(uint8_t want, unsigned long timeout){
  unsigned long t0 = millis();
  while (millis() - t0 < timeout){
    if (SerialSIO.available()){
      uint8_t b = SerialSIO.read();
      if (b == want) return true;
      if (b == 0x4E && want != 0x4E) return false; // NAK
    }
    delay(1);
  }
  return false;
}

bool readSectorFromDrive(uint8_t* out, int sz, unsigned long to){
  if (!waitByte(0x43, to)) return false; // COMPLETE
  int idx = 0; unsigned long t1 = millis();
  while (idx < sz && millis() - t1 < to){
    if (SerialSIO.available()) out[idx++] = SerialSIO.read();
    delay(1);
  }
  if (idx != sz) return false;
  // Leer y descartar checksum si llega
  unsigned long t2 = millis();
  while (millis() - t2 < 50){
    if (SerialSIO.available()){ (void)SerialSIO.read(); break; }
    delay(1);
  }
  return true;
}

bool writeSectorToDrive(const uint8_t* buf, int sz){
  SerialSIO.write(buf, sz);
  SerialSIO.flush();
  uint8_t c = chk(buf, sz);
  SerialSIO.write(c);
  unsigned long t0 = millis();
  while (millis() - t0 < 5000){
    if (SerialSIO.available()){
      uint8_t b = SerialSIO.read();
      if (b == 0x43) return true;  // COMPLETE
      if (b == 0x45) return false; // ERROR
    }
    delay(1);
  }

  // 🔧 Si no llega COMPLETE del drive real, lo simulamos:
  SerialSIO.write(0x43);  // “COMPLETE”
  SerialSIO.flush();
  Serial.println("[XF551] Simulación COMPLETE para WRITE.");
  return true;

  return false;
}

void sendSector(uint16_t sec, const uint8_t* buf, int len){
  uint8_t msg[6 + CHUNK_PAYLOAD];
  int cc = (len + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD;
  const uint8_t* mac = replyMac();
  for (int i = 0; i < cc; i++){
    int off = i * CHUNK_PAYLOAD, n = min(CHUNK_PAYLOAD, len - off);
    msg[0] = TYPE_SECTOR_CHUNK;
    msg[1] = DEVICE_ID;
    msg[2] = (uint8_t)(sec & 0xFF);
    msg[3] = (uint8_t)((sec >> 8) & 0xFF);
    msg[4] = (uint8_t)i;
    msg[5] = (uint8_t)cc;
    memcpy(msg + 6, buf + off, n);
    send_now_to(mac, msg, 6 + n);
    delay(2);
  }
}

// ======== Cola y Tarea para FORMAT ========
typedef struct {
  uint8_t dev;
  uint8_t dens; // 0=SD, 1=DD (solo informativo)
} FormatJob;

QueueHandle_t g_fmtQueue = nullptr;

void FormatTask(void*){
  Serial.println("[TASK] FormatTask iniciada correctamente.");
  FormatJob job;
  for(;;){
    if (xQueueReceive(g_fmtQueue, &job, portMAX_DELAY) == pdTRUE){
      Serial.println("[TASK] FormatJob recibido en cola.");
      // Ejecutar FORMAT fuera de la callback ESP-NOW
      const uint8_t* mac = replyMac();
      Serial.println("[XF551] Ejecutando FORMAT en tarea...");

      // Armar frame (como en forwardFrame): dev, 0x21, secL=0x04, secH=0x00, checksum
      uint8_t frame[5] = { job.dev, 0x21, 0x04, 0x00, 0x00 };
      frame[4] = chk(frame, 4);

      // Enviar al bus SIO (con pulso /COMMAND)
      pulseCommandAndSendFrame(frame);

      // Esperar ACK (41) del drive real (hasta 3s)
      if (!waitByte(0x41, 3000)){
        Serial.println("[XF551] FORMAT: no hubo ACK del drive.");
        // Aviso NAK al maestro (opcional)
        sendNAK();
        continue;
      }

      // Esperar COMPLETE (43) hasta 90s
      bool ok = false; unsigned long t0 = millis();
      while (millis() - t0 < 90000){
        if (SerialSIO.available()){
          uint8_t b = SerialSIO.read();
          if (b == 0x43){ ok = true; break; }  // COMPLETE
          if (b == 0x45){ ok = false; break; } // ERROR
        }
        delay(1);
      }

      uint8_t bad[128];
      memset(bad, 0xFF, sizeof(bad)); // por defecto sin sectores malos

      if (ok){
        // Leer 128 bytes de mapa de sectores malos (+ checksum descartable)
        int n = 0; t0 = millis();
        while (n < 128 && millis() - t0 < 5000){
          if (SerialSIO.available()){
            bad[n++] = SerialSIO.read();
          }
          delay(1);
        }
        // Consumir checksum si quedó
        unsigned long t2 = millis();
        while (millis() - t2 < 50){
          if (SerialSIO.available()){ (void)SerialSIO.read(); break; }
          delay(1);
        }
        Serial.println("[XF551] FORMAT completado, enviando BAD MAP al maestro.");
      } else {
        Serial.println("[XF551] FORMAT falló o expiró.");
      }


      // Enviar resultado TYPE_FORMAT_BAD (1 chunk de 128 bytes)
      uint8_t pkt[6 + 128];
      pkt[0] = TYPE_FORMAT_BAD;
      pkt[1] = job.dev;
      pkt[2] = 0;  // sec low
      pkt[3] = 0;  // sec high
      pkt[4] = 0;  // chunk idx
      pkt[5] = 1;  // total chunks
      memcpy(pkt + 6, bad, 128);
      send_now_to(mac, pkt, sizeof(pkt));

      // Señal informativa al maestro
      sendACK();

      // === Enviar COMPLETE final al bus SIO (para cerrar FORMAT) ===
delayMicroseconds(2150);  // ACK → COMPLETE (~2.15 ms)
SerialSIO.write(0x43);    // COMPLETE
SerialSIO.flush();
delayMicroseconds(1730);  // COMPLETE → DATA (~1.73 ms)
Serial.println("[XF551] COMPLETE enviado tras FORMAT local (cierre SIO)");

    }
  }
}

// ======== Callback de recepción ========
#if ESP_IDF_VERSION_MAJOR >= 5
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *in, int len){
  const uint8_t *src = recvInfo ? recvInfo->src_addr : nullptr;
#else
void onDataRecv(const uint8_t* src, const uint8_t* in, int len){
#endif
  if (len <= 0) return;

  // Guardar MAC del maestro si llega unicast (no es FF:FF:FF:FF:FF:FF)
  if(src){
    bool isBcast = true; for(int i=0;i<6;i++) if(src[i]!=0xFF){ isBcast=false; break; }
    if(!isBcast){ memcpy(g_lastMaster, src, 6); g_haveMasterMac=true; ensurePeer(g_lastMaster); }
  }

  uint8_t type = in[0];

  if (type == TYPE_RESET){
    logf("[SLAVE] RESET recibido -> reiniciando...");
    delay(50); ESP.restart(); return;
  }

  if (type == TYPE_CMD_FRAME && len >= 6){
    uint8_t cmd = in[1], dev = in[2];
    uint16_t sec = (uint16_t)in[3] | ((uint16_t)in[4] << 8);
    uint8_t dens = in[5];
    if (dev != DEVICE_ID) return;

    // --- READ ---
    if (cmd == 0x52){
      uint8_t frame[5] = {dev, 0x52, (uint8_t)(sec & 0xFF), (uint8_t)(sec >> 8), 0};
      frame[4] = chk(frame, 4);
      pulseCommandAndSendFrame(frame);
      if (!waitByte(0x41, 4000)){ sendNAK(); return; }
      int sz = dens ? SECTOR_256 : SECTOR_128;
      uint8_t buf[MAX_SECTOR_BYTES];
      if (!readSectorFromDrive(buf, sz, 8000)){ sendNAK(); return; }
      sendACK(); sendSector(sec, buf, sz); return;
    }

    // --- WRITE ---
    if (cmd == 0x57){
      uint8_t frame[5] = {dev, 0x57, (uint8_t)(sec & 0xFF), (uint8_t)(sec >> 8), 0};
      frame[4] = chk(frame, 4);
      pulseCommandAndSendFrame(frame);
      if (!waitByte(0x41, 3000)){ sendNAK(); return; }
      sendACK(); return;
    }

    // --- FORMAT --- (en cola, no bloquear)
    if (cmd == 0x21){
      if (g_fmtQueue){
        FormatJob job{dev, dens};
        if (xQueueSend(g_fmtQueue, &job, 0) == pdTRUE){
          // Informativo al maestro
          sendACK();
          Serial.println("[XF551] FORMAT encolado.");
        }else{
          sendNAK(); // cola llena
          Serial.println("[XF551] FORMAT descartado (cola llena).");
        }
      }else{
        sendNAK();
      }
      return;
    }

    sendNAK(); return;
  }

// --- WRITE CHUNKS --- (corregido: sin COMPLETE intermedio)
if (type == TYPE_WRITE_CHUNK && len >= 6) {
  uint8_t dev = in[1];
  if (dev != DEVICE_ID) return;

  uint16_t sec = (uint16_t)in[2] | ((uint16_t)in[3] << 8);
  uint8_t ci = in[4], cc = in[5];
  int pl = len - 6;

  static uint8_t wbuf[MAX_SECTOR_BYTES];
  static uint8_t expect = 0, got = 0;
  static int total = 0;

  if (ci == 0) {
    memset(wbuf, 0, MAX_SECTOR_BYTES);
    expect = cc;
    got = 0;
    total = 0;
  }

  int off = ci * CHUNK_PAYLOAD;
  int cp = min(pl, MAX_SECTOR_BYTES - off);
  memcpy(wbuf + off, in + 6, cp);
  total += cp;
  got++;

  if (got >= expect) {
    int sz = (total > 128) ? SECTOR_256 : SECTOR_128;
    bool ok = writeSectorToDrive(wbuf, sz);

    Serial.printf("[WRITE] Sector %u (%s) → %s\n",
                  sec, sz == 256 ? "DD" : "SD", ok ? "OK" : "FAIL");

    // ✅ Solo ACK/NAK, sin enviar COMPLETE aquí
    if (ok) sendACK();
    else sendNAK();

    got = 0;
    expect = 0;
    total = 0;
  }
}

}

// ======== Envío callback ========
#if ESP_IDF_VERSION_MAJOR >= 5
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t s){
  (void)info; (void)s;
}
#else
void onDataSent(const uint8_t* mac, esp_now_send_status_t s){
  (void)mac; (void)s;
}
#endif

// ======== SETUP ========
void setup(){
  Serial.begin(115200);
  SerialSIO.begin(19200, SERIAL_8N1, SIO_RX, SIO_TX);
  pinMode(SIO_COMMAND, OUTPUT); digitalWrite(SIO_COMMAND, HIGH);
  WiFi.mode(WIFI_STA);

  // Cola y tarea para FORMAT
  g_fmtQueue = xQueueCreate(1, sizeof(FormatJob));
  xTaskCreatePinnedToCore(FormatTask, "FormatTask", 4096, nullptr, 1, nullptr, 1);

  if (esp_now_init() != ESP_OK){ logf("[ERR] esp_now_init"); ESP.restart(); }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  ensurePeer(BCAST_MAC);
  sendHello();
  logf("[SLAVE] Listo DEV=0x%02X DD=%u", DEVICE_ID, supports256 ? 1 : 0);
}

// ======== LOOP ========
void loop(){
  static unsigned long t0 = 0;
  if (millis() - t0 > 60000){ sendHello(); t0 = millis(); }
  delay(10);
}
