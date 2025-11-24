/*******************************************************
 * SLAVE XF551 – ESP-NOW (sin TCP)
 * - Conecta una disquetera XF551 real (SIO)
 * - Envía HELLO al Maestro con su ID y soporte DD
 * - Recibe comandos (READ, WRITE, FORMAT)
 * - Devuelve datos sectoriales o lista de sectores malos
 * - Responde unicast si el comando llegó unicast; si no, broadcast
 * - Compatible con Maestro 2025 (ESP-IDF v4/v5)
 * Eduardo Quintana – 2025 (versión dual)
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
void forwardFrame(const uint8_t* frame){
  while (SerialSIO.available()) SerialSIO.read();
  digitalWrite(SIO_COMMAND, LOW); delayMicroseconds(2000);
  SerialSIO.write(frame, 5); SerialSIO.flush();
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
  }
  return false;
}

bool readSectorFromDrive(uint8_t* out, int sz, unsigned long to){
  if (!waitByte(0x43, to)) return false; // COMPLETE
  int idx = 0; unsigned long t1 = millis();
  while (idx < sz && millis() - t1 < to){
    if (SerialSIO.available()) out[idx++] = SerialSIO.read();
  }
  if (idx != sz) return false;
  if (SerialSIO.available()) SerialSIO.read(); // checksum
  return true;
}

bool writeSectorToDrive(const uint8_t* buf, int sz){
  SerialSIO.write(buf, sz); SerialSIO.flush();
  uint8_t c = chk(buf, sz);
  SerialSIO.write(c);
  unsigned long t0 = millis();
  while (millis() - t0 < 5000){
    if (SerialSIO.available()){
      uint8_t b = SerialSIO.read();
      if (b == 0x43) return true;  // COMPLETE
      if (b == 0x45) return false; // ERROR
    }
  }
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

// ======== Callback de recepción ========
#if ESP_IDF_VERSION_MAJOR >= 5
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *in, int len){
  const uint8_t *src = recvInfo ? recvInfo->src_addr : nullptr;
#else
void onDataRecv(const uint8_t* src, const uint8_t* in, int len){
#endif
  if (len <= 0) return;

  // Guardar MAC del maestro si llega unicast (no es  FF:FF:FF:FF:FF:FF )
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
      forwardFrame(frame);
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
      forwardFrame(frame);
      if (!waitByte(0x41, 3000)){ sendNAK(); return; }
      sendACK(); return;
    }

    // --- FORMAT ---
    if (cmd == 0x21){
      uint8_t frame[5] = {dev, 0x21, 0, 0, 0};
      frame[4] = chk(frame, 4);
      forwardFrame(frame);
      if (!waitByte(0x41, 3000)){ sendNAK(); return; }

      unsigned long t0 = millis(); bool got = false;
      while (millis() - t0 < 60000){
        if (SerialSIO.available()){
          uint8_t b = SerialSIO.read();
          if (b == 0x43){ got = true; break; }
          if (b == 0x45){ break; }
        }
        delay(5);
      }
      if (!got){ sendNAK(); return; }

      uint8_t bad[128]; int i = 0; unsigned long t1 = millis();
      while (i < 128 && millis() - t1 < 15000){
        if (SerialSIO.available()) bad[i++] = SerialSIO.read();
      }
      if (i < 128){ sendNAK(); return; }
      if (SerialSIO.available()) SerialSIO.read();

      uint8_t msg[6 + 128];
      msg[0] = TYPE_FORMAT_BAD; msg[1] = DEVICE_ID;
      msg[2] = msg[3] = msg[4] = 0; msg[5] = 1;
      memcpy(msg + 6, bad, 128);
      send_now_to(replyMac(), msg, sizeof(msg));
      sendACK(); return;
    }
    sendNAK(); return;
  }

  // --- WRITE CHUNKS ---
  if (type == TYPE_WRITE_CHUNK && len >= 6){
    uint8_t dev = in[1]; if (dev != DEVICE_ID) return;
    uint16_t sec = (uint16_t)in[2] | ((uint16_t)in[3] << 8);
    uint8_t ci = in[4], cc = in[5];
    int pl = len - 6;

    static uint8_t wbuf[MAX_SECTOR_BYTES];
    static uint8_t expect = 0, got = 0;

    if (ci == 0){ memset(wbuf, 0, MAX_SECTOR_BYTES); expect = cc; got = 0; }

    int off = ci * CHUNK_PAYLOAD;
    int cp = min(pl, MAX_SECTOR_BYTES - off);
    memcpy(wbuf + off, in + 6, cp);
    got++;
    if (got >= expect){
      int sz = (expect * CHUNK_PAYLOAD > 128) ? SECTOR_256 : SECTOR_128;
      bool ok = writeSectorToDrive(wbuf, sz);
      if (ok) sendACK(); else sendNAK();
      got = 0; expect = 0;
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