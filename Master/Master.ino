/*******************************************************
 * MASTER XF551 – ESP-NOW + WebUI (SoftAP) – ESP32 WROOM-32
 * - Multiunidad D1..D4 (0x31..0x34)
 * - READ/WRITE/STATUS/FORMAT + LOCK/UNLOCK
 * - WebUI:
 *     • Reset a esclavos (broadcast)
 *     • Lock/Unlock por nombre 8+3
 *     • Wi-Fi AP (SSID/clave) persistente
 *     • Temporización SIO (persistente, aplica tras reiniciar)
 *     • Diagnóstico SIO (medición/log y Autoajuste +10%)
 *     • Modo de envío ESPNOW: Broadcast / Unicast (persistente)
 *     • Reinicio remoto
 * - ESP-NOW compatible ESP-IDF v4.x / v5.x
 * - Defaults de timing orientados a corregir Error 138 en FORMAT
 * Eduardo Quintana – 2025
 *******************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <Preferences.h>
#include <HardwareSerial.h>
#include <stdarg.h>

// ==================== Preferencias / AP ====================
Preferences prefs;
String AP_SSID = "ATARI_BRIDGE";
String AP_PSK  = "atari123";
WebServer web(80);

// ==================== Pines SIO ====================
#define SIO_DATA_IN   16
#define SIO_DATA_OUT  17
#define SIO_COMMAND   18
#define LED_STATUS     2
#define LED_ACTIVITY   4
HardwareSerial SerialSIO(2);

// ==================== Constantes SIO ====================
#define SIO_ACK      0x41
#define SIO_NAK      0x4E
#define SIO_COMPLETE 0x43
#define SIO_ERROR    0x45
#define SECTOR_128   128
#define SECTOR_256   256
#define MAX_SECTOR   720

// Defaults seguros para arrancar FORMAT sin 138 (µs)
#define DEF_T_ACK_TO_COMPLETE   1300
#define DEF_T_COMPLETE_TO_DATA   950
#define DEF_T_DATA_TO_CHK        120
#define DEF_T_CHUNK             1300

// Rango permitido (ajústalo si necesitas más)
#define SIO_MIN_US   50
#define SIO_MAX_US 8000

// Variables activas (se cargan de NVS en setup)
static uint16_t T_ACK_TO_COMPLETE   = DEF_T_ACK_TO_COMPLETE;
static uint16_t T_COMPLETE_TO_DATA  = DEF_T_COMPLETE_TO_DATA;
static uint16_t T_DATA_TO_CHK       = DEF_T_DATA_TO_CHK;
static uint16_t T_CHUNK             = DEF_T_CHUNK;

// ==================== ESP-NOW protocolo ====================
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

// ==================== Estado de esclavos ====================
struct SlaveInfo {
  bool present;
  bool supports256;
  uint8_t mac[6];
  unsigned long lastSeen;
} slaves[4];

int devIndex(uint8_t dev){ return (dev>=0x31 && dev<=0x34) ? (dev-0x31) : -1; }
const char* devName(uint8_t d){ static const char* n[]={"D1:","D2:","D3:","D4:"}; int i=devIndex(d); return (i>=0)?n[i]:"UNK"; }

// ==================== Buffers y flags ====================
volatile bool replyReady=false;
volatile uint16_t replySec=0;
volatile uint8_t expectedChunks=0, receivedChunks=0;
uint8_t replyBuf[MAX_SECTOR_BYTES];

// ==================== Utilitarios ====================
bool TRACE = true;
void logf(const char* fmt,...){
  char b[384]; va_list a; va_start(a,fmt); vsnprintf(b,sizeof(b),fmt,a); va_end(a);
  Serial.println(b);
}
uint8_t checksum(const uint8_t* d,int len){ uint16_t s=0; for(int i=0;i<len;i++){ s+=d[i]; if(s>0xFF) s=(s&0xFF)+1; } return s&0xFF; }
static inline uint16_t clampu16(uint16_t v, uint16_t lo, uint16_t hi){ return v<lo?lo:(v>hi?hi:v); }

void setSlavePresent(uint8_t dev, const uint8_t* mac, bool supp256){
  int idx=devIndex(dev); if(idx<0) return;
  slaves[idx].present=true; slaves[idx].supports256=supp256;
  memcpy(slaves[idx].mac, mac, 6); slaves[idx].lastSeen=millis();
}
static void ensurePeer(const uint8_t* mac){
  if(esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {}; memcpy(p.peer_addr, mac, 6); p.channel=0; p.encrypt=false; esp_now_add_peer(&p);
}

// ==================== Persistencia de tiempos SIO ====================
void loadSioTiming(){
  prefs.begin("sio_timing", true);
  uint16_t a = prefs.getUShort("ackc",  DEF_T_ACK_TO_COMPLETE);
  uint16_t c = prefs.getUShort("ctd",   DEF_T_COMPLETE_TO_DATA);
  uint16_t d = prefs.getUShort("dtc",   DEF_T_DATA_TO_CHK);
  uint16_t h = prefs.getUShort("chunk", DEF_T_CHUNK);
  prefs.end();
  T_ACK_TO_COMPLETE  = clampu16(a, SIO_MIN_US, SIO_MAX_US);
  T_COMPLETE_TO_DATA = clampu16(c, SIO_MIN_US, SIO_MAX_US);
  T_DATA_TO_CHK      = clampu16(d, SIO_MIN_US, SIO_MAX_US);
  T_CHUNK            = clampu16(h, SIO_MIN_US, SIO_MAX_US);
  if(TRACE) logf("[SIO] Timings cargados: ACK→COMP=%u, COMP→DATA=%u, DATA→CHK=%u, CHUNK=%u",
                 T_ACK_TO_COMPLETE, T_COMPLETE_TO_DATA, T_DATA_TO_CHK, T_CHUNK);
}
void saveSioTiming(uint16_t a,uint16_t c,uint16_t d,uint16_t h){
  prefs.begin("sio_timing", false);
  prefs.putUShort("ackc", a);
  prefs.putUShort("ctd",  c);
  prefs.putUShort("dtc",  d);
  prefs.putUShort("chunk",h);
  prefs.end();
}

// ==================== Persistencia modo TX ====================
enum TxMode : uint8_t { TX_BCAST=0, TX_UCAST=1 };
static TxMode g_txmode = TX_BCAST;       // default
const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

void loadTxMode(){
  prefs.begin("netcfg", true);
  uint8_t m = prefs.getUChar("txmode", (uint8_t)TX_BCAST);
  prefs.end();
  g_txmode = (m==TX_UCAST)?TX_UCAST:TX_BCAST;
  if(TRACE) logf("[NET] TX Mode cargado: %s", g_txmode==TX_UCAST?"Unicast":"Broadcast");
}
void saveTxMode(uint8_t m){
  prefs.begin("netcfg", false);
  prefs.putUChar("txmode", (m?TX_UCAST:TX_BCAST));
  prefs.end();
}

// ==================== ESPNOW callbacks (IDF v4/v5) ====================
#if ESP_IDF_VERSION_MAJOR >= 5
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status){ (void)info; (void)status; }
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incoming, int len){
  if(!recvInfo || !incoming || len<=0) return;
  const uint8_t *mac = recvInfo->src_addr;
#else
void onDataSent(const uint8_t* mac, esp_now_send_status_t status){ (void)mac; (void)status; }
void onDataRecv(const uint8_t * mac, const uint8_t *incoming, int len){
  if(len<=0) return;
#endif
  uint8_t type = incoming[0];

  if(type==TYPE_HELLO && len>=3){
    uint8_t dev=incoming[1]; bool s256=incoming[2];
    setSlavePresent(dev, mac, s256);
    if(TRACE) logf("[HELLO] %s DD=%u %02X:%02X:%02X:%02X:%02X:%02X",
                   devName(dev), s256, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    ensurePeer(mac);
    return;
  }

  if((type==TYPE_SECTOR_CHUNK || type==TYPE_FORMAT_BAD) && len>=6){
    uint8_t dev=incoming[1]; uint16_t sec=(uint16_t)incoming[2]|((uint16_t)incoming[3]<<8);
    uint8_t ci=incoming[4], cc=incoming[5]; int pl=len-6;
    if(ci==0){ memset(replyBuf,0,MAX_SECTOR_BYTES); expectedChunks=cc; receivedChunks=0; replyReady=false; replySec=sec; }
    int off=ci*CHUNK_PAYLOAD; int copy=pl; if(off+copy>MAX_SECTOR_BYTES) copy=MAX_SECTOR_BYTES-off;
    if(copy>0) memcpy(replyBuf+off, incoming+6, copy);
    receivedChunks++; if(receivedChunks>=expectedChunks) replyReady=true;
    return;
  }

  if(type==TYPE_ACK || type==TYPE_NAK){
    if(TRACE) logf("[ESPNOW] %s", (type==TYPE_ACK?"ACK":"NAK"));
    return;
  }
}

// ==================== Envío maestro->esclavo ====================
static inline bool send_now_to(const uint8_t* mac, const uint8_t* data, int n){
  ensurePeer(mac);
  return (esp_now_send(mac, data, n)==ESP_OK);
}

bool sendCmd(uint8_t dev, uint8_t cmd, uint16_t sec, uint8_t dens){
  uint8_t p[6]={TYPE_CMD_FRAME,cmd,dev,(uint8_t)(sec&0xFF),(uint8_t)((sec>>8)&0xFF),dens};
  if(g_txmode==TX_BCAST){
    return send_now_to(BCAST_MAC, p, sizeof(p));
  }else{
    int idx=devIndex(dev); if(idx<0 || !slaves[idx].present){ if(TRACE) logf("[CMD] %s no presente",devName(dev)); return false; }
    return send_now_to(slaves[idx].mac, p, sizeof(p));
  }
}

bool sendWriteChunk(uint8_t dev, uint16_t sec, uint8_t chunk, uint8_t cc, const uint8_t* data, int n){
  uint8_t* p=(uint8_t*)malloc(6+n); if(!p) return false;
  p[0]=TYPE_WRITE_CHUNK; p[1]=dev; p[2]=sec&0xFF; p[3]=sec>>8; p[4]=chunk; p[5]=cc; memcpy(p+6,data,n);
  bool ok=false;
  if(g_txmode==TX_BCAST){
    ok = send_now_to(BCAST_MAC, p, 6+n);
  }else{
    int idx=devIndex(dev); if(idx<0 || !slaves[idx].present){ free(p); return false; }
    ok = send_now_to(slaves[idx].mac, p, 6+n);
  }
  free(p);
  return ok;
}

void sendResetBroadcast(){
  uint8_t p[1]={TYPE_RESET};
  send_now_to(BCAST_MAC, p, 1);
}

// ==================== Local read/write (maestro<->esclavo) ====================
int readLocal(uint8_t dev, uint16_t sec, bool dd, uint8_t* out){
  replyReady=false; expectedChunks=receivedChunks=0; replySec=0;
  if(!sendCmd(dev,0x52,sec,dd?1:0)) return 0;
  unsigned long t0=millis(); int expect=dd?SECTOR_256:SECTOR_128;
  while(millis()-t0<10000){ if(replyReady && replySec==sec) break; delay(2); }
  if(!(replyReady && replySec==sec)) return 0;
  memcpy(out,replyBuf,expect); replyReady=false; return expect;
}
bool writeLocal(uint8_t dev, uint16_t sec, bool dd, const uint8_t* data, int len){
  if(!sendCmd(dev,0x57,sec,dd?1:0)) return false;
  int cc=(len+CHUNK_PAYLOAD-1)/CHUNK_PAYLOAD;
  for(int i=0;i<cc;i++){
    int off=i*CHUNK_PAYLOAD, n=min(CHUNK_PAYLOAD,len-off);
    if(!sendWriteChunk(dev,sec,(uint8_t)i,(uint8_t)cc,data+off,n)) return false;
    delay(2);
  }
  delay(20);
  return true;
}

// ==================== LOCK / UNLOCK ====================
bool lockUnlock(uint8_t dev, const char* name11, bool lock){
  uint8_t dir[SECTOR_128];
  int r = readLocal(dev, 360, false, dir); if(r!=128){ logf("[LOCK] No pude leer sector 360"); return false; }
  int entry=-1;
  for(int i=0;i<128;i+=16){
    bool ok=true; for(int j=0;j<11;j++) if((uint8_t)name11[j]!=dir[i+1+j]){ ok=false; break; }
    if(ok){ entry=i; break; }
  }
  if(entry<0){ logf("[LOCK] Nombre no encontrado"); return false; }
  if(lock) dir[entry]|=0x80; else dir[entry]&=0x7F;
  return writeLocal(dev,360,false,dir,128);
}

// ==================== SIO helpers y diagnóstico ====================
uint8_t sioChk(const uint8_t* f){ return checksum(f,4); }

// variables de diagnóstico (valores medidos en la última transacción)
static uint32_t last_us_ack_to_comp   = 0;
static uint32_t last_us_comp_to_data  = 0;
static uint32_t last_us_data_to_chk   = 0;
static uint32_t last_us_chunk_delay   = 0;

void sendAtariData(const uint8_t* buf,int n){
  uint32_t t0 = micros();
  delayMicroseconds(T_ACK_TO_COMPLETE);
  SerialSIO.write(SIO_COMPLETE);
  uint32_t t1 = micros();
  delayMicroseconds(T_COMPLETE_TO_DATA);
  SerialSIO.write(buf,n);
  uint32_t t2 = micros();
  delayMicroseconds(T_DATA_TO_CHK);
  uint8_t chk = checksum(buf,n);
  SerialSIO.write(chk);
  uint32_t t3 = micros();
  delayMicroseconds(T_CHUNK);
  uint32_t t4 = micros();

  last_us_ack_to_comp  = (t1 - t0);
  last_us_comp_to_data = (t2 - t1);
  last_us_data_to_chk  = (t3 - t2);
  last_us_chunk_delay  = (t4 - t3);

  if(TRACE){
    logf("[SIO][MEASURE] ACK→COMP ~%lu us | COMP→DATA ~%lu us | DATA→CHK ~%lu us | CHUNK ~%lu us",
         (unsigned long)last_us_ack_to_comp,
         (unsigned long)last_us_comp_to_data,
         (unsigned long)last_us_data_to_chk,
         (unsigned long)last_us_chunk_delay);
  }
}

void handleSioFrame(){
  uint8_t f[5];
  for(int i=0;i<5;i++){
    unsigned long t0=micros();
    while(!SerialSIO.available()){ if(micros()-t0>3000) return; }
    f[i]=SerialSIO.read();
  }
  if(TRACE){ Serial.print("[FRAME] "); for(int i=0;i<5;i++) Serial.printf("%02X ",f[i]); Serial.println(); }

  uint8_t dev=f[0], cmd=f[1]; uint16_t sec=(f[3]<<8)|f[2]; uint8_t chk=sioChk(f);

  if(f[4]!=chk){
    delayMicroseconds(1000);
    SerialSIO.write(SIO_NAK);
    if(TRACE) logf("[SIO] ❌ Checksum invalido al inicio (posible 138). Recv=%02X Calc=%02X", f[4], chk);
    return;
  }
  if(dev<0x31||dev>0x34){
    delayMicroseconds(1000);
    SerialSIO.write(SIO_NAK);
    return;
  }

  bool dd=(cmd&0x80)!=0; uint8_t base=cmd&0x7F;

  if(base==0x52){ // READ
    SerialSIO.write(SIO_ACK);
    delayMicroseconds(1000);
    uint8_t buf[MAX_SECTOR_BYTES]; int n=readLocal(dev,sec,dd,buf);
    if(!n){ SerialSIO.write(SIO_ERROR); return; }
    sendAtariData(buf,n);
    return;
  }

  if(base==0x57){ // WRITE
    SerialSIO.write(SIO_ACK);
    delayMicroseconds(1000);
    int n=dd?SECTOR_256:SECTOR_128; uint8_t buf[MAX_SECTOR_BYTES];
    unsigned long t0=millis(); int idx=0;
    while(idx<n && millis()-t0<6000){ if(SerialSIO.available()) buf[idx++]=SerialSIO.read(); }
    if(idx!=n){ SerialSIO.write(SIO_ERROR); return; }
    uint8_t rchk=0; t0=millis(); while(millis()-t0<2000){ if(SerialSIO.available()){ rchk=SerialSIO.read(); break; } }
    if(rchk!=checksum(buf,n)){ SerialSIO.write(SIO_ERROR); return; }
    if(!writeLocal(dev,sec,dd,buf,n)){ SerialSIO.write(SIO_ERROR); return; }
    delayMicroseconds(T_ACK_TO_COMPLETE);
    SerialSIO.write(SIO_COMPLETE);
    return;
  }

  if(base==0x53){ // STATUS
    SerialSIO.write(SIO_ACK);
    delayMicroseconds(1000);
    uint8_t st[4]={0x00,(uint8_t)(dd?0x80:0x00),0x01,0x00};
    sendAtariData(st,4);
    return;
  }

  if(base==0x21){ // FORMAT
    // Mantener /COMMAND bajo durante toda la operación
    pinMode(SIO_COMMAND, OUTPUT);
    digitalWrite(SIO_COMMAND, LOW);

    SerialSIO.write(SIO_ACK);
    delayMicroseconds(1600); // ACK→COMPLETE (ajuste fino anti-138)

    // Espera mínima antes de enviar COMPLETE
    SerialSIO.write(SIO_COMPLETE);
    delayMicroseconds(950); // COMPLETE→DATA

    replyReady = false; expectedChunks = receivedChunks = 0; replySec = 0;

    // Enviar FORMAT al esclavo
    if(!sendCmd(dev, 0x21, 0, dd ? 1 : 0)){
      SerialSIO.write(SIO_ERROR);
      digitalWrite(SIO_COMMAND, HIGH);
      pinMode(SIO_COMMAND, INPUT_PULLUP);
      return;
    }

    // Esperar hasta 90s la respuesta TYPE_FORMAT_BAD
    unsigned long t0 = millis(); bool ok = false;
    uint8_t bad[128]; memset(bad, 0xFF, sizeof(bad));
    while(millis() - t0 < 90000){
      if(replyReady){
        memcpy(bad, replyBuf, 128);
        ok = true;
        break;
      }
      delay(5);
    }

    // Si no llegó o falló, enviar ERROR
    if(!ok){
      SerialSIO.write(SIO_ERROR);
      digitalWrite(SIO_COMMAND, HIGH);
      pinMode(SIO_COMMAND, INPUT_PULLUP);
      return;
    }

    // Enviar datos de sectores malos ($FF $FF si ninguno)
    delayMicroseconds(1200);
    sendAtariData(bad, 128);

    // Liberar /COMMAND con retardo seguro
    delayMicroseconds(300);
    digitalWrite(SIO_COMMAND, HIGH);
    pinMode(SIO_COMMAND, INPUT_PULLUP);
    return;
  }

  delayMicroseconds(1000);
  SerialSIO.write(SIO_ERROR);
}

// ==================== WebUI ====================
String htmlHeader(){
  return F("<!doctype html><html lang='es'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
           "<title>XF551 Bridge</title><style>"
           "body{font-family:system-ui,Arial;margin:20px}"
           "code{background:#eee;padding:2px 6px;border-radius:4px}"
           "button,input,select{padding:6px 10px;margin:4px}"
           "table{border-collapse:collapse}td,th{border:1px solid #ddd;padding:6px 8px}"
           ".box{border:1px solid #ddd;border-radius:8px;padding:12px;margin:12px 0}"
           ".row{display:flex;gap:8px;flex-wrap:wrap}"
           ".col{display:flex;flex-direction:column;min-width:200px}"
           ".warn{color:#b00}"
           "</style></head><body>");
}
String htmlFooter(){ return F("</body></html>"); }

String txModeBox(){
  String s;
  s += "<div class='box'><h2>Modo de Envío ESPNOW</h2>";
  s += "<form method='post' action='/txmode_save'>";
  s += "<label>Modo:</label> ";
  s += "<select name='mode'>";
  s += String("<option value='0' ") + (g_txmode==TX_BCAST?"selected":"") + ">Broadcast</option>";
  s += String("<option value='1' ") + (g_txmode==TX_UCAST?"selected":"") + ">Unicast</option>";
  s += "</select> ";
  s += "<button>Guardar</button>";
  s += "</form>";
  s += "<p><small>Unicast reduce tráfico al aire y colisiones cuando hay varios esclavos. Broadcast es útil para pruebas o cuando el maestro aún no conoce la MAC del esclavo.</small></p>";
  s += "</div>";
  return s;
}

String sioTimingForm(){
  String s;
  s += "<div class='box'><h2>Temporización SIO (µs)</h2>";
  s += "<form method='post' action='/sio_save'>";
  s += "<div class='row'>";
  s += "<div class='col'><label>ACK → COMPLETE</label><input name='ackc' type='number' min='50' max='5000' value='"+String(T_ACK_TO_COMPLETE)+"'></div>";
  s += "<div class='col'><label>COMPLETE → DATA</label><input name='ctd' type='number' min='50' max='5000' value='"+String(T_COMPLETE_TO_DATA)+"'></div>";
  s += "<div class='col'><label>DATA → CHK</label><input name='dtc' type='number' min='50' max='5000' value='"+String(T_DATA_TO_CHK)+"'></div>";
  s += "<div class='col'><label>CHUNK DELAY</label><input name='chunk' type='number' min='50' max='5000' value='"+String(T_CHUNK)+"'></div>";
  s += "</div>";
  s += "<p><small>Se guardan en NVS y se <b>aplican tras reiniciar</b>. Valores altos ayudan a evitar el Error 138 al inicio de FORMAT.</small></p>";
  s += "<button type='submit'>Guardar</button>";
  s += "</form> ";
  s += "<form method='post' action='/sio_reset' style='display:inline'><button>Restaurar por defecto</button></form> ";
  s += "<form method='post' action='/reboot' style='display:inline'><button style='background:#d33;color:#fff'>🔄 Reiniciar ahora</button></form>";
  s += "</div>";
  return s;
}
String sioDiagBox(){
  String s;
  s += "<div class='box'><h2>Diagnóstico SIO</h2>";
  s += "<p>Últimas mediciones aproximadas (incluyen latencia de escritura UART):</p>";
  s += "<ul>";
  s += "<li>ACK → COMPLETE: <b>"+String(last_us_ack_to_comp)+"</b> µs</li>";
  s += "<li>COMPLETE → DATA: <b>"+String(last_us_comp_to_data)+"</b> µs</li>";
  s += "<li>DATA → CHK: <b>"+String(last_us_data_to_chk)+"</b> µs</li>";
  s += "<li>CHUNK DELAY: <b>"+String(last_us_chunk_delay)+"</b> µs</li>";
  s += "</ul>";
  s += "<form method='post' action='/diag_measure' style='display:inline'><button>Medir tiempos (log a Serial)</button></form> ";
  s += "<form method='post' action='/sio_autoplus' style='display:inline'><button>Autoajustar +10% (guardar)</button></form>";
  s += "<p class='warn'><small>“Medir tiempos” escribe las mediciones por Serial en la <i>próxima</i> respuesta SIO que ocurra (p.ej. al iniciar FORMAT).</small></p>";
  s += "</div>";
  return s;
}

void handleRoot(){
  String s = htmlHeader();
  s += F("<h1>XF551 Bridge (ESP-NOW)</h1>");

  // Unidades
  s += F("<div class='box'><h2>Unidades</h2><table><tr><th>ID</th><th>Presente</th><th>MAC</th><th>DD</th><th>Acciones</th></tr>");
  for(int i=0;i<4;i++){
    char macbuf[24]; if(slaves[i].present) sprintf(macbuf,"%02X:%02X:%02X:%02X:%02X:%02X",slaves[i].mac[0],slaves[i].mac[1],slaves[i].mac[2],slaves[i].mac[3],slaves[i].mac[4],slaves[i].mac[5]); else strcpy(macbuf,"—");
    s += "<tr><td>D"+String(i+1)+"</td><td>"+String(slaves[i].present?"Sí":"No")+"</td><td>"+String(macbuf)+"</td><td>"+String(slaves[i].supports256?"Sí":"No")+"</td><td>";
    s += "<form method='post' action='/reset' style='display:inline'><input type='hidden' name='d' value='"+String(i+1)+"'><button>Reset</button></form>";
    s += "</td></tr>";
  }
  s += "</table></div>";

  // Lock/Unlock
  s += F("<div class='box'><h2>LOCK / UNLOCK</h2>"
       "<p>Nombre exacto (11 chars 8+3, sin punto, mayúsculas, con espacios):</p>"
       "<form method='post' action='/lock'><label>Unidad:</label>"
       "<select name='d'><option>1</option><option>2</option><option>3</option><option>4</option></select> "
       "<label>Nombre (11):</label><input name='n' maxlength='11' size='12'> "
       "<button>LOCK</button></form>"
       "<form method='post' action='/unlock' style='margin-top:6px'><label>Unidad:</label>"
       "<select name='d'><option>1</option><option>2</option><option>3</option><option>4</option></select> "
       "<label>Nombre (11):</label><input name='n' maxlength='11' size='12'> "
       "<button>UNLOCK</button></form></div>");

  // Wi-Fi AP
  s += "<div class='box'><h2>Wi-Fi (AP)</h2>"
       "<form method='post' action='/wifi'><label>SSID:</label>"
       "<input name='ssid' value='"+AP_SSID+"'> "
       "<label>Clave:</label><input name='psk' value='"+AP_PSK+"' type='password'> "
       "<button>Cambiar</button></form>"
       "<p><small>Reinicia para aplicar.</small></p></div>";

  // Modo de envío
  s += txModeBox();

  // Temporización SIO
  s += sioTimingForm();

  // Diagnóstico SIO
  s += sioDiagBox();

  s += htmlFooter();
  web.send(200,"text/html",s);
}

// ===== Handlers Web =====
void handleReset(){ sendResetBroadcast(); web.sendHeader("Location","/"); web.send(302); }

void handleLockUnlock(bool isLock){
  if(!web.hasArg("d")||!web.hasArg("n")){ web.send(400,"text/plain","Parámetros requeridos"); return; }
  int dn=web.arg("d").toInt(); if(dn<1||dn>4){ web.send(400,"text/plain","Unidad invalida"); return; }
  String nm=web.arg("n"); if(nm.length()!=11){ web.send(400,"text/plain","Nombre debe tener 11 chars (8+3)"); return; }
  nm.toUpperCase(); char name11[12]; memset(name11,' ',11); name11[11]=0; for(int i=0;i<11;i++) name11[i]=nm.charAt(i);
  bool ok=lockUnlock((uint8_t)(0x30+dn),name11,isLock);
  web.send(200,"text/plain",String(isLock?"LOCK ":"UNLOCK ")+(ok?"OK":"FAIL"));
}
void handleWifi(){
  String ssid=web.arg("ssid"), psk=web.arg("psk");
  if(ssid.length()<3){ web.send(400,"text/plain","SSID muy corto"); return; }
  if(psk.length()<8){ web.send(400,"text/plain","Clave muy corta"); return; }
  prefs.begin("apcfg",false); prefs.putString("ssid",ssid); prefs.putString("psk",psk); prefs.end();
  web.send(200,"text/plain","Guardado. Reinicia el Maestro para aplicar.");
}
void handleSioSave(){
  if(!web.hasArg("ackc")||!web.hasArg("ctd")||!web.hasArg("dtc")||!web.hasArg("chunk")){
    web.send(400,"text/plain","Parámetros requeridos"); return;
  }
  uint16_t a=clampu16(web.arg("ackc").toInt(),  SIO_MIN_US,SIO_MAX_US);
  uint16_t c=clampu16(web.arg("ctd").toInt(),   SIO_MIN_US,SIO_MAX_US);
  uint16_t d=clampu16(web.arg("dtc").toInt(),   SIO_MIN_US,SIO_MAX_US);
  uint16_t h=clampu16(web.arg("chunk").toInt(), SIO_MIN_US,SIO_MAX_US);
  saveSioTiming(a,c,d,h);
  web.send(200,"text/plain","Temporización guardada. Reinicia para aplicar.");
}
void handleSioReset(){ saveSioTiming(DEF_T_ACK_TO_COMPLETE,DEF_T_COMPLETE_TO_DATA,DEF_T_DATA_TO_CHK,DEF_T_CHUNK);
  web.send(200,"text/plain","Temporización restaurada. Reinicia para aplicar."); }
void handleReboot(){ web.send(200,"text/plain","Reiniciando..."); delay(500); ESP.restart(); }
void handleDiagMeasure(){ web.send(200,"text/plain","Listo. Inicia un comando SIO y revisa el Serial."); }

void handleTxModeSave(){
  if(!web.hasArg("mode")){ web.send(400,"text/plain","Falta 'mode'"); return; }
  uint8_t m = (uint8_t)web.arg("mode").toInt();
  saveTxMode(m?1:0);
  web.send(200,"text/plain", String("Modo guardado: ") + (m? "Unicast":"Broadcast") + ". Reinicia para aplicar.");
}

// ==================== Inicialización ====================
void startAP(){
  prefs.begin("apcfg",true);
  String s=prefs.getString("ssid",AP_SSID), p=prefs.getString("psk",AP_PSK);
  prefs.end();
  AP_SSID=s; AP_PSK=p;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID.c_str(), AP_PSK.c_str());
  if(TRACE) logf("[AP] SSID=%s IP=%s", AP_SSID.c_str(), WiFi.softAPIP().toString().c_str());
}
void startWeb(){
  web.on("/",HTTP_GET,handleRoot);
  web.on("/reset",HTTP_POST,handleReset);
  web.on("/lock",HTTP_POST,[](){handleLockUnlock(true);});
  web.on("/unlock",HTTP_POST,[](){handleLockUnlock(false);});
  web.on("/wifi",HTTP_POST,handleWifi);
  web.on("/sio_save",HTTP_POST,handleSioSave);
  web.on("/sio_reset",HTTP_POST,handleSioReset);
  web.on("/reboot",HTTP_POST,handleReboot);
  web.on("/diag_measure",HTTP_POST,handleDiagMeasure);
  web.on("/sio_autoplus",HTTP_POST,[](){
    uint32_t a = (uint32_t)(T_ACK_TO_COMPLETE  * 1.10f);
    uint32_t c = (uint32_t)(T_COMPLETE_TO_DATA * 1.10f);
    uint32_t d = (uint32_t)(T_DATA_TO_CHK      * 1.10f);
    uint32_t h = (uint32_t)(T_CHUNK            * 1.10f);
    saveSioTiming(clampu16(a,SIO_MIN_US,SIO_MAX_US),
                  clampu16(c,SIO_MIN_US,SIO_MAX_US),
                  clampu16(d,SIO_MIN_US,SIO_MAX_US),
                  clampu16(h,SIO_MIN_US,SIO_MAX_US));
    web.send(200,"text/plain","+10% guardado. Reinicia para aplicar.");
  });
  web.on("/txmode_save",HTTP_POST,handleTxModeSave);
  web.begin();
  if(TRACE) logf("[WEB] UI lista en http://%s", WiFi.softAPIP().toString().c_str());
}
void startEspNow(){
  if(esp_now_init()!=ESP_OK){ logf("[ERR] esp_now_init"); ESP.restart(); }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  ensurePeer(BCAST_MAC);
}

void setup(){
  Serial.begin(115200);
  pinMode(SIO_COMMAND,INPUT_PULLUP);
  pinMode(LED_STATUS,OUTPUT); pinMode(LED_ACTIVITY,OUTPUT);
  digitalWrite(LED_STATUS, HIGH);
  // UART SIO 19200 8N1
  SerialSIO.begin(19200, SERIAL_8N1, SIO_DATA_IN, SIO_DATA_OUT);
  for(int i=0;i<4;i++){ slaves[i].present=false; slaves[i].supports256=false; }
  // Timings / TX mode desde NVS
  loadSioTiming();
  loadTxMode();
  // Inicios
  startAP(); startWeb(); startEspNow();
  logf("[MASTER] Listo (WebUI + SIO + TX=%s).", g_txmode==TX_UCAST?"Unicast":"Broadcast");
}

void loop(){
  web.handleClient();

  // Limpieza de esclavos inactivos (30 s)
  for(int i=0;i<4;i++){
    if(slaves[i].present && millis()-slaves[i].lastSeen>30000) slaves[i].present=false;
  }

  // Gestión del bus SIO (Atari)
  if(digitalRead(SIO_COMMAND)==LOW){
    delayMicroseconds(2500);
    if(SerialSIO.available()>=5) handleSioFrame();
    while(SerialSIO.available()) SerialSIO.read();
  }
  delay(1);
} 