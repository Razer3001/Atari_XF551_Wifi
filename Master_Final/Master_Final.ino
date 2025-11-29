/*******************************************************
 * MASTER – STATUS + READ + FORMAT + WRITE + PERCOM (D1..D4) + WEB UI + PREFETCH + NVS
 * - Emula D1..D4 (0x31..0x34) frente al Atari
 * - Soporta:
 *   • STATUS (0x53)
 *   • READ SECTOR (0x52)
 *   • FORMAT (0x21 / 0x22)
 *   • WRITE SECTOR (0x50 / 0x57)
 *   • READ PERCOM BLOCK (0x4E)
 *   • WRITE PERCOM BLOCK (0x4F)
 * - Usa ESP-NOW para delegar en SLAVES con XF551 real
 * - Web UI (HTTP en puerto 80, AP "XF551_MASTER" / "xf551wifi"):
 *   • Ver estado de disqueteras (SLAVES)
 *   • Ajustar tiempos SIO
 *   • Ajustar prefetch por unidad (0..MAX_PREFETCH_SECTORS)
 * - Configuración persistente (NVS):
 *   • Tiempos SIO
 *   • Prefetch D1..D4
 *******************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HardwareSerial.h>
#include <WebServer.h>
#include <stdarg.h>
#include <Preferences.h> // NVS para guardar configuración

// ========= Config SIO MASTER (lado Atari) =========
#define SIO_DATA_IN 16
#define SIO_DATA_OUT 17
#define SIO_COMMAND 18

HardwareSerial SerialSIO(2);

// Códigos SIO
#define SIO_ACK      0x41
#define SIO_NAK      0x4E
#define SIO_COMPLETE 0x43
#define SIO_ERROR    0x45

// D1..D4
#define DEV_MIN 0x31
#define DEV_MAX 0x38

// Tamaños
#define SECTOR_128       128
#define SECTOR_256       256
#define MAX_SECTOR_BYTES 256
#define PERCOM_BLOCK_LEN 12   // Tamaño estándar del Percom block

// ========= Protocolo ESP-NOW =========
#define TYPE_HELLO        0x20
#define TYPE_CMD_FRAME    0x01
#define TYPE_SECTOR_CHUNK 0x10
#define TYPE_ACK          0x11
#define TYPE_NAK          0x12

#define CHUNK_PAYLOAD 240

// Prefetch máximo que vamos a permitir configurar
#define MAX_PREFETCH_SECTORS 4

// ========= Timings SIO (µs) – AJUSTABLES POR WEB =========
// Valores por defecto (si no hay nada en NVS)
uint16_t T_ACK_TO_COMPLETE   = 2000;
uint16_t T_COMPLETE_TO_DATA  = 1600;
uint16_t T_DATA_TO_CHK       = 400;
uint16_t T_CHUNK_DELAY       = 1600;

// ========= Estado de SLAVES (uno por unidad) =========
struct SlaveInfo {
  bool present;
  bool supports256;
  uint8_t mac[6];
  unsigned long lastSeen;
};

SlaveInfo slaves[4]; // D1..D4

// Prefetch configurado por unidad (0..MAX_PREFETCH_SECTORS)
uint8_t prefetchCfg[4] = {1, 1, 1, 1};

int devIndex(uint8_t dev){
  if (dev < DEV_MIN || dev > DEV_MAX) return -1;
  return dev - DEV_MIN; // 0..3
}

const char* devName(uint8_t dev){
  static const char* names[] = {"D1", "D2", "D3", "D4"};
  int idx = devIndex(dev);
  return (idx >= 0) ? names[idx] : "UNK";
}

uint8_t prefetchForDev(uint8_t dev){
  int idx = devIndex(dev);
  if (idx < 0) return 0;
  return prefetchCfg[idx];
}

// ========= Estado de operación remota =========
enum OpType : uint8_t {
  OP_NONE    = 0,
  OP_READ    = 1,
  OP_STATUS  = 2,
  OP_FORMAT  = 3,
  OP_PERCOM  = 4   // READ PERCOM
};

volatile OpType   currentOpType = OP_NONE;
volatile uint8_t  currentOpDev  = 0;
volatile uint16_t currentOpSec  = 0;

// ========= Buffers de respuesta (desde SLAVES) =========
volatile bool     replyReady       = false;
volatile uint16_t replySec         = 0;
volatile uint16_t replyLen         = 0;
volatile uint8_t  expectedChunks   = 0;
volatile uint8_t  receivedChunks   = 0;
uint8_t           replyBuf[MAX_SECTOR_BYTES];

// ========= Seguimiento de ACK/NAK remotos (para WRITE/FORMAT/PERCOM) =========
volatile bool lastAckDone = false;
volatile bool lastAckOk   = false;

// ========= Otros =========
const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ========= WebServer =========
WebServer server(80);

// ========= NVS (Preferences) =========
Preferences prefs;
const uint32_t CFG_MAGIC = 0xCAFEBABE; // para detectar config válida

// ========= HTML responsive (UI móvil) =========

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
 <meta charset="UTF-8" />
 <meta name="viewport" content="width=device-width, initial-scale=1" />
 <title>XF551 WiFi - Panel</title>
 <style>
 :root {
 --bg: #020617;
 --accent: #38bdf8;
 --accent-soft: rgba(56, 189, 248, 0.15);
 --accent-strong: rgba(56, 189, 248, 0.45);
 --text: #e5e7eb;
 --text-soft: #9ca3af;
 --border: #1f2937;
 --danger: #f97373;
 --ok: #4ade80;
 }
 * { box-sizing: border-box; }
 body {
 margin: 0;
 font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
 background: radial-gradient(circle at top left, #0f172a 0, #020617 40%, #000 100%);
 color: var(--text);
 }
 .page { min-height: 100vh; padding: 12px; display: flex; flex-direction: column; align-items: center; }
 .app { width: 100%; max-width: 960px; }
 header { margin-bottom: 10px; text-align: center; }
 header h1 { font-size: 1.4rem; margin: 4px 0; }
 header p { font-size: 0.85rem; color: var(--text-soft); margin: 0; }
 .grid { display: grid; grid-template-columns: 1fr; gap: 10px; }
 @media (min-width: 700px) {
 .grid { grid-template-columns: minmax(0, 1.2fr) minmax(0, 1fr); }
 }
 .card {
 background: linear-gradient(135deg, rgba(15,23,42,0.96), rgba(15,23,42,0.9));
 border-radius: 14px;
 border: 1px solid var(--border);
 box-shadow: 0 16px 40px rgba(15,23,42,0.8);
 padding: 12px;
 }
 .card h2 { font-size: 1.1rem; margin: 0 0 8px; display: flex; align-items: center; gap: 6px; }
 .card h2 span.icon {
 display: inline-flex; width: 20px; height: 20px; border-radius: 999px;
 align-items: center; justify-content: center;
 background: var(--accent-soft); color: var(--accent); font-size: 0.9rem;
 }
 .card small { display: block; font-size: 0.75rem; color: var(--text-soft); margin-bottom: 6px; }
 .drives-list { display: flex; flex-direction: column; gap: 6px; }
 .drive-item {
 border-radius: 10px; border: 1px solid var(--border);
 padding: 8px; background: radial-gradient(circle at top left, rgba(15,23,42,0.9), #020617);
 display: flex; flex-direction: column; gap: 4px;
 }
 .drive-header { display: flex; justify-content: space-between; align-items: center; gap: 6px; }
 .drive-name { font-weight: 600; font-size: 0.95rem; }
 .pill {
 padding: 2px 8px; border-radius: 999px; font-size: 0.7rem;
 border: 1px solid var(--border); background: rgba(15,23,42,0.9);
 color: var(--text-soft); white-space: nowrap;
 }
 .pill.ok {
 color: var(--ok); border-color: rgba(74,222,128,0.5); background: rgba(34,197,94,0.12);
 }
 .pill.bad {
 color: var(--danger); border-color: rgba(248,113,113,0.6); background: rgba(248,113,113,0.12);
 }
 .drive-body { display: flex; flex-wrap: wrap; gap: 4px 10px; font-size: 0.75rem; color: var(--text-soft); }
 .drive-body span.label { color: var(--text-soft); }
 .drive-body span.value { color: var(--text); font-weight: 500; }
 .drive-prefetch {
 margin-top: 4px; display: flex; align-items: center; justify-content: space-between;
 gap: 6px; font-size: 0.75rem;
 }
 .switch { position: relative; display: inline-flex; align-items: center; cursor: pointer; gap: 4px; font-size: 0.75rem; }
 .switch input { opacity: 0; width: 0; height: 0; position: absolute; }
 .switch-slider {
 width: 32px; height: 18px; background-color: #111827;
 border-radius: 999px; border: 1px solid var(--border);
 position: relative; transition: background-color 0.15s ease, border-color 0.15s ease;
 }
 .switch-slider::before {
 content: ""; position: absolute; width: 14px; height: 14px; border-radius: 999px;
 background-color: #f9fafb; left: 1px; top: 1px; transition: transform 0.15s ease;
 box-shadow: 0 1px 2px rgba(0,0,0,0.5);
 }
 .switch input:checked + .switch-slider {
 background-color: var(--accent-strong); border-color: rgba(56,189,248,0.8);
 }
 .switch input:checked + .switch-slider::before { transform: translateX(14px); }
 .small-input {
 width: 66px; padding: 4px 6px; border-radius: 6px;
 border: 1px solid var(--border); background: #020617;
 color: var(--text); font-size: 0.75rem;
 }
 .small-input:focus {
 outline: none; border-color: var(--accent); box-shadow: 0 0 0 1px rgba(56,189,248,0.5);
 }
 .timing-grid {
 display: grid; grid-template-columns: repeat(2, minmax(0, 1fr));
 gap: 6px; margin-top: 8px;
 }
 @media (min-width: 700px) {
 .timing-grid { grid-template-columns: repeat(4, minmax(0, 1fr)); }
 }
 .timing-item {
 background: rgba(15,23,42,0.85); border-radius: 8px;
 border: 1px solid var(--border); padding: 6px; font-size: 0.75rem;
 }
 .timing-item label { display: block; margin-bottom: 2px; color: var(--text-soft); }
 .timing-item input {
 width: 100%; padding: 4px 6px; border-radius: 6px;
 border: 1px solid var(--border); background: #020617;
 color: var(--text); font-size: 0.8rem;
 }
 .timing-item input:focus {
 outline: none; border-color: var(--accent); box-shadow: 0 0 0 1px rgba(56,189,248,0.5);
 }
 .actions { margin-top: 10px; display: flex; flex-wrap: wrap; gap: 6px; }
 button {
 border: none; border-radius: 999px; padding: 8px 14px;
 font-size: 0.8rem; font-weight: 500; cursor: pointer;
 display: inline-flex; align-items: center; gap: 6px;
 background: radial-gradient(circle at top left, #0ea5e9, #0369a1);
 color: #f9fafb; box-shadow: 0 10px 25px rgba(8,47,73,0.9);
 transition: transform 0.12s ease, box-shadow 0.12s ease, filter 0.12s ease;
 }
 button:hover {
 transform: translateY(-1px); filter: brightness(1.05);
 box-shadow: 0 12px 28px rgba(8,47,73,0.9);
 }
 button:active { transform: translateY(0); box-shadow: 0 6px 18px rgba(8,47,73,0.9); }
 button.secondary {
 background: radial-gradient(circle at top left, #111827, #020617);
 color: var(--text-soft); box-shadow: none; border: 1px solid var(--border);
 }
 .status-line {
 margin-top: 8px; font-size: 0.75rem; color: var(--text-soft);
 display: flex; flex-wrap: wrap; gap: 6px; align-items: center;
 }
 .status-dot {
 width: 8px; height: 8px; border-radius: 999px;
 background: #4ade80; box-shadow: 0 0 0 5px rgba(74,222,128,0.15);
 }
 .status-dot.err {
 background: #f97373; box-shadow: 0 0 0 5px rgba(248,113,113,0.18);
 }
 .status-msg { flex: 1; min-width: 0; }
 .footer { margin-top: 10px; text-align: center; font-size: 0.7rem; color: var(--text-soft); opacity: 0.7; }
 </style>
</head>
<body>
 <div class="page">
 <div class="app">
 <header>
 <h1>XF551 WiFi · Panel Maestro</h1>
 <p>Estado de disqueteras · Tiempos SIO · Prefetch remoto</p>
 </header>

 <div class="grid">
 <!-- Columna izquierda: Disqueteras -->
 <section class="card">
 <h2><span class="icon">💾</span> Disqueteras</h2>
 <small>Toca una unidad para ver su MAC, densidad y prefetch.</small>
 <div id="drives" class="drives-list"></div>
 </section>

 <!-- Columna derecha: Configuración -->
 <section class="card">
 <h2><span class="icon">⚙️</span> Configuración avanzada</h2>
 <small>Ajusta los tiempos SIO (μs) y el prefetch por unidad.</small>

 <div class="timing-grid">
 <div class="timing-item">
 <label for="ackToComplete">ACK → COMPLETE</label>
 <input type="number" id="ackToComplete" min="200" step="50" />
 </div>
 <div class="timing-item">
 <label for="completeToData">COMPLETE → DATA</label>
 <input type="number" id="completeToData" min="200" step="50" />
 </div>
 <div class="timing-item">
 <label for="dataToChk">DATA → CHK</label>
 <input type="number" id="dataToChk" min="50" step="50" />
 </div>
 <div class="timing-item">
 <label for="chunkDelay">Delay entre sectores</label>
 <input type="number" id="chunkDelay" min="200" step="50" />
 </div>
 </div>

 <div class="actions">
 <button id="btnSave">💾 Guardar configuración</button>
 <button id="btnReload" class="secondary">🔄 Recargar estado</button>
 </div>

 <div class="status-line">
 <div id="statusDot" class="status-dot"></div>
 <div id="statusMsg" class="status-msg">Listo.</div>
 </div>
 </section>
 </div>

 <div class="footer">
 ESP32 Master · XF551 WiFi · V1.1
 </div>
 </div>
 </div>

 <script>
 function $(id) { return document.getElementById(id); }

 function setStatus(msg, ok = true) {
 const dot = $("statusDot");
 const text = $("statusMsg");
 text.textContent = msg;
 if (ok) dot.classList.remove("err");
 else dot.classList.add("err");
 }

 function renderDrives(drives) {
 const container = $("drives");
 container.innerHTML = "";
 if (!drives || drives.length === 0) {
 container.innerHTML = "<small>No hay disqueteras registradas aún.</small>";
 return;
 }
 drives.forEach(d => {
 const div = document.createElement("div");
 div.className = "drive-item";
 const present = !!d.present;
 const supports256 = !!d.supports256;
 const prefetch = !!d.prefetch;
 const prefetchSectors = d.prefetchSectors || 0;
 const mac = d.mac || "—";

 div.innerHTML = `
 <div class="drive-header">
 <div class="drive-name">${d.dev || "DX"}</div>
 <div style="display:flex; gap:4px; align-items:center;">
 <span class="pill ${present ? "ok" : "bad"}">${present ? "ONLINE" : "OFFLINE"}</span>
 <span class="pill">${supports256 ? "DD 256B" : "SD 128B"}</span>
 </div>
 </div>
 <div class="drive-body">
 <div><span class="label">MAC: </span><span class="value">${mac}</span></div>
 <div><span class="label">Prefetch: </span><span class="value">${prefetch ? "Sí" : "No"}</span></div>
 <div><span class="label">Sectores pref.: </span><span class="value">${prefetchSectors}</span></div>
 </div>
 <div class="drive-prefetch">
 <label class="switch">
 <input type="checkbox" data-dev="${d.dev}" class="pf-toggle" ${prefetch ? "checked" : ""}>
 <span class="switch-slider"></span>
 </label>
 <div style="display:flex; align-items:center; gap:4px;">
 <span style="font-size:0.75rem;">Sectores:</span>
 <input
 type="number"
 min="0"
 max="16"
 step="1"
 value="${prefetchSectors}"
 class="small-input pf-count"
 data-dev="${d.dev}"
 />
 </div>
 </div>
 `;
 container.appendChild(div);
 });
 }

 async function loadStatus() {
 try {
 setStatus("Cargando estado...", true);
 const res = await fetch("/api/status");
 if (!res.ok) throw new Error("HTTP " + res.status);
 const data = await res.json();

 renderDrives(data.drives || []);

 if (data.timings) {
 $("ackToComplete").value = data.timings.ackToComplete ?? "";
 $("completeToData").value = data.timings.completeToData ?? "";
 $("dataToChk").value = data.timings.dataToChk ?? "";
 $("chunkDelay").value = data.timings.chunkDelay ?? "";
 }
 setStatus("Estado actualizado.");
 } catch (e) {
 console.error(e);
 setStatus("Error al obtener estado: " + e.message, false);
 }
 }

 function collectConfig() {
 const drives = [];
 document.querySelectorAll(".drive-item").forEach(div => {
 const dev = div.querySelector(".drive-name").textContent.trim();
 const toggle = div.querySelector(".pf-toggle");
 const inputCount = div.querySelector(".pf-count");
 let sectors = inputCount ? Number(inputCount.value) || 0 : 0;
 if (!toggle.checked) sectors = 0;
 drives.push({ dev, prefetch: sectors > 0, prefetchSectors: sectors });
 });

 const timings = {
 ackToComplete: Number($("ackToComplete").value) || 0,
 completeToData: Number($("completeToData").value) || 0,
 dataToChk: Number($("dataToChk").value) || 0,
 chunkDelay: Number($("chunkDelay").value) || 0
 };

 return { drives, timings };
 }

 function mapPrefetchByDev(drives) {
 const out = { D1: 0, D2: 0, D3: 0, D4: 0 };
 drives.forEach(d => {
 const dev = d.dev;
 if (out.hasOwnProperty(dev)) {
 out[dev] = d.prefetchSectors || 0;
 }
 });
 return out;
 }

 async function saveConfig() {
  try {
    const cfg = collectConfig();
    const pf = mapPrefetchByDev(cfg.drives);

    setStatus("Guardando configuración...", true);

    const urlTiming =
      `/set_timing?t_ack=${cfg.timings.ackToComplete}` +
      `&t_comp=${cfg.timings.completeToData}` +
      `&t_chk=${cfg.timings.dataToChk}` +
      `&t_chunk=${cfg.timings.chunkDelay}`;

    const urlPrefetch =
      `/set_prefetch?pf1=${pf.D1}&pf2=${pf.D2}&pf3=${pf.D3}&pf4=${pf.D4}`;

    const r1 = await fetch(urlTiming);
    if (!r1.ok) throw new Error("Error en /set_timing (" + r1.status + ")");

    const r2 = await fetch(urlPrefetch);
    if (!r2.ok) throw new Error("Error en /set_prefetch (" + r2.status + ")");

    setStatus("Configuración guardada. Recargando estado...");
    await loadStatus();
  } catch (e) {
    console.error(e);
    setStatus("Error al guardar configuración: " + e.message, false);
  }
 }

 document.addEventListener("DOMContentLoaded", () => {
 $("btnReload").addEventListener("click", loadStatus);
 $("btnSave").addEventListener("click", saveConfig);
 loadStatus();
 });
 </script>
</body>
</html>
)rawliteral";

// ========= Utilidades varias =========

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
  esp_err_t e = esp_now_add_peer(&p);
  if (e != ESP_OK){
    logf("[ESPNOW] esp_now_add_peer error=%d", (int)e);
  }
}

const uint8_t* macForDev(uint8_t dev){
  int idx = devIndex(dev);
  if (idx < 0) return BCAST_MAC;
  if (!slaves[idx].present) return BCAST_MAC;
  return slaves[idx].mac;
}

String formatMac(const uint8_t mac[6]){
  char buf[32];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// ========= NVS: cargar/guardar configuración =========

void saveTimingConfigToNvs(){
  if (!prefs.begin("xf551cfg", false)) {
    logf("[NVS] Error al abrir NVS para tiempos");
    return;
  }
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putUShort("t_ack",  T_ACK_TO_COMPLETE);
  prefs.putUShort("t_comp", T_COMPLETE_TO_DATA);
  prefs.putUShort("t_chk",  T_DATA_TO_CHK);
  prefs.putUShort("t_chd",  T_CHUNK_DELAY);
  prefs.end();
  logf("[NVS] Tiempos guardados");
}

void savePrefetchConfigToNvs(){
  if (!prefs.begin("xf551cfg", false)) {
    logf("[NVS] Error al abrir NVS para prefetch");
    return;
  }
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putUChar("pf1", prefetchCfg[0]);
  prefs.putUChar("pf2", prefetchCfg[1]);
  prefs.putUChar("pf3", prefetchCfg[2]);
  prefs.putUChar("pf4", prefetchCfg[3]);
  prefs.end();
  logf("[NVS] Prefetch guardado");
}

void loadConfigFromNvs(){
  if (!prefs.begin("xf551cfg", false)) {
    logf("[NVS] No se pudo abrir NVS, uso defaults");
    return;
  }

  uint32_t magic = prefs.getUInt("magic", 0);
  if (magic != CFG_MAGIC) {
    prefs.putUInt("magic", CFG_MAGIC);
    prefs.putUShort("t_ack",  T_ACK_TO_COMPLETE);
    prefs.putUShort("t_comp", T_COMPLETE_TO_DATA);
    prefs.putUShort("t_chk",  T_DATA_TO_CHK);
    prefs.putUShort("t_chd",  T_CHUNK_DELAY);
    prefs.putUChar("pf1", prefetchCfg[0]);
    prefs.putUChar("pf2", prefetchCfg[1]);
    prefs.putUChar("pf3", prefetchCfg[2]);
    prefs.putUChar("pf4", prefetchCfg[3]);
    prefs.end();
    logf("[NVS] Config inicial grabada (defaults)");
    return;
  }

  T_ACK_TO_COMPLETE   = prefs.getUShort("t_ack",  T_ACK_TO_COMPLETE);
  T_COMPLETE_TO_DATA  = prefs.getUShort("t_comp", T_COMPLETE_TO_DATA);
  T_DATA_TO_CHK       = prefs.getUShort("t_chk",  T_DATA_TO_CHK);
  T_CHUNK_DELAY       = prefs.getUShort("t_chd",  T_CHUNK_DELAY);

  prefetchCfg[0] = prefs.getUChar("pf1", prefetchCfg[0]);
  prefetchCfg[1] = prefs.getUChar("pf2", prefetchCfg[1]);
  prefetchCfg[2] = prefs.getUChar("pf3", prefetchCfg[2]);
  prefetchCfg[3] = prefs.getUChar("pf4", prefetchCfg[3]);

  prefs.end();

  logf("[NVS] Config cargada:");
  logf(" T_ACK_TO_COMPLETE=%u",  T_ACK_TO_COMPLETE);
  logf(" T_COMPLETE_TO_DATA=%u", T_COMPLETE_TO_DATA);
  logf(" T_DATA_TO_CHK=%u",      T_DATA_TO_CHK);
  logf(" T_CHUNK_DELAY=%u",      T_CHUNK_DELAY);
  logf(" Prefetch D1..D4=%u,%u,%u,%u",
       prefetchCfg[0], prefetchCfg[1], prefetchCfg[2], prefetchCfg[3]);
}

// ========= Web UI Handlers =========

void handleRoot(){
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleApiStatus(){
  Serial.println("[WEB] /api/status llamado");

  String json;
  json.reserve(1024);
  json += F("{\"drives\":[");
  unsigned long now = millis();

  bool first = true;
  for (uint8_t dev = DEV_MIN; dev <= DEV_MAX; dev++){
    int idx = devIndex(dev);
    if (idx < 0) continue;

    if (!first) json += ',';
    first = false;

    json += F("{\"dev\":\"");
    json += devName(dev);
    json += F("\",");

    json += F("\"present\":");
    json += (slaves[idx].present ? F("true") : F("false"));
    json += F(",");

    json += F("\"supports256\":");
    json += (slaves[idx].supports256 ? F("true") : F("false"));
    json += F(",");

    json += F("\"mac\":\"");
    if (slaves[idx].present){
      json += formatMac(slaves[idx].mac);
    }
    json += F("\",");

    json += F("\"prefetch\":");
    json += (prefetchCfg[idx] > 0 ? F("true") : F("false"));
    json += F(",");

    json += F("\"prefetchSectors\":");
    json += String(prefetchCfg[idx]);

    json += F(",\"lastSeenMs\":");
    if (slaves[idx].present){
      json += String(now - slaves[idx].lastSeen);
    } else {
      json += F("0");
    }
    json += F("}");
  }

  json += F("],\"timings\":{");
  json += F("\"ackToComplete\":");  json += String(T_ACK_TO_COMPLETE);  json += F(",");
  json += F("\"completeToData\":"); json += String(T_COMPLETE_TO_DATA); json += F(",");
  json += F("\"dataToChk\":");      json += String(T_DATA_TO_CHK);      json += F(",");
  json += F("\"chunkDelay\":");     json += String(T_CHUNK_DELAY);
  json += F("}}");

  Serial.print("[WEB] /api/status JSON = ");
  Serial.println(json);

  server.send(200, "application/json", json);
}

void handleSetTiming(){
  if (server.hasArg("t_ack")){
    T_ACK_TO_COMPLETE = (uint16_t)server.arg("t_ack").toInt();
  }
  if (server.hasArg("t_comp")){
    T_COMPLETE_TO_DATA = (uint16_t)server.arg("t_comp").toInt();
  }
  if (server.hasArg("t_chk")){
    T_DATA_TO_CHK = (uint16_t)server.arg("t_chk").toInt();
  }
  if (server.hasArg("t_chunk")){
    T_CHUNK_DELAY = (uint16_t)server.arg("t_chunk").toInt();
  }

  logf("[WEB] Tiempos: ACK->COMP=%u, COMP->DATA=%u, DATA->CHK=%u, CHUNK=%u",
       T_ACK_TO_COMPLETE, T_COMPLETE_TO_DATA, T_DATA_TO_CHK, T_CHUNK_DELAY);

  saveTimingConfigToNvs();
  server.send(200, "text/plain", "OK");
}

void handleSetPrefetch(){
  for (int idx = 0; idx < 4; idx++){
    String argName = "pf" + String(idx + 1);
    if (server.hasArg(argName)){
      int v = server.arg(argName).toInt();
      if (v < 0) v = 0;
      if (v > MAX_PREFETCH_SECTORS) v = MAX_PREFETCH_SECTORS;
      prefetchCfg[idx] = (uint8_t)v;
    }
  }

  logf("[WEB] Prefetch: D1=%u, D2=%u, D3=%u, D4=%u",
       prefetchCfg[0], prefetchCfg[1], prefetchCfg[2], prefetchCfg[3]);

  savePrefetchConfigToNvs();
  server.send(200, "text/plain", "OK");
}

// ========= Lógica común de recepción/envío ESP-NOW =========

static void handleEspNowRecvCommon(const uint8_t* mac, const uint8_t* data, int len){
  if (!data || len <= 0) return;
  uint8_t type = data[0];

  if (type == TYPE_HELLO && len >= 3){
    uint8_t dev = data[1];
    bool s256 = (data[2] != 0);

    int idx = devIndex(dev);
    if (idx >= 0 && mac){
      slaves[idx].present     = true;
      slaves[idx].supports256 = s256;
      memcpy(slaves[idx].mac, mac, 6);
      slaves[idx].lastSeen = millis();
      ensurePeer(slaves[idx].mac);
      logf("[HELLO] %s presente DD=%u MAC=%s",
           devName(dev), s256, formatMac(slaves[idx].mac).c_str());
    }
    return;
  }

  if (type == TYPE_SECTOR_CHUNK && len >= 6){
    uint8_t dev = data[1];
    uint16_t sec = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    uint8_t ci = data[4];
    uint8_t cc = data[5];
    int payload = len - 6;

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
    lastAckOk   = true;
    lastAckDone = true;
    return;
  } else if (type == TYPE_NAK){
    logf("[ESPNOW] NAK recibido");
    lastAckOk   = false;
    lastAckDone = true;
    return;
  }
}

static void handleEspNowSentCommon(esp_now_send_status_t status){
  if (status == ESP_NOW_SEND_SUCCESS)
    Serial.println("[ESPNOW] Envío OK");
  else
    Serial.println("[ESPNOW] Falla en envío");
}

#if ESP_IDF_VERSION_MAJOR >= 5

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  const uint8_t *mac = info ? info->src_addr : nullptr;
  handleEspNowRecvCommon(mac, data, len);
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status){
  (void)info;
  handleEspNowSentCommon(status);
}

#else

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){
  handleEspNowRecvCommon(mac, data, len);
}

void onDataSent(const uint8_t* mac, esp_now_send_status_t status){
  (void)mac;
  handleEspNowSentCommon(status);
}

#endif

// ========= Envío de comandos al SLAVE =========

// [0]=TYPE_CMD_FRAME,[1]=cmd,[2]=dev,[3]=secL,[4]=secH,[5]=dens,[6]=prefetchCount
bool sendCmd(uint8_t dev, uint8_t cmd, uint16_t sec, bool dd){
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
  logHex("[ESPNOW][TX] bytes:", p, sizeof(p));

  esp_err_t e = esp_now_send(dest, p, sizeof(p));
  if (e != ESP_OK){
    logf("[ESPNOW] esp_now_send error=%d", (int)e);
  }
  return (e == ESP_OK);
}

void sendWriteDataToSlave(uint8_t dev, uint16_t sec, const uint8_t* buf, int len){
  uint8_t pkt[6 + CHUNK_PAYLOAD];
  int cc = (len + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD;
  const uint8_t* dest = macForDev(dev);

  for (int i = 0; i < cc; i++){
    int off = i * CHUNK_PAYLOAD;
    int n = (len - off > CHUNK_PAYLOAD) ? CHUNK_PAYLOAD : (len - off);

    pkt[0] = TYPE_SECTOR_CHUNK;
    pkt[1] = dev;
    pkt[2] = (uint8_t)(sec & 0xFF);
    pkt[3] = (uint8_t)(sec >> 8);
    pkt[4] = (uint8_t)i;
    pkt[5] = (uint8_t)cc;
    memcpy(pkt + 6, buf + off, n);

    logf("[ESPNOW][TX] WRITE_CHUNK dev=%s sec=%u ci=%d/%d len=%d",
         devName(dev), sec, i, cc, n);
    esp_err_t e = esp_now_send(dest, pkt, 6 + n);
    if (e != ESP_OK){
      logf("[ESPNOW] WRITE esp_now_send error=%d", (int)e);
    }
    delay(2);
  }
}

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

int readRemoteStatus(uint8_t dev, bool dd, uint8_t* out4){
  (void)dd;

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

  logf("[STATUS] Solicitando STATUS a %s", devName(dev));

  if (!sendCmd(dev, 0x53, 0, false)){
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

// FORMAT genérico (0x21 / 0x22) – siempre 128 bytes de resultado
int formatRemoteDisk(uint8_t dev, uint8_t cmd, uint16_t param, bool dd, uint8_t* out128){
  (void)dd; // el resultado es siempre 128 bytes, independiente de densidad

  int idx = devIndex(dev);
  if (idx < 0 || !slaves[idx].present){
    logf("[FORMAT] %s no presente", devName(dev));
    return 0;
  }

  replyReady = false;
  replySec   = 0;
  replyLen   = 0;
  expectedChunks = receivedChunks = 0;

  currentOpType = OP_FORMAT;
  currentOpDev  = dev;
  currentOpSec  = param;

  logf("[FORMAT] Solicitando cmd=0x%02X a %s param=%u",
       cmd, devName(dev), param);

  if (!sendCmd(dev, cmd, param, false)){
    logf("[FORMAT] sendCmd 0x%02X falló para %s", cmd, devName(dev));
    currentOpType = OP_NONE;
    return 0;
  }

  unsigned long t0 = millis();
  while (millis() - t0 < 90000){
    if (replyReady) break;
    delay(5);
  }

  currentOpType = OP_NONE;

  if (!replyReady){
    logf("[FORMAT] Timeout esperando resultado FORMAT (cmd=0x%02X) de %s",
         cmd, devName(dev));
    return 0;
  }

  if (replyLen < SECTOR_128){
    logf("[FORMAT] Respuesta corta (%u bytes) de %s", replyLen, devName(dev));
  }

  int n = (replyLen > SECTOR_128) ? SECTOR_128 : replyLen;
  memcpy(out128, replyBuf, n);
  logHex("[FORMAT] bytes:", out128, n);
  return n;
}

// Helper específico para 0x22 (por claridad)
int formatMediumRemoteDisk(uint8_t dev, uint16_t param, uint8_t* out128){
  return formatRemoteDisk(dev, 0x22, param, false, out128);
}

// === READ PERCOM BLOCK remoto (0x4E) – 12 bytes ===
int readRemotePercomBlock(uint8_t dev, uint8_t* out12){
  int idx = devIndex(dev);
  if (idx < 0 || !slaves[idx].present){
    logf("[PERCOM] %s no presente", devName(dev));
    return 0;
  }

  replyReady = false;
  replySec   = 0;
  replyLen   = 0;
  expectedChunks = receivedChunks = 0;

  currentOpType = OP_PERCOM;
  currentOpDev  = dev;
  currentOpSec  = 0;

  logf("[PERCOM] Solicitando READ PERCOM (0x4E) a %s", devName(dev));

  if (!sendCmd(dev, 0x4E, 0, false)){
    logf("[PERCOM] sendCmd 0x4E falló para %s", devName(dev));
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
    logf("[PERCOM] Timeout esperando READ PERCOM de %s", devName(dev));
    return 0;
  }

  if (replyLen < PERCOM_BLOCK_LEN){
    logf("[PERCOM] Respuesta corta (%u bytes) de %s", replyLen, devName(dev));
    return 0;
  }

  memcpy(out12, replyBuf, PERCOM_BLOCK_LEN);
  logHex("[PERCOM] READ bytes:", out12, PERCOM_BLOCK_LEN);
  return PERCOM_BLOCK_LEN;
}

// === WRITE PERCOM BLOCK remoto (0x4F) – 12 bytes ===
bool writeRemotePercomBlock(uint8_t dev, const uint8_t* data, int len){
  int idx = devIndex(dev);
  if (idx < 0 || !slaves[idx].present){
    logf("[PERCOM] %s no presente", devName(dev));
    return false;
  }

  if (len <= 0){
    logf("[PERCOM] len inválido=%d para %s", len, devName(dev));
    return false;
  }

  uint8_t tmp[PERCOM_BLOCK_LEN];
  memset(tmp, 0, sizeof(tmp));
  int copyLen = (len > PERCOM_BLOCK_LEN) ? PERCOM_BLOCK_LEN : len;
  memcpy(tmp, data, copyLen);

  logf("[PERCOM] Enviando WRITE PERCOM a SLAVE %s len=%d", devName(dev), PERCOM_BLOCK_LEN);

  const uint16_t PERCOM_SEC_MAGIC = 0xFFFF;
  sendWriteDataToSlave(dev, PERCOM_SEC_MAGIC, tmp, PERCOM_BLOCK_LEN);

  lastAckDone = false;
  lastAckOk   = false;

  if (!sendCmd(dev, 0x4F, 0, false)){
    logf("[PERCOM] sendCmd 0x4F falló para %s", devName(dev));
    return false;
  }

  unsigned long t0 = millis();
  while (millis() - t0 < 15000){
    if (lastAckDone) break;
    delay(5);
  }

  if (!lastAckDone){
    logf("[PERCOM] Timeout esperando ACK desde SLAVE %s", devName(dev));
    return false;
  }

  if (!lastAckOk){
    logf("[PERCOM] SLAVE NAK %s", devName(dev));
    return false;
  }

  logf("[PERCOM] WRITE OK remoto %s", devName(dev));
  return true;
}

// WRITE remoto – sector completo (128/256) + ACK/NAK
bool writeRemoteSector(uint8_t dev, uint16_t sec, bool dd, const uint8_t* data, int len, bool verify){
  int idx = devIndex(dev);
  if (idx < 0 || !slaves[idx].present){
    logf("[WRITE] %s no presente", devName(dev));
    return false;
  }

  if (len <= 0){
    logf("[WRITE] len inválido=%d para %s sec=%u", len, devName(dev), sec);
    return false;
  }

  logf("[WRITE] Enviando datos a SLAVE %s sec=%u dd=%s len=%d",
       devName(dev), sec, dd ? "DD" : "SD", len);

  sendWriteDataToSlave(dev, sec, data, len);

  lastAckDone = false;
  lastAckOk   = false;

  uint8_t baseCmd = verify ? 0x57 : 0x50;

  if (!sendCmd(dev, baseCmd, sec, dd)){
    logf("[WRITE] sendCmd falló para %s", devName(dev));
    return false;
  }

  unsigned long t0 = millis();
  while (millis() - t0 < 15000){
    if (lastAckDone) break;
    delay(5);
  }

  if (!lastAckDone){
    logf("[WRITE] Timeout esperando ACK desde SLAVE %s sec=%u", devName(dev), sec);
    return false;
  }

  if (!lastAckOk){
    logf("[WRITE] SLAVE NAK %s sec=%u", devName(dev), sec);
    return false;
  }

  logf("[WRITE] OK remoto %s sec=%u", devName(dev), sec);
  return true;
}

// ========= SIO: envío de DATA+CHK al Atari =========

void sendAtariData(const uint8_t* buf, int len){
  delayMicroseconds(T_ACK_TO_COMPLETE);
  SerialSIO.write(SIO_COMPLETE);
  SerialSIO.flush();
  logf("[TX->Atari] COMPLETE");

  delayMicroseconds(T_COMPLETE_TO_DATA);
  SerialSIO.write(buf, len);
  SerialSIO.flush();
  logf("[TX->Atari] DATA len=%d", len);
  logHex("[TX->Atari] DATA bytes:", buf, len);

  delayMicroseconds(T_DATA_TO_CHK);
  uint8_t chk = sioChecksum(buf, len);
  SerialSIO.write(chk);
  SerialSIO.flush();
  logf("[TX->Atari] CHK=%02X", chk);

  delayMicroseconds(T_CHUNK_DELAY);
}

// Envío sólo COMPLETE/ERROR al Atari (para WRITE/PERCOM 4F)
void sendAtariComplete(bool ok){
  if (ok){
    delayMicroseconds(T_ACK_TO_COMPLETE);
    SerialSIO.write(SIO_COMPLETE);
    SerialSIO.flush();
    logf("[TX->Atari] COMPLETE (WRITE/PERCOM)");
  } else {
    SerialSIO.write(SIO_ERROR);
    SerialSIO.flush();
    logf("[TX->Atari] ERROR (WRITE/PERCOM)");
  }
  delayMicroseconds(T_CHUNK_DELAY);
}

// Lee DATA+CHK desde el Atari (WRITE 128/256 o PERCOM 12)
bool readAtariWriteData(uint8_t* buf, int len, bool strictChk){
  int idx = 0;
  unsigned long t0 = millis();
  while (idx < len && (millis() - t0) < 1000){
    if (SerialSIO.available()){
      buf[idx++] = SerialSIO.read();
    } else {
      delay(1);
    }
  }

  if (idx != len){
    logf("[SIO] WRITE: esperados %d bytes, recibidos %d", len, idx);
    if (strictChk) return false;
  }

  uint8_t recvChk = 0;
  bool gotChk = false;
  unsigned long t1 = millis();
  while ((millis() - t1) < 200){
    if (SerialSIO.available()){
      recvChk = SerialSIO.read();
      gotChk = true;
      break;
    }
    delay(1);
  }

  if (!gotChk){
    logf("[SIO] WRITE: checksum no recibido");
    if (strictChk) return false;
  } else {
    uint8_t calcChk = sioChecksum(buf, len);
    if (recvChk != calcChk){
      logf("[SIO] WRITE: checksum inválido recv=%02X calc=%02X", recvChk, calcChk);
      if (strictChk) return false;
    }
  }

  logHex("[SIO] WRITE DATA:", buf, len);
  return true;
}

// ========= Procesar cabecera SIO del Atari =========

void handleSioHeader(uint8_t f[5]){
  uint8_t dev     = f[0];
  uint8_t cmd     = f[1];
  uint8_t aux1    = f[2];
  uint8_t aux2    = f[3];
  uint8_t recvChk = f[4];

  logf("[HDR] %02X %02X %02X %02X %02X", f[0], f[1], f[2], f[3], f[4]);

  if (dev < DEV_MIN || dev > DEV_MAX){
    logf("[SIO] Header hacia dev=%02X (fuera de rango), ignorado", dev);
    return;
  }

  uint8_t calcChk = sioChecksum(f, 4);
  if (recvChk != calcChk){
    logf("[SIO] Checksum inválido (recv=%02X calc=%02X) para %s -> ignorado",
         recvChk, calcChk, devName(dev));
    return;
  }

  bool dd      = (cmd & 0x80) != 0;
  uint8_t base = cmd & 0x7F;
  uint16_t sec = (uint16_t)aux1 | ((uint16_t)aux2 << 8);

  // READ SECTOR
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

  // STATUS
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

  // FORMAT INDEX (0x21)
  if (base == 0x21){
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    logf("[TX->Atari] ACK (FORMAT 0x21) %s aux=%u", devName(dev), sec);
    delayMicroseconds(800);

    uint8_t fmt[SECTOR_128];
    int n = formatRemoteDisk(dev, 0x21, sec, dd, fmt);
    if (n <= 0){
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      logf("[SIO] FORMAT 0x21 remoto falló %s aux=%u", devName(dev), sec);
      return;
    }

    sendAtariData(fmt, n);
    return;
  }

  // FORMAT MEDIUM / ENHANCED (0x22)
  if (base == 0x22){
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    logf("[TX->Atari] ACK (FORMAT MEDIUM 0x22) %s aux=%u", devName(dev), sec);
    delayMicroseconds(800);

    uint8_t fmt[SECTOR_128];
    int n = formatMediumRemoteDisk(dev, sec, fmt);
    if (n <= 0){
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      logf("[SIO] FORMAT MEDIUM remoto falló %s aux=%u", devName(dev), sec);
      return;
    }

    sendAtariData(fmt, n);
    return;
  }

  // READ PERCOM BLOCK (0x4E) – 12 bytes
  if (base == 0x4E){
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    logf("[TX->Atari] ACK (READ PERCOM 0x4E) %s", devName(dev));
    delayMicroseconds(800);

    uint8_t blk[PERCOM_BLOCK_LEN];
    int n = readRemotePercomBlock(dev, blk);
    if (n != PERCOM_BLOCK_LEN){
      SerialSIO.write(SIO_ERROR);
      SerialSIO.flush();
      logf("[SIO] PERCOM READ remoto falló %s", devName(dev));
      return;
    }

    sendAtariData(blk, PERCOM_BLOCK_LEN);
    return;
  }

  // WRITE PERCOM BLOCK (0x4F) – 12 bytes
  if (base == 0x4F){
    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    logf("[TX->Atari] ACK (WRITE PERCOM 0x4F) %s", devName(dev));
    delayMicroseconds(800);

    uint8_t blk[PERCOM_BLOCK_LEN];
    if (!readAtariWriteData(blk, PERCOM_BLOCK_LEN, true)){
      logf("[SIO] Fallo leyendo DATA PERCOM (12 bytes) desde Atari %s", devName(dev));
      sendAtariComplete(false);
      return;
    }

    bool ok = writeRemotePercomBlock(dev, blk, PERCOM_BLOCK_LEN);
    sendAtariComplete(ok);
    return;
  }

  // WRITE SECTOR (0x50 / 0x57)
  if (base == 0x50 || base == 0x57){
    bool verify = (base == 0x57);

    SerialSIO.write(SIO_ACK);
    SerialSIO.flush();
    logf("[TX->Atari] ACK (WRITE%s) %s sec=%u",
         verify ? "+VERIFY" : "", devName(dev), sec);
    delayMicroseconds(800);

    uint8_t buf[MAX_SECTOR_BYTES];
    int expectedLen = dd ? SECTOR_256 : SECTOR_128;

    // 0x50 estricto, 0x57 tolerante (para no matar FORMAT)
    bool strictChk = !verify;

    if (!readAtariWriteData(buf, expectedLen, strictChk)){
      logf("[SIO] Fallo leyendo DATA WRITE desde Atari %s sec=%u", devName(dev), sec);
      sendAtariComplete(false);
      return;
    }

    bool ok = writeRemoteSector(dev, sec, dd, buf, expectedLen, verify);
    sendAtariComplete(ok);
    return;
  }

  // Otros comandos no soportados
  delayMicroseconds(800);
  SerialSIO.write(SIO_NAK);
  SerialSIO.flush();
  logf("[SIO] Cmd no soportado 0x%02X para %s => NAK", base, devName(dev));
}

// ========= Máquina de estados para COMMAND =========

bool     inCommand       = false;
uint8_t  hdrBuf[5];
uint8_t  hdrCount        = 0;
uint32_t cmdStartMicros  = 0;

// ========= SETUP / LOOP =========

void setup(){
  Serial.begin(115200);
  Serial.println("\n[MASTER] STATUS+READ+FORMAT+WRITE+PERCOM + WEB UI + PREFETCH + NVS D1..D4");

  pinMode(SIO_COMMAND, INPUT_PULLUP);
  SerialSIO.begin(19200, SERIAL_8N1, SIO_DATA_IN, SIO_DATA_OUT, false);

  for (int i = 0; i < 4; i++){
    slaves[i].present     = false;
    slaves[i].supports256 = false;
    memset(slaves[i].mac, 0, 6);
    slaves[i].lastSeen = 0;
  }

  loadConfigFromNvs();

  WiFi.mode(WIFI_AP_STA);
  const char* apSsid = "XF551_MASTER";
  const char* apPass = "xf551wifi";
  WiFi.softAP(apSsid, apPass);

  Serial.print("[WIFI] AP SSID: ");
  Serial.println(apSsid);
  Serial.print("[WIFI] AP IP: ");
  Serial.println(WiFi.softAPIP());

  if (esp_now_init() != ESP_OK){
    Serial.println("[ERR] esp_now_init falló");
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  ensurePeer(BCAST_MAC);

  server.on("/",           handleRoot);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/set_timing", HTTP_GET, handleSetTiming);
  server.on("/set_prefetch", HTTP_GET, handleSetPrefetch);
  server.begin();
  Serial.println("[WEB] Servidor HTTP iniciado en puerto 80");
}

void loop(){
  server.handleClient();

  unsigned long now = millis();
  for (int i = 0; i < 4; i++){
    if (slaves[i].present && (now - slaves[i].lastSeen > 60000)){
      slaves[i].present = false;
    }
  }

  int cmdLevel = digitalRead(SIO_COMMAND);

  if (!inCommand && cmdLevel == LOW){
    inCommand      = true;
    hdrCount       = 0;
    cmdStartMicros = micros();
  }

  if (inCommand){
    while (SerialSIO.available() && hdrCount < 5){
      hdrBuf[hdrCount++] = SerialSIO.read();
    }

    if (hdrCount == 5){
      handleSioHeader(hdrBuf);
      inCommand = false;
      while (SerialSIO.available()) SerialSIO.read();
    }

    if (cmdLevel == HIGH && hdrCount < 5 && (micros() - cmdStartMicros) > 3000){
      inCommand = false;
      while (SerialSIO.available()) SerialSIO.read();
    }
  }

  delay(1);
}
