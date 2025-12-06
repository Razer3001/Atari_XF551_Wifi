#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <stdarg.h>

#ifndef ESP_IDF_VERSION_MAJOR
#define ESP_IDF_VERSION_MAJOR 4
#endif

// ===== Protocol =====
#define TYPE_CMD_FRAME    0x01
#define TYPE_SECTOR_CHUNK 0x10
#define TYPE_ACK          0x11
#define TYPE_NAK          0x12
#define TYPE_HELLO        0x20

#define UART_SYNC 0x55

// D1..D4
#define DEV_MIN 0x31
#define DEV_MAX 0x34

// Prefetch máximo
#define MAX_PREFETCH_SECTORS 4

// Broadcast MAC
const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Último SLAVE conocido
uint8_t g_lastSlave[6] = {0};
bool    g_haveSlave    = false;

// UART2 (con RP2040)
const int PIN_RP_RX = 16; // RX2
const int PIN_RP_TX = 17; // TX2

// ========= Estado de SLAVES =========
struct SlaveInfo {
  bool present;
  bool supports256;
  uint8_t mac[6];
  unsigned long lastSeen;
};

SlaveInfo slaves[4]; // D1..D4

// Prefetch configurado por unidad
uint8_t prefetchCfg[4] = {1, 1, 1, 1};

// ========= Timings SIO (µs) – config compartida con RP (por ahora solo NVS/UI) =========
uint16_t T_ACK_TO_COMPLETE   = 600;
uint16_t T_COMPLETE_TO_DATA  = 400;
uint16_t T_DATA_TO_CHK       = 80;
uint16_t T_CHUNK_DELAY       = 600;

// ========= WebServer & NVS =========
WebServer server(80);
Preferences prefs;
const uint32_t CFG_MAGIC = 0xCAFEBABE;

// ========= HTML UI sencillo =========

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
 <p>Estado de disqueteras · Tiempos SIO · Prefetch remoto (bridge)</p>
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
 <small>Tiempos SIO (guardados en NVS) y prefetch por unidad.</small>

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
 ESP32 Master Bridge · XF551 WiFi · V1.1
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

// ========= Utils =========

void logf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
}

uint8_t calcChecksum(const uint8_t* buf, int len) {
  uint16_t s = 0;
  for (int i = 0; i < len; i++) s += buf[i];
  return (uint8_t)s;
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

String formatMac(const uint8_t mac[6]){
  char buf[32];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

const uint8_t* slaveMac() {
  return g_haveSlave ? g_lastSlave : BCAST_MAC;
}

bool sendEspNow(const uint8_t* mac, const uint8_t* data, int len) {
  ensurePeer(mac);
  esp_err_t r = esp_now_send(mac, data, len);
  return (r == ESP_OK);
}

bool sendEspToSlave(const uint8_t* data, int len) {
  return sendEspNow(slaveMac(), data, len);
}

// ========= Pendiente de WRITE desde el RP =========

struct PendingWriteFromRP {
  bool    active;
  uint8_t dev;
  uint16_t sec;
};

PendingWriteFromRP g_pendingWriteRP = { false, 0, 0 };

// ========= NUEVO: último comando importante (READ/WRITE/FORMAT/STATUS/PERCOM) =========
struct LastMasterOp {
  bool    active;
  uint8_t cmd;   // comando SIO original (0x52, 0x50, 0x21, etc.)
  uint8_t dev;   // 0x31..0x34
  uint16_t sec;  // sector lógico (o 0/0xFFFF según cmd)
};

LastMasterOp g_lastMasterOp = { false, 0, 0, 0 };

// ========= NVS: cargar/guardar =========

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

// ========= Web Handlers =========

void handleRoot(){
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleApiStatus(){
  // Construimos JSON a mano para que calce con la página original
  String json = "{";

  // ---- drives ----
  json += "\"drives\":[";
  bool first = true;
  for (int i = 0; i < 4; i++) {
    uint8_t devCode = DEV_MIN + i; // 0x31..0x34
    const char* name = devName(devCode);

    if (!first) json += ",";
    first = false;

    bool present     = slaves[i].present;
    bool supports256 = slaves[i].supports256;
    uint8_t pf       = prefetchCfg[i];
    String macStr    = formatMac(slaves[i].mac);
    unsigned long lastSeen = slaves[i].lastSeen;  // millis() en el momento del último HELLO

    json += "{";
    json += "\"dev\":\"" + String(name) + "\",";
    json += "\"present\":";     json += present ? "true" : "false";      json += ",";
    json += "\"supports256\":"; json += supports256 ? "true" : "false";  json += ",";
    json += "\"prefetch\":";    json += (pf > 0 ? "true" : "false");     json += ",";
    json += "\"prefetchSectors\":" + String((int)pf) + ",";
    json += "\"mac\":\"" + macStr + "\",";
    json += "\"lastSeen\":" + String(lastSeen);
    json += "}";
  }
  json += "],";

  // ---- timings ----
  json += "\"timings\":{";
  json += "\"ackToComplete\":"  + String((int)T_ACK_TO_COMPLETE)   + ",";
  json += "\"completeToData\":" + String((int)T_COMPLETE_TO_DATA)  + ",";
  json += "\"dataToChk\":"      + String((int)T_DATA_TO_CHK)       + ",";
  json += "\"chunkDelay\":"     + String((int)T_CHUNK_DELAY);
  json += "}";

  json += "}";

  server.send(200, "application/json", json);
}


void handleSetTiming(){
  // Tomamos los valores que vengan, si faltan dejamos los actuales
  if (server.hasArg("t_ack")) {
    T_ACK_TO_COMPLETE = (uint16_t) server.arg("t_ack").toInt();
  }
  if (server.hasArg("t_comp")) {
    T_COMPLETE_TO_DATA = (uint16_t) server.arg("t_comp").toInt();
  }
  if (server.hasArg("t_chk")) {
    T_DATA_TO_CHK = (uint16_t) server.arg("t_chk").toInt();
  }
  // La página usa t_chunk -> lo mapeamos a T_CHUNK_DELAY
  if (server.hasArg("t_chunk")) {
    T_CHUNK_DELAY = (uint16_t) server.arg("t_chunk").toInt();
  }

  // Guardar en NVS con tus funciones
  saveTimingConfigToNvs();

  logf("[WEB] /set_timing: t_ack=%u t_comp=%u t_chk=%u t_chunk=%u",
       T_ACK_TO_COMPLETE, T_COMPLETE_TO_DATA, T_DATA_TO_CHK, T_CHUNK_DELAY);

  server.send(200, "text/plain", "OK");
}


void handleSetPrefetch(){
  int pf1 = server.hasArg("pf1") ? server.arg("pf1").toInt() : prefetchCfg[0];
  int pf2 = server.hasArg("pf2") ? server.arg("pf2").toInt() : prefetchCfg[1];
  int pf3 = server.hasArg("pf3") ? server.arg("pf3").toInt() : prefetchCfg[2];
  int pf4 = server.hasArg("pf4") ? server.arg("pf4").toInt() : prefetchCfg[3];

  auto clampPf = [](int v)->uint8_t {
    if (v < 0) v = 0;
    if (v > MAX_PREFETCH_SECTORS) v = MAX_PREFETCH_SECTORS;
    return (uint8_t)v;
  };

  prefetchCfg[0] = clampPf(pf1);
  prefetchCfg[1] = clampPf(pf2);
  prefetchCfg[2] = clampPf(pf3);
  prefetchCfg[3] = clampPf(pf4);

  savePrefetchConfigToNvs();

  logf("[WEB] /set_prefetch: D1=%u D2=%u D3=%u D4=%u",
       prefetchCfg[0], prefetchCfg[1], prefetchCfg[2], prefetchCfg[3]);

  server.send(200, "text/plain", "OK");
}


// ===== UART TX -> RP2040 =====
void sendUartFrameToRP(const uint8_t* payload, uint8_t len) {
  if (len == 0) return;
  uint8_t chk = calcChecksum(payload, len);

  Serial2.write(UART_SYNC);
  Serial2.write(len);
  Serial2.write(payload, len);
  Serial2.write(chk);
  Serial2.flush();

  Serial.print(F("[MASTER] UART->RP len="));
  Serial.print(len);
  Serial.print(F(" tipo=0x"));
  if (payload[0] < 0x10) Serial.print('0');
  Serial.println(payload[0], HEX);
}

// ===== UART RX <- RP2040 FSM =====
static uint8_t  uartState    = 0;
static uint8_t  uartLen      = 0;
static uint8_t  uartIdx      = 0;
static uint32_t uartLastTime = 0;
static uint8_t  uartBuf[255];

void pollUartFromRP() {
  while (Serial2.available() > 0) {
    uint8_t b = (uint8_t)Serial2.read();
    uartLastTime = millis();

    Serial.print(F("[MASTER UART FSM] state="));
    Serial.print(uartState);
    Serial.print(F(" byte=0x"));
    if (b < 0x10) Serial.print('0');
    Serial.println(b, HEX);

    switch (uartState) {
      case 0:
        if (b == UART_SYNC) uartState = 1;
        break;
      case 1:
        uartLen = b;
        if (uartLen == 0) uartState = 0;
        else {
          uartIdx   = 0;
          uartState = 2;
        }
        break;
      case 2:
        uartBuf[uartIdx++] = b;
        if (uartIdx >= uartLen) uartState = 3;
        break;
      case 3: {
        uint8_t chk = b;
        uint8_t sum = calcChecksum(uartBuf, uartLen);
        if (chk != sum) {
          Serial.println(F("[MASTER] UART: checksum inválido, descartando frame."));
          uartState = 0;
        } else {
          uint8_t type = uartBuf[0];
          Serial.print(F("[MASTER] Frame UART válido recibido, len="));
          Serial.print(uartLen);
          Serial.print(F(" tipo=0x"));
          if (type < 0x10) Serial.print('0');
          Serial.println(type, HEX);

          switch (type) {
            case TYPE_CMD_FRAME: {
              if (uartLen >= 7) {
                uint8_t cmd  = uartBuf[1];
                uint8_t dev  = uartBuf[2];
                uint8_t base = cmd & 0x7F;

                // Sector lógico y flags (comparten layout para READ/WRITE)
                uint16_t sec     = (uint16_t)uartBuf[3] | ((uint16_t)uartBuf[4] << 8);
                uint8_t densFlag = uartBuf[5]; // 0=SD, 1=DD (según RP)
                uint8_t pfReq    = uartBuf[6]; // prefetch pedido por RP

                // Prefetch override
                uint8_t pf  = prefetchForDev(dev);
                if (pf > MAX_PREFETCH_SECTORS) pf = MAX_PREFETCH_SECTORS;
                if (pf > 0) {
                  uartBuf[6] = pf;
                  logf("[MASTER] Overwrite PREFETCH para %s = %u sectores",
                       devName(dev), pf);
                }

                // Log específico de READ
                if (base == 0x52) {
                  logf("[MASTER] READ lógico dev=%s (0x%02X) sec=%u dens=%s pfReq=%u pfCfg=%u",
                       devName(dev),
                       (unsigned)dev,
                       (unsigned)sec,
                       densFlag ? "DD" : "SD",
                       (unsigned)pfReq,
                       (unsigned)pf);
                }

                // Log específico de FORMAT
                if (base == 0x21) {
                  logf("[MASTER] FORMAT SD (0x21) dev=%s", devName(dev));
                } else if (base == 0x22) {
                  logf("[MASTER] FORMAT DD (0x22) dev=%s", devName(dev));
                }

                // Detectar WRITE pendiente
                bool isWriteSector = (base == 0x50 || base == 0x57);
                bool isWritePercom = (base == 0x4F);

                if (isWriteSector || isWritePercom) {
                  uint16_t opSec;
                  if (isWritePercom) {
                    opSec = 0xFFFF;
                  } else {
                    opSec = sec;
                  }

                  g_pendingWriteRP.active = true;
                  g_pendingWriteRP.dev    = dev;
                  g_pendingWriteRP.sec    = opSec;
                  logf("[MASTER] WRITE pendiente desde RP dev=%s sec=%u base=0x%02X",
                       devName(dev),
                       (unsigned)opSec,
                       (unsigned)base);
                }

                // ====== NUEVO: registrar último comando importante ======
                bool isRead       = (base == 0x52);
                bool isFormatSD   = (base == 0x21);
                bool isFormatDD   = (base == 0x22);
                bool isStatus     = (base == 0x53);
                bool isReadPercom = (base == 0x4E);
                bool isWritePer   = isWritePercom;
                bool isWriteSecOp = isWriteSector;

                g_lastMasterOp.active = (isRead || isWriteSecOp || isWritePer ||
                                         isFormatSD || isFormatDD || isStatus || isReadPercom);
                if (g_lastMasterOp.active) {
                  g_lastMasterOp.cmd = cmd;
                  g_lastMasterOp.dev = dev;

                  if (isRead || isWriteSecOp) {
                    g_lastMasterOp.sec = sec;
                  } else if (isWritePer || isReadPercom) {
                    g_lastMasterOp.sec = 0xFFFF;
                  } else {
                    g_lastMasterOp.sec = 0;
                  }
                }
                // ====== FIN NUEVO ======
              }

              // Reenviar comando al SLAVE
              sendEspToSlave(uartBuf, uartLen);
              Serial.print(F("[MASTER] Enviado payload a SLAVE tipo=0x"));
              if (uartBuf[0] < 0x10) Serial.print('0');
              Serial.print(uartBuf[0], HEX);
              Serial.print(F(" len="));
              Serial.println(uartLen);
            } break;

            case TYPE_SECTOR_CHUNK: {
              if (uartLen < 6) {
                Serial.println(F("[MASTER] SECTOR_CHUNK desde RP demasiado corto"));
                break;
              }

              uint8_t dev    = uartBuf[1];
              uint16_t sec   = (uint16_t)uartBuf[2] | ((uint16_t)uartBuf[3] << 8);
              uint8_t idx    = uartBuf[4];
              uint8_t count  = uartBuf[5];
              int     dlen   = uartLen - 6;

              Serial.print(F("[MASTER] SECTOR_CHUNK desde RP dev=0x"));
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

              bool isPercom = (sec == 0xFFFF);

              if (isPercom) {
                sendEspToSlave(uartBuf, uartLen);
                Serial.println(F("[MASTER] SECTOR_CHUNK PERCOM reenviado al SLAVE (WRITE PERCOM)"));
                g_pendingWriteRP.active = false;
                break;
              }

              if (!g_pendingWriteRP.active) {
                Serial.println(F("[MASTER] SECTOR_CHUNK desde RP sin WRITE pendiente, se ignora"));
                break;
              }
              if (dev != g_pendingWriteRP.dev || sec != g_pendingWriteRP.sec) {
                Serial.println(F("[MASTER] SECTOR_CHUNK dev/sec no coincide con WRITE pendiente, se ignora"));
                break;
              }

              sendEspToSlave(uartBuf, uartLen);
              Serial.println(F("[MASTER] SECTOR_CHUNK reenviado al SLAVE (WRITE DATA)"));

              g_pendingWriteRP.active = false;
            } break;

            default:
              Serial.println(F("[MASTER] Tipo desconocido desde RP (no reenviado)."));
              break;
          }

          uartState = 0;
        }
        break;
      }
    }
  }

  if (uartState != 0 && (millis() - uartLastTime > 50)) {
    Serial.println(F("[MASTER] UART: timeout mid-frame, reseteando FSM."));
    uartState = 0;
    uartIdx   = 0;
    uartLen   = 0;
  }
}

// ===== ESP-NOW callbacks =====

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t* data, int len) {
  if (!data || len <= 0) return;
  const uint8_t* src = info ? info->src_addr : nullptr;

  if (src) {
    bool isBcast = true;
    for (int i = 0; i < 6; i++) {
      if (src[i] != 0xFF) { isBcast = false; break; }
    }
    if (!isBcast) {
      memcpy(g_lastSlave, src, 6);
      g_haveSlave = true;
      ensurePeer(g_lastSlave);
    }
  }

  uint8_t type = data[0];

  Serial.print(F("[MASTER] onDataRecv type=0x"));
  if (type < 0x10) Serial.print('0');
  Serial.print(type, HEX);
  Serial.print(F(" len="));
  Serial.println(len);

  if (type == TYPE_HELLO && len >= 3) {
    uint8_t dev = data[1];
    uint8_t dd  = data[2];

    Serial.print(F("[MASTER] HELLO de SLAVE DEV=0x"));
    if (dev < 0x10) Serial.print('0');
    Serial.println(dev, HEX);

    int idx = devIndex(dev);
    if (idx >= 0 && src) {
      slaves[idx].present     = true;
      slaves[idx].supports256 = (dd != 0);
      memcpy(slaves[idx].mac, src, 6);
      slaves[idx].lastSeen = millis();
      ensurePeer(slaves[idx].mac);
      logf("[HELLO] %s presente DD=%u MAC=%s",
           devName(dev), dd ? 1 : 0, formatMac(slaves[idx].mac).c_str());
    }

    sendUartFrameToRP(data, len);
  }
  else if (type == TYPE_SECTOR_CHUNK) {
    // Sector data desde SLAVE → RP
    sendUartFrameToRP(data, len);
  }
  else if (type == TYPE_ACK || type == TYPE_NAK) {
    const char* kind = (type == TYPE_ACK) ? "ACK" : "NAK";

    if (g_lastMasterOp.active) {
      logf("[MASTER] %s desde SLAVE para cmd=0x%02X dev=%s sec=%u",
           kind,
           (unsigned)g_lastMasterOp.cmd,
           devName(g_lastMasterOp.dev),
           (unsigned)g_lastMasterOp.sec);
    } else {
      logf("[MASTER] %s desde SLAVE (sin op activa)", kind);
    }

    // después de ACK/NAK, limpiamos la op activa
    g_lastMasterOp.active = false;

    sendUartFrameToRP(data, len);
  }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t s) {
  (void)info;
  if (s == ESP_NOW_SEND_SUCCESS)
    Serial.println(F("[MASTER] ESP-NOW envío OK"));
  else
    Serial.println(F("[MASTER] ESP-NOW fallo en envío"));
}

// ===== SETUP / LOOP =====

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(F("\n=== ESP32 MASTER BRIDGE (RP2040 <-> SLAVE XF551) + WEB ==="));

  Serial2.begin(115200, SERIAL_8N1, PIN_RP_RX, PIN_RP_TX);

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

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[MASTER] ERROR: esp_now_init fallo"));
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

void loop() {
  server.handleClient();

  unsigned long now = millis();
  for (int i = 0; i < 4; i++){
    if (slaves[i].present && (now - slaves[i].lastSeen > 60000)){
      slaves[i].present = false;
    }
  }

  pollUartFromRP();

  delay(2);
}
