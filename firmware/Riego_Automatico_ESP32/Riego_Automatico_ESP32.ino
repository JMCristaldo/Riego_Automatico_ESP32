#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <time.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <DHT.h>


Preferences prefs;

String wifiSsid;
String wifiPass;


#define PIN_LED_HEARTBEAT 2
#define PIN_HUMEDAD       32
#define PIN_BOMBA         18
#define PIN_BTN           16
#define PIN_LED_VERDE     25
#define PIN_LED_AMARILLO  26
#define PIN_LED_ROJO      27
#define PIN_DHT           17
#define DHTTYPE           DHT22
DHT dht(PIN_DHT, DHTTYPE);



// ---------- Estado del sistema ----------
volatile int humedadPct = 0;
volatile int umbralPct  = 50;
volatile bool modoManual = false;
volatile bool regando = false;
volatile int diasMask = 127;
volatile int runMode = 0;

// --- WiFi Setup AP ---
const char* AP_SSID = "RiegoESP32-Setup";
const char* AP_PASS = "12345678";   // mínimo 8 chars (temporal)
volatile bool wifiApMode = false;


// ===================== HISTÓRICO EN RAM (ring buffer) =====================
// Buffer dimensionado para 30 días @ 10 min => 4320 muestras
// En modo prueba podemos muestrear cada 1 min => cubre ~3 días.

#define HISTORY_DAYS             30
#define HISTORY_BASE_INTERVAL_S  (10 * 60)  // 10 minutos (diseño)
#define HISTORY_CAPACITY         (HISTORY_DAYS * 24 * 60 / 10) // 4320

// Para pruebas: ponelo en 60 (1 minuto). Para producción: 600 (10 min).
// Si querés alternarlo rápido, dejalo en 60 mientras probás y luego volvés a 600.
volatile uint32_t historyIntervalSec = 5;  // <-- Intervalo de muestreo del histórico (seg). En producción recomendado 600

// Cada muestra es compacta (sin String, sin malloc)
struct HistorySample {
  uint32_t ts;      // epoch (si NTP ok) o uptime sec (si no)
  uint8_t  soil;    // 0..100
  int16_t  temp10;  // temp * 10 (ej 23.4°C => 234). INT16_MIN = inválido
  uint16_t hum10;   // hum * 10 (ej 55.2% => 552). 0xFFFF = inválido
  uint8_t  flags;   // bits: 0=regando, 1=dhtOk, 2=safety
};

HistorySample hist[HISTORY_CAPACITY];
volatile uint16_t histHead = 0;   // próxima posición a escribir
volatile uint16_t histCount = 0;  // cuántas válidas hay (<= CAPACITY)

unsigned long lastHistorySampleMs = 0;

// --- sensor ambiente (DHT22) ---
unsigned long lastDhtMs = 0;
const unsigned long DHT_PERIOD_MS = 2500;

volatile float tempC = NAN;
volatile float humAirPct = NAN;
volatile bool dhtOk = false;


// Estado previo solo para restore del botón físico
volatile int prevRunMode = 0;
volatile int prevProgMode = 0;


// ---------- NTP / Hora ----------
volatile bool ntpOk = false;
unsigned long lastNtpCheckMs = 0;
const unsigned long NTP_CHECK_PERIOD_MS = 2000;  // cada 2s


// Ventana: inicio + fin (HH:MM)
volatile int startHour = 0;
volatile int startMin  = 0;
volatile int endHour   = 18;
volatile int endMin    = 0;


// Programado: 0 = PROG_SENSOR, 1 = PROG_CICLOS
volatile int progMode = 0;

// Ciclos X/Y (min)
volatile int cycleEveryMin = 30; // cada X minutos
volatile int cycleOnMin = 2;  // riega Y minutos


// --- Anti-ciclo / histéresis ---
const int H = 3;  // histéresis en %
const unsigned long MIN_ON_MS  = 5000;
const unsigned long MIN_OFF_MS = 5000;

unsigned long lastPumpChangeMs = 0;  // cuándo cambió regando por última vez


AsyncWebServer server(80);

// ---------- Botón: debounce + edge ----------
bool btnStable = HIGH;
bool btnRawPrev = HIGH;
unsigned long btnLastChangeMs = 0;
const unsigned long DEBOUNCE_MS = 35;

// ---------- Tareas periódicas ----------
unsigned long lastSampleMs = 0;
const unsigned long SAMPLE_PERIOD_MS = 200;
unsigned long lastHbMs = 0;
const unsigned long HB_PERIOD_MS = 500; // parpadeo cada 500ms
bool hbState = false;

static inline uint32_t getNowTsSec() {
  if (ntpOk) {
    return (uint32_t)time(nullptr);
  }
  return (uint32_t)(millis() / 1000);
}

static inline int16_t encodeTemp10(float tC) {
  if (!isfinite(tC)) return INT16_MIN;
  return (int16_t)lroundf(tC * 10.0f);
}

static inline uint16_t encodeHum10(float h) {
  if (!isfinite(h)) return 0xFFFF;
  int v = (int)lroundf(h * 10.0f);
  if (v < 0) v = 0;
  if (v > 1000) v = 1000; // 100.0%
  return (uint16_t)v;
}


// ---- Prototipos de funciones ----
void setupServer();
void toggleManual();
void handleButton();
void sampleAndControl();
void loadConfigFromNVS();
void saveConfigToNVS();
void initNTP();
void updateNtpStatus();
String getLocalTimeString();
bool isWindowActive();
void applyPumpOutput();
bool safetyCutoffActive();
void enforceSafetyCutoff();        // fuerza OFF y resetea ciclos si hace falta
bool sensorWantsOn();              // histéresis pura (umbral +/- H)
void applyAntiCycle(bool desired); // MIN_ON/MIN_OFF centralizado
void runAutoSensor();
void runProgSensor();
void runProgCycles();
void onConfigChanged(bool stopPumpIfAuto);
void enterManual(bool savePrev);
void exitManual(bool restorePrev);
String getWindowStartString();
String getWindowEndString();
int getWifiRssi();
String getWifiIpString();
void sampleDHT();
void historyAddSample();
void historyTick();
void loadWifiCredsFromNVS();
bool hasWifiCreds();
bool tryConnectSTAFromNVS(uint32_t timeoutMs);
void startWifiAP();


void startWifiAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  IPAddress ip = WiFi.softAPIP();
  Serial.println("=== MODO CONFIGURACION WIFI ===");
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("IP AP: ");
  Serial.println(ip);

  wifiApMode = true;
}


void historyAddSample() {
  HistorySample s;
  s.ts = getNowTsSec();
  s.soil = (uint8_t)constrain(humedadPct, 0, 100);

  // DHT (si no está OK, guardamos inválido)
  if (dhtOk) {
    s.temp10 = encodeTemp10(tempC);
    s.hum10  = encodeHum10(humAirPct);
  } else {
    s.temp10 = INT16_MIN;
    s.hum10  = 0xFFFF;
  }

  uint8_t f = 0;
  if (regando) f |= (1 << 0);
  if (dhtOk)   f |= (1 << 1);
  if (safetyCutoffActive()) f |= (1 << 2);
  s.flags = f;

  // write ring
  hist[histHead] = s;
  histHead = (histHead + 1) % HISTORY_CAPACITY;
  if (histCount < HISTORY_CAPACITY) histCount++;
}

void historyTick() {
  const unsigned long nowMs = millis();
  const unsigned long periodMs = (unsigned long)historyIntervalSec * 1000UL;

  if (nowMs - lastHistorySampleMs >= periodMs) {
    lastHistorySampleMs = nowMs;
    historyAddSample();
  }
}

void loadConfigFromNVS() {
  // Si no existe la key, mantiene el default actual (segundo parámetro)
  umbralPct = prefs.getInt("umbral", umbralPct);

  runMode = prefs.getInt("runMode", runMode);
  diasMask = prefs.getInt("diasMask", diasMask);

  startHour = prefs.getInt("stH", startHour);
  startMin  = prefs.getInt("stM", startMin);
  endHour = prefs.getInt("enH", endHour);
  endMin  = prefs.getInt("enM", endMin);

  // --- Migración desde ventana por duración (legacy) ---
  if (!prefs.isKey("enH") || !prefs.isKey("enM")) {
    int dur = prefs.getInt("durW", -1);
    if (dur > 0) {
      int startTotal = startHour * 60 + startMin;
      int endTotal = (startTotal + dur) % 1440;

      endHour = endTotal / 60;
      endMin  = endTotal % 60;
    }
  }

  progMode = prefs.getInt("progMode", progMode);

  cycleEveryMin = prefs.getInt("cyEvery", cycleEveryMin);
  cycleOnMin    = prefs.getInt("cyOn", cycleOnMin);


  // Sanitizado mínimo (para no cargar basura si alguna vez se guarda mal)
  if (umbralPct < 0) umbralPct = 0;
  if (umbralPct > 100) umbralPct = 100;


  if (runMode < 0) runMode = 0;
  if (runMode > 2) runMode = 2;

  if (diasMask < 0) diasMask = 0;
  if (diasMask > 127) diasMask = 127;

  if (startHour < 0) startHour = 0;
  if (startHour > 23) startHour = 23;

  if (startMin < 0) startMin = 0;
  if (startMin > 59) startMin = 59;

  if (progMode < 0) progMode = 0;
  if (progMode > 1) progMode = 1;

  if (cycleEveryMin < 1) cycleEveryMin = 1;
  if (cycleEveryMin > 1440) cycleEveryMin = 1440;

  if (cycleOnMin < 1) cycleOnMin = 1;
  if (cycleOnMin > 1440) cycleOnMin = 1440;

  if (endHour < 0) endHour = 0;
  if (endHour > 23) endHour = 23;

  if (endMin < 0) endMin = 0;
  if (endMin > 59) endMin = 59;

}

void saveConfigToNVS() {
  prefs.putInt("umbral", umbralPct);

  prefs.putInt("runMode", runMode);
  prefs.putInt("diasMask", diasMask);

  prefs.putInt("stH", startHour);
  prefs.putInt("stM", startMin);

  prefs.putInt("enH", endHour);
  prefs.putInt("enM", endMin);

  prefs.putInt("cyEvery", cycleEveryMin);
  prefs.putInt("cyOn", cycleOnMin);

  prefs.putInt("progMode", progMode);
}

void initNTP() {
  // Argentina UTC-3 (sin DST)
  configTime(-3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
}

void updateNtpStatus() {
  unsigned long nowMs = millis();
  if (nowMs - lastNtpCheckMs < NTP_CHECK_PERIOD_MS) return;
  lastNtpCheckMs = nowMs;

  time_t now = time(nullptr);

  // Si NTP todavía no sincronizó, suele devolver 0 o un valor muy bajo.
  // Usamos un umbral seguro (2023-11 aprox) para evitar falsos positivos.
  if (now > 1700000000) {
    ntpOk = true;
  } else {
    ntpOk = false;
  }
}


void loadWifiCredsFromNVS() {
  wifiSsid = prefs.getString("wifiSsid", "");
  wifiPass = prefs.getString("wifiPass", "");
  wifiSsid.trim();
}

bool hasWifiCreds() {
  return wifiSsid.length() > 0;
}

bool tryConnectSTAFromNVS(uint32_t timeoutMs) {
  loadWifiCredsFromNVS();
  if (!hasWifiCreds()) return false;

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeoutMs) {
    delay(250);
  }
  return (WiFi.status() == WL_CONNECTED);
}



String getWindowStartString() {
  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", startHour, startMin);
  return String(buf);
}

String getWindowEndString() {
  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", endHour, endMin);
  return String(buf);
}

int getWifiRssi() {
  if (WiFi.status() != WL_CONNECTED) return -999;
  return WiFi.RSSI();
}

String getWifiIpString() {
  if (WiFi.status() != WL_CONNECTED) return String("");
  return WiFi.localIP().toString();
}


String getLocalTimeString() {
  if (!ntpOk) return String("--:--:--");

  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);

  char buf[20];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
  return String(buf);
}



bool isWindowActive() {
  if (!ntpOk) return false;

  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);

  const int nowMin = t.tm_hour * 60 + t.tm_min;   // 0..1439
  const int today  = t.tm_wday;                   // 0=Dom..6=Sab

  const int startMinDay = startHour * 60 + startMin;
  const int endMinDay   = endHour   * 60 + endMin;

  // Ventana NO cruza medianoche
  if (startMinDay < endMinDay) {
    if ((diasMask & (1 << today)) == 0) return false;
    return (nowMin >= startMinDay) && (nowMin < endMinDay);
  }

  // Ventana CRUZA medianoche
  // Ej: 20:00 -> 04:00
  if (nowMin >= startMinDay) {
    // Tramo inicial (día de inicio)
    if ((diasMask & (1 << today)) == 0) return false;
    return true;
  }

  // Tramo después de medianoche
  // Pertenece al día anterior
  int yesterday = (today + 6) % 7;
  if ((diasMask & (1 << yesterday)) == 0) return false;
  return (nowMin < endMinDay);
}

int windowElapsedMin() {
  if (!ntpOk) return -1;

  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);

  const int nowMin = t.tm_hour * 60 + t.tm_min; // 0..1439
  const int today  = t.tm_wday;                 // 0=Dom..6=Sab

  const int startMinDay = startHour * 60 + startMin;
  const int endMinDay   = endHour   * 60 + endMin;

  // Ventana NO cruza medianoche
  if (startMinDay < endMinDay) {
    if ((diasMask & (1 << today)) == 0) return -1;
    if (nowMin < startMinDay || nowMin >= endMinDay) return -1;
    return nowMin - startMinDay;
  }

  // Ventana CRUZA medianoche (ej 20:00->04:00)
  if (nowMin >= startMinDay) {
    // tramo antes de medianoche (día de inicio)
    if ((diasMask & (1 << today)) == 0) return -1;
    return nowMin - startMinDay;
  }

  // tramo después de medianoche (pertenece al día anterior)
  const int yesterday = (today + 6) % 7;
  if ((diasMask & (1 << yesterday)) == 0) return -1;
  if (nowMin >= endMinDay) return -1;  // por seguridad
  return (1440 - startMinDay) + nowMin;
}

void sampleDHT() {
  unsigned long now = millis();
  if (now - lastDhtMs < DHT_PERIOD_MS) return;
  lastDhtMs = now;

  float h = dht.readHumidity();
  float t = dht.readTemperature(); // Celsius

  if (isnan(h) || isnan(t)) {
    dhtOk = false;
    tempC = NAN;
    humAirPct = NAN;
    return;
  }

  dhtOk = true;
  humAirPct = h;
  tempC = t;
}


void setupServer() {

  Serial.println("Conectando a WiFi (NVS)...");
  bool ok = tryConnectSTAFromNVS(15000);

  if (ok) {
    Serial.println("\nWiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    initNTP();
    Serial.println("NTP iniciado.");
  } else {
    Serial.println("\nWiFi NO conectado. Iniciando AP de configuración...");
    startWifiAP();
  }

  server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request) {

    const char* html = R"rawliteral(
  <!doctype html>
  <html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Configurar WiFi</title>
    <style>
      body{
        font-family: system-ui, sans-serif;
        background:#0f1113;
        color:#e7e7e7;
        padding:20px;
      }
      .card{
        max-width:360px;
        margin:auto;
        background:#171b1f;
        border-radius:16px;
        padding:20px;
      }
      input,button{
        width:100%;
        padding:12px;
        margin-top:10px;
        border-radius:10px;
        border:none;
        font-size:16px;
      }
      input{
        background:#0b0d0f;
        color:#e7e7e7;
      }
      button{
        background:#35d07f;
        color:#000;
        font-weight:600;
      }
    </style>
  </head>
  <body>
    <div class="card">
      <h2>Configurar WiFi</h2>
      <form method="POST" action="/wifi/save">
        <input name="ssid" placeholder="Nombre de red (SSID)" required>
        <input name="pass" type="password" placeholder="Contraseña">
        <button type="submit">Guardar y conectar</button>
      </form>
    </div>
  </body>
  </html>
  )rawliteral";

    request->send(200, "text/html", html);
  });

  server.on("/wifi/save", HTTP_POST, [](AsyncWebServerRequest *request) {

    if (!request->hasParam("ssid", true)) {
      request->send(400, "text/plain", "SSID faltante");
      return;
    }

    String ssid = request->getParam("ssid", true)->value();
    String pass = "";
    if (request->hasParam("pass", true)) {
      pass = request->getParam("pass", true)->value();
    }

    ssid.trim();

    prefs.putString("wifiSsid", ssid);
    prefs.putString("wifiPass", pass);

    request->send(200, "text/plain", "Credenciales guardadas. Reiniciando ESP32...");

    wifiApMode = false;
    delay(1200);
    ESP.restart();
  });


  // Ruta principal
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (wifiApMode) {
      request->redirect("/wifi");
      return;
    }

  const char* html = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Riego ESP32</title>
  <style>
    :root{
      --bg:#0f1113;
      --panel:#14171a;
      --card:#171b1f;
      --card2:#15191d;
      --border:rgba(255,255,255,.06);
      --text:#e7e7e7;
      --muted:#9aa3ab;
      --green:#35d07f;
      --red:#ff4d4d;
      --radius:16px;
      --pad:14px;
      --gap:12px;
    }

    *{ box-sizing:border-box; }
    body{
      margin:0;
      font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif;
      background:var(--bg);
      color:var(--text);
      padding-bottom:70px; /* espacio para bottom bar */
    }

    /* Layout container */
    .wrap{
      max-width: 560px;
      margin: 0 auto;
      padding: 14px;
    }

    /* Top bar */
    .topbar{
      display:flex;
      align-items:center;
      justify-content:space-between;
      gap:12px;
      padding: 12px 12px;
      border-radius: var(--radius);
      background: linear-gradient(180deg, rgba(255,255,255,.05), rgba(255,255,255,.02));
      border:1px solid var(--border);
      margin-bottom: 12px;
    }
    .title{
      font-weight:600;
      letter-spacing:.2px;
    }
    .topRight{
      display:flex;
      align-items:center;
      gap:10px;
      color:var(--muted);
      font-size: 13px;
      white-space: nowrap;
    }
    .dot{
      width:8px; height:8px; border-radius:50%;
      background: rgba(255,255,255,.2);
      display:inline-block;
      margin-right:6px;
    }
    .wifiOk .dot{ background: var(--green); }
    .wifiNo .dot{ background: var(--red); }

    /* Grid */
    .grid{
      display:grid;
      grid-template-columns: 1fr 1fr;
      gap: var(--gap);
    }

    .card{
      background: linear-gradient(180deg, rgba(255,255,255,.04), rgba(255,255,255,.02));
      border: 1px solid var(--border);
      border-radius: var(--radius);
      padding: var(--pad);
    }

    .square{
      display:flex;
      flex-direction:column;
      justify-content:space-between;
      min-height: 100px;
    }

    .wide{
      grid-column: 1 / -1;
    }

    .label{
      font-size: 12px;
      color: var(--muted);
      letter-spacing:.2px;
    }

    .valueRow{
      display:flex;
      align-items:baseline;
      justify-content:space-between;
      gap:10px;
      margin-top: 6px;
    }

    .value{
      font-size: 36px;
      line-height: 1.0;
      font-weight: 650;
      letter-spacing: .2px;
    }

    .unit{
      font-size: 14px;
      color: var(--muted);
      margin-left: 6px;
    }

    .sub{
      margin-top: 10px;
      font-size: 12px;
      color: var(--muted);
    }

    /* Pill ON/OFF - estilo oscuro con texto de color */
    .pill{
      display:inline-flex;
      align-items:center;
      justify-content:center;
      padding: 10px 14px;
      border-radius: 999px;
      background: rgba(0,0,0,.22);
      border: 1px solid rgba(255,255,255,.08);
      font-weight: 700;
      letter-spacing: .6px;
      font-size: 14px;
      min-width: 86px;
      user-select:none;
    }
    .pill.on{
      color: var(--green);
      border-color: rgba(53,208,127,.35);
      box-shadow: 0 0 0 1px rgba(53,208,127,.05) inset;
    }
    .pill.off{
      color: var(--red);
      border-color: rgba(255,77,77,.35);
      box-shadow: 0 0 0 1px rgba(255,77,77,.05) inset;
    }

    /* Manual button card */
    .btnCard{
      cursor:pointer;
      user-select:none;
      position:relative;
      transition: transform .08s ease, box-shadow .12s ease;
    }
    .btnCard:active{
      transform: translateY(1px);
    }

    /* “presionado” cuando está activo */
    .btnCard.active{
      box-shadow:
        0 0 0 1px rgba(255,255,255,.06) inset,
        0 10px 30px rgba(0,0,0,.35) inset;
    }

    /* ranura LED */
    .ledSlot{
      position:absolute;
      left: 14px;
      right: 14px;
      bottom: 14px;
      height: 6px;
      border-radius: 999px;
      background: rgba(255,255,255,.08);
      overflow:hidden;
    }
    .ledSlot::after{
      content:"";
      display:block;
      height:100%;
      width: 0%;
      background: var(--green);
      box-shadow: 0 0 18px rgba(53,208,127,.45);
      transition: width .18s ease;
    }
    .btnCard.active .ledSlot::after{
      width: 100%;
    }

    /* Select big */
    select{
      width:100%;
      padding: 12px 12px;
      border-radius: 12px;
      background: rgba(0,0,0,.25);
      border: 1px solid rgba(255,255,255,.10);
      color: var(--text);
      font-size: 16px;
      outline:none;
    }
    select:disabled{
      opacity:.55;
    }

    /* Slider */
    input[type="range"]{
      width:100%;
      margin-top: 10px;
      accent-color: var(--green);
    }

    /* Bottom bar */
    #bottomBar{
      position:fixed;
      bottom:0; left:0; right:0;
      background: rgba(10,12,14,.92);
      border-top: 1px solid rgba(255,255,255,.06);
      display:flex;
      gap: 0;
      padding: 8px 10px;
      backdrop-filter: blur(6px);
    }
    #bottomBar button{
      flex:1;
      padding: 12px 10px;
      border-radius: 12px;
      border: 1px solid rgba(255,255,255,.06);
      background: rgba(255,255,255,.02);
      color: var(--muted);
      font-size: 14px;
      transition: background .12s ease, color .12s ease, border-color .12s ease;
    }
    #bottomBar button.active{
      background: rgba(53,208,127,.10);
      border-color: rgba(53,208,127,.30);
      color: var(--text);
    }

    /* Config view minimal dark */
    .sectionTitle{
      margin: 14px 0 8px;
      font-size: 13px;
      color: var(--muted);
      letter-spacing:.3px;
      text-transform: uppercase;
    }
    .row{
      display:flex;
      justify-content:space-between;
      align-items:center;
      gap:10px;
      margin: 10px 0;
    }
    .btn{
      padding: 12px 14px;
      border-radius: 12px;
      border: 1px solid rgba(255,255,255,.10);
      background: rgba(255,255,255,.04);
      color: var(--text);
    }
    .small{
      color: var(--muted);
      font-size: 12px;
    }
    .days{
      display:flex; gap:10px; flex-wrap:wrap; margin: 8px 0;
      color: var(--text);
    }
    input[type="number"], input[type="time"]{
      background: rgba(0,0,0,.25);
      border: 1px solid rgba(255,255,255,.10);
      color: var(--text);
      border-radius: 10px;
      padding: 10px 10px;
    }
  </style>
</head>

<body>
  <div class="wrap">

    <!-- DASHBOARD -->
    <div id="dashboardView">

      <div class="topbar">
        <div class="title">Riego ESP32</div>
        <div class="topRight">
          <div id="wifiBadge" class="wifiNo"><span class="dot"></span><span id="wifiTxt">WiFi</span></div>
          <div id="topTime">--</div>
        </div>
      </div>

      <div class="grid">

        <!-- Fila 1 -->
        <div class="card square">
          <div class="label">HUMEDAD SUELO</div>
          <div class="valueRow">
            <div class="value"><span id="humVal">--</span><span class="unit">%</span></div>
          </div>
          <div class="sub">Sensor</div>
        </div>

        <div class="card square">
          <div class="label">UMBRAL</div>
          <div class="valueRow">
            <div class="value"><span id="umbVal">--</span><span class="unit">%</span></div>
          </div>
          <div class="sub">Objetivo</div>
        </div>

        <!-- Fila 2 -->
        <div class="card square">
          <div class="label">HUM. AIRE</div>
          <div class="valueRow">
            <div class="value"><span id="humAirVal">--</span><span class="unit">%</span></div>
          </div>
          <div class="sub">DHT22</div>
        </div>

        <div class="card square">
          <div class="label">TEMPERATURA</div>
          <div class="valueRow">
            <div class="value"><span id="tempVal">--</span><span class="unit">°C</span></div>
          </div>
          <div class="sub">DHT22</div>
        </div>

        <!-- Fila 3 -->
        <div class="card wide">
          <div class="label">AJUSTE DE UMBRAL</div>
          <div class="valueRow" style="margin-top:8px;">
            <div class="value" style="font-size:32px;"><span id="umbSliderVal">--</span><span class="unit">%</span></div>
          </div>
          <input id="umbSet" type="range" min="0" max="100" value="50">
          <div class="small">Se guarda al soltar.</div>
        </div>

        <!-- Fila 4 -->
        <div class="card wide">
          <div class="label">MODO</div>
          <div style="margin-top:10px;">
            <select id="modeSelect" onchange="onModeChange(this.value)">
              <option value="1">Automático</option>
              <option value="2">Programado + Sensor</option>
              <option value="4">Programado + Ciclos</option>
              <option value="3">Apagado</option>
            </select>
          </div>
          <div class="sub" id="modeHuman">—</div>
        </div>

        <!-- Fila 5 -->
        <div class="card square">
          <div class="label">RIEGO</div>
          <div style="margin-top:12px;">
            <span id="pumpPill" class="pill off">OFF</span>
          </div>
          <div class="sub" id="pumpSub">—</div>
        </div>

        <div class="card square btnCard" id="manualBtn" onclick="manualBtnClicked()">
          <div class="label">MANUAL</div>
          <div class="valueRow" style="margin-top:12px;">
            <div class="value" style="font-size:28px;"><span id="manualTxt">OFF</span></div>
          </div>
          <div class="sub">Forzar bomba</div>
          <div class="ledSlot"></div>
        </div>

        <!-- HISTÓRICO: SUELO -->
        <div class="card wide">
          <div class="label">HISTÓRICO — HUMEDAD SUELO (0–100%)</div>
          <canvas id="soilCanvas" width="520" height="140" style="width:100%; height:140px; margin-top:10px;"></canvas>
          <div class="small" id="soilInfo">—</div>
        </div>

        <!-- HISTÓRICO: HUMEDAD AIRE -->
        <div class="card wide">
          <div class="label">HISTÓRICO — HUMEDAD AIRE (0–100%)</div>
          <canvas id="airHumCanvas" width="520" height="140" style="width:100%; height:140px; margin-top:10px;"></canvas>
          <div class="small" id="airHumInfo">—</div>
        </div>

        <!-- HISTÓRICO: TEMPERATURA -->
        <div class="card wide">
          <div class="label">HISTÓRICO — TEMPERATURA (0–60°C)</div>
          <canvas id="tempCanvas" width="520" height="140" style="width:100%; height:140px; margin-top:10px;"></canvas>
          <div class="small" id="tempInfo">—</div>
        </div>



      </div>
    </div>

    <!-- CONFIG -->
    <div id="configView" style="display:none;">

      <div class="topbar">
        <div class="title">Configuración</div>
        <div class="topRight">
          <div id="cfgTime">--</div>
        </div>
      </div>

      <div class="sectionTitle">Programación (ventana)</div>
      <div class="card">

        <div class="row"><div class="label">NTP</div><div><b id="ntpOkTxt">--</b></div></div>
        <div class="row"><div class="label">Hora ESP</div><div><span id="timeTxt">--:--:--</span></div></div>
        <div class="row"><div class="label">Ventana activa</div><div><b id="winActiveTxt">--</b></div></div>

        <hr style="border:0;border-top:1px solid rgba(255,255,255,.08); margin:14px 0;">

        <div class="label">Días de riego (inicio)</div>
        <div class="days">
          <label><input type="checkbox" class="day" data-bit="0"> Dom</label>
          <label><input type="checkbox" class="day" data-bit="1"> Lun</label>
          <label><input type="checkbox" class="day" data-bit="2"> Mar</label>
          <label><input type="checkbox" class="day" data-bit="3"> Mié</label>
          <label><input type="checkbox" class="day" data-bit="4"> Jue</label>
          <label><input type="checkbox" class="day" data-bit="5"> Vie</label>
          <label><input type="checkbox" class="day" data-bit="6"> Sáb</label>
        </div>

        <div class="row">
          <div class="label">Inicio</div>
          <div><input id="startTime" type="time" value="00:00"></div>
        </div>

        <div class="row">
          <div class="label">Fin</div>
          <div><input id="endTime" type="time" value="18:00"></div>
        </div>

        <div class="sectionTitle" style="margin-top:14px;">Ciclos</div>

        <div class="row">
          <div class="label">Intervalo (min)</div>
          <div><input id="cycleEveryMin" type="number" min="1" max="1440" value="30" style="width:120px"></div>
        </div>

        <div class="row">
          <div class="label">Duración riego (min)</div>
          <div><input id="cycleOnMin" type="number" min="1" max="1440" value="2" style="width:120px"></div>
        </div>

        <div class="small">En “ciclos” se ignora el umbral. Seguridad dura: corta si humedad ≥ 90%.</div>

        <div style="margin-top:14px;">
          <button class="btn" onclick="saveConfig()">Guardar configuración</button>
          <div class="small" id="cfgMsg" style="margin-top:8px;">—</div>
        </div>

      </div>

    </div>

  </div>

  <div id="bottomBar">
    <button id="navDash" class="active" onclick="showDashboard()">Dashboard</button>
    <button id="navCfg" onclick="showConfig()">Configuración</button>
  </div>

<script>
  let draggingUmb = false;
  let manualActive = false;

  function setActiveTab(which){
    const b1 = document.getElementById('navDash');
    const b2 = document.getElementById('navCfg');
    if(which === 'dash'){
      b1.classList.add('active'); b2.classList.remove('active');
    }else{
      b2.classList.add('active'); b1.classList.remove('active');
    }
  }

  function updateTopTime(){
    const d = new Date();
    // Ej: "sáb 13:20"
    const day = d.toLocaleDateString(undefined, { weekday:'short' });
    const time = d.toLocaleTimeString(undefined, { hour:'2-digit', minute:'2-digit' });
    document.getElementById('topTime').textContent = `${day} ${time}`;
    const cfgTime = document.getElementById('cfgTime');
    if(cfgTime) cfgTime.textContent = `${day} ${time}`;
  }

  window.addEventListener('load', () => {
    const s = document.getElementById('umbSet');

    s.addEventListener('input', () => {
      document.getElementById('umbSliderVal').textContent = s.value;
    });

    s.addEventListener('pointerdown', () => { draggingUmb = true; });
    s.addEventListener('pointerup', () => { draggingUmb = false; setUmbral(); });
    s.addEventListener('pointercancel', () => { draggingUmb = false; });

    loadConfig();
    update();
    updateTopTime();
    setInterval(update, 1000);
    setInterval(updateTopTime, 1000);

    fetchHistoryAndDraw();
    setInterval(fetchHistoryAndDraw, 5000); // cada 5s

  });

  function syncUmbralUI(v){
    // Evita que "null/undefined" rompa el slider o muestre "null"
    if (v == null) return;
    const n = Number(v);
    if (!Number.isFinite(n)) return;

    document.getElementById('umbSliderVal').textContent = n;
    if(!draggingUmb){
      document.getElementById('umbSet').value = n;
    }
  }


  function modeToHuman(j){
    if(j.manual) return "Manual (prioridad absoluta)";
    if(j.runMode === 0) return "Automático: decide por humedad de suelo";
    if(j.runMode === 2) return "Apagado: riego deshabilitado";
    if(j.runMode === 1 && j.progMode === 0) return "Programado + Sensor: ventana + humedad";
    if(j.runMode === 1 && j.progMode === 1) return "Programado + Ciclos: slots dentro de ventana";
    return "—";
  }

  function applyPumpUI(isOn, safety){
    const pill = document.getElementById('pumpPill');
    const sub = document.getElementById('pumpSub');

    if(isOn){
      pill.textContent = "ON";
      pill.classList.remove('off'); pill.classList.add('on');
      sub.textContent = "Bomba activa";
    }else{
      pill.textContent = "OFF";
      pill.classList.remove('on'); pill.classList.add('off');
      sub.textContent = safety ? "Cortado por seguridad" : "Bomba detenida";
    }
  }

  function applyManualUI(on){
    const btn = document.getElementById('manualBtn');
    const txt = document.getElementById('manualTxt');
    if(on){
      btn.classList.add('active');
      txt.textContent = "ON";
    }else{
      btn.classList.remove('active');
      txt.textContent = "OFF";
    }
  }

  async function update(){
    const url = '/status?t=' + Date.now();

    try{
      const r = await fetch(url, { cache:'no-store' });
      const j = await r.json();

      manualActive = !!j.manual;

      // Top WiFi simple
      const wifiBadge = document.getElementById('wifiBadge');
      const wifiTxt = document.getElementById('wifiTxt');
      if(j.wifiOk){
        wifiBadge.classList.add('wifiOk');
        wifiBadge.classList.remove('wifiNo');
        wifiTxt.textContent = "WiFi OK";
      }else{
        wifiBadge.classList.remove('wifiOk');
        wifiBadge.classList.add('wifiNo');
        wifiTxt.textContent = "WiFi NO";
      }

      // Cards valores
      document.getElementById('humVal').textContent = (j.humedad != null) ? j.humedad : "--";
      document.getElementById('umbVal').textContent = (j.umbral != null) ? j.umbral : "--";
      if (j.umbral != null) syncUmbralUI(j.umbral);

      if(j.dhtOk && j.humAir != null) document.getElementById('humAirVal').textContent = Number(j.humAir).toFixed(1);
      else document.getElementById('humAirVal').textContent = "--";

      if(j.dhtOk && j.tempC != null) document.getElementById('tempVal').textContent = Number(j.tempC).toFixed(1);
      else document.getElementById('tempVal').textContent = "--";

      // Modo
      const modeSelectEl = document.getElementById('modeSelect');
      modeSelectEl.disabled = manualActive;

      if(!manualActive){
        if(j.runMode === 0) modeSelectEl.value = "1";
        else if(j.runMode === 2) modeSelectEl.value = "3";
        else if(j.runMode === 1) modeSelectEl.value = (j.progMode === 1) ? "4" : "2";
      }
      document.getElementById('modeHuman').textContent = modeToHuman(j);

      // Riego + Manual
      applyPumpUI(j.riego === "ON", !!j.safetyCutoff);
      applyManualUI(manualActive);

      // Config view status (si está visible, lo actualizamos también)
      if(document.getElementById('configView').style.display !== 'none'){
        document.getElementById('ntpOkTxt').textContent = j.ntpOk ? "OK" : "NO";
        document.getElementById('timeTxt').textContent = j.espTime || "--:--:--";
        document.getElementById('winActiveTxt').textContent =
          (j.winActive ? "SI" : "NO") + (j.winStart ? ` (${j.winStart} → ${j.winEnd})` : "");
      }

    }catch(e){
      // Si hay error de fetch, marcamos wifi como NO (no rompemos UI)
      const wifiBadge = document.getElementById('wifiBadge');
      const wifiTxt = document.getElementById('wifiTxt');
      wifiBadge.classList.remove('wifiOk');
      wifiBadge.classList.add('wifiNo');
      wifiTxt.textContent = "WiFi ?";
    }
  }

  async function setUmbral(){
    const v = document.getElementById('umbSet').value;
    try{
      await fetch('/umbral/set', {
        method:'POST',
        headers:{ 'Content-Type':'application/x-www-form-urlencoded' },
        body:'value=' + encodeURIComponent(v),
        cache:'no-store'
      });
      await update();
    }catch(e){}
  }

  async function setMainMode(v){
    try{
      await fetch('/mode/set', {
        method:'POST',
        headers:{ 'Content-Type':'application/x-www-form-urlencoded' },
        body:'value=' + encodeURIComponent(v),
        cache:'no-store'
      });
      await update();
    }catch(e){}
  }

  async function onModeChange(v){
    if(manualActive) return;
    await setMainMode(v);
  }

  function manualBtnClicked(){
    // Toggle manual desde la card (no cambia color, solo efecto presionado + led)
    onManualToggle(!manualActive);
  }

  async function onManualToggle(on){
    try{
      await fetch('/manual/set', {
        method:'POST',
        headers:{ 'Content-Type':'application/x-www-form-urlencoded' },
        body:'value=' + (on ? '1' : '0')
      });
      await update();
    }catch(e){}
  }

  async function loadConfig(){
    try{
      const r = await fetch('/config/get?t=' + Date.now(), { cache:'no-store' });
      const cfg = await r.json();

      // Días
      document.querySelectorAll('input.day').forEach(cb => {
        const bit = parseInt(cb.dataset.bit, 10);
        cb.checked = ((cfg.diasMask >> bit) & 1) === 1;
      });

      // Inicio
      const hh = String(cfg.startHour).padStart(2,'0');
      const mm = String(cfg.startMin).padStart(2,'0');
      document.getElementById('startTime').value = `${hh}:${mm}`;

      // Fin
      const eh = String(cfg.endHour).padStart(2,'0');
      const em = String(cfg.endMin).padStart(2,'0');
      document.getElementById('endTime').value = `${eh}:${em}`;

      // Ciclos
      document.getElementById('cycleEveryMin').value = cfg.cycleEveryMin;
      document.getElementById('cycleOnMin').value = cfg.cycleOnMin;

    }catch(e){}
  }

  function getDiasMaskFromUI(){
    let mask = 0;
    document.querySelectorAll('input.day').forEach(cb => {
      const bit = parseInt(cb.dataset.bit, 10);
      if(cb.checked) mask |= (1 << bit);
    });
    return mask;
  }

  async function saveConfig(){
    const mask = getDiasMaskFromUI();

    const tStart = document.getElementById('startTime').value;
    const s = tStart.split(':');
    const sh = parseInt(s[0], 10);
    const sm = parseInt(s[1], 10);

    const tEnd = document.getElementById('endTime').value;
    const e = tEnd.split(':');
    const eh = parseInt(e[0], 10);
    const em = parseInt(e[1], 10);

    const ce = parseInt(document.getElementById('cycleEveryMin').value, 10);
    const co = parseInt(document.getElementById('cycleOnMin').value, 10);

    if(co >= ce){
      document.getElementById('cfgMsg').textContent = 'Error: la duración debe ser menor que el intervalo';
      return;
    }

    try{
      const r = await fetch('/config/program', {
        method:'POST',
        headers:{ 'Content-Type':'application/x-www-form-urlencoded' },
        body:
          'diasMask=' + encodeURIComponent(mask) +
          '&startHour=' + encodeURIComponent(sh) +
          '&startMin=' + encodeURIComponent(sm) +
          '&endHour=' + encodeURIComponent(eh) +
          '&endMin=' + encodeURIComponent(em) +
          '&cycleEveryMin=' + encodeURIComponent(ce) +
          '&cycleOnMin=' + encodeURIComponent(co),
        cache:'no-store'
      });

      document.getElementById('cfgMsg').textContent = r.ok ? 'Configuración guardada' : 'Error al guardar';
      await loadConfig();
      await update();
    }catch(err){
      document.getElementById('cfgMsg').textContent = String(err);
    }
  }

  function showDashboard(){
    document.getElementById('dashboardView').style.display = 'block';
    document.getElementById('configView').style.display = 'none';
    setActiveTab('dash');
  }

  async function showConfig(){
    document.getElementById('dashboardView').style.display = 'none';
    document.getElementById('configView').style.display = 'block';
    setActiveTab('cfg');
    await loadConfig();
    await update();
  }

  async function fetchHistoryAndDraw(){
    try{
      const r = await fetch('/history?last=360&t=' + Date.now(), { cache:'no-store' });
      const h = await r.json();
      const data = (h && h.data) ? h.data : [];

      const hrs = Math.round((h.count * h.intervalSec) / 3600 * 10)/10;
      const baseInfo = `${h.count} pts · ${h.intervalSec}s · ~${hrs} h · ` + (h.ntpOk ? 'NTP' : 'UPTIME');

      // 1) Suelo
      const soilCanvas = document.getElementById('soilCanvas');
      if(soilCanvas) drawSoil(soilCanvas, data);
      const soilInfo = document.getElementById('soilInfo');
      if(soilInfo) soilInfo.textContent = baseInfo;

      // 2) Humedad aire
      const airHumCanvas = document.getElementById('airHumCanvas');
      if(airHumCanvas) drawAirHum(airHumCanvas, data);
      const airHumInfo = document.getElementById('airHumInfo');
      if(airHumInfo) airHumInfo.textContent = baseInfo;

      // 3) Temperatura
      const tempCanvas = document.getElementById('tempCanvas');
      if(tempCanvas) drawTemp(tempCanvas, data);
      const tempInfo = document.getElementById('tempInfo');
      if(tempInfo) tempInfo.textContent = baseInfo;

    }catch(e){
      const ids = ['soilInfo','airHumInfo','tempInfo'];
      ids.forEach(id => {
        const el = document.getElementById(id);
        if(el) el.textContent = 'No se pudo leer /history';
      });
    }
  }


  function drawBase(ctx, W, H, yMin, yMax, yStep, unit){
    const padL = 38;   // espacio para números del eje Y
    const padR = 10;
    const padT = 10;
    const padB = 18;   // espacio para eje X

    const plotX = padL;
    const plotY = padT;
    const plotW = W - padL - padR;
    const plotH = H - padT - padB;

    ctx.clearRect(0,0,W,H);

    // ejes
    ctx.strokeStyle = 'rgba(255,255,255,.18)';
    ctx.lineWidth = 1;

    // eje Y
    ctx.beginPath();
    ctx.moveTo(plotX, plotY);
    ctx.lineTo(plotX, plotY + plotH);
    ctx.stroke();

    // eje X (abajo)
    ctx.beginPath();
    ctx.moveTo(plotX, plotY + plotH);
    ctx.lineTo(plotX + plotW, plotY + plotH);
    ctx.stroke();

    // ticks eje Y + etiquetas
    ctx.fillStyle = 'rgba(255,255,255,.55)';
    ctx.font = '11px system-ui, sans-serif';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    ctx.strokeStyle = 'rgba(255,255,255,.18)';
    for(let v=yMin; v<=yMax; v+=yStep){
      const norm = (v - yMin) / (yMax - yMin); // 0..1
      const y = plotY + plotH * (1 - norm);

      // tick
      ctx.beginPath();
      ctx.moveTo(plotX - 4, y);
      ctx.lineTo(plotX, y);
      ctx.stroke();

      // label
      ctx.fillText(`${v}`, plotX - 6, y);
    }

    // unidad (arriba del eje Y)
    if(unit){
      ctx.fillStyle = 'rgba(255,255,255,.35)';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'top';
      ctx.fillText(unit, 2, 2);
    }

    return { plotX, plotY, plotW, plotH };
  }


  function drawSoil(canvas, data){
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;

    if(!data || data.length < 2){
      ctx.clearRect(0,0,W,H);
      ctx.fillStyle = 'rgba(255,255,255,.75)';
      ctx.font = '14px system-ui, sans-serif';
      ctx.fillText('Esperando datos…', 12, 22);
      return;
    }

    const yMin = 0, yMax = 100, yStep = 20;
    const { plotX, plotY, plotW, plotH } = drawBase(ctx, W, H, yMin, yMax, yStep, '%');

    const xAt = (i) => plotX + (plotW * i / (data.length-1));
    const yAt = (v) => {
      const norm = (v - yMin) / (yMax - yMin);
      const clamped = Math.max(0, Math.min(1, norm));
      return plotY + (plotH * (1 - clamped));
    };

    ctx.strokeStyle = 'rgba(53,208,127,.95)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    for(let i=0;i<data.length;i++){
      const soil = data[i][1];
      const x = xAt(i);
      const y = yAt(soil);
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }


  function drawAirHum(canvas, data){
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;

    if(!data || data.length < 2){
      ctx.clearRect(0,0,W,H);
      ctx.fillStyle = 'rgba(255,255,255,.75)';
      ctx.font = '14px system-ui, sans-serif';
      ctx.fillText('Esperando datos…', 12, 22);
      return;
    }

    const yMin = 0, yMax = 100, yStep = 20;
    const { plotX, plotY, plotW, plotH } = drawBase(ctx, W, H, yMin, yMax, yStep, '%');

    const xAt = (i) => plotX + (plotW * i / (data.length-1));
    const yAt = (v) => {
      const norm = (v - yMin) / (yMax - yMin);
      const clamped = Math.max(0, Math.min(1, norm));
      return plotY + (plotH * (1 - clamped));
    };

    ctx.strokeStyle = 'rgba(90,160,255,.85)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    let started = false;
    for(let i=0;i<data.length;i++){
      const hum10 = data[i][3];
      if(hum10 === 65535) continue;
      const hum = hum10/10;

      const x = xAt(i);
      const y = yAt(hum);
      if(!started){ ctx.moveTo(x,y); started=true; }
      else ctx.lineTo(x,y);
    }
    if(started) ctx.stroke();
  }


  function drawTemp(canvas, data){
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;

    if(!data || data.length < 2){
      ctx.clearRect(0,0,W,H);
      ctx.fillStyle = 'rgba(255,255,255,.75)';
      ctx.font = '14px system-ui, sans-serif';
      ctx.fillText('Esperando datos…', 12, 22);
      return;
    }

    const yMin = 0, yMax = 60, yStep = 15;
    const { plotX, plotY, plotW, plotH } = drawBase(ctx, W, H, yMin, yMax, yStep, '°C');

    const xAt = (i) => plotX + (plotW * i / (data.length-1));
    const yAt = (v) => {
      const norm = (v - yMin) / (yMax - yMin);
      const clamped = Math.max(0, Math.min(1, norm));
      return plotY + (plotH * (1 - clamped));
    };

    ctx.strokeStyle = 'rgba(255,170,60,.90)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    let started = false;
    for(let i=0;i<data.length;i++){
      const t10 = data[i][2];
      if(t10 === -32768) continue;
      const t = t10/10;

      const x = xAt(i);
      const y = yAt(t);
      if(!started){ ctx.moveTo(x,y); started=true; }
      else ctx.lineTo(x,y);
    }
    if(started) ctx.stroke();
  }




</script>

</body>
</html>
)rawliteral";


  AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html);
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  response->addHeader("Pragma", "no-cache");
  response->addHeader("Expires", "0");
  request->send(response);
});



  // Ruta de salud (útil para debug)
  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "OK");
  });


  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    bool winActive = isWindowActive();
    bool safety = safetyCutoffActive();
    bool wifiOk = (WiFi.status() == WL_CONNECTED);

    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    response->addHeader("Pragma", "no-cache");
    response->addHeader("Expires", "0");

    char tbuf[16];   // para temp/hum con 1 decimal
    char ipbuf[20];  // "255.255.255.255"
    char stbuf[6];
    char enbuf[6];
    char timebuf[12];

    // Window strings (sin Strings intermedias)
    snprintf(stbuf, sizeof(stbuf), "%02d:%02d", startHour, startMin);
    snprintf(enbuf, sizeof(enbuf), "%02d:%02d", endHour, endMin);

    // IP
    if (wifiOk) {
      IPAddress ip = WiFi.localIP();
      snprintf(ipbuf, sizeof(ipbuf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    } else {
      ipbuf[0] = '\0';
    }

    // Hora
    if (ntpOk) {
      time_t now = time(nullptr);
      struct tm t;
      localtime_r(&now, &t);
      snprintf(timebuf, sizeof(timebuf), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
    } else {
      snprintf(timebuf, sizeof(timebuf), "--:--:--");
    }

    response->print("{");

    response->print("\"humedad\":"); response->print(humedadPct); response->print(",");
    response->print("\"umbral\":");  response->print(umbralPct);  response->print(",");
    response->print("\"progMode\":");response->print(progMode);   response->print(",");
    response->print("\"modo\":\"");  response->print(modoManual ? "MANUAL" : "AUTO"); response->print("\",");
    response->print("\"manual\":");  response->print(modoManual ? "true" : "false"); response->print(",");
    response->print("\"runMode\":"); response->print(runMode); response->print(",");
    response->print("\"riego\":\""); response->print(regando ? "ON" : "OFF"); response->print("\",");

    response->print("\"dhtOk\":"); response->print(dhtOk ? "true" : "false"); response->print(",");
    if (dhtOk) {
      // temp/hum 1 decimal sin String()
      dtostrf(tempC, 0, 1, tbuf);
      response->print("\"tempC\":"); response->print(tbuf); response->print(",");

      dtostrf(humAirPct, 0, 1, tbuf);
      response->print("\"humAir\":"); response->print(tbuf); response->print(",");
    } else {
      response->print("\"tempC\":null,");
      response->print("\"humAir\":null,");
    }

    response->print("\"ntpOk\":"); response->print(ntpOk ? "true" : "false"); response->print(",");
    response->print("\"winActive\":"); response->print(winActive ? "true" : "false"); response->print(",");
    response->print("\"winStart\":\""); response->print(stbuf); response->print("\",");
    response->print("\"winEnd\":\"");   response->print(enbuf); response->print("\",");
    response->print("\"safetyCutoff\":"); response->print(safety ? "true" : "false"); response->print(",");

    response->print("\"wifiOk\":"); response->print(wifiOk ? "true" : "false"); response->print(",");
    response->print("\"ip\":\""); response->print(ipbuf); response->print("\",");
    response->print("\"rssi\":"); response->print(getWifiRssi()); response->print(",");
    response->print("\"espTime\":\""); response->print(timebuf); response->print("\"");

    response->print("}");
    request->send(response);
  });



  server.on("/history", HTTP_GET, [](AsyncWebServerRequest *request) {

    // /history?last=360  -> últimas N muestras (opcional)
    int last = -1;
    if (request->hasParam("last")) {
      last = request->getParam("last")->value().toInt();
    }

    // Snapshot consistente del ring buffer
    uint16_t head = histHead;        // volatile -> copia local
    uint16_t available = histCount; // volatile -> copia local

    uint16_t count = available;
    if (last > 0 && last < count) {
      count = (uint16_t)last;
    }


    AsyncResponseStream *response =
      request->beginResponseStream("application/json");

    response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    response->addHeader("Pragma", "no-cache");
    response->addHeader("Expires", "0");

    // --- Header JSON ---
    response->print("{");
    response->print("\"capacity\":"); response->print(HISTORY_CAPACITY); response->print(",");
    response->print("\"count\":"); response->print(count); response->print(",");
    response->print("\"intervalSec\":"); response->print(historyIntervalSec); response->print(",");
    response->print("\"ntpOk\":"); response->print(ntpOk ? "true" : "false"); response->print(",");
    response->print("\"data\":[");

    // Índice inicial (orden temporal ascendente)
    int startIndex = (int)head - (int)count;
    while (startIndex < 0) startIndex += HISTORY_CAPACITY;


    for (uint16_t i = 0; i < count; i++) {
      uint16_t idx = (startIndex + i) % HISTORY_CAPACITY;
      const HistorySample &s = hist[idx];

      response->print("[");
      response->print(s.ts);    response->print(",");
      response->print(s.soil);  response->print(",");
      response->print(s.temp10);response->print(",");
      response->print(s.hum10); response->print(",");
      response->print(s.flags);
      response->print("]");

      if (i + 1 < count) response->print(",");
    }

    response->print("]}");

    request->send(response);
  });



server.on("/config/get", HTTP_GET, [](AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  response->addHeader("Pragma", "no-cache");
  response->addHeader("Expires", "0");

  response->print("{");
  response->print("\"progMode\":"); response->print(progMode); response->print(",");
  response->print("\"diasMask\":"); response->print(diasMask); response->print(",");
  response->print("\"startHour\":"); response->print(startHour); response->print(",");
  response->print("\"startMin\":"); response->print(startMin); response->print(",");
  response->print("\"endHour\":"); response->print(endHour); response->print(",");
  response->print("\"endMin\":"); response->print(endMin); response->print(",");
  response->print("\"cycleEveryMin\":"); response->print(cycleEveryMin); response->print(",");
  response->print("\"cycleOnMin\":"); response->print(cycleOnMin);
  response->print("}");

  request->send(response);
});


  server.on("/config/program", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Requiere todo junto
    const char* keys[] = {"diasMask","startHour","startMin","endHour","endMin","cycleEveryMin","cycleOnMin"};

    for (auto k : keys) {
      if (!request->hasParam(k, true)) {
        request->send(400, "text/plain", "Missing param");

        return;
      }
    }

    int dm = request->getParam("diasMask", true)->value().toInt();
    int sh = request->getParam("startHour", true)->value().toInt();
    int sm = request->getParam("startMin", true)->value().toInt();
    int eh = request->getParam("endHour", true)->value().toInt();
    int em = request->getParam("endMin", true)->value().toInt();

    int ce = request->getParam("cycleEveryMin", true)->value().toInt();
    int co = request->getParam("cycleOnMin", true)->value().toInt();

    // Sanitizado (mismo criterio que venís usando)

    if (dm < 0) dm = 0;
    if (dm > 127) dm = 127;

    if (sh < 0) sh = 0;
    if (sh > 23) sh = 23;

    if (sm < 0) sm = 0;
    if (sm > 59) sm = 59;

    if (eh < 0) eh = 0;
    if (eh > 23) eh = 23;

    if (em < 0) em = 0;
    if (em > 59) em = 59;

    if (ce < 1) ce = 1;
    if (ce > 1440) ce = 1440;

    if (co < 1) co = 1;
    if (co > 1440) co = 1440;
    if (co >= ce) co = ce - 1;
    if (co < 1) co = 1;


    // Aplicar
    diasMask = dm;
    startHour = sh;
    startMin  = sm;
    endHour   = eh;
    endMin    = em;
    cycleEveryMin = ce;
    cycleOnMin = co;

    onConfigChanged(true);
    saveConfigToNVS();

    request->send(200, "text/plain", "OK");
  });


  // ---- Set Umbral ----
  server.on("/umbral/set", HTTP_POST, [](AsyncWebServerRequest *request) {
  if (!request->hasParam("value", true)) {
    request->send(400, "text/plain", "Missing value");
    return;
  }

    String v = request->getParam("value", true)->value();
    int val = v.toInt();

    if (val < 0) val = 0;
    if (val > 100) val = 100;

    umbralPct = val;
    saveConfigToNVS();

    request->send(200, "text/plain", "OK");
  });

  server.on("/mode/set", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("value", true)) {
      request->send(400, "text/plain", "Missing value");
      return;
    }

    int v = request->getParam("value", true)->value().toInt();

    // Códigos UI:
    // 0=MANUAL (ON fijo)
    // 1=AUTO
    // 2=PROGRAMADO + SENSOR
    // 3=APAGADO
    // 4=PROGRAMADO + CICLOS
    if (v < 0) v = 1;
    if (v > 4) v = 1;

    // AUTO / APAGADO / PROGRAMADO
    if (v == 1) {
      runMode = 0;      // AUTO
    } else if (v == 3) {
      runMode = 2;      // APAGADO
    } else if (v == 2) {
      runMode = 1;      // PROGRAMADO
      progMode = 0;     // SENSOR
    } else if (v == 4) {
      runMode = 1;      // PROGRAMADO
      progMode = 1;     // CICLOS
    }

    // Salir siempre de manual al cambiar modo desde UI
    if (modoManual) {
      modoManual = false;
    }


    onConfigChanged(true);
    saveConfigToNVS();
    request->send(200, "text/plain", "OK");
  });

  server.on("/manual/set", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("value", true)) {
      request->send(400, "text/plain", "Missing value");
      return;
    }

    int v = request->getParam("value", true)->value().toInt();

    if (v == 1) {
      enterManual(true);
    } else {
      exitManual(true);
    }

    saveConfigToNVS();
    request->send(200, "text/plain", "OK");
  });


  server.on("/nvs/cleanup", HTTP_POST, [](AsyncWebServerRequest *request) {
    // borra keys viejas (si existen)
    prefs.remove("autoMode");

    request->send(200, "text/plain", "OK");
  });



  server.begin();
  Serial.println("Servidor web iniciado (puerto 80).");
}

void setup() {
  Serial.begin(115200);
  lastPumpChangeMs = millis() - MIN_OFF_MS;  // permite encender inmediatamente si corresponde

  pinMode(PIN_LED_HEARTBEAT, OUTPUT);
  pinMode(PIN_BOMBA, OUTPUT);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_AMARILLO, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);

  digitalWrite(PIN_BOMBA, LOW);
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_AMARILLO, LOW);
  digitalWrite(PIN_LED_ROJO, LOW);
  digitalWrite(PIN_LED_HEARTBEAT, LOW);

  prefs.begin("riego", false);
  loadConfigFromNVS();
  Serial.print("Umbral cargado: ");
  Serial.println(umbralPct);

  pinMode(PIN_BTN, INPUT_PULLUP);

    dht.begin();

  // I2C (pines estándar ESP32)
  Wire.begin(21, 22);

  delay(2000);     // tiempo para abrir Serial Monitor
  lastHistorySampleMs = millis(); // arranca el conteo del histórico desde el inicio
  setupServer();
}

void toggleManual() {
  if (!modoManual) {
    enterManual(true);
    Serial.println(">> BTN: -> MANUAL (restore habilitado)");
  } else {
    exitManual(true);
    saveConfigToNVS();
    Serial.println(">> BTN: MANUAL -> restore");
  }
}




void heartbeat() {
  unsigned long now = millis();
  if (now - lastHbMs >= HB_PERIOD_MS) {
    lastHbMs = now;
    hbState = !hbState;
    digitalWrite(PIN_LED_HEARTBEAT, hbState ? HIGH : LOW);
  }
}


void applyPumpOutput() {
  digitalWrite(PIN_BOMBA, regando ? HIGH : LOW);
}

void applyStatusLeds() {
  // Manual
  digitalWrite(PIN_LED_AMARILLO, modoManual ? HIGH : LOW);

  // Bomba (estado real)
  digitalWrite(PIN_LED_ROJO, regando ? HIGH : LOW);

  // “Sistema habilitado”
  bool habilitado = false;

  if (runMode == 2) {
    habilitado = false; // apagado
  } else if (runMode == 0) {
    habilitado = true;  // auto sensor siempre “habilitado”
  } else {
    habilitado = (ntpOk && isWindowActive()); // programado
  }


  digitalWrite(PIN_LED_VERDE, habilitado ? HIGH : LOW);
}

void onConfigChanged(bool stopPumpIfAuto) {


  // Si estamos en AUTO, lo más seguro es cortar inmediatamente
  // (si estás en MANUAL, respetamos manual salvo seguridad dura).
  if (stopPumpIfAuto && !modoManual) {
    regando = false;

    // Importante: dejá listo el anti-ciclo para permitir encender cuando corresponda
    // (si querés que pueda re-encender enseguida al volver a estar “habilitado”)
    lastPumpChangeMs = millis() - MIN_OFF_MS;
  }
}

void enterManual(bool savePrev) {
  if (savePrev) {
    prevRunMode = runMode;
    prevProgMode = progMode;
  }
  modoManual = true;
  regando = true;
  lastPumpChangeMs = millis();
}

void exitManual(bool restorePrev) {
  modoManual = false;
  if (restorePrev) {
    runMode = prevRunMode;
    progMode = prevProgMode;
  }
  onConfigChanged(true);
}


void handleButton() {
  bool raw = digitalRead(PIN_BTN);

  if (raw != btnRawPrev) {
    btnRawPrev = raw;
    btnLastChangeMs = millis();
  }

  if (millis() - btnLastChangeMs >= DEBOUNCE_MS) {
    if (btnStable == HIGH && raw == LOW) {
      toggleManual();
    }

    btnStable = raw;
  }
}

bool safetyCutoffActive() {
  return (humedadPct >= 90);
}

void enforceSafetyCutoff() {
  if (!safetyCutoffActive()) return;

  // Corta SIEMPRE e inmediato (no respeta MIN_ON por ser seguridad)
  if (regando) {
    regando = false;
    lastPumpChangeMs = millis();
  }

}

bool sensorWantsOn() {
  // Histéresis pura (sin MIN_ON/MIN_OFF)
  if (regando) {
    return !(humedadPct > (umbralPct + H));   // mantiene ON hasta superar umbral+H
  } else {
    return (humedadPct < (umbralPct - H));    // enciende recién bajo umbral-H
  }
}

void forceOffImmediate() {
  if (regando) {
    regando = false;
    lastPumpChangeMs = millis();
  }
}

void applyAntiCycle(bool desired) {
  unsigned long now = millis();
  if (desired == regando) return;

  if (regando) {
    // ON -> OFF
    if (now - lastPumpChangeMs >= MIN_ON_MS) {
      regando = false;
      lastPumpChangeMs = now;
    }
  } else {
    // OFF -> ON
    if (now - lastPumpChangeMs >= MIN_OFF_MS) {
      regando = true;
      lastPumpChangeMs = now;
    }
  }
}

void runAutoSensor() {
  bool desired = sensorWantsOn();
  applyAntiCycle(desired);
}

void runProgSensor() {
  // Igual que AUTO sensor, pero ya sabemos que estamos en ventana
  bool desired = sensorWantsOn();
  applyAntiCycle(desired);
}

void runProgCycles() {
  // En ciclos no se usa umbral. Solo seguridad dura

  int elapsed = windowElapsedMin();

  // Validación: duración < intervalo
  int every = cycleEveryMin;
  int on    = cycleOnMin;

  if (every < 1) every = 1;
  if (every > 1440) every = 1440;

  if (on < 1) on = 1;
  if (on > 1440) on = 1440;

  // Regla clave: debe ser menor que el intervalo (si no, no tiene sentido)
  if (on >= every) on = every - 1;
  if (on < 1) on = 1; // por si every era 1

  // Fase dentro del slot
  int pos = elapsed % every;

  bool desired = (pos < on);

  // Para que corte/arranque “justo” a los límites del slot, conviene NO usar anti-ciclo aquí.
  // Pero si querés mantenerlo por hardware (evitar relay chatter), lo dejamos configurable.
  // Opción A (recomendado para exactitud de horario):
  regando = desired;

  // Si preferís respetar MIN_ON/MIN_OFF también en ciclos:
  // applyAntiCycle(desired);
}


void sampleAndControl() {
  humedadPct = map(analogRead(PIN_HUMEDAD), 0, 4095, 0, 100);

  // Seguridad dura: si >=90% siempre OFF y salir
  if (humedadPct >= 90) {
    forceOffImmediate();
    return;
  }

  if (modoManual) return;

  if (runMode == 2) { forceOffImmediate(); return; }
  if (runMode == 0) { runAutoSensor(); return; }

  if (!isWindowActive()) { forceOffImmediate(); return; }

  if (progMode == 0) runProgSensor();
  else runProgCycles();

}


void loop() {
  
  heartbeat();

  handleButton();

  updateNtpStatus();

  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    sampleDHT();
    sampleAndControl();   // solo decide "regando"
    applyPumpOutput();    // SIEMPRE escribe GPIO18
    applyStatusLeds();    // LEDs de estado (GPIO25/26/27)
  }

  // Histórico independiente del control (cada X segundos)
  historyTick();

}
