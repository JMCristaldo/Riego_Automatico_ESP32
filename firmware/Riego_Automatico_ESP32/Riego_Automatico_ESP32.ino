#include <Wire.h>
#include <WiFi.h>
#include <time.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

Preferences prefs;

#define PIN_LED_HEARTBEAT 2
#define PIN_HUMEDAD       32
#define PIN_BOMBA         18
#define PIN_BTN           16
#define PIN_LED_VERDE     25
#define PIN_LED_AMARILLO  26
#define PIN_LED_ROJO      27



// ---------- Estado del sistema ----------
volatile int humedadPct = 0;
volatile int umbralPct  = 50;
volatile bool modoManual = false;
volatile bool regando = false;
volatile int diasMask = 127;
volatile int runMode = 0;

// Estado previo solo para restore del botón físico
volatile int prevRunMode = 0;
volatile int prevProgMode = 0;


enum CycleState {
  CYCLE_IDLE,   // esperando próximo ciclo
  CYCLE_ON,    // regando Y minutos
  CYCLE_OFF    // esperando X minutos
};
volatile CycleState cycleState = CYCLE_IDLE;
unsigned long cycleStateMs = 0;   // marca de tiempo del último cambio


// ---------- NTP / Hora ----------
volatile bool ntpOk = false;
unsigned long lastNtpCheckMs = 0;
const unsigned long NTP_CHECK_PERIOD_MS = 2000;  // cada 2s


// Ventana: inicio + duración (min)
volatile int startHour = 0;
volatile int startMin = 0;
volatile int durWindowMin = 60;



// Programado: 0 = PROG_SENSOR, 1 = PROG_CICLOS
volatile int progMode = 0;

// Ciclos X/Y (min)
volatile int cycleEveryMin = 30; // cada X minutos
volatile int cycleOnMin = 2;  // riega Y minutos

// Seguridad por sensor en ciclos
volatile bool sensorLimitEnable = true;


// --- Anti-ciclo / histéresis ---
const int H = 3;  // histéresis en %
const unsigned long MIN_ON_MS  = 5000;
const unsigned long MIN_OFF_MS = 5000;

unsigned long lastPumpChangeMs = 0;  // cuándo cambió regando por última vez


// ---------- Servidor ----------
const char* ssid = "WiFi_Fibertel_nnk_2.4GHz";
const char* password = "cn6vxu7bjm";

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




void loadConfigFromNVS() {
  // Si no existe la key, mantiene el default actual (segundo parámetro)
  umbralPct = prefs.getInt("umbral", umbralPct);

  runMode = prefs.getInt("runMode", runMode);
  diasMask = prefs.getInt("diasMask", diasMask);

  startHour = prefs.getInt("stH", startHour);
  startMin  = prefs.getInt("stM", startMin);
  durWindowMin = prefs.getInt("durW", durWindowMin);

  progMode = prefs.getInt("progMode", progMode);

  cycleEveryMin = prefs.getInt("cyEvery", cycleEveryMin);
  cycleOnMin    = prefs.getInt("cyOn", cycleOnMin);

  sensorLimitEnable = prefs.getBool("sensLim", sensorLimitEnable);

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

  if (durWindowMin < 1) durWindowMin = 1;
  if (durWindowMin > 1440) durWindowMin = 1440;

  if (progMode < 0) progMode = 0;
  if (progMode > 1) progMode = 1;

  if (cycleEveryMin < 1) cycleEveryMin = 1;
  if (cycleEveryMin > 1440) cycleEveryMin = 1440;

  if (cycleOnMin < 1) cycleOnMin = 1;
  if (cycleOnMin > 1440) cycleOnMin = 1440;
}

void saveConfigToNVS() {
  prefs.putInt("umbral", umbralPct);

  prefs.putInt("runMode", runMode);
  prefs.putInt("diasMask", diasMask);

  prefs.putInt("stH", startHour);
  prefs.putInt("stM", startMin);
  prefs.putInt("durW", durWindowMin);

  prefs.putInt("progMode", progMode);

  prefs.putInt("cyEvery", cycleEveryMin);
  prefs.putInt("cyOn", cycleOnMin);

  prefs.putBool("sensLim", sensorLimitEnable);
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

  const int nowMinOfDay = t.tm_hour * 60 + t.tm_min; // 0..1439
  const int todayWday   = t.tm_wday;                 // 0=Dom..6=Sab

  const int startOfDayMin = startHour * 60 + startMin; // <-- nombre distinto (clave)
  int dur = durWindowMin;

  // Sanitizado mínimo
  if (dur <= 0) return false;
  if (dur > 1440) dur = 1440;

  // Ventana no cruza medianoche
  if (startOfDayMin + dur <= 1440) {
    if ((diasMask & (1 << todayWday)) == 0) return false;
    return (nowMinOfDay >= startOfDayMin) && (nowMinOfDay < (startOfDayMin + dur));
  }

  // Ventana cruza medianoche
  const int endMinNextDay = (startOfDayMin + dur) - 1440;

  // Tramo antes de medianoche (mismo día)
  if (nowMinOfDay >= startOfDayMin) {
    if ((diasMask & (1 << todayWday)) == 0) return false;
    return true;
  }

  // Tramo después de medianoche: debe haber arrancado AYER (día marcado)
  const int ydayWday = (todayWday + 6) % 7;
  if ((diasMask & (1 << ydayWday)) == 0) return false;
  return (nowMinOfDay < endMinNextDay);
}


void setupServer() {
  Serial.println("[setupServer] Entrando...");

  Serial.println("Conectando a WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  initNTP();
  Serial.println("NTP iniciado.");

  // Ruta principal
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  const char* html = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Riego ESP32 - DEBUG</title>
  <style>
    body { font-family: sans-serif; margin: 16px; }
    .card { border: 1px solid #ddd; border-radius: 10px; padding: 12px; max-width: 560px; }
    .row { display: flex; justify-content: space-between; margin: 6px 0; }
    pre { background:#f4f4f4; padding:10px; border-radius:10px; overflow:auto; }
    .err { color:#b00020; }
    code { background:#f4f4f4; padding:2px 6px; border-radius:6px; }
    .btn{
      padding: 12px 14px;
      margin: 4px 4px 0 0;
      border-radius: 10px;
    }
    .btn { min-height: 44px; }

  </style>
</head>
<body>
  <h2>Riego ESP32 - DEBUG</h2>

  <div class="card">
    <div class="row"><div>Tick JS</div><div><code id="tick">0</code></div></div>
    <div class="row"><div>HTTP</div><div><code id="http">--</code></div></div>
    <div class="row"><div>Error</div><div><code id="err" class="err">--</code></div></div>
    <div class="row"><div>Humedad</div><div><b id="hum">--</b>%</div></div>
    <div class="row"><div>Umbral</div><div><b id="umb">--</b>%</div></div>
    <div class="row"><div>Modo</div><div><b id="modo">--</b></div></div>
    <div class="row"><div>Riego</div><div><b id="riego">--</b></div></div>
    <div class="row"><div>Hora</div><div><code id="ts">--</code></div></div>
  </div>

  <h3>Umbral</h3>
  <div class="card">
    <div class="row">
      <div>Umbral (%)</div>
      <div><b id="umb2">--</b>%</div>
    </div>

    <input id="umbSet" type="range" min="0" max="100" value="55" style="width:100%">
  </div>


  <h3>Modo</h3>
  <div class="card">
    <button class="btn" onclick="setMainMode(1)">Automático</button>
    <button class="btn" onclick="saveAndSetProgram(0)">Programado + Sensor</button>
    <button class="btn" onclick="saveAndSetProgram(1)">Programado + Ciclos</button>
    <button class="btn" onclick="setMainMode(0)">Manual</button>
    <button class="btn" onclick="setMainMode(3)">Apagado</button>

    <div style="margin-top:8px;">
      <code id="modeMsg">--</code>
    </div>
  </div>


  <h3>Programación (ventana)</h3>
  <div class="card">
    <div class="row"><div>NTP</div><div><b id="ntpOkTxt">--</b></div></div>
    <div class="row"><div>Hora local</div><div><code id="timeTxt">--:--:--</code></div></div>
    <div class="row"><div>Ventana activa</div><div><b id="winActiveTxt">--</b></div></div>

    <hr>

    <div><b>Días de riego (inicio)</b> <small>(0=Dom ... 6=Sab)</small></div>
    <div style="display:flex; gap:10px; flex-wrap:wrap; margin:8px 0;">
      <label><input type="checkbox" class="day" data-bit="0">Dom</label>
      <label><input type="checkbox" class="day" data-bit="1">Lun</label>
      <label><input type="checkbox" class="day" data-bit="2">Mar</label>
      <label><input type="checkbox" class="day" data-bit="3">Mié</label>
      <label><input type="checkbox" class="day" data-bit="4">Jue</label>
      <label><input type="checkbox" class="day" data-bit="5">Vie</label>
      <label><input type="checkbox" class="day" data-bit="6">Sáb</label>
    </div>

<h3>Ciclos</h3>
<div class="card" id="cyclesCard">

  <div class="row"><div>Intervalo (min)</div>
    <div><input id="cycleEveryMin" type="number" min="1" max="1440" value="30" style="width:100px"></div>
  </div>

  <div class="row"><div>Duración riego (min)</div>
    <div><input id="cycleOnMin" type="number" min="1" max="1440" value="2" style="width:100px"></div>
  </div>


  <small>Nota: en “ciclos” se ignora el umbral. Seguridad dura: corta si humedad ≥ 90%.</small>
</div>

    <div class="row">
      <div>Inicio</div>
      <div><input id="startTime" type="time" value="00:00"></div>
    </div>

    <div class="row">
      <div>Duración (min)</div>
      <div><input id="durMin" type="number" min="1" max="1440" value="60" style="width:100px"></div>
    </div>


  </div>


  <h3>RAW /status</h3>
  <pre id="raw">(sin datos)</pre>

  <script>
    let n = 0;
    let draggingUmb = false;

    window.addEventListener('load', () => {
      const s = document.getElementById('umbSet');

      // Mostrar el valor mientras se mueve el slider
      s.addEventListener('input', () => {
        document.getElementById('umb2').textContent = s.value;
      });

      // Detectar inicio de arrastre
      s.addEventListener('pointerdown', () => {
        draggingUmb = true;
      });

      // Al soltar: dejar de arrastrar y ENVIAR el valor al ESP32
      s.addEventListener('pointerup', () => {
        draggingUmb = false;
        setUmbral();   
      });

      s.addEventListener('pointercancel', () => {
        draggingUmb = false;
      });
    });

    // Sincroniza UI con el valor real del ESP32
    function syncUmbralUI(v) {
      document.getElementById('umb2').textContent = v;
      if (!draggingUmb) {
        document.getElementById('umbSet').value = v;
      }
    }

    async function updateWindowStatus() {
      try {
        const r = await fetch('/window/status?t=' + Date.now(), { cache: 'no-store' });
        const w = await r.json();

        document.getElementById('ntpOkTxt').textContent = w.ntpOk ? 'OK' : 'NO';
        document.getElementById('timeTxt').textContent = w.time;
        document.getElementById('winActiveTxt').textContent = w.active ? 'SI' : 'NO';
      } catch (e) {
        // no rompas toda la UI por esto
      }
    }


    async function update() {
      n++;
      document.getElementById('tick').textContent = n;

      const url = '/status?t=' + Date.now(); // cache-buster real
      
      try {
        const r = await fetch(url, { cache: 'no-store' });
        document.getElementById('http').textContent = r.status + ' ' + r.statusText;

        const txt = await r.text(); // primero texto para debug
        document.getElementById('raw').textContent = txt;

        const j = JSON.parse(txt);  // después parseo
        

        

        document.getElementById('hum').textContent = j.humedad;
        document.getElementById('umb').textContent = j.umbral;
        syncUmbralUI(j.umbral);
        
        let modoTxt = '';

        if (j.modo === 'MANUAL') {
          modoTxt = 'MANUAL';
        } else {
          // AUTO pero lo desglosamos por runMode + progMode
          if (j.runMode === 0) {
            modoTxt = 'AUTOMÁTICO';
          } else if (j.runMode === 1) {
            modoTxt = (j.progMode === 1) ? 'PROGRAMADO + CICLOS' : 'PROGRAMADO + SENSOR';
          } else if (j.runMode === 2) {
            modoTxt = 'APAGADO';
          } else {
            modoTxt = 'AUTO(?)';
          }
        }

        document.getElementById('modo').textContent = modoTxt;


        document.getElementById('riego').textContent = j.riego;

        document.getElementById('ts').textContent = new Date().toLocaleTimeString();
        document.getElementById('err').textContent = '--';
      } catch (e) {
        document.getElementById('err').textContent = String(e);
      }
      await updateWindowStatus();
    }


    async function setUmbral() {
      const v = document.getElementById('umbSet').value;

      try {
        await fetch('/umbral/set', {
          method: 'POST',
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
          body: 'value=' + encodeURIComponent(v),
          cache: 'no-store'
        });

        update();
        // refresco inmediato
      } catch (e) {
        document.getElementById('err').textContent = String(e);
      }
    }


    async function loadConfig() {
      try {
        const r = await fetch('/config/get?t=' + Date.now(), { cache: 'no-store' });
        const cfg = await r.json();

        // Días
        document.querySelectorAll('input.day').forEach(cb => {
          const bit = parseInt(cb.dataset.bit, 10);
          cb.checked = ((cfg.diasMask >> bit) & 1) === 1;
        });

        // Hora inicio
        const hh = String(cfg.startHour).padStart(2, '0');
        const mm = String(cfg.startMin).padStart(2, '0');
        document.getElementById('startTime').value = `${hh}:${mm}`;

        // Duración
        document.getElementById('durMin').value = cfg.durWindowMin;

        //Ciclos
        document.getElementById('cycleEveryMin').value = cfg.cycleEveryMin;
        document.getElementById('cycleOnMin').value = cfg.cycleOnMin;

      } catch (e) {
        document.getElementById('err').textContent = String(e);
      }
    }

    function getDiasMaskFromUI() {
      let mask = 0;
      document.querySelectorAll('input.day').forEach(cb => {
        const bit = parseInt(cb.dataset.bit, 10);
        if (cb.checked) mask |= (1 << bit);
      });
      return mask;
    }

    async function saveAndSetProgram(pm) {
      // 1) Guardar configuración + progMode
      const mask = getDiasMaskFromUI();

      const t = document.getElementById('startTime').value; // "HH:MM"
      const parts = t.split(':');
      const sh = parseInt(parts[0], 10);
      const sm = parseInt(parts[1], 10);

      const du = parseInt(document.getElementById('durMin').value, 10);
      const ce = parseInt(document.getElementById('cycleEveryMin').value, 10);
      const co = parseInt(document.getElementById('cycleOnMin').value, 10);

      try {
        const r1 = await fetch('/config/program', {
          method: 'POST',
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
          body:
            'progMode=' + encodeURIComponent(pm) +
            '&diasMask=' + encodeURIComponent(mask) +
            '&startHour=' + encodeURIComponent(sh) +
            '&startMin=' + encodeURIComponent(sm) +
            '&durWindowMin=' + encodeURIComponent(du) +
            '&cycleEveryMin=' + encodeURIComponent(ce) +
            '&cycleOnMin=' + encodeURIComponent(co),
          cache: 'no-store'
        });

        // 2) Activar modo PROGRAMADO + (sensor/ciclos)
        const modeValue = (pm === 0) ? 2 : 4;

        const r2 = await fetch('/mode/set', {
          method: 'POST',
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
          body: 'value=' + encodeURIComponent(modeValue),
          cache: 'no-store'
        });


        document.getElementById('modeMsg').textContent =
          `CFG:${r1.status} ${r1.statusText} | MODE:${r2.status} ${r2.statusText}`;

        await loadConfig();  // trae sanitizado
        await update();      // refresca status/ventana
      } catch (e) {
        document.getElementById('modeMsg').textContent = String(e);
      }
    }



    async function setMainMode(v) {
      try {
        const r = await fetch('/mode/set', {
          method: 'POST',
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
          body: 'value=' + encodeURIComponent(v),
          cache: 'no-store'
        });
        document.getElementById('modeMsg').textContent = r.status + ' ' + r.statusText;
        await update();
      } catch (e) {
        document.getElementById('modeMsg').textContent = String(e);
      }
    }


    window.addEventListener('load', () => {
      loadConfig();
      update();
      setInterval(update, 1000);
    });

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


  server.on("/window/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"ntpOk\":" + String(ntpOk ? "true" : "false") + ",";
    json += "\"active\":" + String(isWindowActive() ? "true" : "false") + ",";
    json += "\"diasMask\":" + String(diasMask) + ",";
    json += "\"start\":\"" + String(startHour) + ":" + (startMin < 10 ? "0" : "") + String(startMin) + "\",";
    json += "\"durMin\":" + String(durWindowMin) + ",";
    json += "\"time\":\"" + getLocalTimeString() + "\"";
    json += "}";

    request->send(200, "application/json", json);
  });


  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"humedad\":" + String(humedadPct) + ",";
    json += "\"umbral\":" + String(umbralPct) + ",";
    json += "\"progMode\":" + String(progMode) + ",";
    json += "\"modo\":\"" + String(modoManual ? "MANUAL" : "AUTO") + "\",";
    json += "\"runMode\":" + String(runMode) + ",";
    json += "\"riego\":\"" + String(regando ? "ON" : "OFF") + "\"";
    json += "}";

    AsyncWebServerResponse *response =
      request->beginResponse(200, "application/json", json);

    response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    response->addHeader("Pragma", "no-cache");
    response->addHeader("Expires", "0");

    request->send(response);
  });

  server.on("/config/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"progMode\":" + String(progMode) + ",";
    json += "\"diasMask\":" + String(diasMask) + ",";
    json += "\"startHour\":" + String(startHour) + ",";
    json += "\"startMin\":" + String(startMin) + ",";
    json += "\"durWindowMin\":" + String(durWindowMin) + ",";
    json += "\"cycleEveryMin\":" + String(cycleEveryMin) + ",";
    json += "\"cycleOnMin\":" + String(cycleOnMin) + ",";
    json += "\"sensorLimitEnable\":" + String(sensorLimitEnable ? "true" : "false");
    json += "}";

    request->send(200, "application/json", json);
  });

  server.on("/config/program", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Requiere todo junto
    const char* keys[] = {"progMode","diasMask","startHour","startMin","durWindowMin","cycleEveryMin","cycleOnMin"};
    for (auto k : keys) {
      if (!request->hasParam(k, true)) {
        request->send(400, "text/plain", String("Missing param: ") + k);
        return;
      }
    }

    int pm = request->getParam("progMode", true)->value().toInt();
    int dm = request->getParam("diasMask", true)->value().toInt();
    int sh = request->getParam("startHour", true)->value().toInt();
    int sm = request->getParam("startMin", true)->value().toInt();
    int du = request->getParam("durWindowMin", true)->value().toInt();
    int ce = request->getParam("cycleEveryMin", true)->value().toInt();
    int co = request->getParam("cycleOnMin", true)->value().toInt();

    // Sanitizado (mismo criterio que venís usando)
    if (pm < 0) pm = 0;
    if (pm > 1) pm = 1;

    if (dm < 0) dm = 0;
    if (dm > 127) dm = 127;

    if (sh < 0) sh = 0;
    if (sh > 23) sh = 23;

    if (sm < 0) sm = 0;
    if (sm > 59) sm = 59;

    if (du < 1) du = 1;
    if (du > 1440) du = 1440;

    if (ce < 1) ce = 1;
    if (ce > 1440) ce = 1440;

    if (co < 1) co = 1;
    if (co > 1440) co = 1440;
    if (co > ce) co = ce;

    // Aplicar
    progMode = pm;
    diasMask = dm;
    startHour = sh;
    startMin = sm;
    durWindowMin = du;
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


    // MANUAL UI
    if (v == 0) {
      modoManual = true;
      regando = true;
      lastPumpChangeMs = millis();
      request->send(200, "text/plain", "MANUAL_ON");
      return;
    }

    // salimos de manual al entrar a cualquier modo no-manual
    if (modoManual) modoManual = false;

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


    onConfigChanged(true);
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

  // I2C (pines estándar ESP32)
  Wire.begin(21, 22);

  delay(2000);     // tiempo para abrir Serial Monitor
  setupServer();
}

void toggleManual() {
  if (!modoManual) {
    // Entrar a MANUAL (por botón físico): guardar modo anterior
    prevRunMode = runMode;
    prevProgMode = progMode;
    modoManual = true;
    regando = true;
    lastPumpChangeMs = millis();
    Serial.println(">> BTN: -> MANUAL (restore habilitado)");
    return;
  }

  // Salir de MANUAL (por botón físico): volver al modo anterior
  modoManual = false;
  runMode = prevRunMode;
  progMode = prevprogMode;

  onConfigChanged(true);
  saveConfigToNVS();

  Serial.println(">> BTN: MANUAL -> restore prevRunMode");
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
  // Reset de estados internos para evitar “solapamientos”
  cycleState = CYCLE_IDLE;
  cycleStateMs = millis();

  // Si estamos en AUTO, lo más seguro es cortar inmediatamente
  // (si estás en MANUAL, respetamos manual salvo seguridad dura).
  if (stopPumpIfAuto && !modoManual) {
    regando = false;

    // Importante: dejá listo el anti-ciclo para permitir encender cuando corresponda
    // (si querés que pueda re-encender enseguida al volver a estar “habilitado”)
    lastPumpChangeMs = millis() - MIN_OFF_MS;
  }
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

  // Resetea ciclos para re-arrancar limpio cuando vuelva a ser seguro
  cycleState = CYCLE_IDLE;
  cycleStateMs = millis();

}

bool sensorWantsOn() {
  // Histéresis pura (sin MIN_ON/MIN_OFF)
  if (regando) {
    return !(humedadPct > (umbralPct + H));   // mantiene ON hasta superar umbral+H
  } else {
    return (humedadPct < (umbralPct - H));    // enciende recién bajo umbral-H
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
  
  // Límite por sensor (opcional) en modo ciclos:
  // si ya estamos por arriba del umbral + histéresis, NO regar y resetear ciclos.
  if (sensorLimitEnable && (humedadPct >= (umbralPct + H))) {
    applyAntiCycle(false);     // pedir OFF (respetando MIN_ON si estuviera ON)
    cycleState = CYCLE_IDLE;   // re-arranca limpio cuando baje
    cycleStateMs = millis();
    return;
  }

  // Programado + ciclos: IGNORA UMBRAL (salvo seguridad dura 90% que ya se chequea antes)
  if (cycleOnMin >= cycleEveryMin) cycleOnMin = cycleEveryMin;

  unsigned long now = millis();

  if (cycleState == CYCLE_IDLE) {
    cycleState = CYCLE_ON;
    cycleStateMs = now;
  }

  if (cycleState == CYCLE_ON) {
    // Durante ON: queremos ON sí o sí (sin mirar sensor/umbral)
    applyAntiCycle(true);

    if (now - cycleStateMs >= (unsigned long)cycleOnMin * 60000UL) {
      cycleState = CYCLE_OFF;
      cycleStateMs = now;
      applyAntiCycle(false);
    }
    return;
  }

  // CYCLE_OFF
  applyAntiCycle(false);

  unsigned long offMs = (unsigned long)(cycleEveryMin - cycleOnMin) * 60000UL;
  if (offMs < 1000UL) offMs = 1000UL;

  if (now - cycleStateMs >= offMs) {
    cycleState = CYCLE_ON;
    cycleStateMs = now;
  }
}

void sampleAndControl() {
  humedadPct = map(analogRead(PIN_HUMEDAD), 0, 4095, 0, 100);

  // Seguridad dura: corta siempre (AUTO/MANUAL) y resetea ciclos
  enforceSafetyCutoff();

  // Si estás en MANUAL, no hacemos control automático.
  // (pero la seguridad ya pudo cortar la bomba arriba)
  if (modoManual) return;

    // APAGADO: no se riega por nada (solo manual podría prender, pero manual ya salió arriba)
  if (runMode == 2) {
    if (regando) {
      regando = false;
      lastPumpChangeMs = millis();
    }
    cycleState = CYCLE_IDLE;
    return;
  }

  // AUTO por sensor
  if (runMode == 0) {
    runAutoSensor();
    return;
  }

    // PROGRAMADO: fuera de ventana => OFF + reset ciclos
  if (!isWindowActive()) {
    cycleState = CYCLE_IDLE;
    if (regando) {
      regando = false;
      lastPumpChangeMs = millis();
    }
    return;
  }

  // Dentro de ventana:
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
    sampleAndControl();   // solo decide "regando"
    applyPumpOutput();    // SIEMPRE escribe GPIO18
    applyStatusLeds();    // LEDs de estado (GPIO25/26/27)
  }
}
