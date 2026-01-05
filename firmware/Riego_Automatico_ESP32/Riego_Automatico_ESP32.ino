#include <Wire.h>
#include <math.h>
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
#define PIN_DHT           17  // reservado para DHT22 (DATA)


// ---------- Estado del sistema ----------
volatile int humedadPct = 0;
volatile int umbralPct  = 50;
volatile bool modoManual = false;
volatile bool regando = false;
volatile int diasMask = 127;
volatile int runMode = 0;

// --- Futuro sensor ambiente (DHT22) ---

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
void enterManual(bool savePrev);
void exitManual(bool restorePrev);




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

    #bottomBar {
      position: fixed;
      bottom: 0;
      left: 0;
      right: 0;
      display: flex;
      background: #111;
    }

    #bottomBar button {
      flex: 1;
      padding: 14px;
      border: none;
      color: white;
      background: #222;
    }

    body {
      padding-bottom: 60px;
    }


  </style>
</head>
<body>

<div id="dashboardView">

  <h2>Riego ESP32 - DEBUG</h2>

  <div class="card">
    <div class="row"><div>Tick JS</div><div><code id="tick">0</code></div></div>
    <div class="row"><div>HTTP</div><div><code id="http">--</code></div></div>
    <div class="row"><div>Error</div><div><code id="err" class="err">--</code></div></div>
    <div class="row"><div>Humedad</div><div><b id="hum">--</b>%</div></div>
    <div class="row"><div>Umbral</div><div><b id="umb">--</b>%</div></div>
    <div class="row"><div>Temp</div><div><b id="temp">--</b>°C</div></div>
    <div class="row"><div>Hum. aire</div><div><b id="humAir">--</b>%</div></div> 
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

  <select id="modeSelect" onchange="onModeChange(this.value)">
    <option value="1">Automático</option>
    <option value="2">Programado + Sensor</option>
    <option value="4">Programado + Ciclos</option>
    <option value="3">Apagado</option>
  </select>


    <div style="margin-top:8px;">
      <code id="modeMsg">--</code>
    </div>
  </div>

  <h3>Manual</h3>
  <div class="card">
    <label>
      <input type="checkbox" id="manualSwitch" onchange="onManualToggle(this.checked)">
      Riego manual (forzar ON)
    </label>
    <div style="margin-top:8px;">
      <small>Al apagar vuelve al modo anterior automáticamente.</small>
    </div>
  </div>


</div> <!-- dashboardView -->

<div id="configView" style="display:none">
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
    <div>Fin</div>
    <div><input id="endTime" type="time" value="18:00"></div>
  </div>

  <div style="margin-top:12px;">
    <button class="btn" onclick="saveConfig()">
      Guardar configuración
    </button>
    <div style="margin-top:6px;">
      <small id="cfgMsg">—</small>
    </div>
  </div>


  </div>


  <h3>RAW /status</h3>
  <pre id="raw">(sin datos)</pre>

</div> <!-- configView -->

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
        if (w.start && w.end) {
          document.getElementById('winActiveTxt').textContent =
            (w.active ? 'SI' : 'NO') + ` (${w.start} → ${w.end})`;
        }

      } catch (e) {
        // no rompas toda la UI por esto
      }
    }

    let manualActive = false;

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

        manualActive = j.manual;

        document.getElementById('manualSwitch').checked = manualActive;
        document.getElementById('modeSelect').disabled = manualActive;

        if (!manualActive) {
          if (j.runMode === 0) modeSelect.value = 1;
          else if (j.runMode === 2) modeSelect.value = 3;
          else if (j.runMode === 1) {
            modeSelect.value = (j.progMode === 1) ? 4 : 2;
          }
        }

        
        // DHT (por ahora puede venir null)
        if (j.dhtOk && j.tempC != null) {
          document.getElementById('temp').textContent = Number(j.tempC).toFixed(1);
        } else {
          document.getElementById('temp').textContent = '--';
        }

        if (j.dhtOk && j.humAir != null) {
          document.getElementById('humAir').textContent = Number(j.humAir).toFixed(1);
        } else {
          document.getElementById('humAir').textContent = '--';
        }

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

        // Hora fin
        const eh = String(cfg.endHour).padStart(2, '0');
        const em = String(cfg.endMin).padStart(2, '0');
        document.getElementById('endTime').value = `${eh}:${em}`;


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


    function showDashboard() {
      document.getElementById('dashboardView').style.display = 'block';
      document.getElementById('configView').style.display = 'none';
    }

    async function showConfig() {
      document.getElementById('dashboardView').style.display = 'none';
      document.getElementById('configView').style.display = 'block';

      await loadConfig();
      await updateWindowStatus();
    }

    async function onModeChange(v) {
      if (manualActive) return;
      await setMainMode(v);
    }

    async function onManualToggle(on) {
      await fetch('/manual/set', {
        method: 'POST',
        headers: {'Content-Type':'application/x-www-form-urlencoded'},
        body: 'value=' + (on ? '1' : '0')
      });
    }

    async function saveConfig() {
      const mask = getDiasMaskFromUI();

      // Inicio
      const tStart = document.getElementById('startTime').value;
      const s = tStart.split(':');
      const sh = parseInt(s[0], 10);
      const sm = parseInt(s[1], 10);

      // Fin
      const tEnd = document.getElementById('endTime').value;
      const e = tEnd.split(':');
      const eh = parseInt(e[0], 10);
      const em = parseInt(e[1], 10);

      // Ciclos
      const ce = parseInt(document.getElementById('cycleEveryMin').value, 10);
      const co = parseInt(document.getElementById('cycleOnMin').value, 10);

      if (co >= ce) {
        document.getElementById('cfgMsg').textContent =
          'Error: la duración debe ser menor que el intervalo';
        return;
      }


      try {
        const r = await fetch('/config/program', {
          method: 'POST',
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
          body:
            'diasMask=' + encodeURIComponent(mask) +
            '&startHour=' + encodeURIComponent(sh) +
            '&startMin=' + encodeURIComponent(sm) +
            '&endHour=' + encodeURIComponent(eh) +
            '&endMin=' + encodeURIComponent(em) +
            '&cycleEveryMin=' + encodeURIComponent(ce) +
            '&cycleOnMin=' + encodeURIComponent(co),
          cache: 'no-store'
        });

        document.getElementById('cfgMsg').textContent =
          r.ok ? 'Configuración guardada' : 'Error al guardar';

        await loadConfig();
        await updateWindowStatus();

      } catch (err) {
        document.getElementById('cfgMsg').textContent = String(err);
      }
    }



  </script>

  <div id="bottomBar">
    <button onclick="showDashboard()">Dashboard</button>
    <button onclick="showConfig()">Configuración</button>
  </div>


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
    json += "\"end\":\"" + String(endHour) + ":" + (endMin < 10 ? "0" : "") + String(endMin) + "\","; 
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
    json += "\"manual\":" + String(modoManual ? "true" : "false") + ",";
    json += "\"runMode\":" + String(runMode) + ",";
    json += "\"riego\":\"" + String(regando ? "ON" : "OFF") + "\",";

    // --- DHT (placeholder por ahora) ---
    json += "\"dhtOk\":" + String(dhtOk ? "true" : "false") + ",";
    if (dhtOk) {
      json += "\"tempC\":" + String(tempC, 1) + ",";
      json += "\"humAir\":" + String(humAirPct, 1);
    } else {
      json += "\"tempC\":null,";
      json += "\"humAir\":null";
    }

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
    json += "\"endHour\":" + String(endHour) + ",";
    json += "\"endMin\":" + String(endMin) + ",";
    json += "\"cycleEveryMin\":" + String(cycleEveryMin) + ",";
    json += "\"cycleOnMin\":" + String(cycleOnMin);
    json += "}";

    request->send(200, "application/json", json);
  });

  server.on("/config/program", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Requiere todo junto
    const char* keys[] = {"diasMask","startHour","startMin","endHour","endMin","cycleEveryMin","cycleOnMin"};

    for (auto k : keys) {
      if (!request->hasParam(k, true)) {
        request->send(400, "text/plain", String("Missing param: ") + k);
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

  // I2C (pines estándar ESP32)
  Wire.begin(21, 22);

  delay(2000);     // tiempo para abrir Serial Monitor
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
  // En ciclos: SOLO seguridad dura (90%) actúa, lo demás es por timing.
  // (enforceSafetyCutoff() ya se ejecuta antes)

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

  enforceSafetyCutoff();

  if (modoManual) return;

  if (runMode == 2) {         // APAGADO
    forceOffImmediate();
    return;
  }

  if (runMode == 0) {         // AUTO
    runAutoSensor();
    return;
  }

  // PROGRAMADO
  if (!isWindowActive()) {
    forceOffImmediate();
    return;
  }

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
