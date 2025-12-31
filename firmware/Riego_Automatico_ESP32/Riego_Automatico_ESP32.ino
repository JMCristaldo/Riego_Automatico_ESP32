#include <Wire.h>
//#include <U8g2lib.h>
#include <WiFi.h>
#include <time.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

Preferences prefs;

#define PIN_HUMEDAD 32
#define PIN_BOMBA   18
#define PIN_BTN     15

// OLED SSD1306 128x64 I2C (dirección detectada: 0x3C)
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ---------- Estado del sistema ----------
volatile int humedadPct = 0;
volatile int umbralPct  = 50;
volatile bool modoManual = false;
volatile bool regando = false;

// ---------- NTP / Hora ----------
volatile bool ntpOk = false;
unsigned long lastNtpCheckMs = 0;
const unsigned long NTP_CHECK_PERIOD_MS = 2000;  // cada 2s


// ---------- Config AUTO / Programado (persistente) ----------
// Auto mode: 0 = AUTO_SENSOR, 1 = AUTO_PROGRAMADO
volatile int autoMode = 0;

// Programado: días de INICIO (bits 0=Dom .. 6=Sab)
volatile int diasMask = 0b01111110; // Lun..Vie por defecto

// Ventana: inicio + duración (min)
volatile int startHour = 7;
volatile int startMin  = 30;
volatile int durWindowMin = 60; // 1 hora por defecto

// Programado: 0 = PROG_SENSOR, 1 = PROG_CICLOS
volatile int progMode = 0;

// Ciclos X/Y (min)
volatile int cycleEveryMin = 30; // cada X minutos
volatile int cycleOnMin    = 2;  // riega Y minutos

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
int dayStartMinuteIndex(int wday);



void loadConfigFromNVS() {
  // Si no existe la key, mantiene el default actual (segundo parámetro)
  umbralPct = prefs.getInt("umbral", umbralPct);

  autoMode = prefs.getInt("autoMode", autoMode);
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

  if (autoMode < 0) autoMode = 0;
  if (autoMode > 1) autoMode = 1;

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

  prefs.putInt("autoMode", autoMode);
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

  int nowMinOfDay = t.tm_hour * 60 + t.tm_min; // 0..1439
  int todayWday = t.tm_wday;                   // 0..6

  int startMin = startHour * 60 + startMin;
  int dur = durWindowMin;

  // Seguridad
  if (dur <= 0) return false;
  if (dur > 1440) dur = 1440; // por ahora limitamos a 24h (coherente con el modelo)

  // Caso 1: ventana NO cruza medianoche (start + dur <= 1440)
  if (startMin + dur <= 1440) {
    // Activa solo si HOY es un día marcado y estamos dentro
    if ((diasMask & (1 << todayWday)) == 0) return false;
    return (nowMinOfDay >= startMin) && (nowMinOfDay < (startMin + dur));
  }

  // Caso 2: ventana cruza medianoche (ej 20:00 + 600min => termina al día siguiente)
  int endMinNextDay = (startMin + dur) - 1440; // fin en el día siguiente (0..)

  // a) Si estamos en el tramo "antes de medianoche": hoy debe estar marcado
  if (nowMinOfDay >= startMin) {
    if ((diasMask & (1 << todayWday)) == 0) return false;
    return true;
  }

  // b) Si estamos en el tramo "después de medianoche": debe haber arrancado AYER (marcado)
  int ydayWday = (todayWday + 6) % 7; // ayer
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


  <h3>Control</h3>
  <div class="card">
    <button id="btnToggle" onclick="toggleManualWeb()">--</button>
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
        setUmbral();   // 👈 envío automático
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

    async function toggleManualWeb() {
      const btn = document.getElementById('btnToggle');
      btn.disabled = true;

      try {
        await fetch('/manual/toggle', { method: 'POST', cache: 'no-store' });
        update();
      } catch (e) {
        document.getElementById('err').textContent = String(e);
      }

      setTimeout(() => (btn.disabled = false), 300);
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
        document.getElementById('modo').textContent = j.modo;
        
        const btn = document.getElementById('btnToggle');

        if (j.modo === "AUTO") {
          btn.textContent = "Pasar a MANUAL (Riego ON)";
        } else {
          btn.textContent = "Volver a AUTO";
        }

        document.getElementById('riego').textContent = j.riego;

        document.getElementById('ts').textContent = new Date().toLocaleTimeString();
        document.getElementById('err').textContent = '--';
      } catch (e) {
        document.getElementById('err').textContent = String(e);
      }
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



    update();
    setInterval(update, 1000);
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

  server.on("/time/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"ntpOk\":" + String(ntpOk ? "true" : "false") + ",";
    json += "\"time\":\"" + getLocalTimeString() + "\"";
    json += "}";

    request->send(200, "application/json", json);
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
  json += "\"modo\":\"" + String(modoManual ? "MANUAL" : "AUTO") + "\",";
  json += "\"riego\":\"" + String(regando ? "ON" : "OFF") + "\"";
  json += "}";

  AsyncWebServerResponse *response =
      request->beginResponse(200, "application/json", json);

  // Evita cache en serio (browser/proxy)
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  response->addHeader("Pragma", "no-cache");
  response->addHeader("Expires", "0");

  request->send(response);
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

  server.on("/manual/toggle", HTTP_POST, [](AsyncWebServerRequest *request) {
    toggleManual();
    request->send(200, "text/plain", "OK");
  });



  server.begin();
  Serial.println("Servidor web iniciado (puerto 80).");
}

void setup() {
  Serial.begin(115200);
  lastPumpChangeMs = millis() - MIN_OFF_MS;  // permite encender inmediatamente si corresponde


  pinMode(PIN_BOMBA, OUTPUT);
  digitalWrite(PIN_BOMBA, LOW);
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
    // AUTO -> MANUAL (siempre riego ON)
    modoManual = true;
    regando = true;
    lastPumpChangeMs = millis();
    Serial.println(">> TOGGLE: AUTO -> MANUAL (RIEGO ON)");
    return;
  }

  // MANUAL -> AUTO (riego depende del control)
  modoManual = false;
  lastPumpChangeMs = millis(); // marca transición

  // Forzamos una evaluación inmediata (sin esperar al próximo ciclo)
  // Usamos la misma lógica que en sampleAndControl, pero aplicada ya.
  bool desired = regando;

  if (regando) {
    if (humedadPct > (umbralPct + H)) desired = false;
  } else {
    if (humedadPct < (umbralPct - H)) desired = true;
  }

  // Al volver a AUTO, queremos que "dependa del sensor" ya mismo.
  // Para cumplir tu expectativa, aplicamos el cambio inmediato (sin MIN_ON/OFF en esta transición).
  if (desired != regando) {
    regando = desired;
    lastPumpChangeMs = millis();
  }

  Serial.println(">> TOGGLE: MANUAL -> AUTO (RIEGO segun sensor)");
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

void sampleAndControl() {
  humedadPct = map(analogRead(PIN_HUMEDAD), 0, 4095, 0, 100);

    if (!modoManual) {
    bool desired = regando;

    // Histéresis: si está regando, no corta hasta superar umbral+H
    if (regando) {
      if (humedadPct > (umbralPct + H)) desired = false;
    } else {
      // Si está apagado, no enciende hasta bajar de umbral-H
      if (humedadPct < (umbralPct - H)) desired = true;
    }

    // Anti-ciclo: tiempo mínimo ON/OFF
    unsigned long now = millis();
    if (desired != regando) {
      if (regando) {
        // quiere pasar de ON->OFF
        if (now - lastPumpChangeMs >= MIN_ON_MS) {
          regando = desired;
          lastPumpChangeMs = now;
        }
      } else {
        // quiere pasar de OFF->ON
        if (now - lastPumpChangeMs >= MIN_OFF_MS) {
          regando = desired;
          lastPumpChangeMs = now;
        }
      }
    }
  }


  digitalWrite(PIN_BOMBA, regando ? HIGH : LOW);

  Serial.print("Hum=");
  Serial.print(humedadPct);
  Serial.print("% Umbral=");
  Serial.print(umbralPct);
  Serial.print("% | Modo=");
  Serial.print(modoManual ? "MAN" : "AUTO");
  Serial.print(" | Bomba=");
  Serial.println(regando ? "ON" : "OFF");
}

void loop() {
  handleButton();

  updateNtpStatus();

  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    sampleAndControl();
  }
}
