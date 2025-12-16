#include <Wire.h>
#include <U8g2lib.h>

#define PIN_HUMEDAD 32
#define PIN_UMBRAL  35
#define PIN_BOMBA   18
#define PIN_BTN     15

// OLED SSD1306 128x64 I2C (dirección detectada: 0x3C)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ---------- Estado del sistema ----------
bool regando = false;
bool modoManual = false;

// ---------- Botón: debounce + edge ----------
bool btnStable = HIGH;
bool btnRawPrev = HIGH;
unsigned long btnLastChangeMs = 0;
const unsigned long DEBOUNCE_MS = 35;

// ---------- Tareas periódicas ----------
unsigned long lastSampleMs = 0;
const unsigned long SAMPLE_PERIOD_MS = 200;

unsigned long lastUiMs = 0;
const unsigned long UI_PERIOD_MS = 200;

// Variables “últimas medidas” para mostrar en pantalla
int humedadPct = 0;
int umbralPct  = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_BOMBA, OUTPUT);
  digitalWrite(PIN_BOMBA, LOW);

  pinMode(PIN_BTN, INPUT_PULLUP);

  // I2C (pines estándar ESP32)
  Wire.begin(21, 22);

  // OLED
  oled.begin();
  oled.setI2CAddress(0x3C << 1); // U8g2 usa dirección 8-bit (0x3C*2)
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 12, "PASO 10 - OLED OK");
  oled.drawStr(0, 28, "Iniciando...");
  oled.sendBuffer();

  Serial.println("PASO 10 - OLED + logica no bloqueante");
}

void handleButton() {
  bool raw = digitalRead(PIN_BTN);

  if (raw != btnRawPrev) {
    btnRawPrev = raw;
    btnLastChangeMs = millis();
  }

  if (millis() - btnLastChangeMs >= DEBOUNCE_MS) {
    if (btnStable == HIGH && raw == LOW) {
      if (!regando) {
        regando = true;
        modoManual = true;
        Serial.println(">> MANUAL ON");
      } else {
        regando = false;
        modoManual = false;
        Serial.println(">> RIEGO OFF");
      }
    }
    btnStable = raw;
  }
}

void sampleAndControl() {
  humedadPct = map(analogRead(PIN_HUMEDAD), 0, 4095, 0, 100);
  umbralPct  = map(analogRead(PIN_UMBRAL),  0, 4095, 0, 100);

  if (!modoManual) {
    regando = (humedadPct < umbralPct);
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

void updateUI() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);

  oled.drawStr(0, 12, "RIEGO ESP32 (Paso 10)");

  char l2[24];
  snprintf(l2, sizeof(l2), "Hum: %3d%%", humedadPct);
  oled.drawStr(0, 30, l2);

  char l3[24];
  snprintf(l3, sizeof(l3), "Umbr:%3d%%", umbralPct);
  oled.drawStr(0, 42, l3);

  char l4[24];
  snprintf(l4, sizeof(l4), "Modo:%s  %s",
           modoManual ? "MAN" : "AUTO",
           regando ? "ON" : "OFF");
  oled.drawStr(0, 58, l4);

  oled.sendBuffer();
}

void loop() {
  handleButton();

  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    sampleAndControl();
  }

  if (now - lastUiMs >= UI_PERIOD_MS) {
    lastUiMs = now;
    updateUI();
  }
}
