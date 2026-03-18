/**
 * Radar.ino — MR60BHA2 + trigger raspEYE
 * Otimizado para XIAO ESP32-C6 (flash + RAM reduzidos)
 *
 * Reduções aplicadas:
 *  1. F() em todos os Serial.print   → strings vão para flash (~950 bytes RAM livres)
 *  2. StaticJsonDocument             → sem fragmentação de heap
 *  3. Payload mínimo para Raspberry  → JsonDoc 256b em vez de 3072b
 *  4. doPost() genérico              → elimina código duplicado entre API e Sheets
 *  5. PROGMEM para URLs e credenciais
 */

#include <Arduino.h>
#include "Seeed_Arduino_mmWave.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#ifdef ESP32
#  include <HardwareSerial.h>
HardwareSerial mmWaveSerial(0);
#else
#  define mmWaveSerial Serial1
#endif

SEEED_MR60BHA2 mmWave;

// Strings em PROGMEM (flash) — não ocupam RAM até serem copiadas
static const char SSID[]       PROGMEM = "gm-fast";
static const char PASS[]       PROGMEM = "@newapp2020";
static const char API_URL[]    PROGMEM = "https://device-gateway-dispositivos-165talig.ue.gateway.dev/enviar-dados";
static const char API_KEY[]    PROGMEM = "AIzaSyDi_I-6_3bIbiy1NbrnFZzUmnSP3d6MSro";
static const char SERIAL_NUM[] PROGMEM = "Beluga_BoaViagem";
static const char SHEETS_URL[] PROGMEM = "https://script.google.com/macros/s/AKfycbySi10ZmkMgAeh8GtWMc1SkVwRDS1pvRgeiNT7aaQRYB7UaADc_etAlatUT9k4rI13a/exec";
static const char RASP_URL[]   PROGMEM = "http://192.168.1.100:5001/trigger"; // <- ajuste o IP

#define HTTP_CONNECT_TIMEOUT_MS  10000
#define HTTP_READ_TIMEOUT_MS     30000
#define SEND_INTERVAL_MS         4000
#define HTTP_RETRY_COUNT         2
#define HTTP_RETRY_DELAY_MS      2000
#define RADAR_RESET_PIN          D3
#define RADAR_RESET_INTERVAL_MS  3600000UL

// Struct reordenada: floats juntos, ints juntos, bools no final
// Elimina padding implícito do compilador
typedef struct {
  float   targets_x[10];
  float   targets_y[10];
  float   targets_speed[10];
  float   range_step;
  float   total_phase;
  float   breath_phase;
  float   heart_phase;
  float   breath_rate;
  float   heart_rate;
  float   distance;
  int     targets_dop[10];
  int     targets_cluster[10];
  uint8_t num_targets;
  bool    human_detected;
  bool    vital_signs_available;
} radar_data_t;

radar_data_t radarData;
bool wifiConnected = false;
unsigned long totalMsSinceLastRadarReset = 0;

static inline float safeF(float v) {
  return (isnan(v) || isinf(v)) ? 0.0f : v;
}

void resetarRadarFisico() {
  Serial.println(F("Resetando radar..."));
  digitalWrite(RADAR_RESET_PIN, LOW); delay(100);
  digitalWrite(RADAR_RESET_PIN, HIGH); delay(500);
  Serial.println(F("Radar OK."));
}

void measureVitalSigns() {
  radarData.vital_signs_available = false;
  float v, tp, bp, hp;
  if (mmWave.getHeartBreathPhases(tp, bp, hp)) {
    radarData.total_phase = tp;
    radarData.breath_phase = bp;
    radarData.heart_phase = hp;
  }
  if (mmWave.getBreathRate(v)) radarData.breath_rate = v;
  if (mmWave.getHeartRate(v))  radarData.heart_rate  = v;
  if (mmWave.getDistance(v))   radarData.distance    = v;
  radarData.vital_signs_available = true;
}

void conectarWiFi() {
  char ssid_buf[32], pass_buf[64];
  strncpy_P(ssid_buf, SSID, sizeof(ssid_buf));
  strncpy_P(pass_buf, PASS, sizeof(pass_buf));
  Serial.print(F("WiFi "));
  Serial.print(ssid_buf);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid_buf, pass_buf);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) {
    delay(500); Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.print(F("IP: ")); Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("Falha WiFi. Reiniciando..."));
    delay(3000); ESP.restart();
  }
}

void quickReconnect() {
  wifiConnected = false;
  WiFi.disconnect(false); delay(300);
  WiFi.reconnect();
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) delay(250);
  wifiConnected = (WiFi.status() == WL_CONNECTED);
}

// JSON completo para API e Sheets (StaticJsonDocument - stack, sem heap)
String buildFullJson(const radar_data_t* d) {
  StaticJsonDocument<2048> doc;
  char sn[32]; strncpy_P(sn, SERIAL_NUM, sizeof(sn));
  doc[F("serialNumber")] = sn;
  JsonObject o = doc.createNestedObject(F("data"));
  o[F("timestamp")]      = millis();
  o[F("human_detected")] = d->human_detected;
  o[F("num_targets")]    = d->num_targets;
  o[F("range_step")]     = safeF(d->range_step);
  uint8_t n = min(d->num_targets, (uint8_t)10);
  JsonArray ta = o.createNestedArray(F("targetData"));
  for (int i = 0; i < n; i++) {
    JsonObject t = ta.createNestedObject();
    t[F("X")]             = safeF(d->targets_x[i]);
    t[F("Y")]             = safeF(d->targets_y[i]);
    t[F("dop_index")]     = (float)d->targets_dop[i];
    t[F("cluster_index")] = d->targets_cluster[i];
    t[F("speed")]         = safeF(d->targets_speed[i]);
  }
  o[F("vital_signs_available")] = d->vital_signs_available;
  if (d->vital_signs_available) {
    o[F("distance")]     = (int)d->distance;
    o[F("breath_rate")]  = (int)safeF(d->breath_rate);
    o[F("heart_rate")]   = (int)safeF(d->heart_rate);
    o[F("total_phase")]  = safeF(d->total_phase);
    o[F("breath_phase")] = safeF(d->breath_phase);
    o[F("heart_phase")]  = safeF(d->heart_phase);
  }
  String out; serializeJson(doc, out); return out;
}

// JSON mínimo para o Raspberry — 256 bytes, 8x menor
String buildMiniJson(const radar_data_t* d) {
  StaticJsonDocument<256> doc;
  JsonObject o = doc.createNestedObject(F("data"));
  o[F("human_detected")]        = d->human_detected;
  o[F("num_targets")]           = d->num_targets;
  o[F("vital_signs_available")] = d->vital_signs_available;
  if (d->vital_signs_available) {
    o[F("breath_rate")] = (int)safeF(d->breath_rate);
    o[F("heart_rate")]  = (int)safeF(d->heart_rate);
    o[F("distance")]    = (int)d->distance;
  }
  String out; serializeJson(doc, out); return out;
}

// POST genérico — substitui sendToAPI e sendToGoogleSheets duplicados
void doPost(const char* urlPROGMEM, const String& body, const char* label,
            bool addApiKey = false) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    quickReconnect();
    if (!wifiConnected) return;
  }
  char url[256]; strncpy_P(url, urlPROGMEM, sizeof(url));
  HTTPClient http;
  http.begin(url);
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  http.setConnectTimeout(HTTP_CONNECT_TIMEOUT_MS);
  http.setTimeout(HTTP_READ_TIMEOUT_MS);
  http.addHeader(F("Content-Type"), F("application/json"));
  if (addApiKey) {
    char key[64]; strncpy_P(key, API_KEY, sizeof(key));
    http.addHeader(F("x-api-key"), key);
  }
  Serial.print('['); Serial.print(label); Serial.print(F("] "));
  int code = http.POST(body), retries = 0;
  while ((code == -1 || code == -11) && retries < HTTP_RETRY_COUNT) {
    http.end();
    if (code == -1) quickReconnect();
    delay(HTTP_RETRY_DELAY_MS);
    http.begin(url);
    http.setConnectTimeout(HTTP_CONNECT_TIMEOUT_MS);
    http.setTimeout(HTTP_READ_TIMEOUT_MS);
    http.addHeader(F("Content-Type"), F("application/json"));
    code = http.POST(body);
    retries++;
  }
  if (code == 200 || code == 201) Serial.println(F("OK"));
  else { Serial.print(F("HTTP ")); Serial.println(code); }
  http.end();
}

void sendToRaspberry(const radar_data_t* data) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) return;
  char url[64]; strncpy_P(url, RASP_URL, sizeof(url));
  HTTPClient http;
  http.begin(url);
  http.setConnectTimeout(3000);
  http.setTimeout(3000);
  http.addHeader(F("Content-Type"), F("application/json"));
  int code = http.POST(buildMiniJson(data));
  Serial.print(F("[RaspEYE] "));
  if (code == 200 || code == 201) Serial.println(F("OK"));
  else { Serial.print(F("HTTP ")); Serial.println(code); }
  http.end();
}

void sendToAPI(const radar_data_t* data) {
  doPost(API_URL, buildFullJson(data), "API", true);
}

void sendToGoogleSheets(const radar_data_t* data) {
  doPost(SHEETS_URL, buildFullJson(data), "Sheets", false);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println(F("\n=== RADAR + RASPEYE ==="));
  pinMode(RADAR_RESET_PIN, OUTPUT);
  digitalWrite(RADAR_RESET_PIN, HIGH);
  mmWave.begin(&mmWaveSerial);
  resetarRadarFisico();
  totalMsSinceLastRadarReset = 0;
  conectarWiFi();
  Serial.println(F("Aguardando radar...\n"));
}

void loop() {
  static unsigned long lastSendTime = 0, lastRadarResetCheckMs = 0, lastWiFiCheck = 0;

  if (mmWave.update(100)) {
    memset(&radarData, 0, sizeof(radar_data_t));
    if (mmWave.isHumanDetected()) {
      radarData.human_detected = true;
      float dist;
      if (mmWave.getDistance(dist) && dist <= 150.0f) measureVitalSigns();
    }
    PeopleCounting target_info;
    if (mmWave.getPeopleCountingTargetInfo(target_info)) {
      radarData.num_targets = (uint8_t)min(target_info.targets.size(), (size_t)10);
      radarData.range_step  = RANGE_STEP;
      for (size_t i = 0; i < radarData.num_targets; i++) {
        const auto& t = target_info.targets[i];
        radarData.targets_x[i]       = t.x_point;
        radarData.targets_y[i]       = t.y_point;
        radarData.targets_dop[i]     = t.dop_index;
        radarData.targets_cluster[i] = t.cluster_index;
        radarData.targets_speed[i]   = t.dop_index * RANGE_STEP;
      }
      unsigned long now = millis();
      if (now - lastSendTime >= SEND_INTERVAL_MS) {
        lastSendTime = now;
        sendToRaspberry(&radarData);
        sendToAPI(&radarData);
        delay(500);
        sendToGoogleSheets(&radarData);
      }
    }
    delay(300);
  }

  unsigned long now = millis();
  totalMsSinceLastRadarReset += (now - lastRadarResetCheckMs);
  lastRadarResetCheckMs = now;
  if (totalMsSinceLastRadarReset >= RADAR_RESET_INTERVAL_MS) {
    resetarRadarFisico();
    totalMsSinceLastRadarReset = 0;
  }
  if (now - lastWiFiCheck > 30000) {
    if (WiFi.status() != WL_CONNECTED) { wifiConnected = false; quickReconnect(); }
    lastWiFiCheck = now;
  }
  yield();
  delay(50);
}
