// Smart Child Alert System (Optimized Version)
#define BLYNK_TEMPLATE_ID "TMPL6myAfjq7z"
#define BLYNK_TEMPLATE_NAME "New Template"
#define BLYNK_AUTH_TOKEN "1sUQgErcN7LelUNk2Fs1_KE0DZhBOEaZ"

#include "driver/ledc.h"
#define COUNTDOWN_VPIN V7
#define LOG_ADDRESS "http://192.168.137.198:5000/upload"
#define PRESSURE_THRESHOLD 100

#define LEDC_PIN 13
#define LEDC_RESOLUTION 10
#define BEEP_FREQ 4000
#define CHANNEL 0

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>
#include <HTTPClient.h>

void buzzerOn();
void buzzerOff();

char ssid[] = "iPhoneHotspot";
char pass[] = "12345678";
#define PRESSURE_PIN 32

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// System state and control flags
int state = 0;
unsigned long lastCheckTime = 0;
unsigned long lastGPSCompareTime = 0;
unsigned long alertSentTime = 0;
unsigned long lastBlynkUpdate = 0;
unsigned long delayStartTime = 0;
unsigned long continueStartTime = 0;
unsigned long lastWiFiCheck = 0;
const unsigned long wifiCheckInterval = 10000;

float lastLat = 0.0, lastLng = 0.0;
bool alertActive = false;
bool alertHandled = false;
bool waitDelay = false;
bool continueMonitor = false;
bool secondAlertSent = false;
int delayMinutes = 0;
bool gpsPausedPrinted = false;
bool gpsOutputEnabled = true;
bool systemShutdown = false;
bool gpsStartedEventSent = false;
bool childWarningTriggered = false;

// Haversine formula to calculate distance in meters
float calculateDistance(float lat1, float lng1, float lat2, float lng2) {
  const float R = 6371000;
  float dLat = radians(lat2 - lat1);
  float dLng = radians(lng2 - lng1);
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLng / 2) * sin(dLng / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// Determine if the current location is within a certain distance
bool isSameLocation(float lat1, float lng1, float lat2, float lng2, float tol = 50.0) {
  float d = calculateDistance(lat1, lng1, lat2, lng2);
  Serial.print("📏 Distance difference: ");
  Serial.print(d);
  Serial.println(" m");
  return d <= tol;
}

// Convert WiFi RSSI to signal strength percentage
int rssiToPercentage(int rssi) {
  const int RSSI_MIN = -100;
  const int RSSI_MAX = -50;
  if (rssi <= RSSI_MIN) return 0;
  if (rssi >= RSSI_MAX) return 100;
  float percentage = 100.0 * (rssi - RSSI_MIN) / (RSSI_MAX - RSSI_MIN);
  return round(percentage);
}

// --- Blynk Virtual Pin Event Handlers ---

// V0: Immediate Response
BLYNK_WRITE(V0) {
  if (!childWarningTriggered) {
    Serial.println("❌ Action denied: Alert not triggered yet.");
    return;
  }
  if (param.asInt() == 1 && alertActive) {
    alertHandled = true;
    alertActive = false;
    buzzerOff();
    childWarningTriggered = false;
    Blynk.setProperty(V0, "disabled", true);
    Blynk.setProperty(V5, "disabled", true);
    Blynk.setProperty(V4, "disabled", true);
    Blynk.setProperty(V0, "color", "#A9A9A9");
    Blynk.setProperty(V5, "color", "#A9A9A9");
    Blynk.setProperty(V4, "color", "#A9A9A9");
    Serial.println("✅ Parents have responded, ending the alert");
  }
}

// V4: Short Stay Response
BLYNK_WRITE(V4) {
  delayMinutes = param.asInt();
  if (delayMinutes > 0) {
    waitDelay = true;
    delayStartTime = millis();
    alertHandled = true;
    alertActive = false;
    buzzerOff();
    Serial.println("⏳ Parents chose to stay for a short time " + String(delayMinutes) + " minutes, pause detection");
  }
}

// V5: Handle Immediately Response
BLYNK_WRITE(V5) {
  if (!childWarningTriggered) {
    Serial.println("❌ Action denied: Alert not triggered yet.");
    return;
  }
  continueMonitor = true;
  continueStartTime = millis();
  secondAlertSent = false;
  alertHandled = false;
  alertActive = false;
  buzzerOff();
  Serial.println("⏱️ Parents chose to deal with it immediately and re-test after 20s");
}

// V8: System Shutdown Toggle
BLYNK_WRITE(V8) {
  Blynk.setProperty(V8, "color", systemShutdown ? "#FF0000" : "#64C466");
  systemShutdown = param.asInt() == 1;
  Serial.println(systemShutdown ? "🛑 System shutdown initiated" : "✅ System resumed");

  if (!systemShutdown) {
    Blynk.virtualWrite(COUNTDOWN_VPIN, "00:00");
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Wi-Fi connection is successful");

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  configTime(8 * 3600, 0, "pool.ntp.org");

  IPAddress ip;
  WiFi.hostByName("blynk.cloud", ip);
  Blynk.config(BLYNK_AUTH_TOKEN, ip.toString().c_str(), 80);
  Blynk.connect();

  gpsSerial.begin(9600, SERIAL_8N1, 34, 12);
  pinMode(PRESSURE_PIN, INPUT_PULLUP);
  buzzerOff();

  Blynk.setProperty(V0, "disabled", true);
  Blynk.setProperty(V5, "disabled", true);
  Blynk.setProperty(V4, "disabled", true);
  Blynk.setProperty(V0, "color", "#A9A9A9");
  Blynk.setProperty(V5, "color", "#A9A9A9");
  Blynk.setProperty(V4, "color", "#A9A9A9");

  ledcAttach(LEDC_PIN, BEEP_FREQ, LEDC_RESOLUTION);
}

void loop() {
  Blynk.run();

  // Update WiFi signal strength
  if (millis() - lastWiFiCheck >= wifiCheckInterval) {
    lastWiFiCheck = millis();
    int percent = WiFi.status() == WL_CONNECTED ? rssiToPercentage(WiFi.RSSI()) : 0;
    Blynk.virtualWrite(V9, percent);
  }

  // Shutdown condition
  if (systemShutdown) {
    Blynk.virtualWrite(COUNTDOWN_VPIN, "SHUTDOWN");
    buzzerOff();
    return;
  }

  // Reset countdown if system is active
  if (!systemShutdown && !waitDelay && !continueMonitor && !alertActive) {
    Blynk.virtualWrite(COUNTDOWN_VPIN, "00:00");
  }

  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  if (!gpsStartedEventSent && gps.location.isValid()) {
    Blynk.logEvent("gps_started", "📡 GPS is connected and receiving data");
    gpsStartedEventSent = true;
  }

  unsigned long now = millis();

  if (now - lastBlynkUpdate > 5000) {
    updateBlynkData();
    lastBlynkUpdate = now;
  }

  if (waitDelay && now - delayStartTime < delayMinutes * 60000UL) {
    int secondsLeft = (delayMinutes * 60000UL - (now - delayStartTime)) / 1000;
    char countdown[10];
    sprintf(countdown, "%02d:%02d", secondsLeft / 60, secondsLeft % 60);
    Blynk.virtualWrite(COUNTDOWN_VPIN, countdown);
  }

  if (waitDelay && now - delayStartTime >= delayMinutes * 60000UL) {
    waitDelay = false;
    state = 0;
    Blynk.virtualWrite(COUNTDOWN_VPIN, "00:00");
  }

  if (continueMonitor && now - continueStartTime < 2 * 60000UL) {
    unsigned long timeLeftMs = 2 * 60000UL - (now - continueStartTime);
    int secondsLeft = timeLeftMs / 1000;
    int minutes = secondsLeft / 60;
    int seconds = secondsLeft % 60;
    char countdown[10];
    sprintf(countdown, "%02d:%02d", minutes, seconds);
    Blynk.virtualWrite(COUNTDOWN_VPIN, countdown);
  }

  if (continueMonitor && now - continueStartTime >= 2 * 60000UL) {
    if (isSomeoneSeated() && !secondAlertSent) {
      Blynk.logEvent("child_warning", "🚨 The child hasn't been removed, so check it out now!");
      alertActive = true;
      alertSentTime = now;
      secondAlertSent = true;
      childWarningTriggered = true;
      Blynk.setProperty(V0, "disabled", false);
      Blynk.setProperty(V5, "disabled", false);
      Blynk.setProperty(V4, "disabled", false);
      Blynk.setProperty(V0, "color", "#64C466");
      Blynk.setProperty(V5, "color", "#64C466");
      Blynk.setProperty(V4, "color", "#64C466");
      sendAlertToServer(2);
    }
    continueMonitor = false;
  }

  if (alertActive && !alertHandled && now - alertSentTime > 20000) buzzerOn();

  if (alertActive && !waitDelay && !continueMonitor && !alertHandled && now - alertSentTime > 120000) {
    buzzerOn();
    alertActive = false;
    childWarningTriggered = false;
    Blynk.setProperty(V0, "disabled", true);
    Blynk.setProperty(V5, "disabled", true);
    Blynk.setProperty(V4, "disabled", true);
    Blynk.setProperty(V0, "color", "#A9A9A9");
    Blynk.setProperty(V5, "color", "#A9A9A9");
    Blynk.setProperty(V4, "color", "#A9A9A9");
  }

  if (waitDelay || continueMonitor || alertActive) {
    if (!gpsPausedPrinted) {
      Serial.println("⏸️ Paused waiting for parent response...");
      gpsPausedPrinted = true;
    }
  } else {
    gpsPausedPrinted = false;
    handleGPSStateMachine();
  }
}

// Handle GPS logic based on movement and child presence
void handleGPSStateMachine() {
  unsigned long now = millis();

  if (state == 0 && now - lastCheckTime > 3000 && gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
    if (gpsOutputEnabled)
      Serial.println("📍[Initial Position]" + String(lastLat, 6) + "," + String(lastLng, 6));
    lastCheckTime = now;
    state = 1;
  } else if (state == 1 && now - lastCheckTime > 30000 && gps.location.isValid()) {
    float newLat = gps.location.lat();
    float newLng = gps.location.lng();
    if (gpsOutputEnabled)
      Serial.println("📍[30s Later Position]" + String(newLat, 6) + "," + String(newLng, 6));

    if (!isSameLocation(lastLat, lastLng, newLat, newLng)) {
      Serial.println("Vehicle is moving");
      gpsOutputEnabled = true;
      state = 0;
    } else {
      if (gpsOutputEnabled)
        Serial.println("Location stable, wait another 1 minute...");
      lastGPSCompareTime = now;
      state = 2;
    }

  } else if (state == 2 && now - lastGPSCompareTime > 60000 && gps.location.isValid()) {
    float newLat = gps.location.lat();
    float newLng = gps.location.lng();
    if (gpsOutputEnabled)
      Serial.println("📍[1min Later Position]" + String(newLat, 6) + "," + String(newLng, 6));

    if (!isSameLocation(lastLat, lastLng, newLat, newLng)) {
      Serial.println("📍 Vehicle started moving again, resetting process");
      gpsOutputEnabled = true;
      state = 0;
    } else {
      bool pressure = isSomeoneSeated();
      if (pressure) {
        Blynk.logEvent("child_warning", "⚠️ Child is still in seat and vehicle not moving! Check immediately.");
        alertActive = true;
        alertHandled = false;
        alertSentTime = now;
        childWarningTriggered = true;
        Blynk.setProperty(V0, "disabled", false);
        Blynk.setProperty(V5, "disabled", false);
        Blynk.setProperty(V0, "color", "#64C466");
        Blynk.setProperty(V5, "color", "#64C466");
        sendAlertToServer(1);
      } else {
        if (gpsOutputEnabled)
          Serial.println("✅ No one in seat, ending monitoring");
        gpsOutputEnabled = false;

        state = 0;
        Blynk.setProperty(V0, "disabled", true);
        Blynk.setProperty(V5, "disabled", true);
        Blynk.setProperty(V4, "disabled", true);
        Blynk.setProperty(V0, "color", "#A9A9A9");
        Blynk.setProperty(V5, "color", "#A9A9A9");
        Blynk.setProperty(V4, "color", "#A9A9A9");
      }
      state = 0;
    }
  }
}

// Update data to Blynk dashboard
void updateBlynkData() {
  bool pressure = isSomeoneSeated();
  Blynk.virtualWrite(V1, pressure ? "Child in seat" : "No Child");
  Blynk.virtualWrite(V6, pressure ? "1" : "0");

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Blynk.virtualWrite(V2, buffer);
  } else {
    Blynk.virtualWrite(V2, "Time acquisition failed");
  }

  if (gps.location.isValid()) {
    String gpsData = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    String gmapsLink = "https://maps.google.com/?q=" + gpsData;
    Blynk.setProperty(V11, "url", gmapsLink);
    Blynk.virtualWrite(V3, gpsData);
  } else {
    Blynk.virtualWrite(V3, "No Coordinates");
  }
}

// Read pressure sensor value
bool isSomeoneSeated() {
  int pressureValue = analogRead(PRESSURE_PIN);
  return pressureValue > PRESSURE_THRESHOLD;
}

// Turn buzzer on
void buzzerOn() {
  for (int i = 0; i < 3; i++) {
    ledcWrite(LEDC_PIN, 512);
    delay(300);
    ledcWrite(LEDC_PIN, 0);
    delay(200);
  }
  delay(1500);
}

// Turn buzzer off
void buzzerOff() {
  ledcWrite(LEDC_PIN, 0);
}

// Send alert log to server
void sendAlertToServer(int alertCount) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    Serial.println("[ESP32] Connecting to: http://192.168.137.198:5000/upload");
    http.begin("http://192.168.137.198:5000/upload");
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    String postData = "device=seat001&count=" + String(alertCount);

    Serial.print("[ESP32] POST data: ");
    Serial.println(postData);

    int httpResponseCode = http.POST(postData);
    Serial.print("[ESP32] HTTP Response Code: ");
    Serial.println(httpResponseCode);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.print("[ESP32] Server Response: ");
      Serial.println(response);
    } else {
      Serial.print("❌ [ESP32] Upload failed, error: ");
      Serial.println(http.errorToString(httpResponseCode));
    }

    http.end();
  } else {
    Serial.println("❌ WiFi not connected");
  }
}
