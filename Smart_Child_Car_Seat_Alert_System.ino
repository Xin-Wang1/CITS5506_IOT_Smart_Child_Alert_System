/**
 * @brief Smart Child Alert System 
 * @authors Xin Wang (24201533), Karla Ivkovic (22918911)
 * @version 1.0 
 * @date 25/05/2025
 */

// --- Blynk and system configuration ---
#define BLYNK_TEMPLATE_ID "TMPL6myAfjq7z"
#define BLYNK_TEMPLATE_NAME "New Template"
#define BLYNK_AUTH_TOKEN "1sUQgErcN7LelUNk2Fs1_KE0DZhBOEaZ"
#define LOG_ADDRESS "http://192.168.137.198:5000/upload"

#define PRESSURE_THRESHOLD 600 // Pressure threshold for detecting if someone >3kg is seated
#define CHANNEL 0
#define LEDC_RESOLUTION 10
#define BEEP_FREQ 4000

// --- GPIO Pin Assignments ---
#define COUNTDOWN_VPIN V7
#define LEDC_PIN 13
#define PRESSURE_PIN 32

// --- Library includes ---
#include "driver/ledc.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>
#include <HTTPClient.h>

void buzzerOn();
void buzzerOff();

char ssid[] = "Your_SSID"; // Replace with your Wi-Fi SSID
char pass[] = "Your_PASSWORD"; // Replace with your Wi-Fi password

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

/**
 * @brief Calculates the distance between two GPS coordinates using the Haversine formula.
 */
float calculateDistance(float lat1, float lng1, float lat2, float lng2) {
  const float R = 6371000;
  float dLat = radians(lat2 - lat1);
  float dLng = radians(lng2 - lng1);
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLng / 2) * sin(dLng / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

/**
 * @brief Compares two GPS positions to determine if the vehicle has moved beyond a tolerance.
 */
bool isSameLocation(float lat1, float lng1, float lat2, float lng2, float tol = 50.0) {
  float d = calculateDistance(lat1, lng1, lat2, lng2);
  Serial.print("📏 Distance difference: ");
  Serial.print(d);
  Serial.println(" m");
  return d <= tol;
}

/**
 * @brief Converts Wi-Fi RSSI values to a percentage strength for display on Blynk.
 */
int rssiToPercentage(int rssi) {
  const int RSSI_MIN = -100;
  const int RSSI_MAX = -50;
  if (rssi <= RSSI_MIN) return 0;
  if (rssi >= RSSI_MAX) return 100;
  float percentage = 100.0 * (rssi - RSSI_MIN) / (RSSI_MAX - RSSI_MIN);
  return round(percentage);
}

// --- Blynk Virtual Pin Event Handlers ---

/**
 * @brief Custom delay input by parent (V4)
 */
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

/**
 * @brief 2-minute delay chosen by parent to re-check presence (V5)
 */
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

/**
 * @brief Toggles system shutdown via Blynk interface (V8)
 */
BLYNK_WRITE(V8) {
  Blynk.setProperty(V8, "color", systemShutdown ? "#FF0000" : "#64C466");
  systemShutdown = param.asInt() == 1;
  Serial.println(systemShutdown ? "🛑 System shutdown initiated" : "✅ System resumed");

  if (!systemShutdown) {
    Blynk.virtualWrite(COUNTDOWN_VPIN, "00:00");
  }
}

/**
 * @brief Setup function for initializing system hardware, Wi-Fi, Blynk, GPS, and I/O pins.
 */
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

  Blynk.setProperty(V5, "disabled", true);
  Blynk.setProperty(V4, "disabled", true);
  Blynk.setProperty(V5, "color", "#A9A9A9");
  Blynk.setProperty(V4, "color", "#A9A9A9");

  ledcAttach(LEDC_PIN, BEEP_FREQ, LEDC_RESOLUTION);
}

/**
 * @brief Main loop handling system state, GPS updates, alert triggering, and UI feedback.
 */
void loop() {
  Blynk.run();

  // --- Update Wi-Fi signal strength every 10 seconds ---
  if (millis() - lastWiFiCheck >= wifiCheckInterval) {
    lastWiFiCheck = millis();
    int percent = WiFi.status() == WL_CONNECTED ? rssiToPercentage(WiFi.RSSI()) : 0;
    Blynk.virtualWrite(V9, percent);
  }

  // --- Handle system shutdown state ---
  if (systemShutdown) {
    Blynk.virtualWrite(COUNTDOWN_VPIN, "SHUTDOWN");
    buzzerOff();
    return;
  }

  // --- Reset countdown if system is idle ---
  if (!systemShutdown && !waitDelay && !continueMonitor && !alertActive) {
    Blynk.virtualWrite(COUNTDOWN_VPIN, "00:00");
  }

  // --- Read GPS data if available ---
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  if (!gpsStartedEventSent && gps.location.isValid()) {
    Blynk.logEvent("gps_started", "📡 GPS is connected and receiving data");
    gpsStartedEventSent = true;
  }

  // --- Update Blynk dashboard every 5 seconds ---
  unsigned long now = millis();
  if (now - lastBlynkUpdate > 5000) {
    updateBlynkData();
    lastBlynkUpdate = now;
  }

  // --- Handle custom delay timer (V4) ---
  if (waitDelay && now - delayStartTime < delayMinutes * 60000UL) {
    int secondsLeft = (delayMinutes * 60000UL - (now - delayStartTime)) / 1000;
    char countdown[10];
    sprintf(countdown, "%02d:%02d", secondsLeft / 60, secondsLeft % 60);
    Blynk.virtualWrite(COUNTDOWN_VPIN, countdown);
  }
  // --- End custom delay (V4) and reset state ---
  if (waitDelay && now - delayStartTime >= delayMinutes * 60000UL) {
    waitDelay = false;
    state = 0;
    Blynk.virtualWrite(COUNTDOWN_VPIN, "00:00");
  }

  // --- Handle 2-minute monitoring delay (V5) ---
  if (continueMonitor && now - continueStartTime < 2 * 60000UL) {
    unsigned long timeLeftMs = 2 * 60000UL - (now - continueStartTime);
    int secondsLeft = timeLeftMs / 1000;
    int minutes = secondsLeft / 60;
    int seconds = secondsLeft % 60;
    char countdown[10];
    sprintf(countdown, "%02d:%02d", minutes, seconds);
    Blynk.virtualWrite(COUNTDOWN_VPIN, countdown);
  }

  // --- After 2-minute delay, check seat again and possibly trigger 2nd alert ---
  if (continueMonitor && now - continueStartTime >= 2 * 60000UL) {
    if (isSomeoneSeated() && !secondAlertSent) {
      Blynk.logEvent("child_warning", "🚨 The child hasn't been removed, so check it out now!");
      alertActive = true;
      alertSentTime = now;
      secondAlertSent = true;
      childWarningTriggered = true;
      Blynk.setProperty(V5, "disabled", false);
      Blynk.setProperty(V4, "disabled", false);
      Blynk.setProperty(V5, "color", "#64C466");
      Blynk.setProperty(V4, "color", "#64C466");
      sendAlertToServer(2);
    }
    continueMonitor = false;
  }

  // --- Activate buzzer if alert is active and parent hasn’t responded after 20s ---
  if (alertActive && !alertHandled && now - alertSentTime > 20000) buzzerOn();

  // --- End alert cycle after 2 minutes if still unacknowledged ---
  if (alertActive && !waitDelay && !continueMonitor && !alertHandled && now - alertSentTime > 120000) {
    buzzerOn();
    alertActive = false;
    childWarningTriggered = false;
    Blynk.setProperty(V5, "disabled", true);
    Blynk.setProperty(V4, "disabled", true);
    Blynk.setProperty(V5, "color", "#A9A9A9");
    Blynk.setProperty(V4, "color", "#A9A9A9");
  }

  // --- Print pause message if alert system is waiting for parent input --- 
  if (waitDelay || continueMonitor || alertActive) {
    if (!gpsPausedPrinted) {
      Serial.println("⏸️ Paused waiting for parent response...");
      gpsPausedPrinted = true;
    }
  } else {
    gpsPausedPrinted = false;
    handleGPSStateMachine(); // Continue checking location and seat status
  }
}

/**
 * @brief Handles GPS monitoring logic and determines whether the vehicle is stationary and a child is present.
 */
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
        Blynk.setProperty(V5, "disabled", false);
        Blynk.setProperty(V5, "color", "#64C466");
        sendAlertToServer(1);
      } else {
        if (gpsOutputEnabled)
          Serial.println("✅ No one in seat, ending monitoring");
        gpsOutputEnabled = false;

        state = 0;
        Blynk.setProperty(V5, "disabled", true);
        Blynk.setProperty(V4, "disabled", true);
        Blynk.setProperty(V5, "color", "#A9A9A9");
        Blynk.setProperty(V4, "color", "#A9A9A9");
      }
      state = 0;
    }
  }
}


/**
 * @brief Updates Blynk dashboard with pressure sensor data, system time, and GPS location.
 */
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

/**
 * @brief Reads the pressure sensor and returns true if someone is seated.
 */
bool isSomeoneSeated() {
  int pressureValue = analogRead(PRESSURE_PIN);
  return pressureValue > PRESSURE_THRESHOLD;
}

/**
 * @brief Activates the onboard buzzer to alert bystanders.
 */
void buzzerOn() {
  for (int i = 0; i < 3; i++) {
    ledcWrite(LEDC_PIN, 512);
    delay(300);
    ledcWrite(LEDC_PIN, 0);
    delay(200);
  }
  delay(1500);
}

/**
 * @brief Deactivates the onboard buzzer.
 */
void buzzerOff() {
  ledcWrite(LEDC_PIN, 0);
}

/**
 * @brief Sends alert data to a local logging server over HTTP POST.
 * @param alertCount The number of alerts sent (used to distinguish alert stages).
 */
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
