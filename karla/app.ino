#define BLYNK_TEMPLATE_ID "TMPL60SNIol4o"
#define BLYNK_TEMPLATE_NAME "Smart Child Seat"
#define BLYNK_AUTH_TOKEN "vxP7O6nUsG0DlduR_U9FFyliYMKkhKc3"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <HTTPClient.h>
#include <time.h>
#include <math.h>
#include <attention_audio.h>

char ssid[] = "iPhone (56) karla";
char pass[] = "karlaiscool";

// Pins
#define PRESSURE_PIN 13
#define BUZZER_PIN 25
#define SAMPLE_RATE 8000

#define COUNTDOWN_VPIN V7
#define LOG_ADDRESS "http://192.168.137.198:5000/upload"

// State
float lastLat = 0.0, lastLon = 0.0;
float currLat = 0.0, currLon = 0.0;

bool alertActive = false;
bool alertHandled = false;
bool waitDelay = false;
bool continueMonitor = false;
bool secondAlertSent = false;

int delayMinutes = 0;
unsigned long delayStartTime = 0;
unsigned long continueStartTime = 0;
unsigned long alertSentTime = 0;

unsigned long lastGPSCheck = 0;

int state = 0;

// ───────────── UTILITIES ─────────────

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

bool isSameLocation(float lat1, float lon1, float lat2, float lon2, float tol = 50.0) {
  float d = calculateDistance(lat1, lon1, lat2, lon2);
  Serial.print("📏 Distance: ");
  Serial.print(d);
  Serial.println(" meters");
  return d <= tol;
}

void soundAlarmPattern() {
 for (int i = 0; i < attentionAudioLength; i++) {
    uint8_t sample = pgm_read_byte(&attentionAudio[i]);  // read from flash
    dacWrite(BUZZER_PIN, sample);
    delayMicroseconds(1000000 / SAMPLE_RATE);
  }
}


// ───────────── BLYNK CONTROL ─────────────

BLYNK_WRITE(V0) {
  if (param.asInt() == 1 && alertActive) {
    alertHandled = true;
    alertActive = false;
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("✅ Parents handled the alert");
  }
}

BLYNK_WRITE(V4) {
  delayMinutes = param.asInt();
  if (delayMinutes > 0) {
    waitDelay = true;
    delayStartTime = millis();
    alertHandled = true;
    alertActive = false;
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("⏳ Paused for " + String(delayMinutes) + " min");
  }
}

BLYNK_WRITE(V5) {
  continueMonitor = true;
  continueStartTime = millis();
  secondAlertSent = false;
  alertHandled = false;
  alertActive = false;
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("⏱️ Rechecking in 2 minutes");
}

BLYNK_WRITE(V6) {
  alertHandled = false;
  alertActive = false;
  waitDelay = false;
  continueMonitor = false;
  secondAlertSent = false;
  state = 0;
  digitalWrite(BUZZER_PIN, LOW);
  Blynk.virtualWrite(COUNTDOWN_VPIN, "00:00");
  Serial.println("🔁 Manual reset");
}

BLYNK_WRITE(V1) {
  currLat = param.asFloat();
}

BLYNK_WRITE(V2) {
  currLon = param.asFloat();
  handleGPSData();  // Trigger logic after both lat/lon arrive
}

// ───────────── LOGIC ─────────────

void handleGPSData() {
  if (waitDelay || continueMonitor || alertActive) return;

  unsigned long now = millis();
  if (state == 0) {
    lastLat = currLat;
    lastLon = currLon;
    Serial.println("📍 First GPS recorded.");
    state = 1;
    lastGPSCheck = now;
  } 
  else if (state == 1 && now - lastGPSCheck >= 120000) {  // 2 minutes later
    if (!isSameLocation(lastLat, lastLon, currLat, currLon)) {
      Serial.println("✅ Vehicle moved — all good");
      state = 0;
    } else {
        bool pressure = true;
//      bool pressure = (digitalRead(PRESSURE_PIN) == LOW);
      if (pressure) {
        Serial.println("🚨 No movement + pressure → ALERT!");
        Blynk.logEvent("child_warning", "⚠️ Child still in seat — vehicle not moving!");
        soundAlarmPattern();
        alertActive = true;
        alertHandled = false;
        alertSentTime = now;
        sendAlertToServer(1);
      } else {
        Serial.println("✅ No movement but seat is empty");
      }
      state = 0;
    }
  }
}

void sendAlertToServer(int alertCount) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(LOG_ADDRESS);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    String gpsData = String(currLat, 6) + "," + String(currLon, 6);
    String postData = "device=seat001&count=" + String(alertCount) + "&gps=" + gpsData;

    Serial.print("📤 Sending alert: ");
    Serial.println(postData);

    int code = http.POST(postData);
    Serial.println("📡 Response: " + String(code));
    http.end();
  } else {
    Serial.println("❌ WiFi not connected");
  }
}

// ───────────── SETUP + LOOP ─────────────

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected");

  configTime(8 * 3600, 0, "pool.ntp.org");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  pinMode(PRESSURE_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Request GPS on boot
  Blynk.virtualWrite(V10, 1);
}

void loop() {
  Blynk.run();

  unsigned long now = millis();

  // Delay countdown display
  if (waitDelay && now - delayStartTime < delayMinutes * 60000UL) {
    unsigned long left = delayMinutes * 60000UL - (now - delayStartTime);
    int secondsLeft = left / 1000;
    int minutes = secondsLeft / 60;
    int seconds = secondsLeft % 60;

    char countdown[10];
    sprintf(countdown, "%02d:%02d", minutes, seconds);
    Blynk.virtualWrite(COUNTDOWN_VPIN, countdown);
  }

  // Resume after delay
  if (waitDelay && now - delayStartTime >= delayMinutes * 60000UL) {
    waitDelay = false;
    state = 0;
    Blynk.virtualWrite(COUNTDOWN_VPIN, "00:00");
    Serial.println("⏱️ Delay over — resuming monitoring");
  }

  // Recheck after 2 minutes
  if (continueMonitor && now - continueStartTime >= 2 * 60000UL) {
    bool pressure = true;
    //bool pressure = (digitalRead(PRESSURE_PIN) == LOW);
    if (pressure && !secondAlertSent) {
      Blynk.logEvent("child_warning", "🚨 Child still detected after 2 min!");
      soundAlarmPattern();
      sendAlertToServer(2);
      secondAlertSent = true;
      alertActive = true;
      alertSentTime = now;
    }
    continueMonitor = false;
  }

  // Sound buzzer if alert still active and not handled
  if (alertActive && !alertHandled && now - alertSentTime > 60000UL) {
    Serial.println("🔊 1 min passed — sounding buzzer!");
    soundAlarmPattern();
  }
}
