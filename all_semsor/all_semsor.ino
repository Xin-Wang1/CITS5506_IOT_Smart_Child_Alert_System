// Smart Child Alert System
#define BLYNK_TEMPLATE_ID "TMPL60SNIol4o"
#define BLYNK_TEMPLATE_NAME "Smart Child Seat"
#define BLYNK_AUTH_TOKEN "vxP7O6nUsG0DlduR_U9FFyliYMKkhKc3"

#define COUNTDOWN_VPIN V7
#define LOG_ADDRESS "http://192.168.137.198:5000/upload"

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>
#include <HTTPClient.h>
#include "compressed_audio.h"

void playCompressedAudio();
bool isSomeoneSeated();
void updateBlynkData();
void sendAlertToServer(int alertCount);
void handleGPSStateMachine();

char ssid[] = "iPhoneHotspot";
// char ssid[] = "iot5506";
char pass[] = "12345678";

// 引脚
#define PRESSURE_PIN 32  
#define AUDIO_PIN 25          
#define SAMPLE_RATE 8000     
#define PRESSURE_THRESHOLD 100  

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// 状态变量
int state = 0;
unsigned long lastCheckTime = 0;
unsigned long lastGPSCompareTime = 0;
unsigned long alertSentTime = 0;
unsigned long lastBlynkUpdate = 0;
unsigned long delayStartTime = 0;
unsigned long continueStartTime = 0;

float lastLat = 0.0, lastLng = 0.0;
bool alertActive = false;
bool alertHandled = false;
bool waitDelay = false;
bool continueMonitor = false;
bool secondAlertSent = false;
int delayMinutes = 0;
bool gpsPausedPrinted = false;
bool gpsOutputEnabled = true;


// ---------- 工具函数 ----------
float calculateDistance(float lat1, float lng1, float lat2, float lng2) {
  const float R = 6371000;
  float dLat = radians(lat2 - lat1);
  float dLng = radians(lng2 - lng1);
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLng / 2) * sin(dLng / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

bool isSameLocation(float lat1, float lng1, float lat2, float lng2, float tol = 50.0) {
  float d = calculateDistance(lat1, lng1, lat2, lng2);
  Serial.print("📏 Distance difference: ");
  Serial.print(d);
  Serial.println(" 米");
  return d <= tol;
}

// ---------- Blynk----------
BLYNK_WRITE(V0) {
  if (param.asInt() == 1 && alertActive) {
    alertHandled = true;
    alertActive = false;
    playCompressedAudio();
    Serial.println("✅ Parents have responded, ending the alert");
  }
}

BLYNK_WRITE(V4) {
  delayMinutes = param.asInt();
  if (delayMinutes > 0) {
    waitDelay = true;
    delayStartTime = millis();
    alertHandled = true;
    alertActive = false;
    playCompressedAudio();
    Serial.println("⏳ Parents choose to stay for a short time " + String(delayMinutes) + " minutes, pause detection");
    Blynk.virtualWrite(V0, 0); // 重置按钮状态
  }
}

BLYNK_WRITE(V5) {
  if (param.asInt() == 1) { // 确保只在按钮按下时处理
    continueMonitor = true;
    continueStartTime = millis();
    secondAlertSent = false;
    alertHandled = false;
    alertActive = false;
    playCompressedAudio();
    Serial.println("⏱️ Parents choose to deal with it immediately and re-test after 2 minutes");
    Blynk.virtualWrite(V5, 0); // 重置按钮状态
  }
}

// ---------- init ----------
void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Wi-Fi connection is successful");

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());


  configTime(8 * 3600, 0, "pool.ntp.org");

  IPAddress ip;
  WiFi.hostByName("blynk.cloud", ip);
  Blynk.config(BLYNK_AUTH_TOKEN, ip.toString().c_str(), 80);
  Blynk.connect();

  gpsSerial.begin(9600, SERIAL_8N1, 34, 12);
  pinMode(PRESSURE_PIN, INPUT_PULLUP);
  playCompressedAudio();

  Serial.print("ESP32 IP Address: ");
Serial.println(WiFi.localIP());

if (WiFi.status() != WL_CONNECTED) {
  Serial.println("WiFi not connected!");
} else {
  Serial.println("WiFi connected.");
}

}

// ---------- 主循环 ----------
void loop() {
  Blynk.run();
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  unsigned long now = millis();

  if (now - lastBlynkUpdate > 5000) {
    isSomeoneSeated(); // 先调用读取压力值
    updateBlynkData();
    lastBlynkUpdate = now;
  }

  if (waitDelay && now - delayStartTime < delayMinutes * 60000UL) {
  unsigned long timeLeftMs = delayMinutes * 60000UL - (now - delayStartTime);
  int secondsLeft = timeLeftMs / 1000;
  int minutes = secondsLeft / 60;
  int seconds = secondsLeft % 60;

  char countdown[10];
  sprintf(countdown, "%02d:%02d", minutes, seconds);
  Blynk.virtualWrite(COUNTDOWN_VPIN, countdown);
  }

  // 处理短暂停留结束
  if (waitDelay && now - delayStartTime >= delayMinutes * 60000UL) {
    Serial.println("✅ At the end of the short stay, testing resumes");
    waitDelay = false;
    state = 0;
    Blynk.virtualWrite(COUNTDOWN_VPIN, "00:00");
  }

  // 立即处理后再次检测
  if (continueMonitor && now - continueStartTime >= 2 * 60000UL) {
    if (isSomeoneSeated() && !secondAlertSent) {
      Blynk.logEvent("child_warning", "🚨 The child hasn't been removed, so check it out now!");
      alertActive = true;
      alertSentTime = now;
      secondAlertSent = true;
      sendAlertToServer(2);  // 第二次警报

    }
    continueMonitor = false;
  }

  // 警报未处理超过1分钟
  if (alertActive && !alertHandled && now - alertSentTime > 60000) {
    Serial.println("🔊 Timeout does not respond, start an alert!");
    playCompressedAudio();
  }

  // 警报未响应超2分钟，强制响铃
  if (alertActive && !waitDelay && !continueMonitor && !alertHandled && now - alertSentTime > 120000) {
    Serial.println("🆘 Parents don't choose how to respond, and it will be automatically alerted after 2 minutes!");
    playCompressedAudio();
    alertActive = false;
  }

  // ---------- GPS State machine control ----------
  if (waitDelay || continueMonitor || alertActive) {
    if (!gpsPausedPrinted) {
      Serial.println("⏸️ The GPS status machine has been paused, waiting for a parent to respond...");
      gpsPausedPrinted = true;
    }
  } else {
    gpsPausedPrinted = false;
    handleGPSStateMachine();
  }
}

// ---------- GPS State machine encapsulation ----------
void handleGPSStateMachine() {
  unsigned long now = millis();

  if (state == 0 && now - lastCheckTime > 3000 && gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
    if (gpsOutputEnabled) {
      Serial.print("📍【First Position】"); Serial.println(String(lastLat, 6) + "," + String(lastLng, 6));
    }
    lastCheckTime = now;
    state = 1;

  } else if (state == 1 && now - lastCheckTime > 30000 && gps.location.isValid()) {
    float newLat = gps.location.lat();
    float newLng = gps.location.lng();
    if (gpsOutputEnabled) {
      Serial.print("📍[Position after 30 seconds]"); Serial.println(String(newLat, 6) + "," + String(newLng, 6));
    }

    if (!isSameLocation(lastLat, lastLng, newLat, newLng)) {
      Serial.println("📍 车辆移动，流程重启");
      gpsOutputEnabled = true;  // 位置变化 → 启用串口输出
      state = 0;
    } else {
      if (gpsOutputEnabled) {
        Serial.println("📍 位置稳定，等待2分钟...");
      }
      lastGPSCompareTime = now;
      state = 2;
    }

  } else if (state == 2 && now - lastGPSCompareTime > 120000 && gps.location.isValid()) {
    float newLat = gps.location.lat();
    float newLng = gps.location.lng();
    if (gpsOutputEnabled) {
      Serial.print("📍【2 minutes later position】"); Serial.println(String(newLat, 6) + "," + String(newLng, 6));
    }

    if (!isSameLocation(lastLat, lastLng, newLat, newLng)) {
      Serial.println("📍 The vehicle moves again and the process restarts");
      gpsOutputEnabled = true;  // 恢复打印
      state = 0;
    } else {
      bool pressure = isSomeoneSeated();
      if (pressure) {
        Blynk.logEvent("child_warning", "⚠️ The child is still in the seat and the vehicle is not moving! Please check it out now!");
        alertActive = true;
        alertHandled = false;
        alertSentTime = now;
        sendAlertToServer(1);  // 第一次警报计数为 1

      } else {
        if (gpsOutputEnabled) {
          Serial.println("✅ Unmanned, the process ends, and the GPS output is stopped");
        }
        gpsOutputEnabled = false;  // 无人 → 停止输出
      }
      state = 0;
    }
  }
}


// ---------- Blynk Data display ----------
void updateBlynkData() {
  int pressureValue = analogRead(PRESSURE_PIN);
  bool pressure = pressureValue > PRESSURE_THRESHOLD;
  
  Blynk.virtualWrite(V1, pressure ? "Child in seat" : "Empty seat");
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
    Blynk.virtualWrite(V3, gpsData);
  } else {
    Blynk.virtualWrite(V3, "No positioning");
  }
}

bool isSomeoneSeated() {
  int pressureValue = analogRead(PRESSURE_PIN);
  Serial.print("📟 Pressure reading: ");
  Serial.println(pressureValue);
  return pressureValue > PRESSURE_THRESHOLD;
}


void playCompressedAudio() {
  for (int i = 0; i < COMPRESSED_AUDIO_LENGTH; i++) {
    dacWrite(25, compressedAudio[i]);
    delayMicroseconds(1000000 / COMPRESSED_AUDIO_SAMPLE_RATE);
  }
}

void sendAlertToServer(int alertCount) {
//  void sendAlertToServer() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    Serial.println("[ESP32] Connecting to: http://192.168.137.198:5000/upload");
    http.begin("http://192.168.137.198:5000/upload");  // Flask服务器地址

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