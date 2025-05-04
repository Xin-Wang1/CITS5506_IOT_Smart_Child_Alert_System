// Smart Child Alert System (优化版)
#define BLYNK_TEMPLATE_ID "TMPL6iMZTQG-9"
#define BLYNK_TEMPLATE_NAME "TEST"
#define BLYNK_AUTH_TOKEN "2XLv5UunJqLYp0eWksGo3Xydge41XCT1"

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>

char ssid[] = "TP-Link_2A3A";
char pass[] = "33601533";

// 引脚
#define PRESSURE_PIN 13
#define BUZZER_PIN 2

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
bool gpsPausedPrinted = false; // 防刷屏提示标志
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

// ---------- Blynk 回调 ----------
BLYNK_WRITE(V0) {
  if (param.asInt() == 1 && alertActive) {
    alertHandled = true;
    alertActive = false;
    digitalWrite(BUZZER_PIN, LOW);
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
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("⏳ Parents choose to stay for a short time " + String(delayMinutes) + " minutes, pause detection");
  }
}

BLYNK_WRITE(V5) {
  continueMonitor = true;
  continueStartTime = millis();
  secondAlertSent = false;
  alertHandled = false;
  alertActive = false;
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("⏱️ Parents choose to deal with it immediately and re-test after 2 minutes");
}

// ---------- 初始化 ----------
void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Wi-Fi connection is successful");

  configTime(8 * 3600, 0, "pool.ntp.org");

  IPAddress ip;
  WiFi.hostByName("blynk.cloud", ip);
  Blynk.config(BLYNK_AUTH_TOKEN, ip.toString().c_str(), 80);
  Blynk.connect();

  gpsSerial.begin(9600, SERIAL_8N1, 34, 12);
  pinMode(PRESSURE_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

// ---------- 主循环 ----------
void loop() {
  Blynk.run();
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  unsigned long now = millis();

  if (now - lastBlynkUpdate > 5000) {
    updateBlynkData();
    lastBlynkUpdate = now;
  }

  // 处理短暂停留结束
  if (waitDelay && now - delayStartTime >= delayMinutes * 60000UL) {
    Serial.println("✅ At the end of the short stay, testing resumes");
    waitDelay = false;
    state = 0;
  }

  // 立即处理后再次检测
  if (continueMonitor && now - continueStartTime >= 2 * 60000UL) {
    if (digitalRead(PRESSURE_PIN) == LOW && !secondAlertSent) {
      Blynk.logEvent("child_warning", "🚨 The child hasn't been removed, so check it out now!");
      alertActive = true;
      alertSentTime = now;
      secondAlertSent = true;
    }
    continueMonitor = false;
  }

  // 警报未处理超过1分钟
  if (alertActive && !alertHandled && now - alertSentTime > 60000) {
    Serial.println("🔊 Timeout does not respond, start an alert!");
    digitalWrite(BUZZER_PIN, HIGH);
  }

  // 警报未响应超2分钟，强制响铃
  if (alertActive && !waitDelay && !continueMonitor && !alertHandled && now - alertSentTime > 120000) {
    Serial.println("🆘 Parents don't choose how to respond, and it will be automatically alerted after 2 minutes!");
    digitalWrite(BUZZER_PIN, HIGH);
    alertActive = false;
  }

  // ---------- GPS 状态机控制 ----------
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

// ---------- GPS 状态机封装 ----------
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
      bool pressure = (digitalRead(PRESSURE_PIN) == LOW);
      if (pressure) {
        Blynk.logEvent("child_warning", "⚠️ The child is still in the seat and the vehicle is not moving! Please check it out now!");
        alertActive = true;
        alertHandled = false;
        alertSentTime = now;
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


// ---------- Blynk 数据展示 ----------
void updateBlynkData() {
  bool pressure = (digitalRead(PRESSURE_PIN) == LOW);
  Blynk.virtualWrite(V1, pressure ? "someone" : "nobody");

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Blynk.virtualWrite(V2, buffer);
  } else {
    Blynk.virtualWrite(V2, "时间获取失败");
  }

  if (gps.location.isValid()) {
    String gpsData = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    Blynk.virtualWrite(V3, gpsData);
  } else {
    Blynk.virtualWrite(V3, "无定位");
  }
}
