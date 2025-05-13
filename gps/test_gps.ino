#define BLYNK_TEMPLATE_ID "TMPL60SNIol4o"
#define BLYNK_TEMPLATE_NAME "Smart Child Seat"
#define BLYNK_AUTH_TOKEN "vxP7O6nUsG0DlduR_U9FFyliYMKkhKc3"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "iPhone (56) karla";
char pass[] = "karlaiscool";

BlynkTimer timer;
bool waitingForGPS = false;

float latestLat = 0;
float latestLon = 0;

BLYNK_WRITE(V1) {
  latestLat = param.asFloat();
  Serial.print("📍 Latitude: ");
  Serial.println(latestLat, 6);
}

BLYNK_WRITE(V2) {
  latestLon = param.asFloat();
  Serial.print("📍 Longitude: ");
  Serial.println(latestLon, 6);

  if (waitingForGPS) {
    Serial.println("✅ Full GPS coordinates received:");
    Serial.print("Lat: "); Serial.println(latestLat, 6);
    Serial.print("Lon: "); Serial.println(latestLon, 6);
    waitingForGPS = false;
  }
}

void requestGPS() {
  if (!waitingForGPS) {
    Serial.println("🛰 Requesting GPS from phone...");
    Blynk.virtualWrite(V10, 1);  // Ask phone to send GPS
    waitingForGPS = true;

    timer.setTimeout(60000L, []() {
      if (waitingForGPS) {
        Serial.println("❌ GPS request timed out. Resetting.");
        Blynk.virtualWrite(V10, 0);
        waitingForGPS = false;
      }
    });
  }
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Schedule first GPS request after 10 seconds
  timer.setTimeout(10000L, requestGPS);
}

void loop() {
  Blynk.run();
  timer.run();
}
