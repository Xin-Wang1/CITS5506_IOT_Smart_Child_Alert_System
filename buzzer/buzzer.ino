#define AUDIO_PIN 25

void setup() {
  pinMode(AUDIO_PIN, OUTPUT);
}

void loop() {
  for (int i = 0; i < 1000; i++) {
    dacWrite(AUDIO_PIN, 0);   // LOW
    delayMicroseconds(100);
    dacWrite(AUDIO_PIN, 255); // HIGH
    delayMicroseconds(100);
  }
  delay(1000);
}