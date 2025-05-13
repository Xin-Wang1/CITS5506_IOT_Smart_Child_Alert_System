#include "attention_audio.h"

#define AUDIO_PIN 25
#define SAMPLE_RATE 8000

void setup() {
  pinMode(AUDIO_PIN, OUTPUT);
}

void loop() {
  for (int i = 0; i < attentionAudioLength; i++) {
    dacWrite(AUDIO_PIN, attentionAudio[i]);
    delayMicroseconds(1000000 / SAMPLE_RATE);
  }
  delay(5000);
}