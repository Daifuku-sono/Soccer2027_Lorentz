#include <Arduino.h>

// ピン設定
const int pinA = 3;
const int pinB = 4;
const int pinZ = 5;

volatile long encoderCount = 0; // 回転カウント用
volatile bool zSignalDetected = false;

void updateEncoder() {
  if (digitalRead(pinB) == LOW) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(9600);
  
  // 入力ピン設定（必要に応じて内蔵プルアップを使用）
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinZ, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, RISING);
}

void loop() {
  if (digitalRead(pinZ) == LOW) {
    encoderCount = 0;
  }

  static long lastCount = 0;
  if (encoderCount != lastCount) {
    Serial.println(encoderCount);
    lastCount = encoderCount;
  }
}
