#include <Arduino.h>

// ピン設定
const int pinA = 2;
const int pinB = 3;
const int pinZ = 4;

volatile long encoderCount = 0; // 回転カウント用
volatile bool zSignalDetected = false;

void updateEncoder() {
  // A相立ち上がり時にB相がLOWなら正転、HIGHなら逆転
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

  // A相の立ち上がりエッジで割り込み実行
  attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, RISING);
}

void loop() {
  // Z相の信号が入ったらカウントをリセット（簡易的な実装）
  if (digitalRead(pinZ) == LOW) {
    encoderCount = 0;
  }

  static long lastCount = 0;
  if (encoderCount != lastCount) {
    Serial.println(encoderCount);
    lastCount = encoderCount;
  }
}
