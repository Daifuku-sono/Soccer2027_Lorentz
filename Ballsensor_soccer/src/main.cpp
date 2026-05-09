#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================
// 設定・定数
// ============================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// しきい値設定 (rawDistanceに対する閾値：値が大きいほど近い)
#define TH_NEAR 4200 
#define TH_MID  3000

// 重み付け移動平均の重み
#define WEIGHT_RECENT 10.0f   // 直近5回分
#define WEIGHT_MIDDLE 2.0f   // 中間5回分
#define WEIGHT_OLD    0.4f   // 過去6回分

// センサー設定
const int SENSOR_COUNT = 16;
const int SENSOR_QUEUE_SIZE = 16;
const int DIST_QUEUE_SIZE = 16;
const uint8_t pin[SENSOR_COUNT] = {
  24, 25, 26, 27, 28, 29, 30, 31, 32, 39, 38, 37, 36, 35, 34, 33
};

// ============================
// グローバル変数
// ============================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 各センサーの履歴管理用
uint16_t history[SENSOR_COUNT][SENSOR_QUEUE_SIZE] = {0};
uint16_t head[SENSOR_COUNT] = {0};
uint16_t count[SENSOR_COUNT] = {0};
uint32_t sensorSum[SENSOR_COUNT] = {0};

// 統合された距離の履歴管理用
uint32_t distanceHistory[DIST_QUEUE_SIZE] = {0};
int distanceHead = 0;
int distanceCount = 0;

// ============================
// 予測用履歴
// ============================
const int PRED_QUEUE_SIZE = 6;
float angleHistory[PRED_QUEUE_SIZE] = {0};
float distHistory[PRED_QUEUE_SIZE]  = {0};
int predHead = 0;
int predCount = 0;

float normalizeAngle(float a) {
  while (a < 0.0f)   a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

// 角度差を -180～180 に正規化
float angleDiffDeg(float a, float b) {
  float d = a - b;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

void pushPredictSample(float angleDeg, float dist) {
  angleHistory[predHead] = normalizeAngle(angleDeg);
  distHistory[predHead]  = dist;
  predHead = (predHead + 1) % PRED_QUEUE_SIZE;
  if (predCount < PRED_QUEUE_SIZE) predCount++;
}

// 直近の変化量から、少し先の角度・距離を予測
void getPredictedValue(float &predAngleDeg, float &predDist) {
  int latestIdx = (predHead - 1 + PRED_QUEUE_SIZE) % PRED_QUEUE_SIZE;
  predAngleDeg = angleHistory[latestIdx];
  predDist     = distHistory[latestIdx];

  if (predCount < 2) return;

  float sumDA = 0.0f;
  float sumDD = 0.0f;
  int n = 0;

  // 直近3区間の変化量を平均
  for (int i = 0; i < 3 && i < predCount - 1; i++) {
    int newer = (predHead - 1 - i + PRED_QUEUE_SIZE) % PRED_QUEUE_SIZE;
    int older = (predHead - 2 - i + PRED_QUEUE_SIZE) % PRED_QUEUE_SIZE;

    sumDA += angleDiffDeg(angleHistory[newer], angleHistory[older]);
    sumDD += (distHistory[newer] - distHistory[older]);
    n++;
  }

  float avgDA = sumDA / n;
  float avgDD = sumDD / n;

  // ここで予測の先読み量を調整
  const float PREDICT_STEP = 2.0f;

  predAngleDeg = normalizeAngle(predAngleDeg + avgDA * PREDICT_STEP);
  predDist     = predDist + avgDD * PREDICT_STEP;
  if (predDist < 0.0f) predDist = 0.0f;
}
// ============================
// センサーデータ処理
// ============================

// センサーごとの値をキューに入れ、合計値を更新
void pushSensorValue(int sensorIndex, uint16_t value) {
  uint16_t oldValue = history[sensorIndex][head[sensorIndex]];

  if (count[sensorIndex] == SENSOR_QUEUE_SIZE) {
    sensorSum[sensorIndex] -= oldValue;
  } else {
    count[sensorIndex]++;
  }

  history[sensorIndex][head[sensorIndex]] = value;
  sensorSum[sensorIndex] += value;
  head[sensorIndex] = (head[sensorIndex] + 1) % SENSOR_QUEUE_SIZE;
}

// 全センサーのうち、合計値が高い上位3つの平均を取得
uint32_t getTop3AverageOfSums() {
  uint32_t temp[SENSOR_COUNT];
  for (int i = 0; i < SENSOR_COUNT; i++) {
    temp[i] = sensorSum[i];
  }

  uint32_t sumTop3 = 0;
  for (int k = 0; k < 3; k++) {
    int maxIndex = 0;
    for (int i = 1; i < SENSOR_COUNT; i++) {
      if (temp[i] > temp[maxIndex]) maxIndex = i;
    }
    sumTop3 += temp[maxIndex];
    temp[maxIndex] = 0; // 次点の計算のために0にセット
  }

  // 上位3つの平均からオフセットを引く (※3で割る)
  return (sumTop3 / 3) - 750;
}

// 算出された代表距離を履歴に追加
void pushDistance(uint32_t value) {
  distanceHistory[distanceHead] = value;
  distanceHead = (distanceHead + 1) % DIST_QUEUE_SIZE;
  if (distanceCount < DIST_QUEUE_SIZE) distanceCount++;
}

// 重み付けされた距離を計算
float getWeightedDistance() {
  if (distanceCount == 0) return 0.0f;

  float recentSum = 0.0f;
  float middleSum = 0.0f;
  float oldSum = 0.0f;

  int recentN = 0;
  int middleN = 0;
  int oldN = 0;

  for (int i = 0; i < distanceCount; i++) {
    if (i >= 8) break;   // 直近8回だけ使う

    int idx = (distanceHead - 1 - i + DIST_QUEUE_SIZE) % DIST_QUEUE_SIZE;
    uint32_t v = distanceHistory[idx];

    if (i < 1) {
      recentSum += v;
      recentN++;
    } else if (i < 4) {
      middleSum += v;
      middleN++;
    } else {
      oldSum += v;
      oldN++;
    }
  }

  float recentAvg = (recentN > 0) ? recentSum / recentN : 0.0f;
  float middleAvg = (middleN > 0) ? middleSum / middleN : 0.0f;
  float oldAvg    = (oldN > 0) ? oldSum / oldN : 0.0f;

  return (
    (recentAvg * WEIGHT_RECENT) +
    (middleAvg * WEIGHT_MIDDLE) +
    (oldAvg    * WEIGHT_OLD)
  ) / (WEIGHT_RECENT + WEIGHT_MIDDLE + WEIGHT_OLD);
}
// 距離に基づいたレベル判定
int getDistanceLevel(float d) {
  if (d > TH_NEAR) return 2; // 近い
  if (d > TH_MID)  return 1; // 中
  return 0;                  // 遠い
}

// 全センサーの値からベクトルの合成で角度を算出
float getAngleDegrees() {
  float sumX = 0, sumY = 0, total = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    float w = history[i][(head[i] - 1 + SENSOR_QUEUE_SIZE) % SENSOR_QUEUE_SIZE];
    float angleRad = radians(i * 360.0f / SENSOR_COUNT);
    sumX += cos(angleRad) * w;
    sumY -= sin(angleRad) * w;
    total += w;
  }

  if (total == 0) return 0.0f;
  float ang = atan2(sumY, sumX) * 180.0f / PI;
  if (ang < 0) ang += 360.0f;
  return ang;
}
// ディスプレイ
void drawDirection(float angleDeg, float dist, float predAngleDeg, float predDist) {
  const int cx = 64;
  const int cy = 32;
  const int r  = 22;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // 円
  display.drawCircle(cx, cy, r, SSD1306_WHITE);

  // 現在位置の矢印
  float rad = radians(angleDeg + 90.0f);
  int x = cx + (int)(cos(rad) * r);
  int y = cy + (int)(sin(rad) * r);

  display.drawLine(cx, cy, x, y, SSD1306_WHITE);

  float headLen = 6.0f;
  float leftRad  = rad + radians(150);
  float rightRad = rad - radians(150);

  int x1 = x + (int)(cos(leftRad) * headLen);
  int y1 = y + (int)(sin(leftRad) * headLen);
  int x2 = x + (int)(cos(rightRad) * headLen);
  int y2 = y + (int)(sin(rightRad) * headLen);

  display.drawLine(x, y, x1, y1, SSD1306_WHITE);
  display.drawLine(x, y, x2, y2, SSD1306_WHITE);

  // 予測位置の矢印（点線）
  float prad = radians(predAngleDeg + 90.0f);
  int pr = r - 4;

  for (float t = 0.0f; t < 1.0f; t += 0.20f) {
    float t2 = t + 0.10f;
    if (t2 > 1.0f) t2 = 1.0f;

    int sx = cx + (int)(cos(prad) * pr * t);
    int sy = cy + (int)(sin(prad) * pr * t);
    int ex = cx + (int)(cos(prad) * pr * t2);
    int ey = cy + (int)(sin(prad) * pr * t2);

    display.drawLine(sx, sy, ex, ey, SSD1306_WHITE);
  }

  // テキスト表示
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("ANG:");
  display.print(angleDeg, 1);

  display.setCursor(0, 10);
  display.print("PRED:");
  display.print(predAngleDeg, 1);

  display.setCursor(0, 54);
  display.print("DIST:");
  display.print(dist, 0);

  display.display();
}
// ============================
// メイン処理
// ============================

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(800000);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(pin[i], INPUT);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 init failed"));
    while (1);
  }
  display.clearDisplay();
  display.display();
}

void loop() {
  // 1. 各センサー値の取得
  for (int i = 0; i < SENSOR_COUNT; i++) {
    uint16_t val = pulseIn(pin[i], LOW, 2000); 
    pushSensorValue(i, val);
  }

  // 2. 代表距離の計算と平滑化
  uint32_t rawDistance = getTop3AverageOfSums();
  pushDistance(rawDistance);
  float weightedDist = getWeightedDistance();
  int level = getDistanceLevel(weightedDist);

  // 3. 角度の計算
  float angle = getAngleDegrees();
  // 予測用に履歴へ追加
  pushPredictSample(angle, weightedDist);
  // 予測値を計算
  float predAngle, predDist;
  getPredictedValue(predAngle, predDist);

  // // 4. シリアル出力 (デバッグ用)
   Serial.print(F("Dist: ")); Serial.print(weightedDist);
  // Serial.print(F(" | Level: "));
  // if (level == 2)      Serial.print(F("NEAR"));
  // else if (level == 1) Serial.print(F("MID"));
  // else                 Serial.print(F("FAR"));
  Serial.print(F(" | Ang: ")); Serial.println(angle, 1);
  Serial.print(F(" | PredAng: "));
  Serial.print(predAngle);
  Serial.print(F(" | PredDist: "));
  Serial.println(predDist);

drawDirection(angle, weightedDist, predAngle, predDist);
}