#include <iostream>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <chrono>

// ============================
// Arduino互換モック (C++環境動作用)
// ============================
#define INPUT 0
#define LOW 0
#ifndef PI
#define PI 3.14159265358979323846f
#endif

// F() マクロの無効化（標準文字列にする）
#define F(x) x

// Serialのモック（標準出力へのリダイレクト）
class SerialMock {
public:
    void begin(int baud) {}
    
    template<typename T>
    void print(T val) { std::cout << val; }
    
    template<typename T>
    void println(T val) { std::cout << val << std::endl; }
    
    void println() { std::cout << std::endl; }
} Serial;

// math関数のモック
float radians(float deg) {
    return deg * PI / 180.0f;
}

// pinModeモック
void pinMode(int pin, int mode) {
    // C++環境では何もしない
}

// pulseInモック（頑張って再現）
// ※ 実際のハードウェア信号は読めないため、時間やピン番号に基づくダミーのセンサー値を返します。
// ※ ここを書き換えることで任意のテストケースを作成できます。
uint16_t pulseIn(int pin, int value, int timeout) {
    static int counter = 0;
    counter++;
    // 例: ピン番号とカウンターを使った変動するダミー値 (1000〜3000程度の値)
    return 1500 + (pin * 10) + (counter % 500); 
}

// ============================
// 設定・定数
// ============================
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
// 各センサーの履歴管理用
uint16_t history[SENSOR_COUNT][SENSOR_QUEUE_SIZE] = {0};
uint16_t head_idx[SENSOR_COUNT] = {0}; // C++の標準関数名との衝突を避けるため head -> head_idx
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
  uint16_t oldValue = history[sensorIndex][head_idx[sensorIndex]];

  if (count[sensorIndex] == SENSOR_QUEUE_SIZE) {
    sensorSum[sensorIndex] -= oldValue;
  } else {
    count[sensorIndex]++;
  }

  history[sensorIndex][head_idx[sensorIndex]] = value;
  sensorSum[sensorIndex] += value;
  head_idx[sensorIndex] = (head_idx[sensorIndex] + 1) % SENSOR_QUEUE_SIZE;
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
  // C++では負の数になる可能性がある場合のアンダーフローに注意
  uint32_t avg3 = sumTop3 / 3;
  return (avg3 > 750) ? (avg3 - 750) : 0; 
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
    float w = history[i][(head_idx[i] - 1 + SENSOR_QUEUE_SIZE) % SENSOR_QUEUE_SIZE];
    float angleRad = radians(i * 360.0f / SENSOR_COUNT);
    sumX += std::cos(angleRad) * w;
    sumY -= std::sin(angleRad) * w; // Y軸は反転
    total += w;
  }

  if (total == 0) return 0.0f;
  float ang = std::atan2(sumY, sumX) * 180.0f / PI;
  if (ang < 0) ang += 360.0f;
  return ang;
}

// ============================
// メイン処理
// ============================
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(pin[i], INPUT);
  }
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

  // 4. シリアル出力 (デバッグ用)
  Serial.print(F("Dist: ")); Serial.print(weightedDist);
  // Serial.print(F(" | Level: "));
  // if (level == 2)      Serial.print(F("NEAR"));
  // else if (level == 1) Serial.print(F("MID"));
  // else                 Serial.print(F("FAR"));
  Serial.print(F(" | Ang: ")); Serial.print(angle);
  Serial.print(F(" | PredAng: ")); Serial.print(predAngle);
  Serial.print(F(" | PredDist: ")); Serial.println(predDist);
}

// ============================
// C++ エントリポイント
// ============================
int main() {
  setup();
  
  // デバッグ用に100回ループさせる（実際のArduinoでは無限ループですが、PC環境テスト用に制限）
  for (int i = 0; i < 100; i++) {
      loop();
      
      // ループが早すぎる場合は少しスリープを入れる（必要に応じてコメントアウト解除）
      // std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  
  return 0;
}