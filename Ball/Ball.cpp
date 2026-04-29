#include <cstdio>
#include <cstdint>
#include <cmath>
#include <array>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/time.h"

static constexpr float PI_F = 3.14159265358979323846f;

// ============================
// 設定・定数
// ============================
static constexpr float TH_NEAR = 4200.0f;
static constexpr float TH_MID  = 3000.0f;

static constexpr float WEIGHT_RECENT = 10.0f;
static constexpr float WEIGHT_MIDDLE = 2.0f;
static constexpr float WEIGHT_OLD    = 0.4f;

static constexpr int SENSOR_COUNT = 16;
static constexpr int SENSOR_QUEUE_SIZE = 16;
static constexpr int DIST_QUEUE_SIZE = 16;
static constexpr int PRED_QUEUE_SIZE = 6;

// 【修正点】標準的なPico (RP2040) で利用可能な GPIO 0〜15 に仮変更しています。
// RP2350等を使用する場合は元の配線に合わせて変更してください。
static constexpr std::array<uint, SENSOR_COUNT> PIN = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
};

// ============================
// 数学ユーティリティ
// ============================
static inline float radians(float deg) {
    return deg * PI_F / 180.0f;
}

static float normalizeAngle(float a) {
    while (a < 0.0f)   a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

static float angleDiffDeg(float a, float b) {
    float d = a - b;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// ============================
// pulseIn(LOW) 代替
// ============================
// 低レベルのパルス幅(us)を測る
static uint32_t pulseInLow(uint gpio, uint32_t timeout_us) {
    // タイムアウト計算には 32bit カウンタで十分かつ高速
    uint32_t start = time_us_32();

    // 1. すでにLOWの場合は、正確なパルスの始まりを捉えるために一度HIGHになるのを待つ
    while (gpio_get(gpio) == 0) {
        if ((time_us_32() - start) >= timeout_us) return 0;
    }

    // 2. LOW になるまで待つ (パルスの開始)
    while (gpio_get(gpio) != 0) {
        if ((time_us_32() - start) >= timeout_us) return 0;
    }

    uint32_t pulse_start = time_us_32();

    // 3. LOW のままでいる時間を測る (HIGHになったら終了)
    while (gpio_get(gpio) == 0) {
        if ((time_us_32() - start) >= timeout_us) return 0;
    }

    return time_us_32() - pulse_start;
}

// ============================
// センサ処理
// ============================
class SensorProcessor {
public:
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

    uint32_t getTop3AverageOfSums() const {
        std::array<uint32_t, SENSOR_COUNT> temp{};
        for (int i = 0; i < SENSOR_COUNT; i++) temp[i] = sensorSum[i];

        uint32_t sumTop3 = 0;
        for (int k = 0; k < 3; k++) {
            int maxIndex = 0;
            for (int i = 1; i < SENSOR_COUNT; i++) {
                if (temp[i] > temp[maxIndex]) maxIndex = i;
            }
            sumTop3 += temp[maxIndex];
            temp[maxIndex] = 0;
        }

        uint32_t avg3 = sumTop3 / 3;
        return (avg3 > 750) ? (avg3 - 750) : 0;
    }

    void pushDistance(uint32_t value) {
        distanceHistory[distanceHead] = value;
        distanceHead = (distanceHead + 1) % DIST_QUEUE_SIZE;
        if (distanceCount < DIST_QUEUE_SIZE) distanceCount++;
    }

    float getWeightedDistance() const {
        if (distanceCount == 0) return 0.0f;

        float recentSum = 0.0f;
        float middleSum = 0.0f;
        float oldSum = 0.0f;

        int recentN = 0;
        int middleN = 0;
        int oldN = 0;

        for (int i = 0; i < distanceCount && i < 8; i++) {
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

    int getDistanceLevel(float d) const {
        if (d > TH_NEAR) return 2;
        if (d > TH_MID)  return 1;
        return 0;
    }

    float getAngleDegrees() const {
        float sumX = 0.0f, sumY = 0.0f, total = 0.0f;

        for (int i = 0; i < SENSOR_COUNT; i++) {
            float w = history[i][(head[i] - 1 + SENSOR_QUEUE_SIZE) % SENSOR_QUEUE_SIZE];
            float angleRad = radians(i * 360.0f / SENSOR_COUNT);
            sumX += std::cos(angleRad) * w;
            sumY -= std::sin(angleRad) * w;
            total += w;
        }

        if (total == 0.0f) return 0.0f;

        float ang = std::atan2(sumY, sumX) * 180.0f / PI_F;
        if (ang < 0.0f) ang += 360.0f;
        return ang;
    }

    void pushPredictSample(float angleDeg, float dist) {
        angleHistory[predHead] = normalizeAngle(angleDeg);
        distHistory[predHead]  = dist;
        predHead = (predHead + 1) % PRED_QUEUE_SIZE;
        if (predCount < PRED_QUEUE_SIZE) predCount++;
    }

    void getPredictedValue(float &predAngleDeg, float &predDist) const {
        int latestIdx = (predHead - 1 + PRED_QUEUE_SIZE) % PRED_QUEUE_SIZE;
        predAngleDeg = angleHistory[latestIdx];
        predDist     = distHistory[latestIdx];

        if (predCount < 2) return;

        float sumDA = 0.0f;
        float sumDD = 0.0f;
        int n = 0;

        for (int i = 0; i < 3 && i < predCount - 1; i++) {
            int newer = (predHead - 1 - i + PRED_QUEUE_SIZE) % PRED_QUEUE_SIZE;
            int older = (predHead - 2 - i + PRED_QUEUE_SIZE) % PRED_QUEUE_SIZE;

            sumDA += angleDiffDeg(angleHistory[newer], angleHistory[older]);
            sumDD += (distHistory[newer] - distHistory[older]);
            n++;
        }

        float avgDA = sumDA / n;
        float avgDD = sumDD / n;

        constexpr float PREDICT_STEP = 2.0f;
        predAngleDeg = normalizeAngle(predAngleDeg + avgDA * PREDICT_STEP);
        predDist     = predDist + avgDD * PREDICT_STEP;
        if (predDist < 0.0f) predDist = 0.0f;
    }

private:
    std::array<std::array<uint16_t, SENSOR_QUEUE_SIZE>, SENSOR_COUNT> history{};
    std::array<uint16_t, SENSOR_COUNT> head{};
    std::array<uint16_t, SENSOR_COUNT> count{};
    std::array<uint32_t, SENSOR_COUNT> sensorSum{};

    std::array<uint32_t, DIST_QUEUE_SIZE> distanceHistory{};
    int distanceHead = 0;
    int distanceCount = 0;

    std::array<float, PRED_QUEUE_SIZE> angleHistory{};
    std::array<float, PRED_QUEUE_SIZE> distHistory{};
    int predHead = 0;
    int predCount = 0;
};

static void setupPins() {
    for (auto gpio : PIN) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_IN); // 可読性向上のため `false` -> `GPIO_IN`
        gpio_pull_up(gpio);          // センサ仕様に合わせて変更
    }
}

static void loopOnce(SensorProcessor& proc) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        // 【注意】タイムアウト2000us(=2ms)はセンサによっては短すぎる可能性があります。
        // 値が常に0になる場合は、ここの数値を 20000 などに増やしてください。
        uint32_t val = pulseInLow(PIN[i], 2000);
        proc.pushSensorValue(i, static_cast<uint16_t>(val));
    }

    uint32_t rawDistance = proc.getTop3AverageOfSums();
    proc.pushDistance(rawDistance);

    float weightedDist = proc.getWeightedDistance();
    int level = proc.getDistanceLevel(weightedDist);

    float angle = proc.getAngleDegrees();
    proc.pushPredictSample(angle, weightedDist);

    float predAngle = 0.0f;
    float predDist = 0.0f;
    proc.getPredictedValue(predAngle, predDist);

    printf("Dist: %.1f | Level: %d | Ang: %.1f | PredAng: %.1f | PredDist: %.1f\n",
           weightedDist, level, angle, predAngle, predDist);
}

int main() {
    stdio_init_all();
    setupPins();

    SensorProcessor proc;

    while (true) {
        loopOnce(proc);
        sleep_ms(10);
    }
}