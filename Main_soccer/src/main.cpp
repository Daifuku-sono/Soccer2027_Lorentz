#include <Arduino.h>

// BNO055のUART通信設定（ボーレートは115200固定）
// Arduino UNOの場合はSerial（0, 1番ピン）をPC通信に使うため、
// 別のHardwareSerialを持つボード（ESP32やMegaなど）を推奨します。
#define BNO_SERIAL Serial2

void setup() {
  Serial.begin(115200);     // PCへのシリアルモニター出力用
  BNO_SERIAL.begin(115200); // BNO055との通信用
  
  delay(1000);
  Serial.println("BNO055 UART Test Start");

  // 1. 動作モードを「NDOF（9軸フュージョンモード）」に設定するコマンド
  // レジスタアドレス 0x3D (OPR_MODE) に 0x0C (NDOF) を書き込みます
  byte set_mode[] = {0xAA, 0x00, 0x3D, 0x01, 0x0C};
  BNO_SERIAL.write(set_mode, sizeof(set_mode));
  delay(50); // モード変更のための短い待機
}

void loop() {
  // 2. 角度データ（オイラー角のYaw, Roll, Pitch：計6バイト）を読み出すコマンド
  // レジスタアドレス 0x1A から 6バイト分を要求
  byte read_euler[] = {0xAA, 0x01, 0x1A, 0x06};
  BNO_SERIAL.write(read_euler, sizeof(read_euler));
  delay(20); // BNO055からの返信待ち

  // 返信データの解析（ヘッダ2バイト + データ6バイト = 計8バイトを期待）
  if (BNO_SERIAL.available() >= 8) {
    byte reply_status = BNO_SERIAL.read(); // 0xBB (成功)
    byte reply_len = BNO_SERIAL.read();    // 受信バイト数 (0x06)

    if (reply_status == 0xBB) {
      int16_t heading_raw = BNO_SERIAL.read() | (BNO_SERIAL.read() << 8);
      int16_t roll_raw    = BNO_SERIAL.read() | (BNO_SERIAL.read() << 8);
      int16_t pitch_raw   = BNO_SERIAL.read() | (BNO_SERIAL.read() << 8);

      // 1度 = 16 LSB なので、16.0で割って度数法（°）に変換
      float heading = heading_raw / 16.0;
      float roll    = roll_raw / 16.0;
      float pitch   = pitch_raw / 16.0;

      // シリアルモニターに表示
      Serial.print("Heading: "); Serial.print(heading);
      Serial.print(" | Roll: "); Serial.print(roll);
      Serial.print(" | Pitch: "); Serial.println(pitch);
    }
  }
  delay(100); // 100msごとに更新
}
