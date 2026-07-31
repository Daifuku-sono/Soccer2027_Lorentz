#include <Arduino.h>
#include <TFT_eSPI.h>

// TFT_eSPI およびスプライトのインスタンス作成
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

// メニューの選択肢リスト（8項目）
const char* menuItems[] = {
  "All",
  "Ball",
  "Line",
  "BackCamera",
  "MainCamera",
  "BLE",
  "Encoder",
  "Other"
};
const int itemCount = 8;
int currentIndex = 0; // 現在選択中の項目

// 【自動移動タイマー設定】
unsigned long lastMoveTime = 0;
const unsigned long moveInterval = 300; // 600ミリ秒（0.6秒）ごとに自動で1マス進む

// 画面表示・スクロール設定
const int visibleItems = 5; // 画面内に表示する行数
const int itemHeight = 38;   // 1行の高さ
const int startY = 50;       // メニュー開始Y座標

void setup() {
  Serial.begin(115200);
  // 液晶の初期化
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // 240x240の全画面スプライト（仮想キャンバス）をメモリ上に確保
  spr.createSprite(240, 240);
}

void loop() {
  unsigned long currentMillis = millis();

  // タイマーで自動的に1マス下に移動（一番下に行ったらループ）
  if (currentMillis - lastMoveTime >= moveInterval) {
    lastMoveTime = currentMillis;
    currentIndex = (currentIndex + 1) % itemCount; 
  }

  // 1. 仮想キャンバス（裏画面）を消去
  spr.fillSprite(TFT_BLACK);

  // 2. 固定ヘッダーの描画
  spr.setTextColor(TFT_CYAN, TFT_BLACK);
  spr.setTextSize(2);
  spr.setCursor(20, 12);
  spr.print("AUTO MENU");
  spr.drawFastHLine(0, 38, 240, TFT_DARKGREY); // 区切り線

  // 3. スクロール位置（画面の先頭行）の計算
  int topIndex = 0;
  if (currentIndex >= visibleItems) {
    topIndex = currentIndex - visibleItems + 1;
  }

  // 4. 見えている4行分をスプライトへ描画
  for (int i = 0; i < visibleItems; i++) {
    int itemIdx = topIndex + i;
    if (itemIdx < itemCount) {
      int y = startY + (i * itemHeight);
      char buf[32];

      if (itemIdx == currentIndex) {
        // 【選択中の行】黄色文字 ＋ カーソル「>」
        spr.setTextColor(TFT_CYAN, TFT_BLACK);
        spr.setTextSize(3);
        spr.setCursor(20, y);
        
        snprintf(buf, sizeof(buf), "> %s", menuItems[itemIdx]);
        spr.print(buf);
      } else {
        // 【非選択の行】白文字
        spr.setTextColor(TFT_WHITE, TFT_BLACK);
        spr.setTextSize(3);
        spr.setCursor(20, y);
        
        snprintf(buf, sizeof(buf), "  %s", menuItems[itemIdx]);
        spr.print(buf);
      }
    }
  }

  // 5. 完成した全画面（1枚絵）を液晶へ一撃転送
  spr.pushSprite(0, 0);
}