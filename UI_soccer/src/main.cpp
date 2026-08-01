#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);


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
const unsigned long moveInterval = 300; 

// 画面表示・スクロール設定
const int visibleItems = 5; // 画面内に表示する行数
const int itemHeight = 38;   // 1行の高さ
const int startY = 50;       // メニュー開始Y座標

int A = 0;

void setup() {
  Serial.begin(115200);
  // 液晶の初期化
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);

  // 240x240の全画面スプライト（仮想キャンバス）をメモリ上に確保
  spr.createSprite(240, 240);
}


void line() {
  spr.fillSprite(TFT_BLACK);
for(int i=0;i<36;i++){
spr.drawCircle(cos(i * 10 * PI / 180) * 110 + 120, sin(i * 10 * PI / 180) * 110 + 120, 3, TFT_WHITE);
}
  spr.pushSprite(0, 0);
}

void zikoichi() {
  //コート122*183 外側含め182*243
  spr.fillSprite(TFT_BLACK);
  spr.fillRect(29, 0, 211, 240, TFT_DARKGREY);//背景色
  spr.fillRect(59, 28, 122, 183, TFT_WHITE);//59-120-181,28-119-211
  spr.fillRect(61, 30, 118, 179, TFT_DARKGREEN);//コートの外側の白い線
  spr.drawLine(59, 119, 181, 119, TFT_DARKGREY);//センターライン
  spr.drawCircle(120, 119, 30, TFT_DARKGREY);//センターサークル
  spr.fillRect(90,18, 60, 15, TFT_BLUE);//青ゴール
  spr.fillRect(90, 211, 60, 15, TFT_YELLOW);//黄色ゴール
  spr.fillCircle(120, 180, 2, TFT_ORANGE);//ボール
  spr.pushSprite(0, 0);
}

void guruguru() {
  spr.fillSprite(TFT_BLACK);
  spr.drawCircle(120, 120, 100, TFT_RED);
  spr.drawLine(120, 120, 120 + 100 * cos(A * PI / 180), 120 + 100 * sin(A * PI / 180), TFT_GREEN);
  A ++;
  spr.pushSprite(0, 0);
}

void Setmode() {
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
  spr.print("Lorentz Menu");
  spr.drawFastHLine(0, 38, 240, TFT_DARKGREY); // 区切り線

  int topIndex = 0;
  if (currentIndex >= visibleItems) {
    topIndex = currentIndex - visibleItems + 1;
  }

  for (int i = 0; i < visibleItems; i++) {
    int itemIdx = topIndex + i;
    if (itemIdx < itemCount) {
      int y = startY + (i * itemHeight);
      char buf[32];

      if (itemIdx == currentIndex) {
        spr.fillRect(15, y - 8 , 200, itemHeight, TFT_CYAN); 
        spr.setTextColor(TFT_BLACK, TFT_CYAN);
        spr.setTextSize(3);
        spr.setCursor(20, y);
        
        snprintf(buf, sizeof(buf), menuItems[itemIdx]);
        spr.print(buf);
      } else {
        // 【非選択の行】白文字
        spr.setTextColor(TFT_WHITE, TFT_BLACK);
        spr.setTextSize(3);
        spr.setCursor(20, y);
        
        snprintf(buf, sizeof(buf), menuItems[itemIdx]);
        spr.print(buf);
      }
    }
  }

  // 5. 完成した全画面（1枚絵）を液晶へ一撃転送
  spr.pushSprite(0, 0);


}

void loop(){
  line();
  delay(3000);
  zikoichi();
  delay(3000);
  guruguru();
  delay(3000);
  Setmode();
  delay(3000);
}
