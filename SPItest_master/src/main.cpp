#include <Arduino.h>
#include <SerialPIO.h>

constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t PIO_BAUD = 115200;
constexpr uint8_t CH_NUM = 4;

// Master 側のピン
constexpr uint8_t TX_PINS[CH_NUM] = {2, 10, 6, 14};
constexpr uint8_t RX_PINS[CH_NUM] = {3, 11, 7, 15};

// 変数名を pioUartX に変更して衝突を回避
SerialPIO pioUart0(TX_PINS[0], RX_PINS[0]);
SerialPIO pioUart1(TX_PINS[1], RX_PINS[1]);
SerialPIO pioUart2(TX_PINS[2], RX_PINS[2]);
SerialPIO pioUart3(TX_PINS[3], RX_PINS[3]);

SerialPIO* const uarts[CH_NUM] = {
  &pioUart0, &pioUart1, &pioUart2, &pioUart3
};

struct LineBuffer {
  char buf[96];
  uint8_t len = 0;
};

LineBuffer rxbuf[CH_NUM];

void clearBuffer(LineBuffer &b) {
  b.len = 0;
  b.buf[0] = '\0';
}

void handleLine(uint8_t ch, const char* s) {
  Serial.print("[MASTER] ch");
  Serial.print(ch);
  Serial.print(" <- ");
  Serial.println(s);
}

void pollPort(uint8_t ch) {
  SerialPIO &p = *uarts[ch];
  while (p.available() > 0) {
    char c = (char)p.read();

    if (c == '\r') continue;

    if (c == '\n') {
      rxbuf[ch].buf[rxbuf[ch].len] = '\0';
      if (rxbuf[ch].len > 0) {
        handleLine(ch, rxbuf[ch].buf);
      }
      clearBuffer(rxbuf[ch]);
      continue;
    }

    if (rxbuf[ch].len < sizeof(rxbuf[ch].buf) - 1) {
      rxbuf[ch].buf[rxbuf[ch].len++] = c;
    } else {
      clearBuffer(rxbuf[ch]); // 取りこぼし防止
    }
  }
}

void sendAll(uint32_t seq) {
  for (uint8_t ch = 0; ch < CH_NUM; ch++) {
    uarts[ch]->print("M,");
    uarts[ch]->print(ch);
    uarts[ch]->print(",");
    uarts[ch]->println(seq);
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(1500);

  for (uint8_t i = 0; i < CH_NUM; i++) {
    uarts[i]->begin(PIO_BAUD);
    clearBuffer(rxbuf[i]);
  }

  Serial.println("MASTER start");
}

void loop() {
  static uint32_t lastSend = 0;
  static uint32_t seq = 0;

  // 500msごとに非ブロッキングでデータ送信
  if (millis() - lastSend >= 500) {
    lastSend = millis();
    sendAll(seq++);
  }

  // ループ毎に最速で全ポートをポーリングする
  for (uint8_t ch = 0; ch < CH_NUM; ch++) {
    pollPort(ch);
  }
}