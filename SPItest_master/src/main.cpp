#include <Arduino.h>
#include <SerialPIO.h>
#include <SPI.h>

constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t PIO_BAUD = 115200;
constexpr uint8_t CH_NUM = 3;

// Master 側のピン
constexpr uint8_t TX_PINS[CH_NUM] = {2, 10, 6};
constexpr uint8_t RX_PINS[CH_NUM] = {3, 11, 7};

// ハードウェアSPI(SPI0)のピン
constexpr uint8_t SPI_MISO_PIN = 18;
constexpr uint8_t SPI_CS_PIN   = 19;
constexpr uint8_t SPI_SCK_PIN  = 20;
constexpr uint8_t SPI_MOSI_PIN = 21;

SerialPIO pioUart0(TX_PINS[0], RX_PINS[0]);
SerialPIO pioUart1(TX_PINS[1], RX_PINS[1]);
SerialPIO pioUart2(TX_PINS[2], RX_PINS[2]);

SerialPIO* const uarts[CH_NUM] = {
  &pioUart0, &pioUart1, &pioUart2
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

  // ハードウェアSPIの初期化
  SPI.setRX(SPI_MISO_PIN);
  SPI.setCS(SPI_CS_PIN);
  SPI.setSCK(SPI_SCK_PIN);
  SPI.setTX(SPI_MOSI_PIN);
  SPI.begin();
  
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);

  Serial.println("MASTER start");
}

void loop() {
  static uint32_t lastSend = 0;
  static uint32_t seq = 0;

  if (millis() - lastSend >= 500) {
    lastSend = millis();
    sendAll(seq);

    // SPIでデータ送信
    char spiTxBuf[32];
    int len = snprintf(spiTxBuf, sizeof(spiTxBuf), "SPI,%lu", seq);
    
    digitalWrite(SPI_CS_PIN, LOW);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(spiTxBuf, len);
    SPI.endTransaction();
    digitalWrite(SPI_CS_PIN, HIGH);

    Serial.print("[MASTER] SPI -> ");
    Serial.println(spiTxBuf);

    seq++;
  }

  for (uint8_t ch = 0; ch < CH_NUM; ch++) {
    pollPort(ch);
  }
}