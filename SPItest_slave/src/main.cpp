#include <Arduino.h>
#include <SerialPIO.h>
#include <SPI.h>
#include <SPISlave.h>

constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t PIO_BAUD = 115200;
constexpr uint8_t CH_NUM = 3;

// Master と逆向き
constexpr uint8_t TX_PINS[CH_NUM] = {3, 11, 7};
constexpr uint8_t RX_PINS[CH_NUM] = {2, 10, 6};

// master 側と同じ番号に合わせる
constexpr uint8_t SPI_MISO_PIN = 18;
constexpr uint8_t SPI_CS_PIN    = 19;
constexpr uint8_t SPI_SCK_PIN   = 20;
constexpr uint8_t SPI_MOSI_PIN  = 21;

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
uint32_t recvCount[CH_NUM] = {0, 0, 0};

char spiRxBuf[64];
volatile bool spiDataReady = false;

void clearBuffer(LineBuffer &b) {
  b.len = 0;
  b.buf[0] = '\0';
}

void spiRecvCallback(uint8_t *data, size_t len) {
  size_t copyLen = (len < sizeof(spiRxBuf) - 1) ? len : (sizeof(spiRxBuf) - 1);
  memcpy(spiRxBuf, data, copyLen);
  spiRxBuf[copyLen] = '\0';
  spiDataReady = true;
}

void reply(uint8_t ch, const char* line) {
  recvCount[ch]++;

  Serial.print("[SLAVE] ch");
  Serial.print(ch);
  Serial.print(" <- ");
  Serial.println(line);

  uarts[ch]->print("ACK,");
  uarts[ch]->print(ch);
  uarts[ch]->print(",");
  uarts[ch]->println(recvCount[ch]);
}

void pollPort(uint8_t ch) {
  SerialPIO &p = *uarts[ch];
  while (p.available() > 0) {
    char c = (char)p.read();
    if (c == '\r') continue;

    if (c == '\n') {
      rxbuf[ch].buf[rxbuf[ch].len] = '\0';
      if (rxbuf[ch].len > 0) {
        reply(ch, rxbuf[ch].buf);
      }
      clearBuffer(rxbuf[ch]);
      continue;
    }

    if (rxbuf[ch].len < sizeof(rxbuf[ch].buf) - 1) {
      rxbuf[ch].buf[rxbuf[ch].len++] = c;
    } else {
      clearBuffer(rxbuf[ch]);
    }
  }
}

void initPioUarts() {
  Serial.println("[BOOT] init PIO UART...");
  for (uint8_t i = 0; i < CH_NUM; i++) {
    uarts[i]->begin(PIO_BAUD);
    clearBuffer(rxbuf[i]);
    Serial.print("[BOOT] UART ");
    Serial.print(i);
    Serial.println(" OK");
    delay(50);
  }
}

void initSpiSlave() {
  Serial.println("[BOOT] init SPI SLAVE...");

  SPISlave.setRX(SPI_MISO_PIN);
  SPISlave.setCS(SPI_CS_PIN);
  SPISlave.setSCK(SPI_SCK_PIN);
  SPISlave.setTX(SPI_MOSI_PIN);

  SPISettings spisettings(1000000, MSBFIRST, SPI_MODE0);
  SPISlave.onDataRecv(spiRecvCallback);
  SPISlave.begin(spisettings);

  Serial.println("[BOOT] SPI SLAVE OK");
}

void setup() {
  Serial.begin(USB_BAUD);

  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
    delay(10);
  }

  Serial.println();
  Serial.println("=== SLAVE BOOT ===");

  initPioUarts();
  initSpiSlave();

  Serial.println("SLAVE start");
}

void loop() {
  for (uint8_t ch = 0; ch < CH_NUM; ch++) {
    pollPort(ch);
  }

  if (spiDataReady) {
    Serial.print("[SLAVE] SPI <- ");
    Serial.println(spiRxBuf);
    spiDataReady = false;
  }
}