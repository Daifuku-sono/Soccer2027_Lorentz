#include <Arduino.h>
#include <SerialPIO.h>
#include "hardware/pio.h"
#include "hardware/gpio.h"

// === UART 設定 ===
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t PIO_BAUD = 115200;
constexpr uint8_t CH_NUM = 3; // 14, 15ピンを削除して3chに

constexpr uint8_t TX_PINS[CH_NUM] = {2, 10, 6};
constexpr uint8_t RX_PINS[CH_NUM] = {3, 11, 7};

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

// === SPI 設定 ===
static constexpr uint SCK_PIN  = 20;
static constexpr uint MOSI_PIN = 21;
static constexpr uint MISO_PIN = 18;
static constexpr uint CS_PIN   = 19;

PIO pio = pio0;
uint sm; // 衝突を防ぐため定数から変数に変更
uint pio_offset;

static const uint16_t master_instructions[] = {
    pio_encode_out(pio_pins, 1) | (0 << 12), // 0: MOSIに1bit出力 + SCKをLOW
    pio_encode_in(pio_pins, 1)  | (1 << 12)  // 1: MISOから1bit読取 + SCKをHIGH
};
static const pio_program_t master_program = { .instructions = master_instructions, .length = 2, .origin = -1 };


void spi_master_init() {
    pio_offset = pio_add_program(pio, &master_program);
    sm = pio_claim_unused_sm(pio, true); // 空いているステートマシンを安全に確保

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_offset, pio_offset + 1);

    sm_config_set_sideset_pins(&c, SCK_PIN);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_out_pins(&c, MOSI_PIN, 1);
    sm_config_set_in_pins(&c, MISO_PIN);

    sm_config_set_out_shift(&c, false, true, 8); 
    sm_config_set_in_shift(&c, false, true, 8);  
    sm_config_set_clkdiv(&c, 30.0f);

    pio_gpio_init(pio, SCK_PIN);
    pio_gpio_init(pio, MOSI_PIN);
    pio_gpio_init(pio, MISO_PIN);

    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);

    pio_sm_set_consecutive_pindirs(pio, sm, SCK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, MOSI_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, MISO_PIN, 1, false);

    pio_sm_clear_fifos(pio, sm);
    pio_sm_init(pio, sm, pio_offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

uint8_t spi_master_txrx(uint8_t tx) {
    gpio_put(CS_PIN, 0);
    delayMicroseconds(2);

    pio_sm_put(pio, sm, (uint32_t)tx << 24);

    uint32_t t = millis();
    while (pio_sm_is_rx_fifo_empty(pio, sm)) {
        if (millis() - t > 100) {
            Serial.println("[Error] SPI Rx Timeout!");
            gpio_put(CS_PIN, 1);
            return 0xFF;
        }
    }
    uint8_t rx = (uint8_t)(pio_sm_get(pio, sm) & 0xFF);

    delayMicroseconds(2);
    gpio_put(CS_PIN, 1);
    return rx;
}

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
      if (rxbuf[ch].len > 0) handleLine(ch, rxbuf[ch].buf);
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

  // UART初期化
  for (uint8_t i = 0; i < CH_NUM; i++) {
    uarts[i]->begin(PIO_BAUD);
    clearBuffer(rxbuf[i]);
  }

  // SPI初期化
  spi_master_init();
  
  Serial.println("MASTER start");
}

void loop() {
  static uint32_t lastSend = 0;
  static uint32_t seq = 0;

  // 500msごとにUART送信 ＋ SPI送信
  if (millis() - lastSend >= 500) {
    lastSend = millis();
    
    // UART送信
    sendAll(seq);

    // SPI送信テスト (seq番号の下位8bitを送信)
    uint8_t tx_data = seq & 0xFF;
    Serial.printf("[MASTER] SPI -> Sending: 0x%02X ... ", tx_data);
    uint8_t r = spi_master_txrx(tx_data);
    Serial.printf("Received: 0x%02X\n", r);

    seq++;
  }

  // UARTの高速ポーリング
  for (uint8_t ch = 0; ch < CH_NUM; ch++) {
    pollPort(ch);
  }

  delay(1); // USBシリアルのフリーズ防止
}