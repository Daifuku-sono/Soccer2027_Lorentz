#include <Arduino.h>
#include <SerialPIO.h>
#include "hardware/pio.h"
#include "hardware/gpio.h"

// === UART 設定 ===
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t HW_BAUD  = 115200;
constexpr uint32_t PIO_BAUD = 115200;
constexpr uint8_t CH_NUM = 5; // 合計5チャンネル

// ハードウェアUARTのピン (クロス結線前提)
constexpr uint8_t HW_UART0_TX = 0;
constexpr uint8_t HW_UART0_RX = 1;
constexpr uint8_t HW_UART1_TX = 4;
constexpr uint8_t HW_UART1_RX = 5;

// PIO UARTのピン (ストレート結線前提)
constexpr uint8_t PIO_TX_PINS[3] = {2, 10, 6};
constexpr uint8_t PIO_RX_PINS[3] = {3, 11, 7};

SerialPIO pioUart0(PIO_TX_PINS[0], PIO_RX_PINS[0]);
SerialPIO pioUart1(PIO_TX_PINS[1], PIO_RX_PINS[1]);
SerialPIO pioUart2(PIO_TX_PINS[2], PIO_RX_PINS[2]);

// すべてのUARTをStream型の配列にまとめて共通処理する
Stream* const uarts[CH_NUM] = {
  &Serial1,   // ch0: 組み込みUART0
  &Serial2,   // ch1: 組み込みUART1
  &pioUart0,  // ch2: PIO UART
  &pioUart1,  // ch3: PIO UART
  &pioUart2   // ch4: PIO UART
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
uint sm;
uint pio_offset;

static const uint16_t master_instructions[] = {
    pio_encode_out(pio_pins, 1) | (0 << 12),
    pio_encode_in(pio_pins, 1)  | (1 << 12)
};
static const pio_program_t master_program = { .instructions = master_instructions, .length = 2, .origin = -1 };

void spi_master_init() {
    pio_offset = pio_add_program(pio, &master_program);
    sm = pio_claim_unused_sm(pio, true); 

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
  Stream &p = *uarts[ch];
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

  // ハードウェアUARTの初期化
  Serial1.setTX(HW_UART0_TX);
  Serial1.setRX(HW_UART0_RX);
  Serial1.begin(HW_BAUD);

  Serial2.setTX(HW_UART1_TX);
  Serial2.setRX(HW_UART1_RX);
  Serial2.begin(HW_BAUD);

  // PIO UARTの初期化
  pioUart0.begin(PIO_BAUD);
  pioUart1.begin(PIO_BAUD);
  pioUart2.begin(PIO_BAUD);

  for (uint8_t i = 0; i < CH_NUM; i++) {
    clearBuffer(rxbuf[i]);
  }

  // SPI初期化
  spi_master_init();
  
  Serial.println("MASTER start (5x UART + 1x SPI)");
}

void loop() {
  static uint32_t lastSend = 0;
  static uint32_t seq = 0;

  if (millis() - lastSend >= 500) {
    lastSend = millis();
    
    // UART送信
    sendAll(seq);

    // SPI送信テスト
    uint8_t tx_data = seq & 0xFF;
    Serial.printf("[MASTER] SPI -> Sending: 0x%02X ... ", tx_data);
    uint8_t r = spi_master_txrx(tx_data);
    Serial.printf("Received: 0x%02X\n", r);

    seq++;
  }

  // 5系統すべてのUART受信処理
  for (uint8_t ch = 0; ch < CH_NUM; ch++) {
    pollPort(ch);
  }

  delay(1); 
}