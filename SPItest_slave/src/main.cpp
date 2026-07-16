#include <Arduino.h>
#include <SerialPIO.h>
#include "hardware/pio.h"
#include "hardware/gpio.h"

// === UART 設定 ===
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t PIO_BAUD = 115200;
constexpr uint8_t CH_NUM = 3; 

// Slave 側は Master と TX/RX を逆にする
constexpr uint8_t TX_PINS[CH_NUM] = {3, 11, 7};
constexpr uint8_t RX_PINS[CH_NUM] = {2, 10, 6};

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

// === SPI 設定 ===
static constexpr uint SCK_PIN  = 20;
static constexpr uint MOSI_PIN = 21;
static constexpr uint MISO_PIN = 18;
static constexpr uint CS_PIN   = 19;

PIO pio = pio0;
uint sm; // 衝突を防ぐため定数から変数に変更
uint pio_offset;

uint16_t slave_instructions[4];
pio_program_t slave_program;

void spi_slave_init() {
    slave_instructions[0] = pio_encode_out(pio_pins, 1);           // 1: MISOに1bit出力
    slave_instructions[1] = pio_encode_wait_gpio(true, SCK_PIN);   // 2: SCKがHIGHになるのを待つ
    slave_instructions[2] = pio_encode_in(pio_pins, 1);            // 3: MOSIから1bit読取
    slave_instructions[3] = pio_encode_wait_gpio(false, SCK_PIN);  // 4: SCKがLOWになるのを待つ
    
    slave_program = { .instructions = slave_instructions, .length = 4, .origin = -1 };
    pio_offset = pio_add_program(pio, &slave_program);
    
    sm = pio_claim_unused_sm(pio, true); // 空いているステートマシンを安全に確保

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_offset, pio_offset + 3);

    sm_config_set_out_pins(&c, MISO_PIN, 1);
    sm_config_set_in_pins(&c, MOSI_PIN);
    
    sm_config_set_out_shift(&c, false, true, 8); 
    sm_config_set_in_shift(&c, false, true, 8);  
    sm_config_set_clkdiv(&c, 1.0f);

    pio_gpio_init(pio, SCK_PIN);
    pio_gpio_init(pio, MOSI_PIN);
    pio_gpio_init(pio, MISO_PIN);

    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_IN);
    gpio_pull_up(CS_PIN);

    pio_sm_set_consecutive_pindirs(pio, sm, SCK_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, MOSI_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, MISO_PIN, 1, true);

    pio_sm_init(pio, sm, pio_offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void clearBuffer(LineBuffer &b) {
  b.len = 0;
  b.buf[0] = '\0';
}

void reply(uint8_t ch, const char* line) {
  recvCount[ch]++;
  Serial.print("[SLAVE ] UART ch");
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
      if (rxbuf[ch].len > 0) reply(ch, rxbuf[ch].buf);
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

void setup() {
  Serial.begin(USB_BAUD);
  delay(1500);

  // UART初期化
  for (uint8_t i = 0; i < CH_NUM; i++) {
    uarts[i]->begin(PIO_BAUD);
    clearBuffer(rxbuf[i]);
  }

  // SPI初期化
  spi_slave_init();

  Serial.println("SLAVE start");
  
  // 最初にマスターへ返すダミー値 (MSBファーストのため左シフト)
  pio_sm_put(pio, sm, (uint32_t)0x5A << 24); 
}

void loop() {
  // 1. UARTの受信ポーリング
  for (uint8_t ch = 0; ch < CH_NUM; ch++) {
    pollPort(ch);
  }

  // 2. SPIの受信ポーリング
  if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
      uint8_t cmd = (uint8_t)(pio_sm_get(pio, sm) & 0xFF);
      Serial.printf("[SLAVE ] SPI <- Received cmd = 0x%02X\n", cmd);

      // 次の通信でマスターに返す値をFIFOに入れておく (+1して返す)
      pio_sm_put(pio, sm, (uint32_t)(cmd + 1) << 24);
  }

  delay(1); // USBシリアルのフリーズ防止
}