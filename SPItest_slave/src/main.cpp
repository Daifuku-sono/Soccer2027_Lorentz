#include <Arduino.h>
#include <SerialPIO.h>
#include "hardware/pio.h"
#include "hardware/gpio.h"

// === UART 設定 ===
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t HW_BAUD  = 115200;
constexpr uint32_t PIO_BAUD = 115200;
constexpr uint8_t CH_NUM = 5;

// ハードウェアUARTのピン (ソフトウェア上はMasterと同じ番号でOK、物理でクロス結線する)
constexpr uint8_t HW_UART0_TX = 0;
constexpr uint8_t HW_UART0_RX = 1;
constexpr uint8_t HW_UART1_TX = 4;
constexpr uint8_t HW_UART1_RX = 5;

// Slave側の PIO UART は TX と RX のピン番号を逆にする (ストレート結線前提)
constexpr uint8_t PIO_TX_PINS[3] = {3, 11, 7};
constexpr uint8_t PIO_RX_PINS[3] = {2, 10, 6};

SerialPIO pioUart0(PIO_TX_PINS[0], PIO_RX_PINS[0]);
SerialPIO pioUart1(PIO_TX_PINS[1], PIO_RX_PINS[1]);
SerialPIO pioUart2(PIO_TX_PINS[2], PIO_RX_PINS[2]);

Stream* const uarts[CH_NUM] = {
  &Serial1,   // ch0
  &Serial2,   // ch1
  &pioUart0,  // ch2
  &pioUart1,  // ch3
  &pioUart2   // ch4
};

struct LineBuffer {
  char buf[96];
  uint8_t len = 0;
};
LineBuffer rxbuf[CH_NUM];
uint32_t recvCount[CH_NUM] = {0, 0, 0, 0, 0};

// === SPI 設定 ===
static constexpr uint SCK_PIN  = 20;
static constexpr uint MOSI_PIN = 21;
static constexpr uint MISO_PIN = 16;
static constexpr uint CS_PIN   = 17;

PIO pio = pio0;
uint sm; 
uint pio_offset;

uint16_t slave_instructions[4];
pio_program_t slave_program;

void spi_slave_init() {
    slave_instructions[0] = pio_encode_out(pio_pins, 1);
    slave_instructions[1] = pio_encode_wait_gpio(true, SCK_PIN);
    slave_instructions[2] = pio_encode_in(pio_pins, 1);
    slave_instructions[3] = pio_encode_wait_gpio(false, SCK_PIN);
    
    slave_program = { .instructions = slave_instructions, .length = 4, .origin = -1 };
    pio_offset = pio_add_program(pio, &slave_program);
    
    sm = pio_claim_unused_sm(pio, true); 

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
  Stream &p = *uarts[ch];
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

  // ハードウェアUARTの初期化 (Masterと同じピン定義を設定)
  Serial1.setTX(HW_UART0_TX);
  Serial1.setRX(HW_UART0_RX);
  Serial1.begin(HW_BAUD);

  Serial2.setTX(HW_UART1_TX);
  Serial2.setRX(HW_UART1_RX);
  Serial2.begin(HW_BAUD);

  // PIO UART初期化
  pioUart0.begin(PIO_BAUD);
  pioUart1.begin(PIO_BAUD);
  pioUart2.begin(PIO_BAUD);

  for (uint8_t i = 0; i < CH_NUM; i++) {
    clearBuffer(rxbuf[i]);
  }

  // SPI初期化
  spi_slave_init();

  Serial.println("SLAVE start (5x UART + 1x SPI)");
  
  pio_sm_put(pio, sm, (uint32_t)0x5A << 24); 
}

void loop() {
  // 5系統UARTの受信ポーリング
  for (uint8_t ch = 0; ch < CH_NUM; ch++) {
    pollPort(ch);
  }

  // SPIの受信ポーリング
  if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
      uint8_t cmd = (uint8_t)(pio_sm_get(pio, sm) & 0xFF);
      Serial.printf("[SLAVE ] SPI <- Received cmd = 0x%02X\n", cmd);

      pio_sm_put(pio, sm, (uint32_t)(cmd + 1) << 24);
  }

  delay(1); 
}