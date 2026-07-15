#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/gpio.h"

static constexpr uint SCK_PIN  = 20;
static constexpr uint MOSI_PIN = 21;
static constexpr uint MISO_PIN = 18;
static constexpr uint CS_PIN   = 19;

PIO pio = pio0;
uint sm;
uint pio_offset;

// 2命令の簡易PIO SPI風テスト
static const uint16_t master_instructions[] = {
    pio_encode_out(pio_pins, 1) | (0 << 12), // SCK low, MOSI 1bit出力
    pio_encode_in(pio_pins, 1)  | (1 << 12)  // SCK high, MISO 1bit読取
};

static const pio_program_t master_program = {
    .instructions = master_instructions,
    .length = 2,
    .origin = -1
};

bool spi_master_init() {
    sm = pio_claim_unused_sm(pio, true);

    pio_offset = pio_add_program(pio, &master_program);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_offset, pio_offset + 1);

    sm_config_set_sideset_pins(&c, SCK_PIN);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_out_pins(&c, MOSI_PIN, 1);
    sm_config_set_in_pins(&c, MISO_PIN);

    // MSB先頭で8bit送受信
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_in_shift(&c, false, true, 8);

    sm_config_set_clkdiv(&c, 30.0f);

    pio_gpio_init(pio, SCK_PIN);
    pio_gpio_init(pio, MOSI_PIN);
    pio_gpio_init(pio, MISO_PIN);

    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);

    pio_sm_init(pio, sm, pio_offset, &c);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_set_enabled(pio, sm, true);

    return true;
}

uint8_t spi_master_txrx(uint8_t tx) {
    gpio_put(CS_PIN, 0);
    delayMicroseconds(2);

    pio_sm_put_blocking(pio, sm, (uint32_t)tx << 24);

    uint32_t t0 = millis();
    while (pio_sm_is_rx_fifo_empty(pio, sm)) {
        if (millis() - t0 > 100) {
            Serial.println("[Error] Rx Timeout");
            gpio_put(CS_PIN, 1);
            return 0xFF;
        }
        delay(1);
    }

    uint8_t rx = (uint8_t)(pio_sm_get(pio, sm) & 0xFF);

    delayMicroseconds(2);
    gpio_put(CS_PIN, 1);
    return rx;
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println("BOOT");

    if (!spi_master_init()) {
        Serial.println("[Error] PIO init failed");
        while (true) delay(1000);
    }

    Serial.println("--- Master Ready ---");
}

void loop() {
    uint8_t tx_data = 0x12;

    Serial.print("Sending: 0x");
    if (tx_data < 0x10) Serial.print('0');
    Serial.print(tx_data, HEX);
    Serial.print(" ... ");

    uint8_t r = spi_master_txrx(tx_data);

    Serial.print("Received: 0x");
    if (r < 0x10) Serial.print('0');
    Serial.println(r, HEX);

    delay(1000);
}