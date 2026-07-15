#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/gpio.h"

static constexpr uint SCK_PIN  = 20;
static constexpr uint MOSI_PIN = 21;
static constexpr uint MISO_PIN = 18;
static constexpr uint CS_PIN   = 19;

PIO pio = pio0;
const uint sm = 0;
uint pio_offset;

// 【修正ポイント】16進数の手打ちをやめ、SDKの関数で確実な命令を作る
static const uint16_t master_instructions[] = {
    pio_encode_out(pio_pins, 1) | (0 << 12), // 0: MOSIに1bit出力 + SCKをLOW
    pio_encode_in(pio_pins, 1)  | (1 << 12)  // 1: MISOから1bit読取 + SCKをHIGH
};
static const pio_program_t master_program = { .instructions = master_instructions, .length = 2, .origin = -1 };

void spi_master_init() {
    pio_offset = pio_add_program(pio, &master_program);
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_offset, pio_offset + 1);

    sm_config_set_sideset_pins(&c, SCK_PIN);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_out_pins(&c, MOSI_PIN, 1);
    sm_config_set_in_pins(&c, MISO_PIN);

    // MSBファースト, オートプル/プッシュ有効, 8bit単位
    sm_config_set_out_shift(&c, false, true, 8); 
    sm_config_set_in_shift(&c, false, true, 8);  
    sm_config_set_clkdiv(&c, 30.0f); // クロック速度調整

    pio_gpio_init(pio, SCK_PIN);
    pio_gpio_init(pio, MOSI_PIN);
    pio_gpio_init(pio, MISO_PIN);

    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);

    pio_sm_set_consecutive_pindirs(pio, sm, SCK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, MOSI_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, MISO_PIN, 1, false);

    // 万が一ゴミが残っていた時のためにFIFOをクリア
    pio_sm_clear_fifos(pio, sm);

    pio_sm_init(pio, sm, pio_offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

uint8_t spi_master_txrx(uint8_t tx) {
    gpio_put(CS_PIN, 0);
    delayMicroseconds(2);

    // MSBファーストなので上位ビットに寄せて送信
    pio_sm_put(pio, sm, (uint32_t)tx << 24);

    // フリーズ防止のタイムアウト付き受信
    uint32_t t = millis();
    while (pio_sm_is_rx_fifo_empty(pio, sm)) {
        if (millis() - t > 100) {
            Serial.println("[Error] Rx Timeout!");
            gpio_put(CS_PIN, 1);
            return 0xFF; // エラー時は0xFFを返す
        }
    }
    uint8_t rx = (uint8_t)(pio_sm_get(pio, sm) & 0xFF);

    delayMicroseconds(2);
    gpio_put(CS_PIN, 1);
    return rx;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("--- Master Ready ---");
    spi_master_init();
}

void loop() {
    uint8_t tx_data = 0x12;
    Serial.printf("Sending: 0x%02X ... ", tx_data);
    
    uint8_t r = spi_master_txrx(tx_data);
    
    Serial.printf("Received: 0x%02X\n", r);
    delay(1000);
    tx_data = 0x24;
    Serial.printf("Sending: 0x%02X ... ", tx_data);
    
    r = spi_master_txrx(tx_data);
    
    Serial.printf("Received: 0x%02X\n", r);
    delay(1000);
}