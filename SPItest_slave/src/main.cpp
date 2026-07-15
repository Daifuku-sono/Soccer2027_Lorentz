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

uint16_t slave_instructions[4];
pio_program_t slave_program;

void spi_slave_init() {
    // スレーブ用：双方向(Full-Duplex)の命令
    slave_instructions[0] = pio_encode_out(pio_pins, 1);           // 1: MISOに1bit出力
    slave_instructions[1] = pio_encode_wait_gpio(true, SCK_PIN);   // 2: SCKがHIGHになるのを待つ
    slave_instructions[2] = pio_encode_in(pio_pins, 1);            // 3: MOSIから1bit読取
    slave_instructions[3] = pio_encode_wait_gpio(false, SCK_PIN);  // 4: SCKがLOWになるのを待つ
    
    slave_program = { .instructions = slave_instructions, .length = 4, .origin = -1 };
    pio_offset = pio_add_program(pio, &slave_program);
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_offset, pio_offset + 3);

    sm_config_set_out_pins(&c, MISO_PIN, 1);
    sm_config_set_in_pins(&c, MOSI_PIN);
    
    // MSBファースト, オートプル/プッシュ有効, 8bit単位
    sm_config_set_out_shift(&c, false, true, 8); 
    sm_config_set_in_shift(&c, false, true, 8);  
    sm_config_set_clkdiv(&c, 1.0f); // スレーブは全速でマスターを監視

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

void setup() {
    Serial.begin(115200);
    delay(2000);
    spi_slave_init();

    Serial.println("--- Slave Ready ---");

    // 最初にマスターへ返す値 (MSBファーストのため左シフト)
    pio_sm_put(pio, sm, (uint32_t)0x5A << 24); 
}

void loop() {
    // 受信FIFOにデータがあれば読み込む
    if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        uint8_t cmd = (uint8_t)(pio_sm_get(pio, sm) & 0xFF);
        Serial.printf("Received cmd = 0x%02X\n", cmd);

        // 次の通信でマスターに返す値をFIFOに入れておく (+1して返す)
        pio_sm_put(pio, sm, (uint32_t)(cmd + 1) << 24);
    }
}