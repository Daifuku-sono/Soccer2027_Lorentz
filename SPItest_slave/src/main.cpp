#include <Arduino.h>
#include <hardware/pio.h>

// 任意のピンを指定可能（マスター側と揃えてください）
constexpr uint8_t MISO_PIN = 16; 
constexpr uint8_t CS_PIN   = 17;
constexpr uint8_t SCK_PIN  = 18;
constexpr uint8_t MOSI_PIN = 19;

PIO spi_pio = pio0;
uint spi_sm;
uint pio_offset;

// 動的に命令を生成するためのバッファ（SCKピン番号を埋め込むため）
uint16_t slave_instructions[3];
pio_program_t slave_program;

// データ受信時のコールバック関数
void onReceive(uint8_t *data, size_t len) {
    Serial.print("Received: ");
    for (size_t i = 0; i < len; i++) {
        Serial.print((char)data[i]); // 文字として表示
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    // CSピンは入力（プルアップ）
    pinMode(CS_PIN, INPUT_PULLUP);

    // 指定されたSCK_PINに基づいて、PIO命令を動的に組み立てる
    slave_instructions[0] = pio_encode_wait_gpio(true, SCK_PIN);  // 1: SCKがHIGHになるのを待つ
    slave_instructions[1] = pio_encode_in(pio_pins, 1);           // 2: MOSI（in_pin）を1ビット読み込む
    slave_instructions[2] = pio_encode_wait_gpio(false, SCK_PIN); // 3: SCKがLOWになるのを待つ

    slave_program = {
        .instructions = slave_instructions,
        .length = 3,
        .origin = -1,
    };

    // PIOリソースの確保
    spi_sm = pio_claim_unused_sm(spi_pio, true);
    pio_offset = pio_add_program(spi_pio, &slave_program);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_offset, pio_offset + 2);

    // 入力ピン（MOSI）の設定
    sm_config_set_in_pins(&c, MOSI_PIN);
    
    // シフトレジスタ設定（MSBファースト、自動プッシュ有効、8ビット単位）
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_in_shift(&c, false, true, 8);

    // マスターの高速なクロックを取りこぼさないよう、システム全速（ディバイダ1.0）で動作
    sm_config_set_clkdiv(&c, 1.0f);

    // GPIOをPIOに設定
    pio_gpio_init(spi_pio, MOSI_PIN);
    pio_gpio_init(spi_pio, SCK_PIN);
    pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, MOSI_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, SCK_PIN, 1, false);

    // 開始
    pio_sm_init(spi_pio, spi_sm, pio_offset, &c);
    pio_sm_set_enabled(spi_pio, spi_sm, true);

    Serial.println("PIO SPI Slave Ready");
}

void loop() {
    static uint8_t rx_buf[64];
    static size_t len = 0;

    // CSピンがLOW（マスターが通信中）の間、FIFOからデータを読み出す
    if (digitalRead(CS_PIN) == LOW) {
        while (!pio_sm_is_rx_fifo_empty(spi_pio, spi_sm) && len < sizeof(rx_buf)) {
            rx_buf[len++] = (uint8_t)(pio_sm_get(spi_pio, spi_sm) & 0xFF);
        }
    } 
    // CSピンがHIGH（通信終了）になった瞬間に、溜まったデータを一括処理
    else {
        if (len > 0) {
            onReceive(rx_buf, len);
            len = 0; // バッファをリセット
        }
    }
}