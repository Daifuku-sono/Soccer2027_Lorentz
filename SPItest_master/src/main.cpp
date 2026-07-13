#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>

// 任意のピンを指定可能（配線に合わせて自由に変更してください）
constexpr uint8_t MISO_PIN = 16;
constexpr uint8_t CS_PIN   = 17;
constexpr uint8_t SCK_PIN  = 18;
constexpr uint8_t MOSI_PIN = 19;

PIO spi_pio = pio0;
uint spi_sm;
uint pio_offset;

// SPI Mode 0 (CPOL=0, CPHA=0) を表現するPIO命令（2命令）
static const uint16_t master_instructions[] = {
    0x6101, // 0: out    pins, 1         side 0 [1]  (MOSI出力 + クロックLOW)
    0x2141  // 1: in     pins, 1         side 1 [1]  (MISOキャプチャ + クロックHIGH)
};

static const pio_program_t master_program = {
    .instructions = master_instructions,
    .length = 2,
    .origin = -1,
};

// PIO経由で1バイトを送受信する関数
uint8_t pio_spi_transfer(uint8_t data) {
    // MSBファーストのため、32ビットの最上位（ビット31〜24）にデータを配置して送信
    pio_sm_put_blocking(spi_pio, spi_sm, (uint32_t)data << 24);
    
    // 受信バッファからデータが戻るのを待つ（下位8ビットに入ります）
    return (uint8_t)(pio_sm_get_blocking(spi_pio, spi_sm) & 0xFF);
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    // CSピンの初期化
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    // 空いているステートマシン(SM)を確保し、プログラムを登録
    spi_sm = pio_claim_unused_sm(spi_pio, true);
    pio_offset = pio_add_program(spi_pio, &master_program);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_offset, pio_offset + 1);

    // 各ピンの役割をPIOに割り当て
    sm_config_set_out_pins(&c, MOSI_PIN, 1);
    sm_config_set_in_pins(&c, MISO_PIN);
    sm_config_set_sideset_pins(&c, SCK_PIN);
    sm_config_set_sideset(&c, 1, false, false); // 1ビットのサイドセット（クロック用）

    // シフトレジスタ設定（MSBファースト、自動プッシュ/プル有効、8ビット単位）
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_in_shift(&c, false, true, 8);

    // クロック周波数の設定 (1MHz) ※1ビットの処理に4つのPIOサイクルを使用
    float div = (float)clock_get_hz(clk_sys) / (1000000 * 4);
    sm_config_set_clkdiv(&c, div);

    // GPIOをPIO制御に切り替え
    pio_gpio_init(spi_pio, MOSI_PIN);
    pio_gpio_init(spi_pio, SCK_PIN);
    pio_gpio_init(spi_pio, MISO_PIN);

    // ピンの入出力方向を設定
    pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, MOSI_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, SCK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, MISO_PIN, 1, false);

    // PIOの有効化
    pio_sm_init(spi_pio, spi_sm, pio_offset, &c);
    pio_sm_set_enabled(spi_pio, spi_sm, true);

    Serial.println("PIO SPI Master Ready");
}

void loop() {
    char data[] = "hello";
    Serial.println("Sending: hello");

    digitalWrite(CS_PIN, LOW); // 通信開始

    for (int i = 0; i < 5; i++) {
        pio_spi_transfer(data[i]);
    }

    digitalWrite(CS_PIN, HIGH); // 通信終了

    delay(1000);
}