#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>

constexpr uint8_t MISO_PIN = 18;
constexpr uint8_t CS_PIN   = 19;
constexpr uint8_t SCK_PIN  = 20;
constexpr uint8_t MOSI_PIN = 21;

PIO spi_pio = pio0;
uint spi_sm;
uint pio_offset;

static const uint16_t master_instructions[] = {
    0x6101, // 0: out    pins, 1         side 0 [1]  
    0x2141  // 1: in     pins, 1         side 1 [1]  
};

static const pio_program_t master_program = {
    .instructions = master_instructions,
    .length = 2,
    .origin = -1,
};

uint8_t pio_spi_transfer(uint8_t data) {
    pio_sm_put_blocking(spi_pio, spi_sm, (uint32_t)data << 24);
    return (uint8_t)(pio_sm_get_blocking(spi_pio, spi_sm) & 0xFF);
}

void setup() {
    Serial.begin(115200);
    // PlatformIO等でシリアル接続を待つための処理
    uint32_t t = millis();
   

    Serial.println("\n[MASTER] --- System Booting ---");
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    spi_sm = pio_claim_unused_sm(spi_pio, true);
    pio_offset = pio_add_program(spi_pio, &master_program);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_offset, pio_offset + 1);

    sm_config_set_out_pins(&c, MOSI_PIN, 1);
    sm_config_set_in_pins(&c, MISO_PIN);
    sm_config_set_sideset_pins(&c, SCK_PIN);
    sm_config_set_sideset(&c, 1, false, false); 

    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_in_shift(&c, false, true, 8);

    float div = (float)clock_get_hz(clk_sys) / (1000000 * 4);
    sm_config_set_clkdiv(&c, div);

    pio_gpio_init(spi_pio, MOSI_PIN);
    pio_gpio_init(spi_pio, SCK_PIN);
    pio_gpio_init(spi_pio, MISO_PIN);

    pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, MOSI_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, SCK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, MISO_PIN, 1, false);

    pio_sm_init(spi_pio, spi_sm, pio_offset, &c);
    pio_sm_set_enabled(spi_pio, spi_sm, true);

    Serial.println("[MASTER] PIO SPI Ready");
}

void loop() {
    char data[] = "hello";
    
    Serial.println("\n[MASTER] === Starting Transaction ===");
    
    digitalWrite(CS_PIN, LOW); 
    Serial.println("[MASTER] CS -> LOW (通信開始)");

    for (int i = 0; i < 5; i++) {
        Serial.print("[MASTER] Sending: '");
        Serial.print(data[i]);
        Serial.print("' ... ");
        
        uint8_t rx = pio_spi_transfer(data[i]);
        
        Serial.print("Done. RX: 0x");
        Serial.println(rx, HEX);
    }

    digitalWrite(CS_PIN, HIGH); 
    Serial.println("[MASTER] CS -> HIGH (通信終了)");
    Serial.println("[MASTER] ============================");

    delay(2000);
}