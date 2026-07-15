#include <Arduino.h>
#include <SPISlave.h>
#include <hardware/pio.h>

constexpr uint8_t MISO_PIN = 18; 
constexpr uint8_t CS_PIN   = 19;
constexpr uint8_t SCK_PIN  = 20;
constexpr uint8_t MOSI_PIN = 21;

PIO spi_pio = pio0;
uint spi_sm;
uint pio_offset;

uint16_t slave_instructions[3];
pio_program_t slave_program;

SPISettings spisettings(1000000, MSBFIRST, SPI_MODE0);

void onReceive(uint8_t *data, size_t len) {
    Serial.print("[SLAVE] Callback triggered. Data: ");
    for (size_t i = 0; i < len; i++) {
        Serial.print((char)data[i]); 
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while(!Serial && (millis() - t < 3000)) { delay(10); }

    Serial.println("\n[SLAVE] --- System Booting ---");

    pinMode(CS_PIN, INPUT_PULLUP);

    slave_instructions[0] = pio_encode_wait_gpio(true, SCK_PIN);  
    slave_instructions[1] = pio_encode_in(pio_pins, 1);           
    slave_instructions[2] = pio_encode_wait_gpio(false, SCK_PIN); 

    slave_program = {
        .instructions = slave_instructions,
        .length = 3,
        .origin = -1,
    };

    spi_sm = pio_claim_unused_sm(spi_pio, true);
    pio_offset = pio_add_program(spi_pio, &slave_program);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pio_offset, pio_offset + 2);

    sm_config_set_in_pins(&c, MOSI_PIN);
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_in_shift(&c, false, true, 8);
    sm_config_set_clkdiv(&c, 1.0f);

    pio_gpio_init(spi_pio, MOSI_PIN);
    pio_gpio_init(spi_pio, SCK_PIN);
    pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, MOSI_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(spi_pio, spi_sm, SCK_PIN, 1, false);

    pio_sm_init(spi_pio, spi_sm, pio_offset, &c);
    pio_sm_set_enabled(spi_pio, spi_sm, true);

    Serial.println("[SLAVE] PIO SPI Ready. Waiting for CS to go LOW...");
}

void loop() {
    static uint8_t rx_buf[64];
    static size_t len = 0;
    
    // エッジ検出用の変数
    static int last_cs_state = HIGH;
    int current_cs_state = digitalRead(CS_PIN);

    // CSがHIGH -> LOWになった瞬間（マスターが通信開始した）
    if (current_cs_state == LOW && last_cs_state == HIGH) {
        Serial.println("\n[SLAVE] >> CS went LOW. Transaction Started.");
    }

    // CSがLOWの間
    if (current_cs_state == LOW) {
        while (!pio_sm_is_rx_fifo_empty(spi_pio, spi_sm) && len < sizeof(rx_buf)) {
            rx_buf[len] = (uint8_t)(pio_sm_get(spi_pio, spi_sm) & 0xFF);
            Serial.print("[SLAVE] Read FIFO: 0x");
            Serial.println(rx_buf[len], HEX);
            len++;
        }
    } 
    // CSがLOW -> HIGHに戻った瞬間（マスターが通信終了した）
    else if (current_cs_state == HIGH && last_cs_state == LOW) {
        Serial.println("[SLAVE] << CS went HIGH. Transaction Ended.");
        if (len > 0) {
            Serial.print("[SLAVE] Total bytes received: ");
            Serial.println(len);
            onReceive(rx_buf, len);
            len = 0; 
        } else {
            Serial.println("[SLAVE] WARNING: CS went HIGH but no data was received!");
        }
    }

    // 状態を更新
    last_cs_state = current_cs_state;
}