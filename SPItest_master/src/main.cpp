#include <Arduino.h>
#include <SPI.h>

const int CS_PIN = 17;

int number = 123;

void setup() {

    Serial.begin(115200);

    SPI.setRX(16);
    SPI.setTX(19);
    SPI.setSCK(18);

    SPI.begin();

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    Serial.println("SPI Master Ready");
}

void loop() {

    SPI.beginTransaction(
        SPISettings(1000000, MSBFIRST, SPI_MODE0)
    );

    digitalWrite(CS_PIN, LOW);

    // int送信
    SPI.transfer((uint8_t*)&number, sizeof(number));

    digitalWrite(CS_PIN, HIGH);

    SPI.endTransaction();

    Serial.print("Sent: ");
    Serial.println(number);

    
    delay(1000);
}