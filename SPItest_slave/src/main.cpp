#include <Arduino.h>
#include <SPISlave.h>

SPISettings spisettings(1000000, MSBFIRST, SPI_MODE0);

void onReceive(uint8_t *data, size_t len) {

    if (len == sizeof(int)) {

        int receivedNumber;

        memcpy(&receivedNumber, data, sizeof(int));

        Serial.print("Received Number: ");
        Serial.println(receivedNumber);
    }
}

//確認

void setup() {

    Serial.begin(115200);
    delay(2000);

    SPISlave1.setRX(19);
    SPISlave1.setTX(16);
    SPISlave1.setSCK(18);
    SPISlave1.setCS(17);

    SPISlave1.onDataRecv(onReceive);

    SPISlave1.begin(spisettings);

    Serial.println("SPI Slave Ready");
}

void loop() {
}