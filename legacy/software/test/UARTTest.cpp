//
// Created by Oscar Tesniere on 20/07/2026.
//

#include <Arduino.h>

#if defined(PLATFORM_TEENSY41)
    #define SERIAL_FWD Serial8
#elif defined(PLATFORM_RENESAS)
    #define SERIAL_FWD Serial1
#else
    #error "Unsupported platform: define PLATFORM_TEENSY or PLATFORM_RENESAS"
#endif

constexpr uint32_t USB_BAUD_RATE = 115200;
constexpr uint32_t FORWARD_BAUD_RATE = 9600;

void setup() {
    Serial.begin(USB_BAUD_RATE);
    SERIAL_FWD.begin(FORWARD_BAUD_RATE);
}

void loop() {
    // External UART -> USB serial
    while (SERIAL_FWD.available() > 0) {
        Serial.write(SERIAL_FWD.read());
    }

    // USB serial -> external UART
    while (Serial.available() > 0) {
        SERIAL_FWD.write(Serial.read());
    }
}