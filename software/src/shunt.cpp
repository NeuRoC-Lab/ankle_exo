#include <Arduino.h>
#include <Wire.h>
#include <INA_Series_Sensor.h>

static InaBridge226 sensor("INA232", 0x40); // InaBridge226 is said to support the INA232 model which we use

void setup() {
    Serial.begin(115200);
    sensor.begin(18, 19); // SDA=18,SCL=19
    sensor.setRshunt(0.015);
    sensor.setImax(10.0);   // 10 A max
}

void loop() {
    if (sensor.dataReady()) {
        float voltage = sensor.readBusVoltage();  // V
        float current = sensor.readCurrent();     // A
        float power   = sensor.readPower();       // W

        Serial.print("V="); Serial.print(voltage, 3);
        Serial.print(" I="); Serial.print(current, 4);
        Serial.print(" P="); Serial.println(power, 4);

    }
    delay(100);
}