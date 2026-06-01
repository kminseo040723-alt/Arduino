#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
    Serial.begin(9600);
    Wire.begin();

    Serial.println("VL53L0X test");

    if (!lox.begin()) {
        Serial.println("VL53L0X not found");
        while (1);
    }

    Serial.println("VL53L0X ready");
}

void loop() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    
    Serial.print("Status: ");
    Serial.print(measure.RangeStatus);
    Serial.print(" Distance: ");
    Serial.println(measure.RangeMilliMeter);

    delay(300);
}