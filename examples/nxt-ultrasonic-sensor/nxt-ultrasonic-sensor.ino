#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <Arduino.h>
#include "NXTUltrasonicSensor.h"

const int motor1Pin1 = 27; // 9V Power for US sensor
const int motor1Pin2 = 26;
const int tacho1Pin1 = 18; // = SCL
const int tacho1Pin2 = 19; // = SDA
const int sclPin = 5;      // Additional SCL pin for sensor

NXTUltrasonicSensor s1 = NXTUltrasonicSensor(tacho1Pin2, tacho1Pin1, sclPin);

void setup()
{
    // Configure motor outputs
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);

    // Activate powersupply for ultrasonic sensor
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);

    // Serial comm to the host machine
    Serial.begin(115200);

    // Start the sensor
    s1.begin();
}

void loop()
{

    s1.readVersion();
    delay(10);

    s1.readProductID();
    delay(10);
    s1.readMeasurementUnit();
    delay(10);
    s1.readSensorType();
    delay(10);
    s1.readDistance();
    delay(10);
    delay(550);
}