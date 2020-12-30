#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <Arduino.h>
#include "EV3SensorPort.h"
#include "EV3ColorSensor.h"
#include <SoftwareSerial.h>

const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int tacho1Pin1 = 18;
const int tacho1Pin2 = 19;

SoftwareSerial swSer;
EV3SensorPort sensor(&swSer, [](int v) { swSer.end(); swSer.begin(v, SWSERIAL_8N1, tacho1Pin2, tacho1Pin1); });
// EV3SensorPort sensor(&Serial1, [](int v) { Serial1.begin(v, SERIAL_8N1, tacho1Pin2, tacho1Pin1); });
EV3ColorSensor color(&sensor);
EV3ColorSensorColor prev = EV3ColorSensorColor::NONE;

void setup()
{
    // Configure motor outputs
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);

    // Set motor outputs to coast mode
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);

    // Serial comm to the host machine
    Serial.begin(115200);

    /**
     * This example expects the lego mindestorms color sensors.
     */
    sensor.begin([](EV3SensorPort *p) {
        Serial.print("Found sensor of type ");
        Serial.println(p->getCurrentConfig()->type, HEX);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        color.setMode(EV3ColorSensorMode::COL_COLOR);
        color.setOnColColor([](EV3ColorSensorColor c) {
            if (c != prev)
            {
                prev = c;
                Serial.print("Color ");
                writeEV3ColorSensorColorToStream(c, &Serial);
                Serial.println();
            }
        });
    });
}

void loop()
{
    delay(50);
}