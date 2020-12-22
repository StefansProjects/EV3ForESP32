#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <Arduino.h>
#include "EV3SensorPort.h"
#include "EV3IRSensor.h"

const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int tacho1Pin1 = 18;
const int tacho1Pin2 = 19;

EV3SensorPort sensor(&Serial1, [](int v) { Serial1.begin(v, SERIAL_8N1, tacho1Pin2, tacho1Pin1); });
EV3IRSensor ir1(&sensor);

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
   * This example expects the lego mindestorms IR sensors.
   */
    sensor.begin([](EV3SensorPort *p) {
        Serial.print("Found sensor of type ");
        Serial.println(p->getCurrentConfig()->type, HEX);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        ir1.setMode(EV3IRSensorMode::IR_REMOTE);
        ir1.setOnIRRemote([](uint8_t chan, EV3RemoteState r, EV3RemoteState b) {
            if (r != EV3RemoteState::NONE || b != EV3RemoteState::NONE)
            {
                Serial.print("Channel ");
                Serial.print(chan);
                if (r != EV3RemoteState::NONE)
                {
                    Serial.print(" RED ");
                    Serial.print(r == EV3RemoteState::DOWN ? "DOWN" : "UP");
                }
                if (b != EV3RemoteState::NONE)
                {
                    Serial.print(" BLUE ");
                    Serial.print(b == EV3RemoteState::DOWN ? "DOWN" : "UP");
                }
                Serial.println("");
            }
        });
    });
}

void loop()
{
    delay(50);
}