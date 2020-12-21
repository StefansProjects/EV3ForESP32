#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <Arduino.h>
#include "Lpf2HubEmulation.h"
#include "LegoinoCommon.h"
#include "PowerFunctions.h"
#include <Wire.h>
#include <ESP32Encoder.h>
#include "RegulatedMotor.h"
#include "SoftwareSerial.h"
#include "EV3SensorPort.h"
#include "EV3ColorSensor.h"
#include "EV3IRSensor.h"

static const char *TAG = "main";

const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int tacho1Pin1 = 18;
const int tacho1Pin2 = 19;
const int OE_levelshifter = 23;
// create a hub instance
// Lpf2HubEmulation myEmulatedHub("TrainHub", HubType::POWERED_UP_HUB);

// RegulatedMotor motor(motor1Pin1, motor1Pin2, tacho1Pin1, tacho1Pin2);
// SoftwareSerial swSerial(tacho1Pin2, tacho1Pin1);

EV3SensorPort sensor(&Serial1, [](int v) { Serial1.begin(v, SERIAL_8N1, tacho1Pin2, tacho1Pin1); });
EV3IRSensor ir1(&sensor);
TaskHandle_t sensorHandle;

void writeValueCallback(byte port, byte subcommand, std::string value)
{
  Serial.println("writeValueCallback: ");
  Serial.println(port, HEX);
  Serial.println(subcommand, HEX);
  Serial.println(value.c_str()[0], HEX);

  if (port == 0x00)
  {
    //do something when port 0x00 (A) has received a value
  }

  if (port == 0x01)
  {
    //do something when port 0x01 (B) has received a value
  }

  if (port == 0x32)
  {
    //do something when port 0x32 (LED) has received a value
  }
}

uint8_t current_distance = 0;

void setupSensor1(void *param)
{
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

void setup()
{
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  ESP_LOGD(TAG, "Hello world!!");
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  // OE enable of level shifter
  pinMode(OE_levelshifter, OUTPUT);

  // Set motors to coast mode
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);

  Serial1.begin(2400, SERIAL_8N1, tacho1Pin2, tacho1Pin1);

  // motor.start();
  // swSerial.begin(2400);

  Serial.begin(115200);
  // define the callback function if a write message event on the characteristic occurs
  Serial.println("\nInitalize virtual hub");
  //myEmulatedHub.setWritePortCallback(&writeValueCallback);
  //myEmulatedHub.start();

  digitalWrite(OE_levelshifter, HIGH);

  xTaskCreate(
      &setupSensor1,
      "S1",
      50000,
      nullptr,
      1,
      &sensorHandle // Task handle
  );

  // setupSensor1(nullptr);
}

double Kp = 0, Ki = 0.0, Kd = 0;
int64_t pos = 90;
int console_delay = 0;

void loop()
{
  // if an app is connected, attach some devices on the ports to signalize
  // the app that values could be received/written to that ports
  /*
  if (myEmulatedHub.isConnected && !myEmulatedHub.isPortInitialized)
  {
    delay(1000);
    myEmulatedHub.isPortInitialized = true;
    myEmulatedHub.attachDevice((byte)PoweredUpHubPort::A, DeviceType::TRAIN_MOTOR);
    delay(1000);
    myEmulatedHub.attachDevice((byte)PoweredUpHubPort::LED, DeviceType::HUB_LED);
    delay(1000);
    myEmulatedHub.attachDevice((byte)PoweredUpHubPort::B, DeviceType::TRAIN_MOTOR);
    delay(1000);
  }
  */

  /*
  if (console_delay == 10)
  {
    Serial.print("Position ");
    Serial.println(String((int32_t)motor.getPosition()));
    console_delay = 0;
  }
  console_delay++;

  if (Serial.available() > 0)
  {
    byte rec = Serial.read();
    if (rec == 'q')
    {
      Kp += 5;
    }
    if (rec == 'a')
    {
      Kp -= 5;
    }
    if (rec == 'w')
    {
      Ki += 0.1;
    }
    if (rec == 's')
    {
      Ki -= 0.1;
    }
    if (rec == 'e')
    {
      Kd += 0.1;
    }
    if (rec == 'd')
    {
      Kd -= 0.1;
    }
    if (rec == 'r')
    {
      pos += 60;
    }
    if (rec == 'f')
    {
      pos -= 60;
    }

    Serial.print("Kp ");
    Serial.println(String(Kp));
    Serial.print("Ki ");
    Serial.println(String(Ki));
    Serial.print("Kd ");
    Serial.println(String(Kd));
    Serial.print("Pos ");
    Serial.println(String((int32_t)pos));

    motor.set(RegulationType::POSITION, pos, Kp, Ki, Kd);
  }
  */

  delay(50);
}