#include <Arduino.h>
#include "Lpf2HubEmulation.h"
#include "LegoinoCommon.h"
#include "PowerFunctions.h"
#include <Wire.h>
#include <ESP32Encoder.h>

const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int tacho1Pin1 = 18;
const int tacho1Pin2 = 19;
int dutyCycle = 200;
// create a hub instance
Lpf2HubEmulation myEmulatedHub("TrainHub", HubType::POWERED_UP_HUB);

ESP32Encoder encoder;

//Specify the links and initial tuning parameters
double Kp = 10, Ki = 0.0, Kd = 1;
int target = 90;
double cumError = 0.0;
double lastError = 0.0;
unsigned long previousTime;

int consoleCnt = 0;

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

void setup()
{
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  digitalWrite(motor1Pin2, LOW);

  ledcSetup(0, 5000, 8);
  ledcAttachPin(motor1Pin1, 0);

  encoder.attachHalfQuad(tacho1Pin1, tacho1Pin2);

  Serial.begin(115200);
  // define the callback function if a write message event on the characteristic occurs
  Serial.println("Initalize virtual hub");
  myEmulatedHub.setWritePortCallback(&writeValueCallback);
  myEmulatedHub.start();
}

void setMotor(boolean forward, uint8_t value)
{
  if (forward)
  {
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(0, (value));
  }
  else
  {
    digitalWrite(motor1Pin2, HIGH);
    ledcWrite(0, (255 - value));
  }
}

void loop()
{
  // if an app is connected, attach some devices on the ports to signalize
  // the app that values could be received/written to that ports
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

  /*
  while (dutyCycle <= 255)
  {
    ledcWrite(0, dutyCycle);
    Serial.print("Forward with duty cycle: ");
    Serial.println(dutyCycle);
    dutyCycle = dutyCycle + 5;
    delay(500);
    Serial.print("Encoder count: ");
    Serial.println(String((int32_t)encoder.getCount()));
  }
  dutyCycle = 0;
  */

  int32_t input = (int32_t)encoder.getCount();
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - previousTime;

  double error = (target - input);
  cumError += error * elapsedTime;
  double rateError = (error - lastError) / elapsedTime;

  int32_t output = Kp * error + Ki * cumError + Kd * rateError;
  if (output > 255)
  {
    output = 255;
  }
  if (output < -255)
  {
    output = -255;
  }

  setMotor(output >= 0, abs(output));
  consoleCnt++;
  lastError = error;
  previousTime = currentTime;
  delay(5);

  if (consoleCnt == 200)
  {
    Serial.print("Encoder count: ");
    Serial.println(String(input));
    Serial.print("Set point: ");
    Serial.println(String(target));
    Serial.print("Forward with duty cycle: ");
    Serial.println(output);

    Serial.print("Error: ");
    Serial.println(error);

    Serial.print("Rate error: ");
    Serial.println(rateError);
    consoleCnt = 0;
  }
}