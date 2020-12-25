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
  // Wire.begin(tacho1Pin2, tacho1Pin1, 9768);
}

void loop()
{

  /*
  Wire.beginTransmission(0x02 >> 1);
  Wire.write(0x0);
  Wire.endTransmission(true);
  Wire.flush();

  delayMicroseconds(60); //Needed for receiving to work
  pinMode(sclPin, OUTPUT);
  digitalWrite(sclPin, LOW);
  delayMicroseconds(34);
  pinMode(sclPin, INPUT);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(60);

  const auto l = Wire.requestFrom(0x02 >> 1, 8);
  ESP_LOGV(TAG, "Read %d bytes from slave", l);

  byte *result = new byte[8];
  std::fill(result, result + 8, 0);
  Wire.readBytes(result, l);

  ESP_LOGV(TAG, "NXT ultrasonic sensor version: '%s'", result);
  */
  s1.readVersion();
  s1.readProductID();
  s1.readMeasurementUnit();
  s1.readSensorType();
  s1.readDistance();
  delay(550);
}