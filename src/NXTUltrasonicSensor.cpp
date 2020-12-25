#include "NXTUltrasonicSensor.h"
#include <memory>
#include <Wire.h>

constexpr static char *TAG = "NXTUltrasonicSensor";

boolean NXTUltrasonicSensor::sendCommmand(uint8_t command)
{
    pinMode(_clkpin, INPUT); //Needed for writing to work
    digitalWrite(_clkpin, HIGH);

    _wire->beginTransmission(ADDR);
    const auto result = _wire->write(command);
    if (result != 1)
    {
        ESP_LOGE(TAG, "  Error '%d' sending command '%h' to NXT ultrasonic sensor", result, command);
        return false;
    }
    _wire->endTransmission(true);

    _wire->flush();

    delayMicroseconds(60); //Needed for receiving to work
    pinMode(_clkpin, OUTPUT);
    digitalWrite(_clkpin, LOW);
    delayMicroseconds(34);
    pinMode(_clkpin, INPUT);
    digitalWrite(_clkpin, HIGH);
    delayMicroseconds(60);
    return true;
}

void NXTUltrasonicSensor::begin()
{
    pinMode(_clkpin, INPUT);
    digitalWrite(_clkpin, HIGH);
    _wire->begin(_sda, _scl, I2C_FREQ);
}

std::unique_ptr<byte[]> NXTUltrasonicSensor::readVersion()
{
    sendCommmand(READ_VERSION);

    const auto l = _wire->requestFrom(ADDR, 8);

    byte *result = new byte[l];
    std::fill(result, result + l, 0);
    _wire->readBytes(result, l);
    _wire->flush();

    ESP_LOGD(TAG, "NXT ultrasonic sensor version: '%s'", result);

    return std::unique_ptr<byte[]>(result);
}

std::unique_ptr<byte[]> NXTUltrasonicSensor::readProductID()
{
    sendCommmand(READ_ID);

    const auto l = _wire->requestFrom(ADDR, 8);

    byte *result = new byte[l];
    std::fill(result, result + l, 0);
    _wire->readBytes(result, l);
    _wire->flush();

    ESP_LOGD(TAG, "NXT ultrasonic product id: '%s'", result);

    return std::unique_ptr<byte[]>(result);
}

std::unique_ptr<byte[]> NXTUltrasonicSensor::readSensorType()
{
    sendCommmand(READ_TYPE);

    const auto l = _wire->requestFrom(ADDR, 8);

    byte *result = new byte[l];
    std::fill(result, result + l, 0);
    _wire->readBytes(result, l);
    _wire->flush();

    ESP_LOGD(TAG, "NXT ultrasonic sensor type: '%s'", result);

    return std::unique_ptr<byte[]>(result);
}

std::unique_ptr<byte[]> NXTUltrasonicSensor::readMeasurementUnit()
{
    sendCommmand(READ_MEASURMENT_UNIT);

    const auto l = _wire->requestFrom(ADDR, 8);

    byte *result = new byte[l];
    std::fill(result, result + l, 0);
    _wire->readBytes(result, l);
    _wire->flush();

    ESP_LOGD(TAG, "NXT ultrasonic measurment unit: '%s'", result);

    return std::unique_ptr<byte[]>(result);
}

byte NXTUltrasonicSensor::readDistance()
{
    sendCommmand(READ_MEASURMENT0);
    _wire->requestFrom(ADDR, 1);

    byte result;
    _wire->readBytes(&result, 1);
    _wire->flush();
    ESP_LOGD(TAG, "NXT ultrasonic distance: %d cm", result);
    return result;
}