#include <Arduino.h>
#include <memory>
#include <Wire.h>

#ifndef NXTUltrasonicSensor_h
#define NXTUltrasonicSensor_h

/*
 * Access a Lego Mindstorms NXT ultrasonic sensors.
 * The sensor has a bug, which requires an additional SCL wiggle after each command.
 * 
 * Wires on NXT jack plug.
 * Wire colours may vary. Pin 1 is always end nearest latch.
 * 1 White +9V
 * 2 Black GND
 * 3 Red GND
 * 4 Green +5V
 * 5 Yellow SCL - also connect clkpin to give a extra low impuls
 * 6 Blue SDA
 * Do not use i2c pullup resistor - already provided within sensor.
 * All read operations require restart after sending the command!

@see http://blog.tkjelectronics.dk/2011/10/nxt-shield-ver2/
*/
class NXTUltrasonicSensor
{
private:
    const static byte ADDR = 0x02 >> 1;
    const static uint I2C_FREQ = 9768; // 9600 by spec, but minium freq is 9768

    // Commands for reading constans
    const static byte READ_VERSION = 0x00; // = 'V1.0'
    const static byte READ_ID = 0x08;      // = 'LEGO'
    const static byte READ_TYPE = 0x10;    // = 'Sonar'
    const static byte READ_FACTORY_ZERO = 0x11;
    const static byte READ_FACTORY_SCALE_FACTOR = 0x12;
    const static byte READ_FACTORY_SCALE_DIVISOR = 0x13;
    const static byte READ_MEASURMENT_UNIT = 0x14; // = 10E-2m

    // Commands for reading variables
    const static byte READ_WRITE_CONTINOUS_MEASURMENT_INTERVAAL = 0x40;
    const static byte READ_WRITE_COMMAND_STATE = 0x41;
    const static byte READ_MEASURMENT0 = 0x42;
    const static byte READ_MEASURMENT1 = READ_MEASURMENT0 + 1;
    const static byte READ_MEASURMENT2 = READ_MEASURMENT0 + 2;
    const static byte READ_MEASURMENT3 = READ_MEASURMENT0 + 3;
    const static byte READ_MEASURMENT4 = READ_MEASURMENT0 + 4;
    const static byte READ_MEASURMENT5 = READ_MEASURMENT0 + 5;
    const static byte READ_MEASURMENT6 = READ_MEASURMENT0 + 6;
    const static byte READ_MEASURMENT7 = READ_MEASURMENT0 + 7;
    const static byte READ_WRITE_ACTUAL_ZERO = 0x50;
    const static byte READ_WRITE_SCALE_FACTOR = 0x51;
    const static byte READ_WRITE_SCALE_DIVISOR = 0x52;

    // Command states for 0x41
    const static byte CMD_OFF = 0x0;
    const static byte CMD_SINGLE_SHOT = 0x01;
    const static byte CMD_CONTINOUS_MEASURMENT = 0x02; // Default!
    const static byte CMD_EVENT_CAPTURE = 0x03;
    const static byte CMD_WARM_RESET = 0x04;

    const uint8_t _sda;
    const uint8_t _scl;
    const uint8_t _clkpin;
    TwoWire *_wire;

    /**
     * Sends a command to the sensor. Performs the additional SCL wiggle necessary to support the broken implementation.
     */
    boolean sendCommmand(uint8_t command);

public:
    /**
     * Creates an NXTUltrasonicSensor using Wire hardware twi.
     */
    NXTUltrasonicSensor(uint8_t sda, uint8_t scl, uint8_t clkpin) : _sda(sda), _scl(scl), _clkpin(clkpin), _wire(&Wire)
    {
    }

    /**
     * Creates an NXTUltrasonicSensor using the given TwoWire instance.
     */
    NXTUltrasonicSensor(TwoWire *wire, uint8_t sda, uint8_t scl, uint8_t clkpin) : _sda(sda), _scl(scl), _clkpin(clkpin), _wire(wire)
    {
    }

    /**
     * Initalizes all pins and the TWI transmission.
     */
    void begin();

    /**
     * Reads the sensor version as string (V1.0)
     */
    std::unique_ptr<byte[]> readVersion();

    /**
     * Reads the product ID (LEGO)
     */
    std::unique_ptr<byte[]> readProductID();

    /**
     * Reads the sensor type (Sonar)
     */
    std::unique_ptr<byte[]> readSensorType();
    /**
     * Reads the measurment unit as string. (10E-2m)
     */
    std::unique_ptr<byte[]> readMeasurementUnit();

    /**
     * Reads the current distance in cm (see readMeasurementUnit() for actual unit)
     */
    byte readDistance();
};

#endif