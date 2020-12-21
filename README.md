# EV3ForESP32

*Disclaimer*: LEGO® is a trademark of the LEGO Group of companies which does not sponsor, authorize or endorse this project.

__EV3ForESP32__ is an ESP32 Arduino libary for using both EV3 sensors and servo motors with a ESP32. 

## Hardware requirments

The library is implemented for __ESP32__ board. It may work with other Arduino compatible boards but currently relies on a __FreeRTOS__ implementation to be present.
Further hardware is required to drive motors and retrieve data from sensors. It is important to add the hardware because the sensor work on 5V but the ESP32 requires 3.3V at its inputs!

The levelshifter to convert both quadrature encoder signals and uart communiction between 5V and 3V3 is a simple circuit using a mosfet (for prototypes you can use a 2N7000) and two resistor per line.

![Sensor connection scheme][https://github.com/StefansProjects/EV3ForESP32/raw/master/doc/sensor_connection.png]

To drive the motors I use a `DRV8833 Dual H-Bridge Motor Driver` (you can find simple boards for prototyping on the usual platforms). Its limits for both voltage and current are close to the ones for the EV3 motors, so its a perfect fit to drive two EV3 motors. But you can use any other motor driver.

You can connect both the inputs of the h-bridge and the 3V3 outputs of the level-shift circuit to any GPIO of the ESP32. 



## Quickstart

### EV3 regulated motor

To use a motor simply create an instance of `EV3RegulatedMotor` with the GPIOs of the h-bridge and sensor.

```C++
EV3RegulatedMotor motor(ESP32_PWM1, ESP32_PWM2, ESP32_TACHO1, ESP32_TACHO2);
```

In the `setup()` function just start the motor control

```C++
motor.start();
```

Then you can simply set a direction and simple PWM value by using

```C++
void setPWM(boolean direction, uint8_t value);
```

Or you can use the advanced features by retrieving the current positon or setting a target position the motor will head for using PID control.

```C++
 int64_t getPosition();
 void setPosition(int64_t position);
```

### EV3 sensor

**WORK IN PROGRESS**
