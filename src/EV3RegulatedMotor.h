#include <Arduino.h>
#include <ESP32Encoder.h>

#ifndef EV3RegulatedMotor_h
#define EV3RegulatedMotor_h

/**
 * Types of PID regulations available for a regulated motor.
 */
enum struct EV3RegulationType : uint8_t
{
    NONE,
    POSITION,
    SPEED
};

/**
 * State of the motor regulation.
 */
enum struct EV3MotorState
{
    COAST,
    FORWARD,
    REVERSE,
    BRAKE
};

class EV3RegulatedMotor
{
private:
    // Pins for the motors
    int _motorPin1;
    int _motorPin2;
    int _tachoPin1;
    int _tachoPin2;

    // We use 8 bit pwm, defines maximum values
    constexpr static double PWM_MAX = 255;
    constexpr static double PWM_MIN = -1 * PWM_MAX;

    // PID control variables for position control.
    const double KP_POS_CTRL = 25.0;
    const double KI_POS_CTRL = -2.0;
    const double KD_POS_CTRL = 20.0;

    // PID control parameters
    double _Kp = KP_POS_CTRL, _Ki = KI_POS_CTRL, _Kd = KD_POS_CTRL;
    // PID error sum
    double _outputSum = 0.0;

    // Quadrature encoder used to determine position
    ESP32Encoder encoder;

    // Prevent simultanious access quadrature encoder
    SemaphoreHandle_t _positionMutex;
    // Prevent simultanious access to speed variable
    SemaphoreHandle_t _speedMutex;
    // Prevent simultanious access to pid parameters
    SemaphoreHandle_t _pidMutex;
    // Handle for the motor control task.
    TaskHandle_t _motorCtrlHandle = NULL;

    // Type of PID regulration currently active
    EV3RegulationType _reg_type;
    // Target value for the PID regulration
    int64_t _reg_target;

    // Previous position after before the last call
    int64_t _prev_pos = 0;

    //Time of the last motor control loop
    unsigned long _prev_time = 0;

    //Current speed of the motor
    double _speed = 0;
    // Speed in last regulation cycle
    double _prev_speed = 0;
    // Current motor stae.
    EV3MotorState _motorState = EV3MotorState::COAST;

    /*
     * Utility method to bind a class method to a FreeRTOS task.
     * 
     * @see https://www.freertos.org/FreeRTOS_Support_Forum_Archive/July_2010/freertos_Is_it_possible_create_freertos_task_in_c_3778071.html
     * @see https://forum.arduino.cc/index.php?topic=674975.0
     * 
     * @param parm Reference to the actual RegulatedMotor object to handle.
     */
    static void motorCtrlHelper(void *parm)
    {
        static_cast<EV3RegulatedMotor *>(parm)->motorCtrl();
    }

    /**
     * Actual FreeRTOS task to control the motor. Determines current speed and performs PID operation.
     */
    void motorCtrl();

public:
    EV3RegulatedMotor(int motorPin1, int motorPin2, int tachoPin1, int tachoPin2)
    {
        _motorPin1 = motorPin1;
        _motorPin2 = motorPin2;
        _tachoPin1 = tachoPin1;
        _tachoPin2 = tachoPin2;
        _positionMutex = xSemaphoreCreateMutex();
        _speedMutex = xSemaphoreCreateMutex();
        _pidMutex = xSemaphoreCreateMutex();
    }

    /**
     * Starts and setups this regulated motor and the corresponding management task.
     */
    void start();

    /**
     * Stops this regulated motor and the corresponding managment task.
     */
    void stop();

    /**
     * Returns the current position of the motor (720 ticks = 360°)
     */
    int64_t getPosition();

    /**
     * Returns the speed in rpm
     */
    int32_t getSpeed();

    /**
     * Manually configures a PID target type and the corresponding PID parameters.
     */
    void set(EV3RegulationType type, int64_t value, double Kp, double Ki, double Kd);

    /**
     * Set raw pwm value for the motor.
     * @param direction forward = true, backward = false
     * @param value raw pwm value to set.
     */
    void setPWM(boolean direction, uint8_t value);

    /**
     * Set motor to coast mode (both h-bridge outputs high-Z)
     */
    void coast();

    /**
     * Set motor to brake mode (both h-bridge outputs low)
     */
    void brake();

    /**
     * Moves the motor to the given relative position (PID regulated.)
     */
    void setPosition(int64_t position);

    /**
     * Sets the speed of the motor (PID regulated)
     */
    void setSpeed(int32_t speed);
};
#endif