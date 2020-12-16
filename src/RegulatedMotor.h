#include <Arduino.h>
#include <ESP32Encoder.h>

/**
 * Types of PID regulations available for a regulated motor.
 */
enum struct RegulationType : uint8_t
{
    NONE,
    POSITION,
    SPEED
};

enum struct MotorState
{
    COAST,
    FORWARD,
    REVERSE,
    BRAKE
};

class RegulatedMotor
{
private:
    // Pins for the motors
    int _motorPin1;
    int _motorPin2;
    int _tachoPin1;
    int _tachoPin2;

    // We use 8 bit pwm, defines maximum values
    const double PWM_MAX = 255;
    const double PWM_MIN = -1 * PWM_MAX;

    /**
     * PID control variables for position control.
     */
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
    RegulationType _reg_type;
    // Target value for the PID regulration
    int64_t _reg_target;

    // Previous position after before the last call
    int64_t _prev_pos = 0;

    //Time of the last motor control loop
    unsigned long _prev_time = 0;

    //Current speed of the motor
    double _speed = 0;
    double _prev_speed = 0;
    MotorState _motorState = MotorState::COAST;

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
        static_cast<RegulatedMotor *>(parm)->motorCtrl();
    }

    /**
     * Actual FreeRTOS task to control the motor. Determines current speed and performs PID operation.
     */
    void motorCtrl()
    {
        for (;;)
        {
            // Fetch position
            xSemaphoreTake(_positionMutex, portMAX_DELAY);
            auto pos = encoder.getCount();
            xSemaphoreGive(_positionMutex);

            auto time = millis();
            auto delta_pos = pos - _prev_pos;
            auto delta_time = time - _prev_time;

            // 720 ticks per round and 60*1000 ms per minute
            int32_t speed = (abs(delta_pos) * 1000 * 60) / (720 * delta_time);

            // Update speed
            xSemaphoreTake(_speedMutex, portMAX_DELAY);
            _speed = speed;
            xSemaphoreGive(_speedMutex);

            // Perform pid control if necessary.
            xSemaphoreTake(_pidMutex, portMAX_DELAY);
            if (_reg_type == RegulationType::POSITION)
            {
                double output = 0;
                double dInput = delta_pos;
                double error = (double(_reg_target)) - ((double)pos);

                _outputSum += (_Ki * error);
                if (_outputSum > PWM_MAX)
                {
                    _outputSum = PWM_MAX;
                }
                else if (_outputSum < PWM_MIN)
                {
                    _outputSum = PWM_MIN;
                }

                output += _Kp * error;

                output += _outputSum - _Kd * dInput;

                if (output > PWM_MAX)
                {
                    output = PWM_MAX;
                }
                else if (output < PWM_MIN)
                {
                    output = PWM_MIN;
                }

                setPWM(output > 0, abs(output));
            }
            else if (_reg_type == RegulationType::SPEED)
            {
                double output = 0;
                double dInput = speed - _prev_speed;
                double error = (double(_reg_target)) - ((double)speed);

                _outputSum += (_Ki * error);
                if (_outputSum > PWM_MAX)
                {
                    _outputSum = PWM_MAX;
                }
                else if (_outputSum < PWM_MIN)
                {
                    _outputSum = PWM_MIN;
                }

                output += _Kp * error;

                output += _outputSum - _Kd * dInput;

                if (output > PWM_MAX)
                {
                    output = PWM_MAX;
                }
                else if (output < PWM_MIN)
                {
                    output = PWM_MIN;
                }

                // TODO calculate pwm from output
            }
            xSemaphoreGive(_pidMutex);

            _prev_pos = pos;
            _prev_time = time;

            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }

public:
    RegulatedMotor(int motorPin1, int motorPin2, int tachoPin1, int tachoPin2)
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
     *  Starts and setups this regulated motor and the corresponding management task.
     */
    void start()
    {
        pinMode(_motorPin1, OUTPUT);
        pinMode(_motorPin2, OUTPUT);

        // Setup channel 0 with 5khz pwm freq with 8 bit resolution.
        ledcSetup(0, 5000, 8);

        // Set motors to coast mode
        digitalWrite(_motorPin1, LOW);
        digitalWrite(_motorPin2, LOW);

        // Setup motor encoder
        encoder.attachFullQuad(_tachoPin1, _tachoPin2);

        if (_motorCtrlHandle)
        {
            vTaskDelete(_motorCtrlHandle);
        }

        xTaskCreate(
            &motorCtrlHelper,
            "Controlling motor",
            10000,
            this,
            1,
            &_motorCtrlHandle // Task handle
        );
    }

    /**
     *  Stops this regulated motor and the corresponding managment task.
     */
    void stop()
    {
        if (_motorCtrlHandle)
        {
            vTaskDelete(_motorCtrlHandle);
            _motorCtrlHandle = nullptr;
        }
    }

    /**
     *  Returns the current position of the motor (720 = full round)
     */
    int64_t getPosition()
    {
        xSemaphoreTake(_positionMutex, portMAX_DELAY);
        auto pos = encoder.getCount();
        xSemaphoreGive(_positionMutex);
        return pos;
    }

    /**
     *  Returns the speed in rpm
     */
    int32_t getSpeed()
    {
        xSemaphoreTake(_speedMutex, portMAX_DELAY);
        auto speed = _speed;
        xSemaphoreGive(_speedMutex);
        return speed;
    }

    /**
     * Manually configures a PID target type and the corresponding PID parameters.
     */
    void set(RegulationType type, int64_t value, double Kp, double Ki, double Kd)
    {
        xSemaphoreTake(_pidMutex, portMAX_DELAY);
        _reg_type = type;
        _reg_target = value;
        _Kp = Kp;
        _Ki = Ki;
        _Kd = Kd;
        _outputSum = 0.0;
        xSemaphoreGive(_pidMutex);
    }

    /**
    * Set raw pwm value for the motor.
    * @param direction forward = true, backward = false
    * @param value raw pwm value to set.
    */
    void setPWM(boolean direction, uint8_t value)
    {
        if (direction)
        {
            if (_motorState == MotorState::REVERSE)
            {
                ledcDetachPin(_motorPin2);
            }
            if (_motorState != MotorState::FORWARD)
            {
                ledcAttachPin(_motorPin1, 0);
                digitalWrite(_motorPin2, LOW);
                _motorState = MotorState::FORWARD;
            }
        }
        else
        {
            if (_motorState == MotorState::FORWARD)
            {
                ledcDetachPin(_motorPin1);
            }
            if (_motorState != MotorState::REVERSE)
            {
                ledcAttachPin(_motorPin2, 0);
                digitalWrite(_motorPin1, LOW);
                _motorState = MotorState::REVERSE;
            }
        }
        ledcWrite(0, value);
    }

    /**
     * Set motor to coast mode (both h-bridge outputs high-Z)
     */
    void coast()
    {
        switch (_motorState)
        {
        case MotorState::COAST:
            break;              // Already coasting
        case MotorState::BRAKE: // Move from coasting to brake
            digitalWrite(_motorPin1, LOW);
            digitalWrite(_motorPin2, LOW);
            break;
        case MotorState::FORWARD:
            ledcDetachPin(_motorPin1);
            digitalWrite(_motorPin1, LOW);
            digitalWrite(_motorPin2, LOW);
            break;
        case MotorState::REVERSE:
            ledcDetachPin(_motorPin2);
            digitalWrite(_motorPin1, LOW);
            digitalWrite(_motorPin2, LOW);
            break;
        }
        _motorState = MotorState::COAST;
    }

    /**
     * Set motor to brake mode (both h-bridge outputs low)
     */
    void brake()
    {
        switch (_motorState)
        {
        case MotorState::BRAKE:
            break;              // Already breaking
        case MotorState::COAST: // Move from coasting to brake
            digitalWrite(_motorPin1, HIGH);
            digitalWrite(_motorPin2, HIGH);
            break;
        case MotorState::FORWARD:
            ledcDetachPin(_motorPin1);
            digitalWrite(_motorPin1, HIGH);
            digitalWrite(_motorPin2, HIGH);
            break;
        case MotorState::REVERSE:
            ledcDetachPin(_motorPin2);
            digitalWrite(_motorPin1, HIGH);
            digitalWrite(_motorPin2, HIGH);
            break;
        }
        _motorState = MotorState::BRAKE;
    }

    /**
     * Moves the motor to the given relative position (PID regulated.)
     */
    void setPosition(int64_t position)
    {
        set(RegulationType::POSITION, position, KP_POS_CTRL, KI_POS_CTRL, KD_POS_CTRL);
    }

    /**
     * Sets the speed of the motor (PID regulated)
     */
    void setSpeed(int32_t speed)
    {
    }
};