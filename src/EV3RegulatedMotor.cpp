#include "EV3RegulatedMotor.h"

const static char *TAG = "EV3RegulatedMotor";

void EV3RegulatedMotor::start()
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
        "EV3 motor",
        10000,
        this,
        1,
        &_motorCtrlHandle // Task handle
    );
}

void EV3RegulatedMotor::setSpeed(int32_t speed)
{
    ESP_LOGE(TAG, "PID speed control not yet implemented!");
}

void EV3RegulatedMotor::setPosition(int64_t position)
{
    ESP_LOGD(TAG, "Set motor target position to %d ticks", position);
    set(EV3RegulationType::POSITION, position, KP_POS_CTRL, KI_POS_CTRL, KD_POS_CTRL);
}

void EV3RegulatedMotor::brake()
{
    ESP_LOGD(TAG, "Set motor to brake mode");
    xSemaphoreTake(_pidMutex, portMAX_DELAY);
    _reg_type = EV3RegulationType::NONE;
    xSemaphoreGive(_pidMutex);
    switch (_motorState)
    {
    case EV3MotorState::BRAKE:
        break;                 // Already breaking
    case EV3MotorState::COAST: // Move from coasting to brake
        digitalWrite(_motorPin1, HIGH);
        digitalWrite(_motorPin2, HIGH);
        break;
    case EV3MotorState::FORWARD:
        ledcDetachPin(_motorPin1);
        digitalWrite(_motorPin1, HIGH);
        digitalWrite(_motorPin2, HIGH);
        break;
    case EV3MotorState::REVERSE:
        ledcDetachPin(_motorPin2);
        digitalWrite(_motorPin1, HIGH);
        digitalWrite(_motorPin2, HIGH);
        break;
    }
    _motorState = EV3MotorState::BRAKE;
}

void EV3RegulatedMotor::coast()
{
    ESP_LOGD(TAG, "Set motor to coast mode");
    xSemaphoreTake(_pidMutex, portMAX_DELAY);
    _reg_type = EV3RegulationType::NONE;
    xSemaphoreGive(_pidMutex);
    switch (_motorState)
    {
    case EV3MotorState::COAST:
        break;                 // Already coasting
    case EV3MotorState::BRAKE: // Move from coasting to brake
        digitalWrite(_motorPin1, LOW);
        digitalWrite(_motorPin2, LOW);
        break;
    case EV3MotorState::FORWARD:
        ledcDetachPin(_motorPin1);
        digitalWrite(_motorPin1, LOW);
        digitalWrite(_motorPin2, LOW);
        break;
    case EV3MotorState::REVERSE:
        ledcDetachPin(_motorPin2);
        digitalWrite(_motorPin1, LOW);
        digitalWrite(_motorPin2, LOW);
        break;
    }
    _motorState = EV3MotorState::COAST;
}

void EV3RegulatedMotor::setPWM(boolean direction, uint8_t value)
{
    if (direction)
    {
        if (_motorState == EV3MotorState::REVERSE)
        {
            ledcDetachPin(_motorPin2);
        }
        if (_motorState != EV3MotorState::FORWARD)
        {
            ledcAttachPin(_motorPin1, 0);
            digitalWrite(_motorPin2, LOW);
            _motorState = EV3MotorState::FORWARD;
        }
    }
    else
    {
        if (_motorState == EV3MotorState::FORWARD)
        {
            ledcDetachPin(_motorPin1);
        }
        if (_motorState != EV3MotorState::REVERSE)
        {
            ledcAttachPin(_motorPin2, 0);
            digitalWrite(_motorPin1, LOW);
            _motorState = EV3MotorState::REVERSE;
        }
    }
    ledcWrite(0, value);
}

void EV3RegulatedMotor::set(EV3RegulationType type, int64_t value, double Kp, double Ki, double Kd)
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

int32_t EV3RegulatedMotor::getSpeed()
{
    xSemaphoreTake(_speedMutex, portMAX_DELAY);
    auto speed = _speed;
    xSemaphoreGive(_speedMutex);
    return speed;
}

int64_t EV3RegulatedMotor::getPosition()
{
    xSemaphoreTake(_positionMutex, portMAX_DELAY);
    auto pos = encoder.getCount();
    xSemaphoreGive(_positionMutex);
    return pos;
}

void EV3RegulatedMotor::stop()
{
    ESP_LOGD(TAG, "Motor control started");
    if (_motorCtrlHandle)
    {
        vTaskDelete(_motorCtrlHandle);
        _motorCtrlHandle = nullptr;
    }
}

void EV3RegulatedMotor::motorCtrl()
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
        if (_reg_type == EV3RegulationType::POSITION)
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
        else if (_reg_type == EV3RegulationType::SPEED)
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