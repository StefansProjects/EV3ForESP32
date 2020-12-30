#include "EV3SensorPort.h"

static const char *TAG = "EV3SensorPort";

uint8_t EV3SensorPort::calculateChecksum(uint8_t data[], int length)
{
    uint8_t result = 0xff;
    for (int i = 0; i < length; i++)
    {
        result ^= data[i];
    }
    return result;
}

char *EV3SensorPort::makeStringFromPayload(uint8_t data[], int maxlength)
{

    int actualLength = strlen((char *)data);
    actualLength = actualLength > maxlength ? maxlength : actualLength;
    char *result = new char[actualLength + 1]; // +1 for null termination;

    for (int i = 0; i < actualLength; i++)
    {
        result[i] = data[i];
    }
    result[actualLength] = 0;

    return result;
}

/**
 * Utlity method the read the next available byte from the connection.
 */
byte EV3SensorPort::readNextAvailableByte()
{
    byte message;
    while (true)
    {
        if (_connection->available() > 0)
        {
            message = _connection->read();
            break;
        }
    }
    return message;
}

bool EV3SensorPort::parseSpeed(byte header, EV3SensorConfig *config)
{
    _buffer[0] = header;

    _connection->readBytes(_buffer + 1, 5);
    if (calculateChecksum(_buffer, 5) == _buffer[5])
    {
        config->speed = (_buffer[4] << 24) + (_buffer[3] << 16) + (_buffer[2] << 8) + (_buffer[1] << 0);
        ESP_LOGV(TAG, "Found EV3 sensor to expect %d baud as communication baudrate", config->speed);
        return true;
    }
    else
    {
        ESP_LOGD(TAG, "Wrong checksum for SPEED system message.");
        return false;
    }
}

bool EV3SensorPort::parseModeCount(byte header, EV3SensorConfig *config)
{
    _buffer[0] = header;
    _connection->readBytes(_buffer + 1, 3);
    if (calculateChecksum(_buffer, 3) == _buffer[3])
    {
        config->modes = _buffer[1] + 1;
        config->modes_shown = _buffer[2] + 1;
        ESP_LOGV(TAG, "Found EV3 sensor with %d modes and %d modes presented.", config->modes, config->modes_shown);
        return true;
    }
    else
    {
        ESP_LOGD(TAG, "Wrong checksum for MODES system message.");
        return false;
    }
}

bool EV3SensorPort::parseType(byte message, EV3SensorConfig *config)
{
    _buffer[0] = message;

    _connection->readBytes(_buffer + 1, 2);
    if (calculateChecksum(_buffer, 2) == _buffer[2])
    {
        config->type = _buffer[1];
        ESP_LOGD(TAG, "Found EV3 sensor: %d.", config->type);
        return true;
    }
    else
    {
        ESP_LOGD(TAG, "Wrong checksum for TYPE system message.");
        return false;
    }
}

float EV3SensorPort::makeFloatFromPayload(uint8_t data[])
{
    uint32_t flt = (float)(data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0] << 0);
    float result = *reinterpret_cast<float *>(&flt);
    return result;
}

bool EV3SensorPort::parseInfoMessage(byte message, EV3SensorInfo *info)
{
    _buffer[0] = message;
    _connection->readBytes(_buffer + 1, 1);

    byte infoType = _buffer[1];

    if (infoType == 0)
    {
        ESP_LOGV(TAG, "-----------------------------------------------------");
    }

    switch (infoType)
    {
    case 0:
        return parseModeNameMessage(_buffer, info);
        break;
    case 1:
    case 2:
    case 3:
        return parseModeRangeMessage(_buffer, info);
        break;

    case 0x80:
        return parseFormatMessage(_buffer, info);
        break;
    case 4:
        return parseSymbolNameMessage(_buffer, info);
        break;
    default:
        return parseUnknownMessage(_buffer);
    }
}

bool EV3SensorPort::parseUnknownMessage(byte *header)
{
    _buffer[0] = header[0];
    _buffer[1] = header[1];

    // Read bytes until the checksum fits
    uint8_t msgLenght = 1 << ((header[0] & 0b00111000) >> 3); // 2^LLL;
    uint8_t mode = _buffer[0] & 0b111;
    uint8_t msglength = 2;
    uint8_t checksum = 0;

    for (;;)
    {
        checksum = _buffer[msglength - 1];
        if (calculateChecksum(_buffer, msglength - 1) == checksum)
        {
            // Message found !!!
            ESP_LOGD(TAG, "Unknown message %x (mode = %d, length = %d) for sensor mode %d", _buffer[0], mode, msgLenght, _buffer[1]);
            break;
        }
        else
        {
            // fetch next byte
            _connection->readBytes(_buffer + msglength, 1);
            msglength++;
        }
    }
    return true;
}

bool EV3SensorPort::parseSymbolNameMessage(byte *header, EV3SensorInfo *info)
{
    uint8_t msgLenght = 1 << ((header[0] & 0b00111000) >> 3); // 2^LLL;

    // Copy header to payload to simplify checksum calculation
    _buffer[0] = header[0];
    _buffer[1] = header[1];

    // Read the message string
    _connection->readBytes(_buffer + 2, msgLenght + 1); // Msg + checksum

    if (calculateChecksum(_buffer, msgLenght + 2) == _buffer[msgLenght + 2])
    {
        info->mode = _buffer[0] & 0b111;
        // Check actual name length
        info->siSymbol = makeStringFromPayload(_buffer + 2, msgLenght);
        ESP_LOGV(TAG, "Found symbol '%s' for sensor mode %d", info->siSymbol, info->mode);
        return true;
    }
    else
    {
        ESP_LOGD(TAG, "Wrong checksum for INFO system message 4.");
        return false;
    }
}

bool EV3SensorPort::parseModeNameMessage(byte *header, EV3SensorInfo *info)
{
    uint8_t msgLenght = 1 << ((header[0] & 0b00111000) >> 3); // 2^LLL;

    // Copy header to payload to simplify checksum calculation
    _buffer[0] = header[0];
    _buffer[1] = header[1];

    // Read the message string
    _connection->readBytes(_buffer + 2, msgLenght + 1); // Msg + checksum

    if (calculateChecksum(_buffer, msgLenght + 2) == _buffer[msgLenght + 2])
    {
        info->mode = _buffer[0] & 0b111;
        // Check actual name length
        info->name = makeStringFromPayload(_buffer + 2, msgLenght);
        ESP_LOGV(TAG, "Found name '%s' for sensor mode %d", info->name, info->mode);
        return true;
    }
    else
    {
        ESP_LOGD(TAG, "Wrong checksum for INFO system message 0.");
        return false;
    }
}

bool EV3SensorPort::parseFormatMessage(byte *header, EV3SensorInfo *info)
{
    _buffer[0] = header[0];
    _buffer[1] = header[1];
    _connection->readBytes(_buffer + 2, 5);

    if (calculateChecksum(_buffer, 6) == _buffer[6])
    {
        info->numberOfItems = _buffer[2];
        info->dataTypeOfItem = static_cast<EV3Datatype>(_buffer[3]);
        info->numberOfDigits = _buffer[4];
        info->numberOfDecimals = _buffer[5];
        info->mode = _buffer[0] & 0b111;

        ESP_LOGV(TAG, "Number of items per message %d of type %d with %d digits and with % decimals for sensor mode %d", info->numberOfItems, static_cast<uint8_t>(info->dataTypeOfItem), info->numberOfDigits, info->numberOfDecimals, info->mode);
        return true;
    }
    else
    {
        ESP_LOGD(TAG, "Wrong checksum for INFO system message 0x80.");
        return false;
    }
}

bool EV3SensorPort::parseModeRangeMessage(byte *header, EV3SensorInfo *info)
{
    _buffer[0] = header[0];
    _buffer[1] = header[1];
    _connection->readBytes(_buffer + 2, 11 - 2);
    byte infoType = _buffer[1];

    if (calculateChecksum(_buffer, 10) == _buffer[10])
    {
        info->mode = _buffer[0] & 0b111;
        float lowest = makeFloatFromPayload(_buffer + 2);
        float highest = makeFloatFromPayload(_buffer + 2 + 4);

        switch (infoType)
        {
        case 1:
            info->rawLowest = lowest;
            info->rawHighest = highest;
            ESP_LOGV(TAG, "RAW value between %f and %f for sensor mode %d", info->rawLowest, info->rawHighest, info->mode);
            break;
        case 2:
            info->pctLowest = lowest;
            info->pctHighest = highest;
            ESP_LOGV(TAG, "PCT value between %f and %f for sensor mode %d", info->pctLowest, info->pctHighest, info->mode);
            break;
        case 3:
            info->siLowest = lowest;
            info->siHighest = highest;
            ESP_LOGV(TAG, "SI value between %f and %f for sensor mode %d", info->siLowest, info->siHighest, info->mode);
            break;
        default:
            ESP_LOGD(TAG, "Unexpected info type %d for sensor mode %d", infoType, info->mode);
            return false;
        }
        return true;
    }
    else
    {
        ESP_LOGD(TAG, "Wrong checksum for INFO system message %d.", infoType);
        return false;
    }
}

void EV3SensorPort::selectSensorMode(uint8_t mode)
{
    ESP_LOGD(TAG, "Setting sensor to mode %d", mode);
    xSemaphoreTake(_serialMutex, portMAX_DELAY);
    _buffer[0] = SELECT;
    _buffer[1] = mode;
    _buffer[2] = this->calculateChecksum(_buffer, 2);
    _connection->write(_buffer, 3);
    _connection->flush();
    xSemaphoreGive(_serialMutex);
}

void EV3SensorPort::stop()
{
    ESP_LOGD(TAG, "Stopping sensor");
    if (_sensorHandle)
    {
        vTaskDelete(_sensorHandle);
        _sensorHandle = nullptr;
    }
}

void EV3SensorPort::sensorInitHelper(void *parm)
{
    static_cast<EV3SensorPort *>(parm)->sensorInit();
}

void EV3SensorPort::sensorInit()
{
    byte message = 0;
    xSemaphoreTake(_serialMutex, portMAX_DELAY);
    this->_baudrateSetter(2400);
    ESP_LOGV(TAG, "Waiting from the first TYPE message from the EV3 sensor. %d retries left", retries);
    // First wait for the first TYPE message. Its always the first message!!!!
    while (message != TYPE)
    {
        message = this->readNextAvailableByte();
    }

    if (this->parseType(message, &_config))
    {

        // Wait for the next message
        bool waitingForConfig = true;
        while (waitingForConfig)
        {
            // Clear buffer
            std::fill(_buffer, _buffer + BUFFER_SIZE, 0);
            message = this->readNextAvailableByte();
            if (message == MODES)
            {
                if (!this->parseModeCount(message, &_config))
                {
                    ESP_LOGE(TAG, "Failed to parse mode count -> restart (%d retries left)", retries - 1);
                    xSemaphoreGive(_serialMutex);
                    vTaskDelay(TIME_BEFORE_RESTART);
                    retries--;
                    this->sensorInit();
                    return;
                }
                _config.infos = new EV3SensorInfo[_config.modes];
            }
            else if (message == SPEED)
            {
                if (!this->parseSpeed(message, &_config))
                {
                    ESP_LOGE(TAG, "Failed to parse sensor uart speed -> restart (%d retries left)", retries - 1);
                    xSemaphoreGive(_serialMutex);
                    vTaskDelay(TIME_BEFORE_RESTART);
                    retries--;
                    this->sensorInit();
                    return;
                }
            }
            else if (message == ACK)
            {
                waitingForConfig = false;
                ESP_LOGV(TAG, "-----------------------------------------------------");
                ESP_LOGV(TAG, "Fully received sensor config");
            }
            else if (message & 0b10000000)
            {
                // Found info message
                byte modeNumber = message & 0b111;
                EV3SensorInfo info = _config.infos[modeNumber];
                if (!this->parseInfoMessage(message, &info))
                {
                    ESP_LOGE(TAG, "Failed to parse sensor info message -> restart (%d retries left)", retries - 1);
                    xSemaphoreGive(_serialMutex);
                    vTaskDelay(TIME_BEFORE_RESTART);
                    retries--;
                    this->sensorInit();
                    return;
                }
            }
        }

        ESP_LOGV(TAG, "Reply sensor config with ACK");
        _connection->write(ACK);
        _connection->flush();

        ESP_LOGV(TAG, "Set new UART speed to %d baud", this->_config.speed);
        this->_baudrateSetter(this->_config.speed);
        xSemaphoreGive(_serialMutex);
        _onSuccess(this);

        ESP_LOGV(TAG, "Initalization phase finished - switching to communication phase");
        this->sensorCommThread();
    }
    ESP_LOGD(TAG, "Restarting sensor init phase ...");
    sensorInit();
}

void EV3SensorPort::begin(std::function<void(EV3SensorPort *)> onSuccess, int retries)
{
    stop();
    this->_onSuccess = onSuccess;
    this->retries = retries;

    xTaskCreate(
        &sensorInitHelper,
        "EV3Sensor",
        50000,
        this,
        1,
        &_sensorHandle // Task handle
    );
}

/**
 * Utility method to get get EV3SensorInfo for a mode
 */
EV3SensorInfo *EV3SensorPort::getInfoForMode(uint8_t mode)
{
    if (getCurrentConfig())
    {
        auto config = getCurrentConfig();
        for (int i = 0; i < config->modes; i++)
        {
            if (config->infos[i].mode == mode)
            {
                return &config->infos[i];
            }
        }
    }
    return nullptr;
}

void EV3SensorPort::sensorCommThread()
{
    for (;;)
    {
        xSemaphoreTake(_serialMutex, portMAX_DELAY);
        if (_connection->available() > 0)
        {
            // Clear buffer
            std::fill(_buffer, _buffer + BUFFER_SIZE, 0);
            uint8_t message = _connection->read();
            if (message & 0b11000000)
            {
                uint8_t mode = message & 0b111;
                uint8_t msgLenght = 1 << ((message & 0b00111000) >> 3); // 2^LLL;

                _buffer[0] = message;
                _connection->readBytes(_buffer + 1, msgLenght + 1);
                if (calculateChecksum(_buffer, msgLenght + 1) == _buffer[msgLenght + 1])
                {
                    if (this->_onMessage)
                    {
                        _onMessage(mode, _buffer + 1, msgLenght);
                    }
                }
                else
                {
                    ESP_LOGV(TAG, "Got data message from sensor for mode %d with length %d but wrong checksum.", mode, msgLenght);
                }
            }
        }
        xSemaphoreGive(_serialMutex);

        auto current_timestamp = millis();
        auto delta_time = current_timestamp - this->lastNACKSended;
        if (delta_time > 90)
        {
            xSemaphoreTake(_serialMutex, portMAX_DELAY);
            _connection->write(NACK);
            xSemaphoreGive(_serialMutex);
            this->lastNACKSended = current_timestamp;
        }
        vTaskDelay(1);
    }
}
