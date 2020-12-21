#include "EV3SensorPort.h"

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
    // Check actual name length
    int actualLenght = maxlength;
    for (int i = 0; i < maxlength; i++)
    {
        if ((data[i]) == 0)
        {
            actualLenght = i + 1;
            break;
        }
    }
    char *result = new char[actualLenght];

    for (int i = 0; i < actualLenght; i++)
    {
        result[i] = data[i];
    }

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
    if (_buffer[0] != SPEED)
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.println("Trying to parse speed system message but not found ");
#endif
        return false;
    }
    if (calculateChecksum(_buffer, 5) == _buffer[5])
    {
        config->speed = (_buffer[4] << 24) + (_buffer[3] << 16) + (_buffer[2] << 8) + (_buffer[1] << 0);
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Found EV3 sensor baudrate ");
        Serial.println(config->speed);
#endif
        return true;
    }
    else
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Wrong checksum for EV3 sensor baudrate ");
#endif
        return false;
    }
}

bool EV3SensorPort::parseModeCount(byte header, EV3SensorConfig *config)
{
    _buffer[0] = header;
    _connection->readBytes(_buffer + 1, 3);
    if (_buffer[0] != MODES)
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.println("Trying to parse modes system message but not found ");
#endif
        return false;
    }
    if (calculateChecksum(_buffer, 3) == _buffer[3])
    {
        config->modes = _buffer[1] + 1;
        config->modes_shown = _buffer[2] + 1;
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Found EV3 sensor ");
        Serial.print(config->modes);
        Serial.print(" modes with ");
        Serial.print(config->modes_shown);
        Serial.println(" modes presented");
#endif
        return true;
    }
    else
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.println("Trying to parse modes system message but checksum wrong");
#endif
        return false;
    }
}

bool EV3SensorPort::parseType(byte message, EV3SensorConfig *config)
{
    _buffer[0] = message;

    _connection->readBytes(_buffer + 1, 2);
    if (calculateChecksum(_buffer, 2) == _buffer[2])
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Found EV3 sensor ");
        Serial.print(_buffer[1], HEX);
        Serial.println(" with correct checksum");
#endif

        config->type = _buffer[1];
        return true;
    }
    else
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Found EV3 sensor ");
        Serial.print(_buffer[1], HEX);
        Serial.print(" with wrong checksum!!");
        Serial.print(" Should be ");
        Serial.print(calculateChecksum(_buffer, 2));
        Serial.print(" was ");
        Serial.println(_buffer[2]);
#endif
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

#ifdef EV3SENSOR_SERIAL_DEBUG
    if (infoType == 0)
    {
        Serial.println("-----------------------------------------------------");
    }
    Serial.print("info message ");
    Serial.print(infoType);
    Serial.print(" ");
#endif

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
        /*
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Unsupported message type !!");
#endif
        return false;
        */
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
#ifdef EV3SENSOR_SERIAL_DEBUG
            Serial.print(" Unknown message ");
            Serial.print(_buffer[0]);
            Serial.print(" for sensor mode ");
            Serial.print(_buffer[1]);
            Serial.print(" (mode = ");
            Serial.print(mode);
            Serial.print(" length = ");
            Serial.print(msgLenght);
            Serial.print(") of overall size ");
            Serial.print(msglength);
            Serial.print(" and content ");
            for (int i = 2; i < msglength; i++)
            {
                Serial.print(_buffer[i], HEX);
                Serial.print(" ");
            }
            Serial.println("");

#endif
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
        info->name = makeStringFromPayload(_buffer + 2, msgLenght);
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Found Symbol ");
        Serial.print(info->name);
        Serial.print(" for sensor mode ");
        Serial.println(info->mode);

#endif
        return true;
    }
    else
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Wrong checksum for sensor info 0 ");
#endif
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
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Found name ");
        Serial.print(info->name);
        Serial.print(" for sensor mode ");
        Serial.println(info->mode);

#endif
        return true;
    }
    else
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Wrong checksum for sensor info 0 ");
#endif
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

#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Number of items per message ");
        Serial.print(info->numberOfItems);
        Serial.print(" with type ");
        Serial.print(static_cast<uint8_t>(info->dataTypeOfItem));
        Serial.print(" with digits ");
        Serial.print(info->numberOfDigits);
        Serial.print(" with decimals ");
        Serial.print(info->numberOfDecimals);
        Serial.print(" for sensor mode ");
        Serial.println(info->mode);
#endif
        return true;
    }
    else
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Wrong checksum for sensor info ");
        Serial.println(_buffer[6]);
#endif
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
#ifdef EV3SENSOR_SERIAL_DEBUG
            Serial.print("Found RAW lowest value ");
            Serial.print(info->rawLowest);
            Serial.print(" and RAW highest value ");
            Serial.print(info->rawHighest);
            Serial.print(" for sensor mode ");
            Serial.println(info->mode);
#endif
            break;
        case 2:
            info->pctLowest = lowest;
            info->pctHighest = highest;
#ifdef EV3SENSOR_SERIAL_DEBUG
            Serial.print("Found PCT lowest value ");
            Serial.print(info->pctLowest);
            Serial.print(" and PCT highest value ");
            Serial.print(info->pctHighest);
            Serial.print(" for sensor mode ");
            Serial.println(info->mode);
#endif
            break;
        case 3:
            info->siLowest = lowest;
            info->siHighest = highest;
#ifdef EV3SENSOR_SERIAL_DEBUG
            Serial.print("Found SI lowest value ");
            Serial.print(info->siLowest);
            Serial.print(" and SI highest value ");
            Serial.print(info->siHighest);
            Serial.print(" for sensor mode ");
            Serial.println(info->mode);
#endif
            break;
        default:
#ifdef EV3SENSOR_SERIAL_DEBUG
            Serial.print("Unexpected info type ");
            Serial.print(infoType);
            Serial.print(" for sensor mode ");
            Serial.println(info->mode);
#endif
            return false;
        }

        return true;
    }
    else
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Wrong checksum for sensor info ");
        Serial.println(infoType);
#endif
        return false;
    }
}

void EV3SensorPort::selectSensorMode(uint8_t mode)
{
#ifdef EV3SENSOR_SERIAL_DEBUG
    Serial.print("Setting sensor to mode ");
    Serial.println(mode);
#endif
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
}

void EV3SensorPort::begin(std::function<void(EV3SensorPort *)> onSuccess, int retries)
{
    stop();
    this->_baudrateSetter(2400);
    byte message = 0;
    xSemaphoreTake(_serialMutex, portMAX_DELAY);
    // First wait for the first TYPE message. Its always the first message!!!!
    while (message != TYPE)
    {
        message = this->readNextAvailableByte();
    }

    this->parseType(message, &_config);

    // Wait for the next message
    bool waitingForConfig = true;
    while (waitingForConfig)
    {
        message = this->readNextAvailableByte();
        if (message == MODES)
        {
            if (!this->parseModeCount(message, &_config))
            {
#ifdef EV3SENSOR_SERIAL_DEBUG
                Serial.println("Failed to parse mode count -> restart!");
#endif
                xSemaphoreGive(_serialMutex);

                vTaskDelay(TIME_BEFORE_RESTART);
                return this->begin(onSuccess, retries - 1);
            }
            _config.infos = new EV3SensorInfo[_config.modes];
        }
        else if (message == SPEED)
        {
            if (!this->parseSpeed(message, &_config))
            {
#ifdef EV3SENSOR_SERIAL_DEBUG
                Serial.println("Failed to parse sensor uart speed -> restart!");
#endif
                xSemaphoreGive(_serialMutex);
                vTaskDelay(TIME_BEFORE_RESTART);
                return this->begin(onSuccess, retries - 1);
            }
        }
        else if (message == ACK)
        {
            waitingForConfig = false;
#ifdef EV3SENSOR_SERIAL_DEBUG
            Serial.println("-----------------------------------------------------");
            Serial.println("Received ACK - end of sensor config!!");
#endif
        }
        else if (message & 0b10000000)
        {
            // Found info message
            byte modeNumber = message & 0b111;
            EV3SensorInfo info = _config.infos[modeNumber];
            if (!this->parseInfoMessage(message, &info))
            {
#ifdef EV3SENSOR_SERIAL_DEBUG
                Serial.println("Failed to parse sensor mode -> restart!");
#endif
                xSemaphoreGive(_serialMutex);
                vTaskDelay(TIME_BEFORE_RESTART);
                return this->begin(onSuccess, retries - 1);
            }
        }
    }
#ifdef EV3SENSOR_SERIAL_DEBUG
    Serial.println("Reply with ACK ");
#endif
    _connection->write(ACK);
    _connection->flush();

#ifdef EV3SENSOR_SERIAL_DEBUG
    Serial.print("Switching UART baudrate to ");
    Serial.println(this->_config.speed);
#endif
    this->_baudrateSetter(this->_config.speed);
    xSemaphoreGive(_serialMutex);
    onSuccess(this);

    this->sensorCommThread();
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
        if (_connection->available() > 0)
        {
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
#ifdef EV3SENSOR_SERIAL_DEBUG
                    Serial.print("Got data message from sensor for mode ");
                    Serial.print(mode);
                    Serial.print(" with length ");
                    Serial.print(msgLenght);
                    Serial.println(" but with wrong checksum :/.");
#endif
                }
            }
        }

        xSemaphoreTake(_serialMutex, portMAX_DELAY);
        _connection->write(NACK);
        xSemaphoreGive(_serialMutex);
        vTaskDelay(90 / portTICK_PERIOD_MS);
    }
}
