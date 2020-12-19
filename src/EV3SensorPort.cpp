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

bool EV3SensorPort::parseSpeed(SensorConfig *config)
{
    byte payload[6];
    _connection->readBytes(payload, 6);
    if (payload[0] != SPEED)
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.println("Trying to parse speed system message but not found ");
#endif
        return false;
    }
    if (calculateChecksum(payload, 5) == payload[5])
    {
        config->speed = (payload[4] << 24) + (payload[3] << 16) + (payload[2] << 8) + (payload[1] << 0);
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

bool EV3SensorPort::parseModeCount(SensorConfig *config)
{
    byte payload[4];
    _connection->readBytes(payload, 4);
    if (payload[0] != MODES)
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.println("Trying to parse modes system message but not found ");
#endif
        return false;
    }
    if (calculateChecksum(payload, 3) == payload[3])
    {
        config->modes = payload[1] + 1;
        config->modes_shown = payload[2] + 1;
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

bool EV3SensorPort::parseType(SensorConfig *config, bool waitForType)
{
#ifdef EV3SENSOR_SERIAL_DEBUG
    Serial.println("Start searching for EV3 sensor type ...");
#endif
    byte payload[3];
    while (true)
    {
        if (_connection->available() > 0)
        {
            byte v = _connection->read();
            if (v == TYPE)
            {
                payload[0] = v;
                break;
            }
            else
            {
                if (waitForType)
                {
                    continue;
                }
                else
                {
                    return false;
                }
            }
        }
    }
    _connection->readBytes(payload + 1, 2);
    if (calculateChecksum(payload, 2) == payload[2])
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Found EV3 sensor ");
        Serial.print(payload[1], HEX);
        Serial.println(" with correct checksum");
#endif

        config->type = payload[1];
        return true;
    }
    else
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Found EV3 sensor ");
        Serial.print(payload[1], HEX);
        Serial.print(" with wrong checksum!!");
        Serial.print(" Should be ");
        Serial.print(calculateChecksum(payload, 2));
        Serial.print(" was ");
        Serial.println(payload[2]);
#endif
        return false;
    }
}

float EV3SensorPort::makeFloatFromPayload(uint8_t data[])
{
    return (float)(data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0] << 0);
}

uint8_t EV3SensorPort::parseInfoMessage(EV3SensorInfo *info)
{
    byte header[2];
    // First check if the next message can be an info message at all.
    _connection->readBytes(header, 1);
    byte message = header[0];

    if (message & 0b10000000)
    {
        // This might be a info message, check the info byte.
        byte infoType;
        _connection->readBytes(header + 1, 1);

        infoType = header[1];

#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Parsing info message ");
        Serial.println(infoType);
#endif

        switch (infoType)
        {
        case 0:
            return parseModeNameMessage(header, info);
            break;
        case 1:
        case 2:
        case 3:
            return parseModeRangeMessage(header, info);
            break;

        case 0x80:
            return parseFormatMessage(header, info);
            break;
        case 4:
            return -1; // TODO implement;
            break;
        default:
            return -1;
        }
    }
    return message;
}

uint8_t EV3SensorPort::parseModeNameMessage(byte *header, EV3SensorInfo *info)
{
    uint8_t msgLenght = 1 << ((header[0] & 0b00111000) >> 3); // 2^LLL;
    byte *payload = new byte[2 + msgLenght + 1];

    // Copy header to payload to simplify checksum calculation
    memcpy(payload, header, 2);

    // Read the message string
    _connection->readBytes(payload + 2, msgLenght + 1); // Msg + checksum

    if (calculateChecksum(payload, msgLenght + 2) == payload[msgLenght + 2])
    {
        info->mode = payload[0] & 0b111;
        // Check actual name length
        info->name = makeStringFromPayload(payload + 2, msgLenght);
        //  delete[] payload;
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Found name ");
        Serial.print(info->name);
        Serial.print(" (length = ");
        Serial.print(strlen(info->name));
        Serial.print(" )");
        Serial.print(" for sensor mode ");
        Serial.println(info->mode);

#endif
        return info->mode;
    }
    else
    {
        delete[] payload;
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Wrong checksum for sensor info 0 ");
#endif
        return -1;
    }
}

uint8_t EV3SensorPort::parseFormatMessage(byte *header, EV3SensorInfo *info)
{
    byte payload[2 + 4 + 1];
    memcpy(payload, header, 2);
    _connection->readBytes(payload + 2, 5);

    if (calculateChecksum(payload, 6) == payload[6])
    {
        info->numberOfItems = payload[2];
        info->dataTypeOfItem = payload[3];
        info->numberOfDigits = payload[4];
        info->numberOfDecimals = payload[5];
        info->mode = payload[0] & 0b111;

#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Number of items per message ");
        Serial.print(info->numberOfItems);
        Serial.print(" with type ");
        Serial.print(info->dataTypeOfItem);
        Serial.print(" with digits ");
        Serial.print(info->numberOfDigits);
        Serial.print(" with decimals ");
        Serial.print(info->numberOfDecimals);
        Serial.print(" for sensor mode ");
        Serial.println(info->mode);
#endif
        return info->mode;
    }
    else
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Wrong checksum for sensor info ");
        Serial.println(payload[1]);
#endif
        return -1;
    }
}

uint8_t EV3SensorPort::parseModeRangeMessage(byte *header, EV3SensorInfo *info)
{
    byte payload[11];
    memcpy(payload, header, 2);
    _connection->readBytes(payload + 2, 11 - 2);
    byte infoType = payload[1];

    if (calculateChecksum(payload, 10) == payload[10])
    {
        info->mode = payload[0] & 0b111;
        float lowest = makeFloatFromPayload(payload + 2);
        float highest = makeFloatFromPayload(payload + 2 + 4);

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
            return -1;
        }

        return info->mode;
    }
    else
    {
#ifdef EV3SENSOR_SERIAL_DEBUG
        Serial.print("Wrong checksum for sensor info ");
        Serial.println(infoType);
#endif
        return -1;
    }
}
