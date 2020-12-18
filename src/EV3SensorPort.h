#include <stdint.h>
#include <Arduino.h>

#define EV3SENSOR_SERIAL_DEBUG

struct SensorConfig
{
    uint8_t type;
    uint8_t modes;
    uint8_t modes_shown;
    uint32_t speed;
};

struct EV3SensorInfo
{
    byte mode;
    char *name;
    float rawLowest;
    float rawHighest;
    float pctLowest;
    float pctHighest;
    float siLowest;
    float siHighest;
    char *siSymbol;
};

/**
 * 
 * @see https://sourceforge.net/p/lejos/wiki/UART%20Sensor%20Protocol/
 */
class EV3SensorPort
{
private:
    Stream *_connection;
    byte payload[350];
    int pos = 0;

    SensorConfig _config;

    const uint8_t SYNC = 0b00000000;
    const uint8_t NACK = 0b00000010;
    const uint8_t ACK = 0b00000100;
    const uint8_t TYPE = 0b01000000;
    const uint8_t MODES = 0b01001001;
    const uint8_t SPEED = 0b01010010;
    const uint8_t SELECT = 0b01000011;

    const uint8_t TYPE_COLOR_SENSOR = 29;
    const uint8_t TYPE_IR_SENSOR = 33;

    /**
     * Calculates the checksum
     */
    uint8_t calculateChecksum(uint8_t data[], int offset, int length)
    {
        uint8_t result = 0xff;
        for (int i = offset; i < offset + length; i++)
        {
            result ^= data[i];
        }
        return result;
    }

    bool parseInfoMessage(EV3SensorInfo *info)
    {
    }

    bool parseSpeed(Stream *con, SensorConfig *config)
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
        if (calculateChecksum(payload, 0, 5) == payload[5])
        {
            config->speed = payload[7 + 24] + (payload[7 + 3] << 16) + (payload[7 + 2] << 8) + (payload[7 + 1] << 0);
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

    bool parseType(Stream *con, SensorConfig *config, bool waitForType)
    {
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
        if (calculateChecksum(payload, 0, 2) == payload[2])
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
            Serial.print(payload[0], HEX);
            Serial.println(" with wrong checksum!!");
#endif
            return false;
        }
    }

public:
    EV3SensorPort(Stream *connection) : _connection(connection)
    {
    }

    void start()
    {
        // First wait for the TYPE message
        Serial.println("Waiting for TYPE header ...");
        while (true)
        {
            if (_connection->available() > 0)
            {
                byte v = _connection->read();
                if (v == TYPE)
                {
                    payload[pos++] = v;
                    break;
                }
            }
        }
        Serial.println("Found TYPE. Start reading payload ...");
        // Read rest of the message

        while (pos < 309)
        {
            if (_connection->available() > 0)
            {
                byte v = _connection->read();
                payload[pos++] = v;
            }
        }

        Serial.println("Payload read. Start waiting for ACK ...");

        // Wait for ACK
        while (true)
        {
            if (_connection->available() > 0)
            {
                byte v = _connection->read();
                if (v == ACK)
                {
                    break;
                }
                else
                {
                    payload[pos++] = v;
                }
            }
        }

        Serial.print("ACK found. Read ");
        Serial.print(pos);
        Serial.println(" bytes");

        Serial.print("READ sensor infos ");
        Serial.print("Type ");
        Serial.println(payload[1], HEX);

        for (int i = 0; i < pos; i++)
        {
            byte v = payload[i];
            if (v == TYPE)
            {
                Serial.print("TYPE ");
            }
            else if (v == MODES)
            {
                Serial.print("MODES ");
            }
            else if (v == SPEED)
            {
                Serial.print("SPEED ");
            }
            else
            {
                Serial.print(v, HEX);
                Serial.print(" ");
            }
        }

        // Checking type message
        byte checksum = calculateChecksum(payload, 0, 2);
        if (checksum == payload[2])
        {
            Serial.println("\nType checksum ok");
        }
        else
        {
            Serial.print("\nType checksum nok. Got ");
            Serial.print(payload[2], HEX);
            Serial.print(" expected ");
            Serial.print(checksum, HEX);
            return;
        }

        _config.type = payload[1];

        // Checking modes message
        if (payload[3] != MODES)
        {
            Serial.println("Expected MODES message, but none found ...");
            return;
        }
        _config.modes = payload[3 + 1] + 1;
        _config.modes_shown = payload[3 + 2] + 1;
        // pos == 3 +3 is checksum

        // Checking speed message
        if (payload[7] != SPEED)
        {
            Serial.println("Expected SPEED message, but none found ...");
            return;
        }

        // TODO Check claculartion
        _config.speed = payload[7 + 24] + (payload[7 + 3] << 16) + (payload[7 + 2] << 8) + (payload[7 + 1] << 0);
        Serial.print("Found uart speed ");
        Serial.println(_config.speed);
        // pos = 7 + 5 is checksum

        // Now the info messages come
        // First he name of the mode
        uint8_t modeMessage = payload[13];
        uint8_t msgLenght = 1 << ((modeMessage & 0b00111000) >> 3); // 2^LLL;
        uint8_t msgMode = modeMessage & 0b111;

        Serial.print("Info for mode = ");
        Serial.print(msgMode);
        Serial.print(" with length ");
        Serial.print(msgLenght);
        Serial.print(": ");

        // 13+1 is the infobyte 0

        for (int i = 13 + 2; i < 13 + 2 + msgLenght; i++)
        {
            Serial.print((char)payload[i]);
        }
        Serial.println();

        // Switch baudrate and set mode
    }
};