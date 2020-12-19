#include <stdint.h>
#include <Arduino.h>
#include <functional>

#define EV3SENSOR_SERIAL_DEBUG 1

struct EV3SensorInfo
{
    byte mode;
    char *name = nullptr;
    float rawLowest = 0.0f;
    float rawHighest = 0.0f;
    float pctLowest = 0.0f;
    float pctHighest = 0.0f;
    float siLowest = 0.0f;
    float siHighest = 0.0f;
    char *siSymbol = nullptr;

    uint8_t numberOfItems = 0;
    uint8_t dataTypeOfItem = 0;
    uint8_t numberOfDigits = 0;
    uint8_t numberOfDecimals = 0;
};

struct SensorConfig
{
    uint8_t type;
    uint8_t modes;
    uint8_t modes_shown;
    uint32_t speed;

    EV3SensorInfo *infos;
};

/**
 * 
 * @see https://sourceforge.net/p/lejos/wiki/UART%20Sensor%20Protocol/
 */
class EV3SensorPort
{
private:
    Stream *_connection;

    SensorConfig _config;

    std::function<void(int)> _baudrateSetter;

    const uint8_t SYNC = 0b00000000;
    const uint8_t NACK = 0b00000010;
    const uint8_t ACK = 0b00000100;
    const uint8_t TYPE = 0b01000000;
    const uint8_t MODES = 0b01001001;
    const uint8_t SPEED = 0b01010010;
    const uint8_t SELECT = 0b01000011;

    const uint8_t TYPE_COLOR_SENSOR = 29;
    const uint8_t TYPE_IR_SENSOR = 33;

    // Prevent simultanious access quadrature encoder
    SemaphoreHandle_t _serialMutex;
    // Handle for the motor control task.
    TaskHandle_t _sensorCommThreadHandle = nullptr;

    /**
     * Calculates the checksum
     */
    uint8_t calculateChecksum(uint8_t data[], int length);

    /**
     * Makes a string from a sensor payload. Since the payload is padded with zeros, the method tries first to determine the true size of the string.
     */
    char *  EV3SensorPort::makeStringFromPayload(uint8_t data[], int maxlength);
    /*
     * Utility method to bind a class method to a FreeRTOS task.
     * 
     * @see https://www.freertos.org/FreeRTOS_Support_Forum_Archive/July_2010/freertos_Is_it_possible_create_freertos_task_in_c_3778071.html
     * @see https://forum.arduino.cc/index.php?topic=674975.0
     * 
     * @param parm Reference to the actual EV3SensorPort object to handle.
     */
    static void sensorCommThreadHelper(void *parm)
    {
        static_cast<EV3SensorPort *>(parm)->sensorCommThread();
    }

    void sensorCommThread()
    {
        for (;;)
        {
#ifdef EV3SENSOR_SERIAL_DEBUG
            Serial.println("Sending nack");
#endif
            //xSemaphoreTake(_serialMutex, portMAX_DELAY);
            _connection->write(NACK);
            vTaskDelay(90 / portTICK_PERIOD_MS);
            // xSemaphoreGive(_serialMutex);
        }
    }

    /**
     * Makes a float from a sensor payload
     */
    float makeFloatFromPayload(uint8_t data[]);
    /**
     * Determines wheter the next message is a info message. If yes trie parse it and return the mode number.
     * If it is not, return the message byte.
     */
    uint8_t parseInfoMessage(EV3SensorInfo *info);

    /**
     *  Parses the first info message (= mode name) for a supported mode
     */
    uint8_t parseModeNameMessage(byte *header, EV3SensorInfo *info);

    /**
     * Pares the format message containing the format of the data message from sensor -> host.
     */
    uint8_t parseFormatMessage(byte *header, EV3SensorInfo *info);

    /**
     *  Parses the second info message (= mode raw sensor readings) for a supported mode
     */
    uint8_t parseModeRangeMessage(byte *header, EV3SensorInfo *info);
    /**
     * Parses the next uart speed from the stream.
     */
    bool parseSpeed(SensorConfig *config);

    /**
     * Parses the mode counts from the stream.
     */
    bool parseModeCount(SensorConfig *config);

    /**
     * Parses the type of  sensor from the stream.
     * If configures, also waits for the unique type byte to come.
     */
    bool parseType(SensorConfig *config, bool waitForType);

public:
    EV3SensorPort(Stream *connection, std::function<void(int)> baudrateSetter) : _connection(connection), _baudrateSetter(baudrateSetter)
    {
        this->_serialMutex = xSemaphoreCreateMutex();
    }

    /**
     * Returns the sensor configuration found during the first phase of the EV3 protocol.
     */
    SensorConfig *getCurrentConfig()
    {
        return &_config;
    }

    /**
     *  Stops the EV3 Sensor.   
     */
    void stop()
    {
        if (_sensorCommThreadHandle)
        {
            vTaskDelete(_sensorCommThreadHandle);
            _sensorCommThreadHandle = nullptr;
        }
    }

    void selectSensorMode(uint8_t mode)
    {
        byte payload[3];
        payload[0] = SELECT;
        payload[1] = mode;
        payload[3] = this->calculateChecksum(payload, 2);
        xSemaphoreTake(_serialMutex, portMAX_DELAY);
        this->_connection->write(payload, 3);
        xSemaphoreGive(_serialMutex);
    }

    /**
     * Starts the EV3 Sensor communication protocol.
     */
    bool begin(int retries = 9)
    {
        stop();

        xSemaphoreTake(_serialMutex, portMAX_DELAY);
        if (!this->parseType(&_config, true))
        {
            if (retries > 0)
            {
#ifdef EV3SENSOR_SERIAL_DEBUG
                Serial.println("Retry again to find start sensor (Failed to parse type)");
#endif
                xSemaphoreGive(_serialMutex);
                return begin(retries - 1);
            }
            else
            {
                xSemaphoreGive(_serialMutex);
                return false;
            }
        }
        if (!this->parseModeCount(&_config))
        {
            if (retries > 0)
            {
#ifdef EV3SENSOR_SERIAL_DEBUG
                Serial.println("Retry again to find start sensor (Failed to parse mode count)");
#endif
                xSemaphoreGive(_serialMutex);
                return begin(retries - 1);
            }
            else
            {
                xSemaphoreGive(_serialMutex);
                return false;
            }
        }
        if (!this->parseSpeed(&_config))
        {
            if (retries > 0)
            {
#ifdef EV3SENSOR_SERIAL_DEBUG
                Serial.println("Retry again to find start sensor (Failed to parse sensor baudrate)");
#endif
                xSemaphoreGive(_serialMutex);
                return begin(retries - 1);
            }
            else
            {
                xSemaphoreGive(_serialMutex);
                return false;
            }
        }
        _config.infos = new EV3SensorInfo[_config.modes_shown];

        // Mode we currrently get infos for
        byte currentMode = -1;
        // Position in the info array
        byte pos = -1;

        for(int i = 0;i<)



        this->_baudrateSetter(this->_config.speed);
        xSemaphoreGive(_serialMutex);

        // Pass control to own communication thread.

        delay(40);
        xTaskCreate(
            &sensorCommThreadHelper,
            "EV3Sensor",
            10000,
            this,
            1,
            &_sensorCommThreadHandle // Task handle
        );

        return true;
    }
};