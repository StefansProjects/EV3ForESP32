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
    uint8_t modes = 0;
    uint8_t modes_shown = 0;
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

    const static uint8_t SYNC = 0b00000000;
    const static uint8_t NACK = 0b00000010;
    const static uint8_t ACK = 0b00000100;
    const static uint8_t TYPE = 0b01000000;
    const static uint8_t MODES = 0b01001001;
    const static uint8_t SPEED = 0b01010010;
    const static uint8_t SELECT = 0b01000011;

    const static uint8_t TYPE_COLOR_SENSOR = 29;
    const static uint8_t TYPE_IR_SENSOR = 33;

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
    char *makeStringFromPayload(uint8_t data[], int maxlength);
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

    void sensorCommThread();

    /**
     * Makes a float from a sensor payload
     */
    float makeFloatFromPayload(uint8_t data[]);
    /**
     * Determines wheter the next message is a info message. If yes trie parse it and return the mode number.
     * If it is not, return the message byte.
     */
    bool parseInfoMessage(byte message, EV3SensorInfo *info);

    /**
     *  Parses the first info message (= mode name) for a supported mode
     */
    bool parseModeNameMessage(byte *header, EV3SensorInfo *info);

    /**
     * Parses the symbol name of the SI unit of a supported mode.
     */
    bool parseSymbolNameMessage(byte *header, EV3SensorInfo *info);

    /**
     * Pares the format message containing the format of the data message from sensor -> host.
     */
    bool parseFormatMessage(byte *header, EV3SensorInfo *info);

    /**
     *  Parses the second info message (= mode raw sensor readings) for a supported mode
     */
    bool parseModeRangeMessage(byte *header, EV3SensorInfo *info);
    /**
     * Parses the next uart speed from the stream.
     */
    bool parseSpeed(byte header, SensorConfig *config);

    /**
     * Parses the mode counts from the stream.
     */
    bool parseModeCount(byte header, SensorConfig *config);

    /**
     * Parses the type of  sensor from the stream.
     * If configures, also waits for the unique type byte to come.
     */
    bool parseType(byte message, SensorConfig *config);

    /**
     * Utlity method the read the next available byte from the connection.
     */
    byte readNextAvailableByte();

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
    void stop();

    /**
     * Selects the working mode of the sensor
     */
    void selectSensorMode(uint8_t mode);

    /**
     * Starts the EV3 Sensor communication protocol.
     */
    bool begin(int retries = 9);
};