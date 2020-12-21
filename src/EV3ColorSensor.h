#include "EV3SensorPort.h"

#ifndef EV3ColorSensor_h
#define EV3ColorSensor_h

/**
 * Color codes for the COL_COLOR mode of the EV3 color sensor
 */
enum struct EV3ColorSensorColor : uint8_t
{
    NONE = 0,
    BLACK = 1,
    BLUE = 2,
    GREEN = 3,
    YELLOW = 4,
    RED = 5,
    WHITE = 6,
    BROWN = 7
};

/**
 * Utility method to create a humand readable string of a color.
 */
void writeEV3ColorSensorColorToStream(EV3ColorSensorColor col, Stream *stream);

/**
 *  Available modes of the EV3 color sensor.
 */
enum struct EV3ColorSensorMode : uint8_t
{
    COL_CAL = 5,
    RGB_RAW = 4,
    REF_RAW = 3,
    COL_COLOR = 2,
    COL_AMBIENT = 1,
    COL_REFLECT = 0
};

/**
 * Handler of the EV3 color sensor.
 */
class EV3ColorSensor
{
private:
    EV3SensorPort *_port;

    std::function<void(byte)> onColReflect;
    std::function<void(byte)> onColAmbient;
    std::function<void(EV3ColorSensorColor)> onColColor;

    /**
     * Handler for single messages from the EV3 sensor port
     */
    void messageHandler(uint8_t mode, uint8_t *message, int length);

public:
    EV3ColorSensor(EV3SensorPort *port) : _port(port)
    {
        _port->setMessageHandler([this](uint8_t mode, uint8_t *message, int length) { this->messageHandler(mode, message, length); });
    }

    /**
     * Set the mode of the color sensor
     */
    void setMode(EV3ColorSensorMode mode)
    {
        _port->selectSensorMode(static_cast<uint8_t>(mode));
    }

    /**
     * Set the handler for COL_REFLECT messages
     */
    void setOnColReflect(std::function<void(byte)> h)
    {
        this->onColReflect = h;
    }

    /**
     * Set the handler for COL_AMBIENT messages
     */
    void setOnColAmbient(std::function<void(byte)> h)
    {
        this->onColAmbient = h;
    }

    /**
     * Set the handler for COL_COLOR messages
     */
    void setOnColColor(std::function<void(EV3ColorSensorColor)> h)
    {
        this->onColColor = h;
    }
};
#endif