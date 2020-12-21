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
void writeEV3ColorSensorColorToStream(EV3ColorSensorColor col, Stream *stream)
{
    switch (col)
    {
    case EV3ColorSensorColor::NONE:
        stream->print("none");
        break;
    case EV3ColorSensorColor::BLACK:
        stream->print("black");
        break;
    case EV3ColorSensorColor::BLUE:
        stream->print("blue");
        break;
    case EV3ColorSensorColor::GREEN:
        stream->print("green");
        break;
    case EV3ColorSensorColor::YELLOW:
        stream->print("yellow");
        break;
    case EV3ColorSensorColor::RED:
        stream->print("red");
        break;
    case EV3ColorSensorColor::WHITE:
        stream->print("white");
        break;
    case EV3ColorSensorColor::BROWN:
        stream->print("brown");
        break;
    }
}
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

    void messageHandler(uint8_t mode, uint8_t *message, int length)
    {
        switch (mode)
        {
        case 0:
            if (onColReflect)
                onColReflect(message[0]);
            break;
        case 1:
            if (onColAmbient)
                onColAmbient(message[0]);
            break;
        case 2:
            if (onColColor)
                onColColor(static_cast<EV3ColorSensorColor>(message[0]));
            break;
        default:
#ifdef EV3SENSOR_SERIAL_DEBUG
            Serial.print("Currently not supported EV3 color sensor mode ");
            Serial.println(mode);
#endif
        }
    }

public:
    EV3ColorSensor(EV3SensorPort *port) : _port(port)
    {
        _port->setMessageHandler([this](uint8_t mode, uint8_t *message, int length) { this->messageHandler(mode, message, length); });
    }

    void setMode(EV3ColorSensorMode mode)
    {
        _port->selectSensorMode(static_cast<uint8_t>(mode));
    }

    void setOnColReflect(std::function<void(byte)> h)
    {
        this->onColReflect = h;
    }

    void setOnColAmbient(std::function<void(byte)> h)
    {
        this->onColAmbient = h;
    }

    void setOnColColor(std::function<void(EV3ColorSensorColor)> h)
    {
        this->onColColor = h;
    }
};
#endif