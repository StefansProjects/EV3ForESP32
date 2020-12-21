#include "EV3ColorSensor.h"

void EV3ColorSensor::messageHandler(uint8_t mode, uint8_t *message, int length)
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