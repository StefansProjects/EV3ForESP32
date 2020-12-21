#include "esp_log.h"
#include "EV3IRSensor.h"

static const char *TAG = "EV3IRSensor";

void EV3IRSensor::messageHandler(uint8_t mode, uint8_t *message, int length)
{
    switch (mode)
    {
    case 2:
        handleIRRemote(message, length);
        break;
    }
}

void EV3IRSensor::handleIRRemote(uint8_t *message, int length)
{
    // Iterate over the 4 channels
    for (int i = 0; i < 4; i++)
    {
        auto value = message[i];
        EV3RemoteState red = EV3RemoteState::NONE;
        EV3RemoteState blue = EV3RemoteState::NONE;

        if (value != 0)
        {
            ESP_LOGV(TAG, "EV3 IR Remote message %d on channel %d (msg length %d)", value, i, length);
        }

        // Parse remote message
        switch (value)
        {
        case 0:
            red = EV3RemoteState::NONE;
            blue = EV3RemoteState::NONE;
            break;
        case 1:
            red = EV3RemoteState::UP;
            break;
        case 2:
            red = EV3RemoteState::DOWN;
            break;
        case 3:
            blue = EV3RemoteState::UP;
            break;
        case 4:
            blue = EV3RemoteState::DOWN;
            break;
        case 5:
            red = EV3RemoteState::UP;
            blue = EV3RemoteState::UP;
            break;
        case 6:
            red = EV3RemoteState::UP;
            blue = EV3RemoteState::DOWN;
            break;

        case 7:
            red = EV3RemoteState::DOWN;
            blue = EV3RemoteState::UP;
            break;

        case 8:
            red = EV3RemoteState::DOWN;
            blue = EV3RemoteState::DOWN;
            break;

        case 9:
            red = EV3RemoteState::BEACON;
            blue = EV3RemoteState::BEACON;
            break;

        case 10:
            this->onIRRemote(i, EV3RemoteState::UP, blue);
            red = EV3RemoteState::DOWN;
            break;

        case 11:
            this->onIRRemote(i, red, EV3RemoteState::UP);
            blue = EV3RemoteState::DOWN;
            break;
        default:
            ESP_LOGE(TAG, "Unkown value from IR remote: %x", value);
        }
        // finally call the callback with the new values.
        this->onIRRemote(i, red, blue);
    }
}
