#include "EV3SensorPort.h"

#ifndef EV3IRSensor_h
#define EV3IRSensor_h

enum struct EV3IRSensorMode : uint8_t
{
    IR_CAL = 5,
    IR_S_ALT = 4,
    IR_REM_A = 3,
    IR_REMOTE = 2,
    IR_SEEK = 1,
    IR_PROX = 0
};

enum struct EV3RemoteState
{
    NONE,
    UP,
    DOWN,
    BEACON
};

class EV3IRSensor
{
private:
    EV3SensorPort *_port;

    std::function<void(uint8_t, EV3RemoteState, EV3RemoteState)> onIRRemote;

    /**
     * Handler for single messages from the EV3 sensor port
     */
    void messageHandler(uint8_t mode, uint8_t *message, int length);

    /**
     * Handles the IR Remote mode (=2)
     */
    void handleIRRemote(uint8_t *message, int length);

public:
    EV3IRSensor(EV3SensorPort *port) : _port(port)
    {
        _port->setMessageHandler([this](uint8_t mode, uint8_t *message, int length) { this->messageHandler(mode, message, length); });
    }

    /**
     * Set the mode of the color sensor
     */
    void setMode(EV3IRSensorMode mode)
    {
        _port->selectSensorMode(static_cast<uint8_t>(mode));
    }

    /**
     * Sets the handler for IR_REMOTE mode data packets.
     * The handler gets the IR remote channel and the state of the red and blue button.
     */
    void setOnIRRemote(std::function<void(uint8_t, EV3RemoteState, EV3RemoteState)> handler)
    {
        this->onIRRemote = handler;
    }
};

#endif