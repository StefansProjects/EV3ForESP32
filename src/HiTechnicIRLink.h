#include <Arduino.h>
#include <memory>
#include <Wire.h>

#ifndef HiTechnicIRLink_h
#define HiTechnicIRLink_h

constexpr static char *TAG = "HiTechnicIRLink";

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)       \
    (byte & 0x80 ? '1' : '0'),     \
        (byte & 0x40 ? '1' : '0'), \
        (byte & 0x20 ? '1' : '0'), \
        (byte & 0x10 ? '1' : '0'), \
        (byte & 0x08 ? '1' : '0'), \
        (byte & 0x04 ? '1' : '0'), \
        (byte & 0x02 ? '1' : '0'), \
        (byte & 0x01 ? '1' : '0')

/**
 * PF functions motor operations in Combo direct mode
 */
enum struct IRLinkComboDirectMode : byte
{
    FLOAT = 0,
    FORWARD = 1,
    BACKWARD = 2,
    BRAKE = 3
};

/**
 * Control LEGO R/C trains and other motorized LEGO sets with the HiTechnic IRLink Sensor for the Mindstorms NXT. The IRLink uses Infrared signals to communicate with trains, Power Functions Motor controller and the Mindstorms RCX .
 * @see http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NIL1046
 * @see lejos.nxt.addon.IRLink from LeJOS NXJ
 */
class HiTechnicIRLink
{

private:
    const static byte ADDR = 0x02 >> 1;
    const static uint I2C_FREQ = 9768; // 9600 by spec, but minium freq is 9768

    //Registers
    const static byte TX_BUFFER = 0x40; // 40 to 4C
    const static byte TX_BUFFER_LEN = 0x4D;
    const static byte TX_MODE = 0x4E;
    const static byte TX_BUFFER_FLAG = 0x4F;

    const static byte TX_MAX_BUFFER_LEN = 13;

    // IRLink transmission modes
    const static byte TX_MODE_RCX = 0;
    const static byte TX_MODE_TRAIN = 1;
    const static byte TX_MODE_PF = 2;

    // PF Modes
    const static byte PF_MODE_COMBO_DIRECT = 1;

    // IR PF signal encoding parameters
    const byte MAX_BITS = TX_MAX_BUFFER_LEN * 8;
    const byte STOP_START_PAUSE = 7;
    const byte LOW_BIT_PAUSE = 2;
    const byte HIGH_BIT_PAUSE = 4;

    // RCX Remote operation code
    const static int RCX_REMOTE_BEEP = 0x8000;
    const static int RCX_REMOTE_STOP = 0x4000;
    const static int RCX_REMOTE_P5 = 0x2000;
    const static int RCX_REMOTE_P4 = 0x1000;
    const static int RCX_REMOTE_P3 = 0x0800;
    const static int RCX_REMOTE_P2 = 0x0400;
    const static int RCX_REMOTE_P1 = 0x0200;
    const static int RCX_REMOTE_C_BWD = 0x0100;
    const static int RCX_REMOTE_B_BWD = 0x0080;
    const static int RCX_REMOTE_A_BWD = 0x0040;
    const static int RCX_REMOTE_C_FWD = 0x0020;
    const static int RCX_REMOTE_B_FWD = 0x0010;
    const static int RCX_REMOTE_A_FWD = 0x0008;
    const static int RCX_REMOTE_MSG3 = 0x0004;
    const static int RCX_REMOTE_MSG2 = 0x0002;
    const static int RCX_REMOTE_MSG1 = 0x0001;

    const uint8_t _sda;
    const uint8_t _scl;
    TwoWire *_wire;

    byte toggle = 0;

    /**
     * Sets the k-bit in in array of bytes
     * @see https://stackoverflow.com/questions/2525310/how-to-define-and-work-with-an-array-of-bits-in-c
     */
    inline void setBit(byte A[], int k)
    {
        int i = k / 8;   //gives the corresponding index in the array A
        int pos = k % 8; //gives the corresponding bit position in A[i]

        unsigned int flag = 1; // flag = 0000.....00001

        flag = flag << (7 - pos); // flag = 0000...010...000   (shifted k positions)

        A[i] = A[i] | flag; // Set the bit at the k-th position in A[i]
    }

    /**
     * Clears the k-bit in in array of bytes
     * @see https://stackoverflow.com/questions/2525310/how-to-define-and-work-with-an-array-of-bits-in-c
     */
    inline void
    ClearBit(byte A[], int k)
    {
        A[k / 8] &= ~(1 << (k % 8));
    }

    void sendPFCommand(int channel, byte mode, uint16_t data)
    {
        ESP_LOGV(TAG, "Sending PF command to channel %d with mode %d and data %h", channel, mode, data);
        byte nibble1 = (byte)((toggle << 3) | channel);
        byte lrc = (byte)(0xF ^ nibble1 ^ mode ^ data);
        int pfData = (nibble1 << 12) | (mode << 8) | (data << 4) | lrc;

        byte pfCommand[TX_MAX_BUFFER_LEN + 3];
        std::fill(pfCommand, pfCommand + TX_MAX_BUFFER_LEN + 3, 0);
        int nextBit = 0;

        setBit(pfCommand, nextBit++);
        nextBit += STOP_START_PAUSE;
        for (int i = 15; i >= 0; i--)
        {
            setBit(pfCommand, nextBit++);
            nextBit += ((pfData >> i) & 1) == 0 ? LOW_BIT_PAUSE : HIGH_BIT_PAUSE;
        }
        setBit(pfCommand, nextBit++);
        nextBit += STOP_START_PAUSE;
        toggle ^= 1;

        ESP_LOGV(TAG, "PF command " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN "",
                 BYTE_TO_BINARY(pfCommand[0]),
                 BYTE_TO_BINARY(pfCommand[1]), BYTE_TO_BINARY(pfCommand[2]), BYTE_TO_BINARY(pfCommand[3]), BYTE_TO_BINARY(pfCommand[4]), BYTE_TO_BINARY(pfCommand[5]), BYTE_TO_BINARY(pfCommand[6]), BYTE_TO_BINARY(pfCommand[7]), BYTE_TO_BINARY(pfCommand[8]), BYTE_TO_BINARY(pfCommand[9]), BYTE_TO_BINARY(pfCommand[10]), BYTE_TO_BINARY(pfCommand[11]), BYTE_TO_BINARY(pfCommand[12]));

        pfCommand[13] = TX_MAX_BUFFER_LEN;
        pfCommand[14] = TX_MODE_PF;
        pfCommand[15] = 1;

        _wire->beginTransmission(ADDR);
        _wire->write(TX_BUFFER);
        _wire->write(pfCommand, TX_MAX_BUFFER_LEN + 3);
        _wire->endTransmission(true);

        _wire->flush();
    }

public:
    // PF motor operations
    const static byte PF_FLOAT = 0;
    const static byte PF_FORWARD = 1;
    const static byte PF_BACKWARD = 2;
    const static byte PF_BRAKE = 3;

    /**
     * Creates an HiTechnicIRLink using Wire hardware twi.
     */
    HiTechnicIRLink(uint8_t sda, uint8_t scl) : _sda(sda), _scl(scl), _wire(&Wire)
    {
    }

    /**
     * Creates an HiTechnicIRLink using the given TwoWire instance.
     */
    HiTechnicIRLink(TwoWire *wire, uint8_t sda, uint8_t scl, uint8_t clkpin) : _sda(sda), _scl(scl), _wire(wire)
    {
    }

    /**
     * Initalizes all pins and the TWI transmission.
     */
    void begin()
    {
        _wire->begin(_sda, _scl, I2C_FREQ);
    }

    void sendPFComboDirect(int channel, IRLinkComboDirectMode opA, IRLinkComboDirectMode opB)
    {
        sendPFCommand(channel, PF_MODE_COMBO_DIRECT, static_cast<byte>(opB) << 2 | static_cast<byte>(opA));
    }
};

#endif