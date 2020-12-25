#include <Arduino.h>
#include <memory>
#include <Wire.h>

#ifndef HiTechnicIRLink_h
#define HiTechnicIRLink_h

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

enum struct IRLinkOutput : byte
{
    // RED
    A = 0,
    // BLUE
    B = 1
};

enum struct IRLinkPWM : byte
{
    FLOAT = 0b0000,
    FORWARD_STEP1 = 0b0001,
    FORWARD_STEP2 = 0b0010,
    FORWARD_STEP3 = 0b0011,
    FORWARD_STEP4 = 0b0100,
    FORWARD_STEP5 = 0b0101,
    FORWARD_STEP6 = 0b0110,
    FORWARD_STEP7 = 0b0111,
    BRAKE = 0b1000,
    BACKWARD_STEP7 = 0b1001,
    BACKWARD_STEP6 = 0b1010,
    BACKWARD_STEP5 = 0b1011,
    BACKWARD_STEP4 = 0b1100,
    BACKWARD_STEP3 = 0b1101,
    BACKWARD_STEP2 = 0b1110,
    BACKWARD_STEP1 = 0b1111,
};

/**
 * Control LEGO R/C trains and other motorized LEGO sets with the HiTechnic IRLink Sensor for the Mindstorms NXT. The IRLink uses Infrared signals to communicate with trains, Power Functions Motor controller and the Mindstorms RCX .
 * @see http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NIL1046
 * @see lejos.nxt.addon.IRLink from LeJOS NXJ
 * @see LEGO Power Functions RC specification (offical document from LEGO group)
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
    const static byte PF_MODE_COMBO_DIRECT = 0b001;
    const static byte PF_SINGLE_OUTPUT_MODE_PWM = 0b100;
    const static byte PF_SINGLE_OUTPUT_MODE_CLR_SET_TGL_INC_DEC = 0b110;

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

    /**
     * The toggle bit must switch between 0 and 1 between two following PF commands.
     */
    byte toggle = 0;

    /**
     * Sets the k-bit in in array of bytes
     * @see https://stackoverflow.com/questions/2525310/how-to-define-and-work-with-an-array-of-bits-in-c
     */
    void setBit(byte A[], int k);

    /**
     * Sends a raw power functions command. For the command format see the LEGO Power Functions RC specification
     */
    void sendPFCommand(int channel, byte mode, uint16_t data);

public:
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
    void begin();

    /**
     * Sends a power functions combo direct command to the given channel.
     */
    void sendPFComboDirect(int channel, IRLinkComboDirectMode opA, IRLinkComboDirectMode opB);

    /**
     * Sends a power functions single output mode command to the given channel and motor output.
     */
    void sendPFSingleOutputMode(int channel, IRLinkOutput output, IRLinkPWM value);
};

#endif