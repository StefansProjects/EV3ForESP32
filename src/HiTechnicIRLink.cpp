#include "HiTechnicIRLink.h"

constexpr static char *TAG = "HiTechnicIRLink";

void HiTechnicIRLink::setBit(byte A[], int k)
{
    int i = k / 8;   //gives the corresponding index in the array A
    int pos = k % 8; //gives the corresponding bit position in A[i]

    unsigned int flag = 1; // flag = 0000.....00001

    flag = flag << (7 - pos); // flag = 0000...010...000   (shifted k positions)

    A[i] = A[i] | flag; // Set the bit at the k-th position in A[i]
}

void HiTechnicIRLink::sendPFCommand(int channel, byte mode, uint16_t data)
{
    ESP_LOGV(TAG, "Sending PF command to channel %d with mode %d and data %x", channel, mode, data);
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

    pfCommand[13] = TX_MAX_BUFFER_LEN;
    pfCommand[14] = TX_MODE_PF;
    pfCommand[15] = 1;

    _wire->beginTransmission(ADDR);
    _wire->write(TX_BUFFER);
    _wire->write(pfCommand, TX_MAX_BUFFER_LEN + 3);
    _wire->endTransmission(true);

    _wire->flush();
}

void HiTechnicIRLink::begin()
{
    _wire->begin(_sda, _scl, I2C_FREQ);
}

char *comboDirectModeToString(IRLinkComboDirectMode op)
{
    switch (static_cast<byte>(op))
    {
    case 0b00:
        return "float";
    case 0b01:
        return "forward";
    case 0b10:
        return "backward";
    case 0b11:
        return "brake";
    default:
        return "[unknown]";
    }
}

void HiTechnicIRLink::sendPFComboDirect(int channel, IRLinkComboDirectMode opA, IRLinkComboDirectMode opB)
{
    ESP_LOGD(TAG, "IRLink PF combo direct mode for channel %d red = %s and blue = %s", channel, comboDirectModeToString(opA), comboDirectModeToString(opB));
    sendPFCommand(channel, PF_MODE_COMBO_DIRECT, static_cast<byte>(opB) << 2 | static_cast<byte>(opA));
}

void HiTechnicIRLink::sendPFSingleOutputMode(int channel, IRLinkOutput output, IRLinkPWM value)
{
    ESP_LOGD(TAG, "IRLink PF single output mode for channel %d, %s output and value %x", channel, output == IRLinkOutput::A ? "red" : "blue", static_cast<byte>(value));
    sendPFCommand(channel, output == IRLinkOutput::A ? PF_SINGLE_OUTPUT_MODE_PWM : (PF_SINGLE_OUTPUT_MODE_PWM + 1), static_cast<byte>(value));
}