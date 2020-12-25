#include "HiTechnicIRLink.h"

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

void HiTechnicIRLink::setBit(byte A[], int k)
{
    int i = k / 8;   //gives the corresponding index in the array A
    int pos = k % 8; //gives the corresponding bit position in A[i]

    unsigned int flag = 1; // flag = 0000.....00001

    flag = flag << (7 - pos); // flag = 0000...010...000   (shifted k positions)

    A[i] = A[i] | flag; // Set the bit at the k-th position in A[i]
}

void HiTechnicIRLink::clearBit(byte A[], int k)
{
    A[k / 8] &= ~(1 << (k % 8));
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

void HiTechnicIRLink::begin()
{
    _wire->begin(_sda, _scl, I2C_FREQ);
}

void HiTechnicIRLink::sendPFComboDirect(int channel, IRLinkComboDirectMode opA, IRLinkComboDirectMode opB)
{
    sendPFCommand(channel, PF_MODE_COMBO_DIRECT, static_cast<byte>(opB) << 2 | static_cast<byte>(opA));
}

void HiTechnicIRLink::sendPFSingleOutputMode(int channel, IRLinkOutput output, IRLinkPWM value)
{
    sendPFCommand(channel, output == IRLinkOutput::A ? PF_SINGLE_OUTPUT_MODE_PWM : (PF_SINGLE_OUTPUT_MODE_PWM + 1), static_cast<byte>(value));
}