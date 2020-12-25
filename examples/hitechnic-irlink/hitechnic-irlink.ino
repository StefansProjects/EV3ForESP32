#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <Arduino.h>
#include "HiTechnicIRLink.h"

const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int tacho1Pin1 = 18; // = SCL
const int tacho1Pin2 = 19; // = SDA

HiTechnicIRLink s1 = HiTechnicIRLink(tacho1Pin2, tacho1Pin1);

void setup()
{
    // Configure motor outputs
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);

    // Now power supply necessary for sensor
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);

    // Serial comm to the host machine
    Serial.begin(115200);

    // Start the sensor
    s1.begin();
}

void loop()
{
    // Speed motor slowly up and down again.
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP1);
    delay(250);
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP2);
    delay(250);
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP3);
    delay(250);
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP4);
    delay(250);
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP5);
    delay(250);
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP6);
    delay(250);
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP7);
    delay(250);
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP6);
    delay(250);
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP5);
    delay(250);
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP4);
    delay(250);
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP3);
    delay(250);
    s1.sendPFSingleOutputMode(1, IRLinkOutput::A, IRLinkPWM::FORWARD_STEP2);
    delay(250);
}