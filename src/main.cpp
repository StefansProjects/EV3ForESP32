#include <Arduino.h>
#include "Lpf2HubEmulation.h"
#include "LegoinoCommon.h"

// create a hub instance
Lpf2HubEmulation myEmulatedHub("TrainHub", HubType::POWERED_UP_HUB);

void writeValueCallback(byte port, byte subcommand, std::string value)
{
  Serial.println("writeValueCallback: ");
  Serial.println(port, HEX);
  Serial.println(subcommand, HEX);
  Serial.println(value.c_str()[0], HEX);

  if (port == 0x00)
  {
    //do something when port 0x00 (A) has received a value
  }

  if (port == 0x01)
  {
    //do something when port 0x01 (B) has received a value
  }

  if (port == 0x32)
  {
    //do something when port 0x32 (LED) has received a value
  }
}

void setup()
{
  Serial.begin(115200);
  // define the callback function if a write message event on the characteristic occurs
  Serial.println("Initalize virtual hub");
  myEmulatedHub.setWritePortCallback(&writeValueCallback);
  myEmulatedHub.start();
}

void loop()
{
  // if an app is connected, attach some devices on the ports to signalize
  // the app that values could be received/written to that ports
  if (myEmulatedHub.isConnected && !myEmulatedHub.isPortInitialized)
  {
    delay(1000);
    myEmulatedHub.isPortInitialized = true;
    myEmulatedHub.attachDevice((byte)PoweredUpHubPort::A, DeviceType::TRAIN_MOTOR);
    delay(1000);
    myEmulatedHub.attachDevice((byte)PoweredUpHubPort::LED, DeviceType::HUB_LED);
    delay(1000);
    myEmulatedHub.attachDevice((byte)PoweredUpHubPort::B, DeviceType::TRAIN_MOTOR);
    delay(1000);
  }
}