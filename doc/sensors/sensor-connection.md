
# Notes on Sensor connection

## Pinout

### EV3 Sensor

NC -> 2
GND -> 3,1
VCC5V -> 4
TX -> 6
RX -> 5

### NXT Sensor

NC (9V, 5V Pullup) -> 1
GND -> 3,2
VCC5V -> 4
SDA -> 6
SCL -> 5

### EV3 Motor

Tacho1 (5V) -> 5
Tacho2 (5V) -> 6
GND -> 3
VCC5V -> 4
Motor1 -> 1
Motor2 -> 2

### Kabel

Weiß (M1) -> 1
Schwarz (M2) -> 2
Rot (GND) -> 3
Grün (5V) -> 4
Gelb (DATA1) -> 5
Blau (DATA2) -> 6

## Wireing

Directly connect Motor1/2 to output of motor driver `DRV8833`.
Directly connect VCC5V to 5V.

Connect Tacho1/2 to either `74HCT14D` Hex inverting Schmitt trigger for use as tacho signals only, or *TEST* a TXB0108 level converter when using both as motor tachos and sensor port!


## Protocol

https://sourceforge.net/p/lejos/wiki/UART%20Sensor%20Protocol/