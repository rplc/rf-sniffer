# 303.875 MHz Sniffer/Decoder/Replayer

## Hardware
- Wemos D1 Mini
- CC1101 module
- Antenna (copper wire, 25.5cm long)
  - Antenna needs to be shortened until rssi is best -> probably somewhere around 27.7cm
  - alternativly 300 or 315MHz antenna (good enough)

## Wiring
CC1101 Module: Side with components up, top row has 3 PINS, Bottom row 8 PINS
Top row:
- 1 & 3 = GND
- 2 = Antenna
Bottom row:
- 1 = 3.3V
- 2 = GND
- 3 = MOSI
- 4 = SCLK
- 5 = MISO
- 6 = GDO0
- 7 = GDO2
- 8 = CSN

CC1101  ->  Wemos
3.3 V   ->  3.3V
GND     ->  GND
MOSI    ->  D7
SCLK    ->  D5
MISO    ->  D6
GDO2 not needed
GDO0    ->  D2
CSN     ->  D8