#pragma once
#include <Arduino.h>
#include <PubSubClient.h>

// Zweiter CC1101 auf 868 MHz – empfängt Bresser 7-in-1 Wetterstation
// RX-Only, eigener SPI-Bus (HSPI)
void bresserInit(PubSubClient& client);
void bresserLoop();
