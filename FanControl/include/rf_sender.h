#pragma once
#include <Arduino.h>

// Initialisiert CC1101 auf 303.875 MHz OOK/ASK
bool rfSenderInit();

// Sendet ein Kommando n-mal (blockierend, µs-genaues Timing via delayMicroseconds)
// id[2]  = { ID_High, ID_Low }
// cmd    = Kommando-Byte (B2); B3 = ~cmd wird intern berechnet
// repeat = Anzahl Frame-Wiederholungen
void rfSend(const uint8_t id[2], uint8_t cmd, uint8_t repeat = 3);
