#pragma once
#include <Arduino.h>

// Initialisiert CC1101 (Frequenz, Modulation, PA-Leistung)
bool rfSenderInit();

// Konfiguriert CC1101-Register für RX-Modus und schaltet SetRx()
// Wird von rfReceiverEnterRx() aufgerufen
void rfConfigRx();

// Sendet ein Kommando (blockierend, schaltet intern TX→RX)
void rfSend(const uint8_t id[2], uint8_t cmd, uint8_t repeat = 3);
