#pragma once
#include <Arduino.h>

// Callbacks
using FanRxCallback  = void (*)(const uint8_t remoteId[2], uint8_t cmd);
using LampRxCallback = void (*)(uint32_t code);

// Init: CC1101 konfigurieren, startet sofort im RX-Modus
bool rfInit(FanRxCallback fanCb, LampRxCallback lampCb);

// TX-Helfer: Umschalten IDLE→TX→RX mit Settling-Zeiten
// Aufgerufen von fan_handler und lamp_handler vor/nach dem Senden
void rfEnterTx();   // RX deaktivieren, CC1101 in TX schalten
void rfEnterRx();   // CC1101 in RX schalten, ISR aktivieren

// Im loop() aufrufen
void rfLoop();

// ISR (public für IRAM_ATTR)
void IRAM_ATTR rfPulseISR();
