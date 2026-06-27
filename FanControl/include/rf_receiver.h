#pragma once
#include <Arduino.h>

// Callback-Typ: wird aufgerufen wenn ein gültiger Frame empfangen wurde.
// remoteId[2] = { B0, B1 }, cmd = B2
using RfRxCallback = void (*)(const uint8_t remoteId[2], uint8_t cmd);

// Initialisiert RX-Modus und registriert Callback.
// Muss NACH rfSenderInit() aufgerufen werden.
void rfReceiverInit(RfRxCallback callback);

// Im loop() aufrufen – wertet gepufferte Pulse aus (ISR-safe).
void rfReceiverLoop();

// TX/RX-Umschaltung – wird von rf_sender.cpp aufgerufen
void rfReceiverEnterRx();
void rfReceiverLeaveRx();

// Intern: vom ISR aufgerufen (public damit IRAM_ATTR-Makro greift)
void IRAM_ATTR rfPulseISR();
