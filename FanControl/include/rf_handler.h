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

// Sendeleistung setzen (PATABLE-Rohwert, siehe CC1101_PWR_* in config.h).
// Kann jederzeit aufgerufen werden (auch während TX-Idle zwischen Frames);
// wirkt erst auf den nächsten tatsächlich gesendeten Frame.
void rfSetTxPower(uint8_t patableValue);

// Im loop() aufrufen
void rfLoop();

// Live-RSSI (dBm) zum Zeitpunkt des Aufrufs. Wichtig: der 433MHz-CC1101 läuft
// im rohen Async-Pulsmodus (kein RadioLib-Paketmodus, kein readData()), daher
// ist RadioLibs eigener getRSSI()-Cache hier NIE gültig (bleibt permanent auf
// dem Initialwert stehen). Diese Funktion liest das RSSI-Statusregister immer
// live und direkt – von fan_handler/lamp_handler direkt nach dem Decodieren
// eines Frames aufzurufen, um den RSSI mit ins MQTT-JSON zu packen.
float rfGetRSSI();

// ISR (public für IRAM_ATTR)
void IRAM_ATTR rfPulseISR();
