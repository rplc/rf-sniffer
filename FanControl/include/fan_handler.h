#pragma once
#include <Arduino.h>
#include <PubSubClient.h>

// Wird vom rf_handler aufgerufen wenn ein Fan-Frame empfangen wurde
void fanOnRfFrame(const uint8_t remoteId[2], uint8_t cmd);

// Wird von mqtt_handler aufgerufen wenn eine Fan-Topic-Nachricht ankommt
void fanOnMqttMessage(const char* topic, const uint8_t* payload, unsigned int length);

// Im loop() aufrufen (Dimmer-Tick)
void fanLoop();

void fanInit(PubSubClient& client);
