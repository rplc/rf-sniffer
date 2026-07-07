#pragma once
#include <Arduino.h>
#include <PubSubClient.h>

// Initialisiert den Lamp-Handler
void lampInit(PubSubClient& client);

// Im loop() aufrufen
void lampLoop();

// Callback vom rf_handler wenn ein Lamp-Frame empfangen wurde
void lampOnRfFrame(uint32_t code);

// Wird von mqtt_handler aufgerufen wenn eine Lamp-Topic-Nachricht ankommt
void lampOnMqttMessage(const char* topic, const uint8_t* payload, unsigned int length);
