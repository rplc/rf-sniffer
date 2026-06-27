#pragma once
#include <Arduino.h>
#include <PubSubClient.h>

// Initialisiert MQTT-Client und abonniert alle Fan-Topics
void mqttInit(PubSubClient& client);

// Im loop() aufrufen – hält Verbindung aufrecht und verarbeitet Nachrichten
void mqttLoop(PubSubClient& client);
