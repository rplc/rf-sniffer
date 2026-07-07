#pragma once
#include <Arduino.h>
#include <PubSubClient.h>

void mqttInit(PubSubClient& client);
void mqttLoop(PubSubClient& client);
