#pragma once
#include <Arduino.h>

void binaryClockOnMqttMessage(const char* topic, const uint8_t* payload, unsigned int length);

void binaryClockLoop();

void binaryClockInit();
