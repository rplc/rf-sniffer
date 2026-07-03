#pragma once
#include <Arduino.h>

bool rfSenderInit();
void rfConfigRx();
void rfSend(const uint8_t id[2], uint8_t cmd, uint8_t repeat = 3);
