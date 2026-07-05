#pragma once
#include <Arduino.h>

using RfRxCallback = void (*)(const uint8_t remoteId[2], uint8_t cmd);

void rfReceiverInit(RfRxCallback callback);
void rfReceiverLoop();
void rfReceiverEnterRx();
void rfReceiverLeaveRx();

void IRAM_ATTR rfRisingISR();
void IRAM_ATTR rfFallingISR();
void IRAM_ATTR rfPulseISR();  // unused, kept for compatibility
