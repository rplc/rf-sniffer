#pragma once
#include <Arduino.h>

// ─── Callback-Typen ──────────────────────────────────────────────────────────

// Fan-Frame empfangen (eigenes PWM-Protokoll)
using FanRxCallback  = void (*)(const uint8_t remoteId[2], uint8_t cmd);

// Lamp-Frame empfangen (Protokoll 2 kompatibel)
using LampRxCallback = void (*)(uint32_t code);

// ─── API ─────────────────────────────────────────────────────────────────────

void rfReceiverInit(FanRxCallback fanCb, LampRxCallback lampCb);
void rfReceiverLoop();
void rfReceiverEnterRx();
void rfReceiverLeaveRx();

void IRAM_ATTR rfPulseISR();
