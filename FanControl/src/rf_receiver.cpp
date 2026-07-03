#include "rf_receiver.h"
#include "rf_sender.h"
#include "config.h"
#include <SmartRC_CC1101.h>

// ─── Timing-Toleranzen ───────────────────────────────────────────────────────
static constexpr uint32_t HEADER_MIN   = 3000;
static constexpr uint32_t HEADER_MAX   = 6000;
static constexpr uint32_t BIT_ONE_MIN  =  900;
static constexpr uint32_t BIT_ZERO_MIN =  300;
static constexpr uint32_t BIT_MAX      = 2500;
static constexpr uint32_t GAP_MIN      = 10000;

// ─── ISR Double-Buffer ───────────────────────────────────────────────────────
static constexpr uint8_t MAX_PULSES = 40;

struct PulseBuffer {
    volatile uint32_t pulses[MAX_PULSES];
    volatile uint8_t  count;
    volatile bool     ready;
};

static PulseBuffer       s_buf[2];
static volatile uint8_t  s_writeIdx = 0;
static volatile uint32_t s_lastEdge = 0;
static volatile bool     s_inFrame  = false;

static RfRxCallback s_callback = nullptr;
static bool         s_rxActive = false;

// ─── ISR ─────────────────────────────────────────────────────────────────────

void IRAM_ATTR rfPulseISR() {
    const uint32_t now  = micros();
    const uint32_t diff = now - s_lastEdge;
    s_lastEdge = now;

    const bool pinHigh = digitalRead(CC1101_GDO0_PIN);

    if (pinHigh) {
        if (s_inFrame && diff >= GAP_MIN) {
            PulseBuffer& wb = s_buf[s_writeIdx];
            if (wb.count >= 33) {
                wb.ready   = true;
                s_writeIdx ^= 1;
                s_buf[s_writeIdx].count = 0;
                s_buf[s_writeIdx].ready = false;
            }
            s_inFrame = false;
        }
    } else {
        if (diff >= HEADER_MIN && diff <= HEADER_MAX) {
            PulseBuffer& wb = s_buf[s_writeIdx];
            wb.count     = 0;
            wb.ready     = false;
            wb.pulses[0] = diff;
            wb.count     = 1;
            s_inFrame    = true;
        } else if (s_inFrame && diff >= BIT_ZERO_MIN && diff <= BIT_MAX) {
            PulseBuffer& wb = s_buf[s_writeIdx];
            if (wb.count < MAX_PULSES) {
                wb.pulses[wb.count++] = diff;
            } else {
                s_inFrame = false;
            }
        } else if (s_inFrame) {
            s_inFrame = false;
            s_buf[s_writeIdx].count = 0;
        }
    }
}

// ─── Frame dekodieren ────────────────────────────────────────────────────────

static bool decodeFrame(const uint32_t* pulses, uint8_t count,
                        uint8_t outId[2], uint8_t& outCmd) {
    if (count < 33) {
        Serial.printf("[RX] Zu wenig Pulse: %d\n", count);
        return false;
    }

    uint8_t bytes[4] = {};
    for (uint8_t i = 0; i < 32; i++) {
        const uint32_t p = pulses[1 + i];
        bool bit;
        if      (p >= BIT_ONE_MIN  && p <= BIT_MAX)    bit = true;
        else if (p >= BIT_ZERO_MIN && p < BIT_ONE_MIN) bit = false;
        else {
            Serial.printf("[RX] Ungültiger Puls Bit%d: %uµs\n", i, (unsigned)p);
            return false;
        }
        bytes[i / 8] |= (bit << (7 - (i % 8)));
    }

    if (bytes[3] != (uint8_t)(~bytes[2])) {
        Serial.printf("[RX] Checksum fail: B2=0x%02X B3=0x%02X\n", bytes[2], bytes[3]);
        return false;
    }

    outId[0] = bytes[0];
    outId[1] = bytes[1];
    outCmd   = bytes[2];
    return true;
}

// ─── RX-Modus ────────────────────────────────────────────────────────────────

void rfReceiverEnterRx() {
    // CC1101 in RX-Modus (SmartRC setzt IOCFG0 korrekt)
    ELECHOUSE_cc1101.SetRx();
    if (s_rxActive) return;
    pinMode(CC1101_GDO0_PIN, INPUT);
    s_inFrame  = false;
    s_lastEdge = micros();
    attachInterrupt(CC1101_GDO0_PIN, rfPulseISR, CHANGE);
    s_rxActive = true;
    Serial.println("[RX] RX aktiv");
}

void rfReceiverLeaveRx() {
    if (!s_rxActive) return;
    detachInterrupt(CC1101_GDO0_PIN);
    s_rxActive = false;
}

// ─── Öffentliche API ─────────────────────────────────────────────────────────

void rfReceiverInit(RfRxCallback callback) {
    s_callback = callback;
    s_buf[0]   = {};
    s_buf[1]   = {};
    s_writeIdx = 0;
}

void rfReceiverLoop() {
    const uint8_t readIdx = s_writeIdx ^ 1;
    PulseBuffer&  rb      = s_buf[readIdx];

    if (!rb.ready) return;

    uint32_t pulses[MAX_PULSES];
    uint8_t  count;
    noInterrupts();
    count = rb.count;
    memcpy(pulses, (const void*)rb.pulses, count * sizeof(uint32_t));
    rb.ready = false;
    interrupts();

    uint8_t id[2], cmd;
    if (decodeFrame(pulses, count, id, cmd)) {
        Serial.printf("[RX] OK: ID=%02X%02X CMD=0x%02X\n", id[0], id[1], cmd);
        if (s_callback) s_callback(id, cmd);
    }
}
