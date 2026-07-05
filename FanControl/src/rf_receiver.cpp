#include "rf_receiver.h"
#include "rf_sender.h"
#include "config.h"
#include <SmartRC_CC1101.h>

// ─── Timing-Toleranzen (±40%) ────────────────────────────────────────────────
// Protokoll: Bit im LOW-Puls, HIGH = konstanter Marker (~560µs)
// Header HIGH: ~8120µs → Fenster 5000-12000µs
// LOW Bit-0:   ~560µs  → < 900µs
// LOW Bit-1:  ~1680µs  → >= 900µs
// Frame-Gap:  ~20000µs → > 10000µs

static constexpr uint32_t HEADER_HIGH_MIN = 5000;
static constexpr uint32_t HEADER_HIGH_MAX = 12000;
static constexpr uint32_t BIT_THRESHOLD   =  900;   // LOW >= 900µs → Bit 1
static constexpr uint32_t BIT_MIN         =  200;   // LOW >= 200µs → gültiges Bit
static constexpr uint32_t BIT_MAX         = 3000;   // LOW <= 3000µs → gültiges Bit
static constexpr uint32_t GAP_MIN         = 10000;  // LOW > 10ms → Frame-Ende

// ─── ISR Double-Buffer ───────────────────────────────────────────────────────
// Wir speichern nur LOW-Pulse (die Bit-Information)
// plus den Header-HIGH zur Frame-Erkennung
// Format: Bit31=1 → war HIGH-Puls, Bit31=0 → war LOW-Puls

static constexpr uint8_t MAX_PULSES = 80;

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
static volatile uint32_t s_isrCount      = 0;
static volatile uint32_t s_maxHighSeen   = 0;
static volatile uint8_t  s_lastFrameCount = 0;  // Pulse im letzten unvollständigen Frame
static volatile uint32_t s_lastGapDur   = 0;
static volatile uint8_t  s_maxFrameCount = 0;
static volatile uint8_t  s_lastBitLow = 0;
static bool         s_rxActive = false;

// ─── ISR ─────────────────────────────────────────────────────────────────────

// ─── ISR (einzelner CHANGE-Interrupt) ────────────────────────────────────────
// ESP32 unterstützt nur einen Interrupt pro Pin.
// Pegel wird via direktem GPIO-Register gelesen – zuverlässig auf ESP32.

void IRAM_ATTR rfRisingISR()  {}  // unused
void IRAM_ATTR rfFallingISR() {}  // unused

void IRAM_ATTR rfPulseISR() {
    const uint32_t now  = micros();
    const uint32_t diff = now - s_lastEdge;
    s_lastEdge = now;
    s_isrCount++;

    // Direktes GPIO-Register-Read: sicher und schnell auf ESP32
    // Liest den Pegel NACH der Flanke
    const bool pinHigh = (GPIO.in >> CC1101_GDO0_PIN) & 0x1;

    if (pinHigh) {
        // Soeben auf HIGH gegangen → vorheriger LOW-Puls ist jetzt bekannt
        if (!s_inFrame) return;

        if (diff >= GAP_MIN) {
            // Frame-Gap (>10ms)
            s_lastGapDur = diff;
            PulseBuffer& wb = s_buf[s_writeIdx];
            if (wb.count > s_maxFrameCount) s_maxFrameCount = wb.count;
            if (wb.count >= 33) {
                wb.ready   = true;
                s_writeIdx ^= 1;
                s_buf[s_writeIdx].count = 0;
                s_buf[s_writeIdx].ready = false;
            } else {
                s_lastFrameCount = wb.count;
            }
            s_inFrame = false;
        } else if (diff >= BIT_MIN && diff <= BIT_MAX) {
            // Bit-LOW (200-3000µs) → speichern
            s_lastBitLow = diff;
            PulseBuffer& wb = s_buf[s_writeIdx];
            if (wb.count < MAX_PULSES) {
                wb.pulses[wb.count++] = diff;
            } else {
                s_inFrame = false;
            }
        } else if (diff > BIT_MAX && diff < GAP_MIN) {
            // Header-LOW (~4490µs) oder anderer langer LOW zwischen BIT_MAX und GAP_MIN
            // → ignorieren (nicht als Frame-Abbruch werten)
            // Nur beim ersten LOW nach Header (count==1) ist das der Header-LOW
        }
        // diff < BIT_MIN → Rauschen, ignorieren
    } else {
        // Soeben auf LOW gegangen → vorheriger HIGH-Puls ist jetzt bekannt
        if (diff > s_maxHighSeen) s_maxHighSeen = diff;

        if (diff >= HEADER_HIGH_MIN && diff <= HEADER_HIGH_MAX) {
            PulseBuffer& wb = s_buf[s_writeIdx];
            wb.count     = 0;
            wb.ready     = false;
            wb.pulses[0] = diff | 0x80000000;
            wb.count     = 1;
            s_inFrame    = true;
        }
        // HIGH-Marker (~560µs) und andere HIGH-Pulse ignorieren
    }
}

// ─── Frame dekodieren ────────────────────────────────────────────────────────

static bool decodeFrame(const uint32_t* pulses, uint8_t count,
                        uint8_t outId[2], uint8_t& outCmd) {
    // pulses[0] = Header HIGH (mit 0x80000000 markiert)
    // pulses[1..32] = 32 LOW-Pulse (Bit-Werte)
    if (count < 33) {
        Serial.printf("[RX] Zu wenig Pulse: %d\n", count);
        return false;
    }

    uint8_t bytes[4] = {};
    for (uint8_t i = 0; i < 32; i++) {
        uint32_t low = pulses[1 + i] & 0x7FFFFFFF;
        bool bit = (low >= BIT_THRESHOLD);
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
    rfConfigRx();
    if (s_rxActive) return;
    s_inFrame  = false;
    s_lastEdge = micros();
    attachInterrupt(CC1101_GDO0_PIN, rfPulseISR, CHANGE);
    s_rxActive = true;
    Serial.println("[RX] RX aktiv");
}

void rfReceiverLeaveRx() {
    if (!s_rxActive) return;
    detachInterrupt(CC1101_GDO0_PIN);
    // Beide Interrupts werden durch einen detachInterrupt deaktiviert
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
    // Periodisch ISR-Aktivität loggen (alle 5s)
    static uint32_t lastLog = 0;
    static uint32_t lastCount = 0;
    if (millis() - lastLog > 5000) {
        uint32_t cnt = s_isrCount;
        uint32_t maxH = s_maxHighSeen;
        s_maxHighSeen = 0;
        uint8_t mfc = s_maxFrameCount; s_maxFrameCount = 0;
        Serial.printf("[RX] ISR: %u Flanken/5s maxHIGH=%uµs inFrame=%d "
                      "lastFC=%d maxFC=%d lastGap=%u lastBitLow=%u\n",
                      cnt - lastCount, maxH, (int)s_inFrame,
                      (int)s_lastFrameCount, (int)mfc,
                      (unsigned)s_lastGapDur, (unsigned)s_lastBitLow);
        lastCount = cnt;
        lastLog = millis();
    }

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
