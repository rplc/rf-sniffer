#include "rf_receiver.h"
#include "config.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>

// ─── Timing-Toleranzen ───────────────────────────────────────────────────────
// Werte aus Captures: Header ~4520µs, Bit0 ~580µs, Bit1 ~1710µs, Sep ~540µs
// Toleranz ±30 % für robuste Dekodierung bei schwachem Signal

static constexpr uint16_t HEADER_MIN  = 3500;
static constexpr uint16_t HEADER_MAX  = 5500;
static constexpr uint16_t BIT_SEP_MAX =  900;   // LOW-Puls: Bit-Separator
static constexpr uint16_t BIT_ONE_MIN =  900;   // HIGH-Puls: Bit=1 wenn > 900µs
static constexpr uint16_t BIT_ZERO_MIN=  350;   // HIGH-Puls: Bit=0 wenn 350-900µs
static constexpr uint16_t GAP_MIN     = 10000;  // Frame-Ende (langer LOW-Puls)

// ─── ISR-Buffer ──────────────────────────────────────────────────────────────
// Double-Buffer: ISR schreibt in einen, Loop liest aus dem anderen

static constexpr uint8_t  MAX_PULSES  = 72;     // 2 + 32*2 + 1 Echtpulse + Reserve

struct PulseBuffer {
    volatile uint16_t pulses[MAX_PULSES];
    volatile uint8_t  count;
    volatile bool     ready;   // vollständiger Frame vorhanden
};

static PulseBuffer s_buf[2];
static volatile uint8_t s_writeIdx = 0;   // ISR schreibt hier
static volatile uint32_t s_lastEdge = 0;
static volatile bool s_inFrame = false;

static RfRxCallback s_callback = nullptr;

// ─── ISR ─────────────────────────────────────────────────────────────────────

void IRAM_ATTR rfPulseISR() {
    const uint32_t now  = micros();
    const uint32_t diff = now - s_lastEdge;
    s_lastEdge = now;

    const bool pinHigh = digitalRead(CC1101_GDO0_PIN);  // aktueller Pegel NACH Flanke

    // --- Flanke LOW→HIGH: war ein LOW-Puls (Separator oder Gap) ----------------
    if (pinHigh) {
        if (!s_inFrame) {
            // Noch kein Frame aktiv – ignorieren
            return;
        }
        // Frame-Gap: langer LOW-Puls → Frame abgeschlossen
        if (diff >= GAP_MIN) {
            PulseBuffer& wb = s_buf[s_writeIdx];
            if (wb.count >= 34) {   // mind. Header + 32 Datenpulse
                wb.ready = true;
                s_writeIdx ^= 1;    // auf anderen Buffer wechseln
                s_buf[s_writeIdx].count = 0;
                s_buf[s_writeIdx].ready = false;
            }
            s_inFrame = false;
        }
        // normaler Separator – ignorieren (wir messen nur HIGH-Pulse)
        return;
    }

    // --- Flanke HIGH→LOW: war ein HIGH-Puls (Header oder Daten-Bit) -----------
    if (diff >= HEADER_MIN && diff <= HEADER_MAX) {
        // Header erkannt → neuen Frame starten
        PulseBuffer& wb = s_buf[s_writeIdx];
        wb.count = 0;
        wb.ready = false;
        wb.pulses[wb.count++] = (uint16_t)diff;
        s_inFrame = true;
        return;
    }

    if (!s_inFrame) return;

    PulseBuffer& wb = s_buf[s_writeIdx];
    if (wb.count < MAX_PULSES) {
        wb.pulses[wb.count++] = (uint16_t)diff;
    } else {
        // Überlauf → Frame verwerfen
        s_inFrame = false;
        wb.count = 0;
    }
}

// ─── Frame dekodieren ────────────────────────────────────────────────────────

static bool decodeFrame(const uint16_t* pulses, uint8_t count,
                        uint8_t outId[2], uint8_t& outCmd) {
    // Erwartet: [Header, Bit0_high, Bit1_high, ..., Bit31_high]  (33 HIGH-Pulse)
    // (LOW-Pulse/Separatoren werden im ISR nicht gespeichert)
    if (count < 33) return false;

    uint8_t bytes[4] = {0, 0, 0, 0};
    for (uint8_t i = 0; i < 32; i++) {
        const uint16_t p = pulses[1 + i];   // pulses[0] = Header
        bool bit;
        if (p >= BIT_ONE_MIN) {
            bit = true;
        } else if (p >= BIT_ZERO_MIN) {
            bit = false;
        } else {
            return false;   // ungültiger Puls
        }
        const uint8_t byteIdx = i / 8;
        const uint8_t bitPos  = 7 - (i % 8);   // MSB first
        if (bit) bytes[byteIdx] |= (1 << bitPos);
    }

    // Checksumme: B3 == ~B2
    if (bytes[3] != (uint8_t)(~bytes[2])) {
        return false;
    }

    outId[0] = bytes[0];
    outId[1] = bytes[1];
    outCmd   = bytes[2];
    return true;
}

// ─── RX-Modus ein-/ausschalten ───────────────────────────────────────────────

static bool s_rxActive = false;

void rfReceiverEnterRx() {
    if (s_rxActive) return;
    ELECHOUSE_cc1101.SetRx();
    pinMode(CC1101_GDO0_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0_PIN), rfPulseISR, CHANGE);
    s_inFrame  = false;
    s_rxActive = true;
}

void rfReceiverLeaveRx() {
    if (!s_rxActive) return;
    detachInterrupt(digitalPinToInterrupt(CC1101_GDO0_PIN));
    s_rxActive = false;
    // Aufrufer schaltet anschließend in TX und setzt GDO0 als OUTPUT
}

// ─── Öffentliche API ─────────────────────────────────────────────────────────

void rfReceiverInit(RfRxCallback callback) {
    s_callback = callback;
    s_buf[0].count = 0; s_buf[0].ready = false;
    s_buf[1].count = 0; s_buf[1].ready = false;
    s_writeIdx = 0;
    rfReceiverEnterRx();
    Serial.println("[RX] Empfänger aktiv");
}

void rfReceiverLoop() {
    // Lese-Buffer: immer der, in den ISR gerade NICHT schreibt
    const uint8_t readIdx = s_writeIdx ^ 1;
    PulseBuffer& rb = s_buf[readIdx];

    if (!rb.ready) return;

    // Lokale Kopie anlegen, dann Buffer freigeben
    uint16_t pulses[MAX_PULSES];
    uint8_t  count;
    noInterrupts();
    count = rb.count;
    memcpy(pulses, (const void*)rb.pulses, count * sizeof(uint16_t));
    rb.ready = false;
    interrupts();

    uint8_t id[2], cmd;
    if (decodeFrame(pulses, count, id, cmd)) {
        if (s_callback) s_callback(id, cmd);
    } else {
        Serial.printf("[RX] Frame verworfen (count=%d)\n", count);
    }
}
