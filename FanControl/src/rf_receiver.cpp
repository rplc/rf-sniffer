#include "rf_receiver.h"
#include "rf_sender.h"
#include "config.h"

// ─── Protokoll-Konstanten ────────────────────────────────────────────────────

// Fan-Protokoll (eigenes PWM, Bit im LOW-Puls)
static constexpr uint32_t FAN_HEADER_MIN  =  5000;  // Header-HIGH µs
static constexpr uint32_t FAN_HEADER_MAX  = 12000;
static constexpr uint32_t FAN_HDR_LOW_MIN =  3000;  // Header-LOW (ignorieren)
static constexpr uint32_t FAN_HDR_LOW_MAX = 10000;
static constexpr uint32_t FAN_BIT_MIN     =   200;  // LOW-Puls Bit-Dauer
static constexpr uint32_t FAN_BIT_MAX     =  3000;
static constexpr uint32_t FAN_BIT_ONE     =   900;  // LOW >= 900µs → Bit 1
static constexpr uint32_t FAN_GAP_MIN     = 10000;  // Frame-Ende

// Lamp-Protokoll (Protokoll 2 kompatibel, Bit im HIGH-Puls)
// pulseLength=248µs
// sync:  HIGH=248µs,  LOW=7688µs (31×248)
// bit-0: HIGH=248µs,  LOW=744µs  (3×248)
// bit-1: HIGH=744µs,  LOW=248µs  (1×248)
static constexpr uint32_t LAMP_PULSE       =  248;
static constexpr uint32_t LAMP_SYNC_LOW_MIN = 5000;  // >5ms LOW → Sync erkannt
static constexpr uint32_t LAMP_HIGH_ONE_MIN =  450;  // HIGH >450µs → Bit 1 (3×248=744)
static constexpr uint32_t LAMP_HIGH_MAX     = 1500;  // HIGH <1500µs → gültiges Bit
static constexpr uint32_t LAMP_HIGH_MIN     =  100;  // HIGH >100µs → kein Rauschen
static constexpr uint32_t LAMP_BITS         =   24;

// ─── ISR-Buffer ──────────────────────────────────────────────────────────────

static constexpr uint8_t MAX_PULSES = 80;

struct PulseBuffer {
    volatile uint32_t pulses[MAX_PULSES];  // Bit31=1: war HIGH-Puls
    volatile uint8_t  count;
    volatile bool     ready;
};

static PulseBuffer       s_buf[2];
static volatile uint8_t  s_writeIdx = 0;
static volatile uint32_t s_lastEdge = 0;

// Fan-Frame-State
static volatile bool     s_fanInFrame = false;

// Lamp-Frame-State
static volatile bool     s_lampInFrame  = false;
static volatile uint8_t  s_lampBitCount = 0;
static volatile uint32_t s_lampBits     = 0;  // akkumulierte Lamp-Bits (24-Bit)

static bool              s_rxActive = false;
static FanRxCallback     s_fanCb    = nullptr;
static LampRxCallback    s_lampCb   = nullptr;

// ─── ISR ─────────────────────────────────────────────────────────────────────

void IRAM_ATTR rfPulseISR() {
    const uint32_t now  = micros();
    const uint32_t diff = now - s_lastEdge;
    s_lastEdge = now;

    // Pegel NACH der Flanke via GPIO-Register (ESP32)
    const bool pinHigh = (GPIO.in >> CC1101_GDO0_PIN) & 0x1;

    if (pinHigh) {
        // Steigende Flanke → vorheriger LOW-Puls bekannt

        // ── Fan-Protokoll: LOW-Puls auswerten ────────────────────────────────
        if (s_fanInFrame) {
            if (diff >= FAN_GAP_MIN) {
                // Frame-Gap → abschließen
                PulseBuffer& wb = s_buf[s_writeIdx];
                if (wb.count >= 33) {
                    wb.ready   = true;
                    s_writeIdx ^= 1;
                    s_buf[s_writeIdx].count = 0;
                    s_buf[s_writeIdx].ready = false;
                }
                s_fanInFrame = false;
            } else if (diff >= FAN_BIT_MIN && diff <= FAN_BIT_MAX) {
                // Bit-LOW speichern
                PulseBuffer& wb = s_buf[s_writeIdx];
                if (wb.count < MAX_PULSES) {
                    wb.pulses[wb.count++] = diff;  // LOW, kein Bit31
                } else {
                    s_fanInFrame = false;
                }
            } else if (diff > FAN_BIT_MAX && diff < FAN_GAP_MIN) {
                // Header-LOW (~4490µs) → ignorieren
            }
        }

        // ── Lamp-Protokoll: LOW-Puls = Sync-Erkennung ────────────────────────
        if (diff >= LAMP_SYNC_LOW_MIN) {
            // Langer LOW → Sync → neuen Lamp-Frame starten
            s_lampInFrame  = true;
            s_lampBitCount = 0;
            s_lampBits     = 0;
        }

    } else {
        // Fallende Flanke → vorheriger HIGH-Puls bekannt

        // ── Fan-Protokoll: Header-HIGH erkennen ──────────────────────────────
        if (diff >= FAN_HEADER_MIN && diff <= FAN_HEADER_MAX) {
            // Fan-Header → Frame starten
            // Lamp-Frame abbrechen falls aktiv (Priorität: Fan)
            s_lampInFrame = false;
            PulseBuffer& wb = s_buf[s_writeIdx];
            wb.count     = 0;
            wb.ready     = false;
            wb.pulses[0] = diff | 0x80000000;  // Header markieren
            wb.count     = 1;
            s_fanInFrame = true;
        }

        // ── Lamp-Protokoll: HIGH-Puls = Bit-Wert ─────────────────────────────
        if (s_lampInFrame && !s_fanInFrame) {
            if (diff >= LAMP_HIGH_MIN && diff <= LAMP_HIGH_MAX) {
                uint32_t bit = (diff >= LAMP_HIGH_ONE_MIN) ? 1 : 0;
                s_lampBits = (s_lampBits << 1) | bit;
                s_lampBitCount++;

                if (s_lampBitCount == LAMP_BITS) {
                    // 24 Bits vollständig → in Buffer schreiben
                    PulseBuffer& wb = s_buf[s_writeIdx];
                    wb.count     = 0;
                    wb.ready     = false;
                    // Lamp-Frame: Bit31=1 im ersten Element als Marker
                    // Code in pulses[0] (low 30 Bit) + Bit30=0 (Fan=Bit30=1)
                    // Wir nutzen Bit30=0 für Lamp, Bit30=1 für Fan-Header
                    wb.pulses[0] = s_lampBits & 0x3FFFFFFF;  // Bit31=0, Bit30=0 → Lamp
                    wb.count     = 0xFF;  // Sonderkennung für Lamp-Frame
                    wb.ready     = true;
                    s_writeIdx  ^= 1;
                    s_buf[s_writeIdx].count = 0;
                    s_buf[s_writeIdx].ready = false;
                    s_lampInFrame  = false;
                    s_lampBitCount = 0;
                }
            } else {
                // Ungültiger HIGH → Lamp-Frame verwerfen
                s_lampInFrame = false;
            }
        }
    }
}

// ─── Frame-Dekodierung ───────────────────────────────────────────────────────

static bool decodeFanFrame(const uint32_t* pulses, uint8_t count,
                           uint8_t outId[2], uint8_t& outCmd) {
    if (count < 33) return false;

    uint8_t bytes[4] = {};
    for (uint8_t i = 0; i < 32; i++) {
        uint32_t low = pulses[1 + i] & 0x7FFFFFFF;
        bool bit = (low >= FAN_BIT_ONE);
        bytes[i / 8] |= (bit << (7 - (i % 8)));
    }

    if (bytes[3] != (uint8_t)(~bytes[2])) {
        Serial.printf("[RX] Fan checksum fail: B2=0x%02X B3=0x%02X\n",
                      bytes[2], bytes[3]);
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
    s_fanInFrame   = false;
    s_lampInFrame  = false;
    s_lampBitCount = 0;
    s_lastEdge     = micros();
    attachInterrupt(CC1101_GDO0_PIN, rfPulseISR, CHANGE);
    s_rxActive = true;
    Serial.println("[RX] aktiv");
}

void rfReceiverLeaveRx() {
    if (!s_rxActive) return;
    detachInterrupt(CC1101_GDO0_PIN);
    s_rxActive = false;
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void rfReceiverLoop() {
    const uint8_t readIdx = s_writeIdx ^ 1;
    PulseBuffer&  rb      = s_buf[readIdx];

    if (!rb.ready) return;

    // Lokale Kopie
    uint32_t pulses[MAX_PULSES];
    uint8_t  count;
    noInterrupts();
    count = rb.count;
    memcpy(pulses, (const void*)rb.pulses, sizeof(uint32_t) * min((uint8_t)MAX_PULSES, count == 0xFF ? (uint8_t)1 : count));
    rb.ready = false;
    interrupts();

    if (count == 0xFF) {
        // Lamp-Frame
        uint32_t code = pulses[0] & 0x3FFFFFFF;
        Serial.printf("[RX] Lamp: code=%u\n", code);
        if (s_lampCb) s_lampCb(code);
    } else {
        // Fan-Frame
        uint8_t id[2], cmd;
        if (decodeFanFrame(pulses, count, id, cmd)) {
            // Debounce: gleiche ID+CMD innerhalb 800ms ignorieren
            static uint8_t  lastId[2] = {0,0};
            static uint8_t  lastCmd   = 0;
            static uint32_t lastTime  = 0;
            uint32_t now = millis();
            if (!(lastCmd == cmd && lastId[0] == id[0] && lastId[1] == id[1]
                  && (now - lastTime) < 800)) {
                Serial.printf("[RX] Fan: ID=%02X%02X CMD=0x%02X\n",
                              id[0], id[1], cmd);
                lastId[0] = id[0]; lastId[1] = id[1];
                lastCmd   = cmd;
                lastTime  = now;
                if (s_fanCb) s_fanCb(id, cmd);
            }
        }
    }
}

// ─── Init ────────────────────────────────────────────────────────────────────

void rfReceiverInit(FanRxCallback fanCb, LampRxCallback lampCb) {
    s_fanCb  = fanCb;
    s_lampCb = lampCb;
    s_buf[0] = {};
    s_buf[1] = {};
    s_writeIdx = 0;
}
