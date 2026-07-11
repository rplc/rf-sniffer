#include "rf_handler.h"
#include "config.h"
#include <SPI.h>
#include <RadioLib.h>

// ─── CC1101 (RadioLib) ───────────────────────────────────────────────────────
// Eigener SPI-Bus (VSPI), damit der Bresser-CC1101 (HSPI) unangetastet bleibt.
static SPIClass s_vspi(VSPI);
static Module   s_mod(CC1101_CS_PIN, CC1101_GDO0_PIN, RADIOLIB_NC, RADIOLIB_NC, s_vspi);
static CC1101   s_radio(&s_mod);

// ─── State ───────────────────────────────────────────────────────────────────

static FanRxCallback  s_fanCb  = nullptr;
static LampRxCallback s_lampCb = nullptr;
static bool           s_rxActive = false;

// ─── ISR-Buffer (Double-Buffer) ──────────────────────────────────────────────

static constexpr uint8_t MAX_PULSES = 80;

struct PulseBuffer {
    volatile uint32_t pulses[MAX_PULSES];
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
static volatile uint32_t s_lampBits     = 0;

// ─── ISR ─────────────────────────────────────────────────────────────────────

void IRAM_ATTR rfPulseISR() {
    const uint32_t now  = micros();
    const uint32_t diff = now - s_lastEdge;
    s_lastEdge = now;

    const bool pinHigh = (GPIO.in >> CC1101_GDO0_PIN) & 0x1;

    if (pinHigh) {
        // Steigende Flanke → vorheriger LOW-Puls bekannt

        // Fan: LOW-Puls auswerten
        if (s_fanInFrame) {
            if (diff >= FAN_RX_GAP_MIN) {
                PulseBuffer& wb = s_buf[s_writeIdx];
                if (wb.count >= 33) {
                    wb.ready   = true;
                    s_writeIdx ^= 1;
                    s_buf[s_writeIdx].count = 0;
                    s_buf[s_writeIdx].ready = false;
                }
                s_fanInFrame = false;
            } else if (diff >= FAN_RX_BIT_MIN && diff <= FAN_RX_BIT_MAX) {
                PulseBuffer& wb = s_buf[s_writeIdx];
                if (wb.count < MAX_PULSES) {
                    wb.pulses[wb.count++] = diff;
                } else {
                    s_fanInFrame = false;
                }
            } else if (diff > FAN_RX_BIT_MAX && diff < FAN_RX_GAP_MIN) {
                // Header-LOW (~4490µs) → ignorieren
            }
        }

        // Lamp: langer LOW → Sync
        if (diff >= LAMP_RX_SYNC_MIN) {
            s_lampInFrame  = true;
            s_lampBitCount = 0;
            s_lampBits     = 0;
        }

    } else {
        // Fallende Flanke → vorheriger HIGH-Puls bekannt

        // Fan: Header erkennen
        if (diff >= FAN_RX_HEADER_MIN && diff <= FAN_RX_HEADER_MAX) {
            s_lampInFrame = false;  // Fan hat Priorität
            PulseBuffer& wb = s_buf[s_writeIdx];
            wb.count     = 0;
            wb.ready     = false;
            wb.pulses[0] = diff | 0x80000000;
            wb.count     = 1;
            s_fanInFrame = true;
        }

        // Lamp: HIGH-Puls = Bit-Wert
        if (s_lampInFrame && !s_fanInFrame) {
            if (diff >= LAMP_RX_HIGH_MIN && diff <= LAMP_RX_HIGH_MAX) {
                uint32_t bit = (diff >= LAMP_RX_BIT_ONE) ? 1 : 0;
                s_lampBits = (s_lampBits << 1) | bit;
                s_lampBitCount++;

                if (s_lampBitCount == 24) {
                    PulseBuffer& wb = s_buf[s_writeIdx];
                    wb.pulses[0] = s_lampBits & 0x3FFFFFFF;
                    wb.count     = 0xFF;  // Lamp-Frame Kennung
                    wb.ready     = true;
                    s_writeIdx  ^= 1;
                    s_buf[s_writeIdx].count = 0;
                    s_buf[s_writeIdx].ready = false;
                    s_lampInFrame  = false;
                    s_lampBitCount = 0;
                }
            } else {
                s_lampInFrame = false;
            }
        }
    }
}

// ─── Frame dekodieren ────────────────────────────────────────────────────────

static bool decodeFanFrame(const uint32_t* pulses, uint8_t count,
                           uint8_t outId[2], uint8_t& outCmd) {
    if (count < 33) return false;

    uint8_t bytes[4] = {};
    for (uint8_t i = 0; i < 32; i++) {
        uint32_t low = pulses[1 + i] & 0x7FFFFFFF;
        bool bit = (low >= FAN_RX_BIT_ONE);
        bytes[i / 8] |= (bit << (7 - (i % 8)));
    }

    if (bytes[3] != (uint8_t)(~bytes[2])) return false;

    outId[0] = bytes[0];
    outId[1] = bytes[1];
    outCmd   = bytes[2];
    return true;
}

// ─── TX/RX Umschaltung ───────────────────────────────────────────────────────

void rfEnterTx() {
    if (s_rxActive) {
        detachInterrupt(CC1101_GDO0_PIN);
        s_rxActive = false;
    }
    s_radio.standby();               // SIDLE
    delay(2);                         // Settling-Zeit
    s_radio.transmitDirectAsync();    // IOCFG0/PKTCTRL0 = async OOK, STX-Strobe
    pinMode(CC1101_GDO0_PIN, OUTPUT);
    digitalWrite(CC1101_GDO0_PIN, LOW);
    delayMicroseconds(500);
}

void rfEnterRx() {
    s_radio.standby();               // SIDLE
    delay(2);                         // Settling-Zeit
    s_radio.receiveDirectAsync();     // IOCFG0/PKTCTRL0 = async OOK, SRX-Strobe
    delay(2);
    pinMode(CC1101_GDO0_PIN, INPUT);

    if (!s_rxActive) {
        s_fanInFrame   = false;
        s_lampInFrame  = false;
        s_lampBitCount = 0;
        s_lastEdge     = micros();
        attachInterrupt(CC1101_GDO0_PIN, rfPulseISR, CHANGE);
        s_rxActive = true;
    }
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void rfLoop() {
    const uint8_t readIdx = s_writeIdx ^ 1;
    PulseBuffer&  rb      = s_buf[readIdx];

    if (!rb.ready) return;

    uint32_t pulses[MAX_PULSES];
    uint8_t  count;
    noInterrupts();
    count = rb.count;
    uint8_t copyCount = (count == 0xFF) ? 1 : count;
    memcpy(pulses, (const void*)rb.pulses, copyCount * sizeof(uint32_t));
    rb.ready = false;
    interrupts();

    if (count == 0xFF) {
        // Lamp-Frame
        uint32_t code = pulses[0] & 0x3FFFFFFF;

        // Lamp-Debounce
        static uint32_t lastCode = 0;
        static uint32_t lastTime = 0;
        uint32_t now = millis();
        if (code == lastCode && (now - lastTime) < LAMP_RX_DEBOUNCE_MS) return;
        lastCode = code;
        lastTime = now;

        Serial.printf("[RF] Lamp RX: code=%u\n", code);
        if (s_lampCb) s_lampCb(code);
    } else {
        // Fan-Frame
        uint8_t id[2], cmd;
        if (!decodeFanFrame(pulses, count, id, cmd)) return;

        // Fan-Debounce
        static uint8_t  lastId[2] = {0, 0};
        static uint8_t  lastCmd   = 0;
        static uint32_t lastTime  = 0;
        uint32_t now = millis();
        if (lastCmd == cmd && lastId[0] == id[0] && lastId[1] == id[1]
            && (now - lastTime) < FAN_RX_DEBOUNCE_MS) return;
        lastId[0] = id[0]; lastId[1] = id[1];
        lastCmd   = cmd;
        lastTime  = now;

        Serial.printf("[RF] Fan RX: ID=%02X%02X CMD=0x%02X\n", id[0], id[1], cmd);
        if (s_fanCb) s_fanCb(id, cmd);
    }
}

// ─── Init ────────────────────────────────────────────────────────────────────

bool rfInit(FanRxCallback fanCb, LampRxCallback lampCb) {
    s_fanCb  = fanCb;
    s_lampCb = lampCb;
    s_buf[0] = {};
    s_buf[1] = {};
    s_writeIdx = 0;

    s_vspi.begin(CC1101_SCK_PIN, CC1101_MISO_PIN, CC1101_MOSI_PIN, CC1101_CS_PIN);

    int16_t state = s_radio.begin(RF_FREQ_MHZ);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[RF] CC1101 nicht gefunden – SPI prüfen (Code %d)\n", state);
        return false;
    }

    s_radio.setOOK(true);                // MDMCFG2: ASK/OOK-Modulation
    s_radio.disableSyncWordFiltering();   // MDMCFG2: kein Sync-Wort (== altes 0x30)
    s_radio.setOutputPower(12);           // ~12 dBm, wie altes setPA(12)

    Serial.printf("[RF] CC1101 OK @ %.2f MHz | Ver=0x%02X\n",
                  RF_FREQ_MHZ, s_radio.getChipVersion());

    rfEnterRx();  // direkt in RX starten
    return true;
}
