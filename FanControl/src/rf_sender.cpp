#include "rf_sender.h"
#include "rf_receiver.h"
#include "config.h"
#include <SmartRC_CC1101.h>

// Exakt gleicher Ansatz wie die funktionierende Lampen-Sketch:
// SmartRC für Init + Frequenz + SetTx/SetRx
// GPIO direkt für Puls-Timing (kein Async-Serial-Mode)

void rfConfigRx() {
    ELECHOUSE_cc1101.SetRx();
    pinMode(CC1101_GDO0_PIN, INPUT);
}

bool rfSenderInit() {
    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK_PIN, CC1101_MISO_PIN,
                                CC1101_MOSI_PIN, CC1101_CS_PIN);
    ELECHOUSE_cc1101.Init();

    if (!ELECHOUSE_cc1101.getCC1101()) {
        Serial.println("[RF] CC1101 nicht gefunden – SPI prüfen");
        return false;
    }

    ELECHOUSE_cc1101.setMHZ(RF_FREQ_MHZ);
    ELECHOUSE_cc1101.setModulation(2);  // OOK/ASK
    ELECHOUSE_cc1101.setPA(12);

    uint8_t part = ELECHOUSE_cc1101.SpiReadReg(0x30 | 0x80);  // PARTNUM
    uint8_t ver  = ELECHOUSE_cc1101.SpiReadReg(0x31 | 0x80);  // VERSION
    Serial.printf("[RF] CC1101 @ %.2f MHz | Part=0x%02X Ver=0x%02X\n",
                  RF_FREQ_MHZ, part, ver);

    // TX-Modus aktivieren (genau wie Lampen-Sketch)
    ELECHOUSE_cc1101.SetTx();
    pinMode(CC1101_GDO0_PIN, OUTPUT);
    digitalWrite(CC1101_GDO0_PIN, LOW);

    return true;
}

// ─── Puls-Ausgabe direkt via GPIO ────────────────────────────────────────────
// CC1101 ist im TX-Modus → GDO0 HIGH = Träger an, LOW = Träger aus

static inline void IRAM_ATTR txHigh(uint32_t us) {
    digitalWrite(CC1101_GDO0_PIN, HIGH);
    delayMicroseconds(us);
}

static inline void IRAM_ATTR txLow(uint32_t us) {
    digitalWrite(CC1101_GDO0_PIN, LOW);
    delayMicroseconds(us);
}

static void IRAM_ATTR sendFrame(const uint8_t id[2], uint8_t cmd) {
    const uint8_t payload[4] = { id[0], id[1], cmd, (uint8_t)(~cmd) };

    txHigh(RF_HEADER_HIGH);
    txLow(RF_HEADER_LOW);

    for (int b = 0; b < 4; b++) {
        for (int bit = 7; bit >= 0; bit--) {
            bool v = (payload[b] >> bit) & 0x01;
            txHigh(v ? RF_BIT_ONE : RF_BIT_ZERO);
            txLow(RF_BIT_SEP);
        }
    }

    txLow(RF_FRAME_GAP);
}

void rfSend(const uint8_t id[2], uint8_t cmd, uint8_t repeat) {
    rfReceiverLeaveRx();

    // Zurück in TX-Modus (nach RX war SetRx() aktiv)
    ELECHOUSE_cc1101.SetTx();
    pinMode(CC1101_GDO0_PIN, OUTPUT);
    digitalWrite(CC1101_GDO0_PIN, LOW);
    delayMicroseconds(200);

    for (uint8_t i = 0; i < repeat; i++) {
        noInterrupts();
        sendFrame(id, cmd);
        interrupts();
    }

    digitalWrite(CC1101_GDO0_PIN, LOW);
    Serial.printf("[RF] TX: ID=%02X%02X CMD=%02X (%dx)\n",
                  id[0], id[1], cmd, repeat);

    rfReceiverEnterRx();
}
