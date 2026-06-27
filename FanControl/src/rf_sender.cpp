#include "rf_sender.h"
#include "rf_receiver.h"
#include "config.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>

// ─── CC1101 Initialisierung ──────────────────────────────────────────────────

bool rfSenderInit() {
    ELECHOUSE_cc1101.setSpiPin(CC1101_CS_PIN, CC1101_GDO0_PIN);
    ELECHOUSE_cc1101.Init();

    if (!ELECHOUSE_cc1101.getCC1101()) {
        Serial.println("[RF] CC1101 nicht gefunden – SPI-Verbindung prüfen");
        return false;
    }

    ELECHOUSE_cc1101.setMHZ(RF_FREQ_MHZ);
    ELECHOUSE_cc1101.setModulation(2);   // OOK/ASK
    ELECHOUSE_cc1101.setDRate(1.2);
    ELECHOUSE_cc1101.setPktFormat(3);    // Async serial mode

    // Startet im RX-Modus – rfReceiverInit() übernimmt das Aktivieren
    Serial.printf("[RF] CC1101 bereit @ %.3f MHz OOK\n", RF_FREQ_MHZ);
    return true;
}

// ─── Low-Level Puls-Ausgabe ──────────────────────────────────────────────────

static inline void txHigh(uint32_t us) {
    digitalWrite(CC1101_GDO0_PIN, HIGH);
    delayMicroseconds(us);
}

static inline void txLow(uint32_t us) {
    digitalWrite(CC1101_GDO0_PIN, LOW);
    delayMicroseconds(us);
}

// ─── Frame senden ────────────────────────────────────────────────────────────

static void sendFrame(const uint8_t id[2], uint8_t cmd) {
    const uint8_t payload[4] = { id[0], id[1], cmd, (uint8_t)(~cmd) };

    txHigh(RF_HEADER_HIGH);
    txLow(RF_HEADER_LOW);

    for (int byteIdx = 0; byteIdx < 4; byteIdx++) {
        for (int bitIdx = 7; bitIdx >= 0; bitIdx--) {
            bool bit = (payload[byteIdx] >> bitIdx) & 0x01;
            txHigh(bit ? RF_BIT_ONE : RF_BIT_ZERO);
            txLow(RF_BIT_SEP);
        }
    }

    txLow(RF_FRAME_GAP);
}

// ─── Öffentliche API ─────────────────────────────────────────────────────────

void rfSend(const uint8_t id[2], uint8_t cmd, uint8_t repeat) {
    // RX deaktivieren: Interrupt trennen, CC1101 in TX schalten
    rfReceiverLeaveRx();
    ELECHOUSE_cc1101.SetTx();
    pinMode(CC1101_GDO0_PIN, OUTPUT);
    digitalWrite(CC1101_GDO0_PIN, LOW);

    // Interrupts aus für µs-genaues Timing
    noInterrupts();
    for (uint8_t i = 0; i < repeat; i++) {
        sendFrame(id, cmd);
    }
    interrupts();

    Serial.printf("[RF] Gesendet: ID=%02X%02X CMD=%02X (%dx)\n",
                  id[0], id[1], cmd, repeat);

    // Zurück in RX
    rfReceiverEnterRx();
}
