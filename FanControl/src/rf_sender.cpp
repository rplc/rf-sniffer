#include "rf_sender.h"
#include "rf_receiver.h"
#include "config.h"
#include <SmartRC_CC1101.h>
#include <rom/ets_sys.h>

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
    // MDMCFG2 = 0x30: OOK ohne Sync-Word und Preamble
    ELECHOUSE_cc1101.SpiWriteReg(0x12, 0x30);

    uint8_t ver = ELECHOUSE_cc1101.SpiReadReg(0x31 | 0x80);
    Serial.printf("[RF] CC1101 OK @ %.2f MHz | Ver=0x%02X\n", RF_FREQ_MHZ, ver);
    return true;
}

// ─── Frame senden ─────────────────────────────────────────────────────────────
// Protokoll: Bit-Wert steckt im LOW-Puls
//   Header: HIGH(8120µs) + LOW(4490µs)
//   Bit:    HIGH(560µs)  + LOW(560µs=0 / 1680µs=1)
//   Ende:   HIGH(560µs)  + LOW(20000µs Gap)

static void IRAM_ATTR sendFrame(const uint8_t id[2], uint8_t cmd) {
    const uint8_t payload[4] = { id[0], id[1], cmd, (uint8_t)(~cmd) };

    digitalWrite(CC1101_GDO0_PIN, HIGH); ets_delay_us(RF_HEADER_HIGH);
    digitalWrite(CC1101_GDO0_PIN, LOW);  ets_delay_us(RF_HEADER_LOW);

    for (int b = 0; b < 4; b++) {
        for (int bit = 7; bit >= 0; bit--) {
            bool v = (payload[b] >> bit) & 0x01;
            digitalWrite(CC1101_GDO0_PIN, HIGH); ets_delay_us(RF_PULSE);
            digitalWrite(CC1101_GDO0_PIN, LOW);  ets_delay_us(v ? RF_BIT_1 : RF_BIT_0);
        }
    }
    // Abschließender HIGH-Marker + Frame-Gap
    digitalWrite(CC1101_GDO0_PIN, HIGH); ets_delay_us(RF_PULSE);
    digitalWrite(CC1101_GDO0_PIN, LOW);  ets_delay_us(RF_FRAME_GAP);
}

void rfSend(const uint8_t id[2], uint8_t cmd, uint8_t repeat) {
    rfReceiverLeaveRx();

    ELECHOUSE_cc1101.SetTx();
    pinMode(CC1101_GDO0_PIN, OUTPUT);
    digitalWrite(CC1101_GDO0_PIN, LOW);
    ets_delay_us(200);

    for (uint8_t i = 0; i < repeat; i++) {
        noInterrupts();
        sendFrame(id, cmd);
        interrupts();
    }

    Serial.printf("[RF] TX: ID=%02X%02X CMD=%02X (%dx)\n",
                  id[0], id[1], cmd, repeat);

    rfReceiverEnterRx();
}
