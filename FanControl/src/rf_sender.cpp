#include "rf_sender.h"
#include "rf_receiver.h"
#include "config.h"
#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

static void cc1101WriteReg(uint8_t addr, uint8_t val) {
    digitalWrite(CC1101_CS_PIN, LOW);
    delayMicroseconds(2);
    SPI.transfer(addr);
    SPI.transfer(val);
    digitalWrite(CC1101_CS_PIN, HIGH);
    delayMicroseconds(2);
}

static uint8_t cc1101ReadReg(uint8_t addr) {
    digitalWrite(CC1101_CS_PIN, LOW);
    delayMicroseconds(2);
    SPI.transfer(addr | 0x80);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(CC1101_CS_PIN, HIGH);
    delayMicroseconds(2);
    return val;
}

// ─── TX/RX Registerkonfiguration ─────────────────────────────────────────────

static void configTx() {
    cc1101WriteReg(CC1101_IOCFG0,   0x2E);  // GDO0 = High-Z → wir steuern als Output
    cc1101WriteReg(CC1101_PKTCTRL0, 0x12);  // Async serial TX
    cc1101WriteReg(CC1101_MDMCFG2,  0x30);  // OOK, kein Sync-Word
    cc1101WriteReg(CC1101_FREND0,   0x11);  // PA-Table: 0=OFF, 1=max Power
    ELECHOUSE_cc1101.SetTx();
    pinMode(CC1101_GDO0_PIN, OUTPUT);
    digitalWrite(CC1101_GDO0_PIN, LOW);
}

void rfConfigRx() {
    cc1101WriteReg(CC1101_IOCFG0,   0x0D);  // GDO0 = demodulierter OOK-Datenstrom
    cc1101WriteReg(CC1101_PKTCTRL0, 0x30);  // Async serial RX
    cc1101WriteReg(CC1101_MDMCFG2,  0x30);  // OOK, kein Sync-Word
    ELECHOUSE_cc1101.SetRx();
    pinMode(CC1101_GDO0_PIN, INPUT);
}

// ─── Initialisierung ─────────────────────────────────────────────────────────

bool rfSenderInit() {
    // ESP32 VSPI mit expliziten Pins initialisieren
    SPI.begin(CC1101_SCK_PIN, CC1101_MISO_PIN, CC1101_MOSI_PIN, CC1101_CS_PIN);
    pinMode(CC1101_CS_PIN, OUTPUT);
    digitalWrite(CC1101_CS_PIN, HIGH);

    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK_PIN, CC1101_MISO_PIN,
                                CC1101_MOSI_PIN, CC1101_CS_PIN);
    ELECHOUSE_cc1101.setGDO0(CC1101_GDO0_PIN);
    ELECHOUSE_cc1101.Init();

    if (!ELECHOUSE_cc1101.getCC1101()) {
        Serial.println("[RF] CC1101 nicht gefunden – SPI prüfen");
        return false;
    }

    ELECHOUSE_cc1101.setMHZ(RF_FREQ_MHZ);
    ELECHOUSE_cc1101.setModulation(2);   // OOK/ASK
    ELECHOUSE_cc1101.setPA(12);          // max. TX-Leistung

    uint8_t iocfg = cc1101ReadReg(CC1101_IOCFG0);
    Serial.printf("[RF] CC1101 OK @ %.2f MHz | IOCFG0=0x%02X\n", RF_FREQ_MHZ, iocfg);
    return true;
}

// ─── Low-Level Puls-Ausgabe ──────────────────────────────────────────────────
// IRAM_ATTR: Funktion liegt im IRAM → kein Cache-Miss während noInterrupts()

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

// ─── Öffentliche API ─────────────────────────────────────────────────────────

void rfSend(const uint8_t id[2], uint8_t cmd, uint8_t repeat) {
    rfReceiverLeaveRx();
    configTx();

    // Auf ESP32: portDISABLE_INTERRUPTS() statt noInterrupts() für Core-0-Interrupts
    // noInterrupts() funktioniert aber ebenfalls auf dem aktuellen Core
    noInterrupts();
    for (uint8_t i = 0; i < repeat; i++) {
        sendFrame(id, cmd);
    }
    interrupts();

    digitalWrite(CC1101_GDO0_PIN, LOW);
    Serial.printf("[RF] TX: ID=%02X%02X CMD=%02X (%dx)\n",
                  id[0], id[1], cmd, repeat);

    rfReceiverEnterRx();
}
