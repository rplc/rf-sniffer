#include "rf_sender.h"
#include "rf_receiver.h"
#include "config.h"
#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

// ─── CC1101 SPI-Zugriff über SmartRC-interne Methode ─────────────────────────
// ELECHOUSE_cc1101.SpiWriteReg() ist public in der Lib – wir nutzen es direkt
// statt eigenem SPI.transfer(), um Bus-Konflikte zu vermeiden.

static void regWrite(uint8_t addr, uint8_t val) {
    ELECHOUSE_cc1101.SpiWriteReg(addr, val);
}

static uint8_t regRead(uint8_t addr) {
    return ELECHOUSE_cc1101.SpiReadReg(addr);
}

// Register-Adressen (eigener Prefix, kein Konflikt mit Lib-Makros)
static constexpr uint8_t REG_IOCFG0   = 0x02;
static constexpr uint8_t REG_PKTCTRL0 = 0x08;
static constexpr uint8_t REG_MDMCFG2  = 0x12;
static constexpr uint8_t REG_FREND0   = 0x22;

// ─── TX/RX Registerkonfiguration ─────────────────────────────────────────────

static void configTx() {
    regWrite(REG_IOCFG0,   0x2E);  // GDO0 = High-Z (wir steuern manuell als Output)
    regWrite(REG_PKTCTRL0, 0x12);  // Async serial TX mode
    regWrite(REG_MDMCFG2,  0x30);  // OOK, no sync word, no preamble
    regWrite(REG_FREND0,   0x11);  // PA table: index0=off, index1=max power
    ELECHOUSE_cc1101.SetTx();
    delayMicroseconds(100);         // CC1101 Settling-Zeit nach Moduswechsel
    pinMode(CC1101_GDO0_PIN, OUTPUT);
    digitalWrite(CC1101_GDO0_PIN, LOW);
}

void rfConfigRx() {
    regWrite(REG_IOCFG0,   0x0D);  // GDO0 = demodulierter OOK-Datenstrom (async)
    regWrite(REG_PKTCTRL0, 0x30);  // Async serial RX mode
    regWrite(REG_MDMCFG2,  0x30);  // OOK, no sync word
    ELECHOUSE_cc1101.SetRx();
    delayMicroseconds(100);
    pinMode(CC1101_GDO0_PIN, INPUT);
}

// ─── Initialisierung ─────────────────────────────────────────────────────────

bool rfSenderInit() {
    SPI.begin(CC1101_SCK_PIN, CC1101_MISO_PIN, CC1101_MOSI_PIN, CC1101_CS_PIN);
    pinMode(CC1101_CS_PIN, OUTPUT);
    digitalWrite(CC1101_CS_PIN, HIGH);
    delay(10);

    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK_PIN, CC1101_MISO_PIN,
                                CC1101_MOSI_PIN, CC1101_CS_PIN);
    ELECHOUSE_cc1101.setGDO0(CC1101_GDO0_PIN);
    ELECHOUSE_cc1101.Init();
    delay(10);  // CC1101 nach Reset stabil lassen

    if (!ELECHOUSE_cc1101.getCC1101()) {
        Serial.println("[RF] CC1101 nicht gefunden – SPI prüfen");
        return false;
    }

    ELECHOUSE_cc1101.setMHZ(RF_FREQ_MHZ);
    ELECHOUSE_cc1101.setModulation(2);   // OOK/ASK
    ELECHOUSE_cc1101.setPA(12);
    delay(5);

    // Sanity-Check: IOCFG0 sollte nach Init 0x3F sein (Default)
    uint8_t iocfg = regRead(REG_IOCFG0);
    Serial.printf("[RF] CC1101 OK @ %.2f MHz | IOCFG0=0x%02X (erwartet 0x3F)\n",
                  RF_FREQ_MHZ, iocfg);
    if (iocfg == 0x00) {
        Serial.println("[RF] WARNUNG: IOCFG0=0x00 – SPI-Verbindung prüfen!");
    }
    return true;
}

// ─── Low-Level Puls-Ausgabe ──────────────────────────────────────────────────

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
