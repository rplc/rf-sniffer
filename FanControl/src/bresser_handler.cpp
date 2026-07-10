#include "bresser_handler.h"
#include "config.h"
#include <SPI.h>
#include <ArduinoJson.h>

static SPIClass      s_hspi(HSPI);
static PubSubClient* s_client  = nullptr;
static bool          s_ready   = false;

// ─── CC1101 SPI-Zugriff (HSPI) ───────────────────────────────────────────────

static uint8_t cc1101ReadReg(uint8_t addr) {
    digitalWrite(BRESSER_CS_PIN, LOW);
    // Warte bis MISO LOW (CC1101 bereit)
    uint32_t t = micros();
    while (digitalRead(BRESSER_MISO_PIN) && (micros() - t < 1000));
    s_hspi.transfer(addr | 0x80);   // R-Bit setzen
    uint8_t val = s_hspi.transfer(0x00);
    digitalWrite(BRESSER_CS_PIN, HIGH);
    return val;
}

static uint8_t cc1101ReadStatus(uint8_t addr) {
    digitalWrite(BRESSER_CS_PIN, LOW);
    uint32_t t = micros();
    while (digitalRead(BRESSER_MISO_PIN) && (micros() - t < 1000));
    s_hspi.transfer(addr | 0xC0);   // R-Bit + Burst-Bit (Status Register)
    uint8_t val = s_hspi.transfer(0x00);
    digitalWrite(BRESSER_CS_PIN, HIGH);
    return val;
}

static void cc1101WriteReg(uint8_t addr, uint8_t val) {
    digitalWrite(BRESSER_CS_PIN, LOW);
    uint32_t t = micros();
    while (digitalRead(BRESSER_MISO_PIN) && (micros() - t < 1000));
    s_hspi.transfer(addr);
    s_hspi.transfer(val);
    digitalWrite(BRESSER_CS_PIN, HIGH);
}

static void cc1101Strobe(uint8_t cmd) {
    digitalWrite(BRESSER_CS_PIN, LOW);
    uint32_t t = micros();
    while (digitalRead(BRESSER_MISO_PIN) && (micros() - t < 1000));
    s_hspi.transfer(cmd);
    digitalWrite(BRESSER_CS_PIN, HIGH);
}

static void cc1101ReadBurst(uint8_t addr, uint8_t* data, uint8_t len) {
    digitalWrite(BRESSER_CS_PIN, LOW);
    uint32_t t = micros();
    while (digitalRead(BRESSER_MISO_PIN) && (micros() - t < 1000));
    s_hspi.transfer(addr | 0xC0);
    for (uint8_t i = 0; i < len; i++) data[i] = s_hspi.transfer(0x00);
    digitalWrite(BRESSER_CS_PIN, HIGH);
}

// ─── CC1101 Reset ─────────────────────────────────────────────────────────────

static bool cc1101Reset() {
    // Korrekter Reset-Ablauf laut CC1101 Datasheet Section 10.1:
    // 1. CS HIGH
    // 2. CS LOW, warte 1µs
    // 3. CS HIGH, warte 40µs
    // 4. CS LOW
    // 5. Warte bis MISO LOW
    // 6. Sende SRES Strobe (0x30)
    // 7. Warte bis MISO LOW (Reset abgeschlossen)

    digitalWrite(BRESSER_CS_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(BRESSER_CS_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(BRESSER_CS_PIN, HIGH);
    delayMicroseconds(40);

    digitalWrite(BRESSER_CS_PIN, LOW);
    // Warte bis MISO LOW (Chip bereit)
    uint32_t t = micros();
    while (digitalRead(BRESSER_MISO_PIN)) {
        if (micros() - t > 5000) {
            Serial.println("[Bresser] MISO timeout beim Reset – MISO bleibt HIGH");
            Serial.printf("  MISO Pin %d = %d\n", BRESSER_MISO_PIN, digitalRead(BRESSER_MISO_PIN));
            digitalWrite(BRESSER_CS_PIN, HIGH);
            return false;
        }
    }
    s_hspi.transfer(0x30);  // SRES
    // Warte bis MISO wieder LOW (Reset abgeschlossen)
    t = micros();
    while (digitalRead(BRESSER_MISO_PIN)) {
        if (micros() - t > 5000) break;
    }
    digitalWrite(BRESSER_CS_PIN, HIGH);
    delay(10);

    // Verifikation: PARTNUM=0x00, VERSION=0x14
    uint8_t part = cc1101ReadStatus(0x30);  // PARTNUM
    uint8_t ver  = cc1101ReadStatus(0x31);  // VERSION
    Serial.printf("[Bresser] Nach Reset: PARTNUM=0x%02X VERSION=0x%02X\n", part, ver);

    // Zusatz-Diagnose: einige Config-Register lesen
    uint8_t iocfg2 = cc1101ReadReg(0x00);
    uint8_t iocfg0 = cc1101ReadReg(0x02);
    Serial.printf("[Bresser] IOCFG2=0x%02X IOCFG0=0x%02X (Default: 0x29, 0x3F)\n",
                  iocfg2, iocfg0);

    // VERSION 0x14 = standard CC1101, 0x04 = 868MHz Chip-Revision – beide gültig
    return (ver == 0x14 || ver == 0x04);
}

// ─── CC1101 für FSK @ 868.35 MHz konfigurieren ───────────────────────────────

static void cc1101Configure868() {
    cc1101WriteReg(0x00, 0x06);  // IOCFG2
    cc1101WriteReg(0x02, 0x0D);  // IOCFG0: async serial data out
    cc1101WriteReg(0x07, 0x04);  // PKTCTRL1
    cc1101WriteReg(0x08, 0x32);  // PKTCTRL0: async serial, infinite
    cc1101WriteReg(0x0B, 0x06);  // FSCTRL1
    cc1101WriteReg(0x0C, 0x00);  // FSCTRL0
    cc1101WriteReg(0x0D, 0x21);  // FREQ2  
    cc1101WriteReg(0x0E, 0x66);  // FREQ1   > 868.3 MHz
    cc1101WriteReg(0x0F, 0x0A);  // FREQ0 
    cc1101WriteReg(0x10, 0xC8);  // MDMCFG4: BW=203kHz
    cc1101WriteReg(0x11, 0x93);  // MDMCFG3: ~8054 bps
    cc1101WriteReg(0x12, 0x10);  // MDMCFG2: FSK, no sync
    cc1101WriteReg(0x13, 0x22);  // MDMCFG1
    cc1101WriteReg(0x14, 0xF8);  // MDMCFG0
    cc1101WriteReg(0x15, 0x41);  // DEVIATN: ~90kHz
    cc1101WriteReg(0x18, 0x18);  // MCSM0
    cc1101WriteReg(0x19, 0x1D);  // FOCCFG
    cc1101WriteReg(0x1A, 0x1C);  // BSCFG
    cc1101WriteReg(0x1B, 0xC7);  // AGCCTRL2
    cc1101WriteReg(0x1C, 0x00);  // AGCCTRL1
    cc1101WriteReg(0x1D, 0xB0);  // AGCCTRL0
    cc1101WriteReg(0x21, 0xB6);  // FREND1
    cc1101WriteReg(0x22, 0x10);  // FREND0
    cc1101WriteReg(0x23, 0xEA);  // FSCAL3
    cc1101WriteReg(0x24, 0x2A);  // FSCAL2
    cc1101WriteReg(0x25, 0x00);  // FSCAL1
    cc1101WriteReg(0x26, 0x1F);  // FSCAL0

    cc1101Strobe(0x34);  // SRX
    delay(5);
    pinMode(BRESSER_GDO0_PIN, INPUT);
    Serial.printf("[Bresser] CC1101 konfiguriert @ %.2f MHz FSK\n", BRESSER_FREQ_MHZ);
}

// ─── Bresser 7-in-1 Decoder ───────────────────────────────────────────────────

static uint16_t lfsr_digest16(const uint8_t* data, int len,
                               uint16_t gen, uint16_t key) {
    uint16_t sum = 0;
    for (int k = 0; k < len; k++) {
        uint8_t byte = data[k];
        for (int i = 7; i >= 0; i--) {
            if ((byte >> i) & 1) sum ^= key;
            if (key & 1) key = (key >> 1) ^ gen;
            else         key >>= 1;
        }
    }
    return sum;
}

static constexpr int MSG_BYTES     = 25;
static constexpr int PREAMBLE_LEN  = 5;
static const uint8_t PREAMBLE[]    = { 0xAA, 0xAA, 0xAA, 0x2D, 0xD4 };
static constexpr int BIT_BUF_BYTES = 64;

static uint8_t  s_bitBuf[BIT_BUF_BYTES] = {};
static int      s_bitPos = 0;

static void pushBit(uint8_t bit) {
    int byteIdx = (s_bitPos / 8) % BIT_BUF_BYTES;
    int bitIdx  = 7 - (s_bitPos % 8);
    if (bitIdx == 7) s_bitBuf[byteIdx] = 0;
    s_bitBuf[byteIdx] |= (bit << bitIdx);
    s_bitPos++;
}

static bool findAndExtract(uint8_t* msg) {
    int totalBytes = s_bitPos / 8;
    if (totalBytes < PREAMBLE_LEN + MSG_BYTES) return false;

    int checkBytes = min(totalBytes, BIT_BUF_BYTES);
    uint8_t linear[BIT_BUF_BYTES];
    for (int i = 0; i < checkBytes; i++) {
        linear[i] = s_bitBuf[(s_bitPos / 8 - checkBytes + i + BIT_BUF_BYTES) % BIT_BUF_BYTES];
    }

    for (int i = 0; i <= checkBytes - PREAMBLE_LEN - MSG_BYTES; i++) {
        bool match = true;
        for (int j = 0; j < PREAMBLE_LEN; j++) {
            if (linear[i+j] != PREAMBLE[j]) { match = false; break; }
        }
        if (match) {
            memcpy(msg, &linear[i + PREAMBLE_LEN], MSG_BYTES);
            return true;
        }
    }
    return false;
}

static bool bresserDecode(const uint8_t* rawMsg, char* jsonOut, size_t jsonLen) {
    uint8_t msg[MSG_BYTES];
    memcpy(msg, rawMsg, MSG_BYTES);

    int s_type   = msg[6] >> 4;
    int nstartup = (msg[6] & 0x08) >> 3;

    for (int i = 0; i < MSG_BYTES; i++) msg[i] ^= 0xAA;  // de-whitening

    uint16_t chk    = ((uint16_t)msg[0] << 8) | msg[1];
    uint16_t digest = lfsr_digest16(&msg[2], 23, 0x8810, 0xBA95);
    if ((chk ^ digest) != 0x6DF1) {
        Serial.printf("[Bresser] Digest fail\n");
        return false;
    }

    int id      = ((int)msg[2] << 8) | msg[3];
    int flags   = msg[15] & 0x0F;
    int bat_low = (flags & 0x06) == 0x06;

    StaticJsonDocument<384> doc;
    doc["id"]         = id;
    doc["battery_ok"] = !bat_low;
    doc["startup"]    = !nstartup;

    if (s_type == 1 || s_type == 12 || s_type == 13) {
        int wdir     = (msg[4] >> 4) * 100 + (msg[4] & 0x0F) * 10 + (msg[5] >> 4);
        int wgst_raw = (msg[7] >> 4) * 100 + (msg[7] & 0x0F) * 10 + (msg[8] >> 4);
        int wavg_raw = (msg[8] & 0x0F) * 100 + (msg[9] >> 4) * 10 + (msg[9] & 0x0F);
        int rain_raw = (msg[10] >> 4) * 100000 + (msg[10] & 0x0F) * 10000
                     + (msg[11] >> 4) * 1000   + (msg[11] & 0x0F) * 100
                     + (msg[12] >> 4) * 10     + (msg[12] & 0x0F);
        int temp_raw = (msg[14] >> 4) * 100 + (msg[14] & 0x0F) * 10 + (msg[15] >> 4);
        float temp_c = temp_raw > 600 ? (temp_raw - 1000) * 0.1f : temp_raw * 0.1f;
        int   hum    = (msg[16] >> 4) * 10 + (msg[16] & 0x0F);
        int lght_raw = (msg[17] >> 4) * 100000 + (msg[17] & 0x0F) * 10000
                     + (msg[18] >> 4) * 1000   + (msg[18] & 0x0F) * 100
                     + (msg[19] >> 4) * 10     + (msg[19] & 0x0F);
        int uv_raw   = (msg[20] >> 4) * 100 + (msg[20] & 0x0F) * 10 + (msg[21] >> 4);

        doc["model"]         = (s_type == 12) ? "Bresser-3in1" : "Bresser-7in1";
        doc["temperature_C"] = serialized(String(temp_c, 1));
        doc["humidity"]      = hum;
        doc["rain_mm"]       = serialized(String(rain_raw * 0.1f, 1));
        if (s_type != 12) {
            doc["wind_max_m_s"] = serialized(String(wgst_raw * 0.1f, 1));
            doc["wind_avg_m_s"] = serialized(String(wavg_raw * 0.1f, 1));
            doc["wind_dir_deg"] = wdir;
            doc["light_lux"]    = (float)lght_raw;
            doc["uvi"]          = serialized(String(uv_raw * 0.1f, 1));
        }
    } else if (s_type == 8) {
        int pm25 = (msg[10] & 0x0F) * 1000 + (msg[11] >> 4) * 100
                 + (msg[11] & 0x0F) * 10   + (msg[12] >> 4);
        int pm10 = (msg[12] & 0x0F) * 1000 + (msg[13] >> 4) * 100
                 + (msg[13] & 0x0F) * 10   + (msg[14] >> 4);
        doc["model"]        = "Bresser-AirPM";
        doc["pm2_5_ug_m3"]  = pm25;
        doc["pm10_0_ug_m3"] = pm10;
    } else if (s_type == 10) {
        int co2 = ((msg[4] & 0xF0) >> 4) * 1000 + (msg[4] & 0x0F) * 100
                + ((msg[5] & 0xF0) >> 4) * 10   + (msg[5] & 0x0F);
        doc["model"]   = "Bresser-CO2";
        doc["co2_ppm"] = co2;
    } else if (s_type == 11) {
        int hcho = ((msg[4] & 0xF0) >> 4) * 1000 + (msg[4] & 0x0F) * 100
                 + ((msg[5] & 0xF0) >> 4) * 10   + (msg[5] & 0x0F);
        int voc  = msg[22] & 0x0F;
        doc["model"]    = "Bresser-HCHOVOC";
        doc["hcho_ppb"] = hcho;
        if (voc != 0x0F) doc["voc_level"] = voc;
    } else {
        Serial.printf("[Bresser] Unbekannter Sensor-Typ: %d\n", s_type);
        return false;
    }

    serializeJson(doc, jsonOut, jsonLen);
    return true;
}

// ─── Hardware-Timer: Bit-Sampling alle 124µs ──────────────────────────────────

static hw_timer_t*      s_timer  = nullptr;
static volatile uint8_t s_curBit = 0;
static volatile bool    s_newBit = false;

void IRAM_ATTR bresserTimerISR() {
    s_curBit = digitalRead(BRESSER_GDO0_PIN);
    s_newBit = true;
}

// ─── Öffentliche API ─────────────────────────────────────────────────────────

void bresserInit(PubSubClient& client) {
    s_client = &client;

    pinMode(BRESSER_CS_PIN,   OUTPUT);
    pinMode(BRESSER_MISO_PIN, INPUT);
    digitalWrite(BRESSER_CS_PIN, HIGH);
    delay(10);

    s_hspi.begin(BRESSER_SCK_PIN, BRESSER_MISO_PIN, BRESSER_MOSI_PIN, BRESSER_CS_PIN);
    s_hspi.setFrequency(1000000);  // 1MHz – stabil auf allen Breakout-Boards
    s_hspi.setDataMode(SPI_MODE0);
    s_hspi.setBitOrder(MSBFIRST);
    delay(10);

    bool found = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        if (cc1101Reset()) { found = true; break; }
        Serial.printf("[Bresser] Versuch %d fehlgeschlagen\n", attempt);
        delay(100);
    }

    if (!found) {
        Serial.println("[Bresser] CC1101 nicht gefunden! Pins prüfen:");
        Serial.printf("  SCK=%d MISO=%d MOSI=%d CS=%d GDO0=%d\n",
                      BRESSER_SCK_PIN, BRESSER_MISO_PIN,
                      BRESSER_MOSI_PIN, BRESSER_CS_PIN, BRESSER_GDO0_PIN);
        // MISO-Pegel direkt messen als Diagnose
        Serial.printf("  MISO direkt: %d (soll beim Reset 0 werden)\n",
                      digitalRead(BRESSER_MISO_PIN));
        return;
    }

    cc1101Configure868();

    s_timer = timerBegin(1, 80, true);
    timerAttachInterrupt(s_timer, &bresserTimerISR, true);
    timerAlarmWrite(s_timer, 124, true);
    timerAlarmEnable(s_timer);

    s_ready = true;
    Serial.println("[Bresser] Bereit");
}

void bresserLoop() {
    if (!s_ready || !s_newBit) return;
    uint8_t bit = s_curBit;
    s_newBit = false;

    pushBit(bit);

    uint8_t msg[MSG_BYTES];
    if (!findAndExtract(msg)) return;

    static uint8_t  lastMsg[MSG_BYTES] = {};
    static uint32_t lastTime = 0;
    if (memcmp(msg, lastMsg, MSG_BYTES) == 0 && (millis() - lastTime) < 60000) return;
    memcpy(lastMsg, msg, MSG_BYTES);
    lastTime = millis();

    char json[384];
    if (!bresserDecode(msg, json, sizeof(json))) return;

    Serial.printf("[Bresser] → %s\n", json);
    if (s_client && s_client->connected()) {
        s_client->publish(BRESSER_MQTT_TOPIC, json, false);
    }

    s_bitPos = 0;
    memset(s_bitBuf, 0, sizeof(s_bitBuf));
}
