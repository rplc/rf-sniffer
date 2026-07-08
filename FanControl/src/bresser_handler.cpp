#include "bresser_handler.h"
#include "config.h"
#include <SPI.h>
#include <SmartRC_CC1101.h>
#include <ArduinoJson.h>

// ─── Zweiter CC1101 auf HSPI ─────────────────────────────────────────────────
// Der 433MHz CC1101 nutzt VSPI (Pins 18/19/23).
// Der Bresser CC1101 nutzt HSPI (Pins 14/12/13) mit eigenem CS.
// SmartRC_CC1101 nutzt eine globale Instanz – wir steuern den zweiten
// CC1101 direkt über SPI-Direktzugriff mit HSPI.

static SPIClass        s_hspi(HSPI);
static PubSubClient*   s_client = nullptr;

// ─── CC1101 SPI-Zugriff (HSPI) ───────────────────────────────────────────────

static void cc1101Write(uint8_t addr, uint8_t val) {
    s_hspi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(BRESSER_CS_PIN, LOW);
    delayMicroseconds(2);
    s_hspi.transfer(addr);
    s_hspi.transfer(val);
    digitalWrite(BRESSER_CS_PIN, HIGH);
    s_hspi.endTransaction();
}

static uint8_t cc1101Read(uint8_t addr) {
    s_hspi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(BRESSER_CS_PIN, LOW);
    delayMicroseconds(2);
    s_hspi.transfer(addr | 0x80);
    uint8_t val = s_hspi.transfer(0x00);
    digitalWrite(BRESSER_CS_PIN, HIGH);
    s_hspi.endTransaction();
    return val;
}

static void cc1101Strobe(uint8_t cmd) {
    s_hspi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(BRESSER_CS_PIN, LOW);
    delayMicroseconds(2);
    s_hspi.transfer(cmd);
    digitalWrite(BRESSER_CS_PIN, HIGH);
    s_hspi.endTransaction();
}

static void cc1101WriteBurst(uint8_t addr, const uint8_t* data, uint8_t len) {
    s_hspi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(BRESSER_CS_PIN, LOW);
    delayMicroseconds(2);
    s_hspi.transfer(addr | 0x40);  // burst write
    for (uint8_t i = 0; i < len; i++) s_hspi.transfer(data[i]);
    digitalWrite(BRESSER_CS_PIN, HIGH);
    s_hspi.endTransaction();
}

static void cc1101ReadBurst(uint8_t addr, uint8_t* data, uint8_t len) {
    s_hspi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(BRESSER_CS_PIN, LOW);
    delayMicroseconds(2);
    s_hspi.transfer(addr | 0xC0);  // burst read
    for (uint8_t i = 0; i < len; i++) data[i] = s_hspi.transfer(0x00);
    digitalWrite(BRESSER_CS_PIN, HIGH);
    s_hspi.endTransaction();
}

// ─── CC1101 Reset ─────────────────────────────────────────────────────────────

static bool cc1101Reset() {
    digitalWrite(BRESSER_CS_PIN, HIGH);
    delay(1);
    digitalWrite(BRESSER_CS_PIN, LOW);
    delay(1);
    digitalWrite(BRESSER_CS_PIN, HIGH);
    delay(1);
    cc1101Strobe(0x30);  // SRES
    delay(10);

    // Partnum sollte 0x00, Version 0x14 sein
    uint8_t part = cc1101Read(0xF0);  // PARTNUM
    uint8_t ver  = cc1101Read(0xF1);  // VERSION
    Serial.printf("[Bresser] CC1101 Part=0x%02X Ver=0x%02X\n", part, ver);
    return (ver == 0x14);
}

// ─── CC1101 für FSK PCM @ 868.35 MHz konfigurieren ───────────────────────────
// Bresser 7-in-1: FSK_PULSE_PCM, short_width=124µs, long_width=124µs
// Bitrate: 1/124µs ≈ 8065 bps
// Deviation: 90kHz (typisch für Bresser)
// Bandbreite: ~200kHz

static void cc1101Configure868() {
    // Register-Konfiguration für FSK, 8065 bps, 868.35 MHz
    cc1101Write(0x00, 0x06);  // IOCFG2: GDO2 = sync word received
    cc1101Write(0x02, 0x0D);  // IOCFG0: GDO0 = serial data output (async)
    cc1101Write(0x07, 0x04);  // PKTCTRL1: no status bytes, no addr check
    cc1101Write(0x08, 0x32);  // PKTCTRL0: async serial mode, infinite length
    cc1101Write(0x0B, 0x06);  // FSCTRL1: IF = 152 kHz
    cc1101Write(0x0C, 0x00);  // FSCTRL0: freq offset = 0
    // Frequenz: 868.35 MHz → FREQ = 868.35e6 / (26e6/2^16) = 2189848
    // = 0x2169B8 ... close enough, use 868.3 MHz
    cc1101Write(0x0D, 0x21);  // FREQ2
    cc1101Write(0x0E, 0x6B);  // FREQ1
    cc1101Write(0x0F, 0xD0);  // FREQ0  → 868.35 MHz
    // Bitrate: 8065 bps → DRATE = (256 + DRATE_M) * 2^DRATE_E / 2^28 * 26e6
    // 8065 = (256+131) * 2^5 / 2^28 * 26e6 → DRATE_M=131(0x83), E=5
    cc1101Write(0x10, 0xC8);  // MDMCFG4: BW=203kHz, DRATE_E=8
    cc1101Write(0x11, 0x93);  // MDMCFG3: DRATE_M=0x93 → ~8054 bps
    cc1101Write(0x12, 0x10);  // MDMCFG2: FSK, no sync word, no Manchester
    cc1101Write(0x13, 0x22);  // MDMCFG1: preamble=4 bytes
    cc1101Write(0x14, 0xF8);  // MDMCFG0: channel spacing
    cc1101Write(0x15, 0x41);  // DEVIATN: FSK deviation = 90kHz
    cc1101Write(0x18, 0x18);  // MCSM0: auto-calibrate on IDLE→RX/TX
    cc1101Write(0x19, 0x1D);  // FOCCFG: freq offset compensation
    cc1101Write(0x1A, 0x1C);  // BSCFG: bit sync
    cc1101Write(0x1B, 0xC7);  // AGCCTRL2
    cc1101Write(0x1C, 0x00);  // AGCCTRL1
    cc1101Write(0x1D, 0xB0);  // AGCCTRL0
    cc1101Write(0x21, 0xB6);  // FREND1
    cc1101Write(0x22, 0x10);  // FREND0
    cc1101Write(0x23, 0xEA);  // FSCAL3
    cc1101Write(0x24, 0x2A);  // FSCAL2
    cc1101Write(0x25, 0x00);  // FSCAL1
    cc1101Write(0x26, 0x1F);  // FSCAL0

    cc1101Strobe(0x34);  // SRX – in Empfangsmodus
    delay(5);

    pinMode(BRESSER_GDO0_PIN, INPUT);
    Serial.printf("[Bresser] CC1101 konfiguriert @ %.2f MHz FSK\n", BRESSER_FREQ_MHZ);
}

// ─── Bresser 7-in-1 Protokoll-Decoder ────────────────────────────────────────
// Portiert aus rtl_433/src/devices/bresser_7in1.c
// Lizenz: GPL-2.0 (Christian W. Zuckschwerdt)
//
// Preamble: AA AA AA 2D D4
// Payload:  25 Bytes nach Preamble
// Whitening: XOR 0xAA
// Integrity: LFSR-16 digest, generator 0x8810, key 0xba95, final XOR 0x6df1

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

// Bit-Buffer: wir sammeln Bits aus dem async seriellen Datenstrom
static constexpr int MSG_BYTES = 25;
static constexpr int PREAMBLE_LEN = 5;  // AA AA AA 2D D4

static const uint8_t PREAMBLE[] = { 0xAA, 0xAA, 0xAA, 0x2D, 0xD4 };

// Bit-Ring-Buffer für den seriellen Datenstrom
static constexpr int BIT_BUF_BYTES = 64;
static uint8_t  s_bitBuf[BIT_BUF_BYTES] = {};
static int      s_bitPos  = 0;   // aktuelle Bit-Position (gesamt)
static uint32_t s_lastBit = 0;   // Zeitpunkt letztes Bit

// Fügt ein Bit in den Ringbuffer
static void pushBit(uint8_t bit) {
    int byteIdx = (s_bitPos / 8) % BIT_BUF_BYTES;
    int bitIdx  = 7 - (s_bitPos % 8);
    if (bitIdx == 7) s_bitBuf[byteIdx] = 0;
    s_bitBuf[byteIdx] |= (bit << bitIdx);
    s_bitPos++;
}

// Sucht Preamble und extrahiert Nachricht
static bool findAndExtract(uint8_t* msg) {
    // Suche Preamble im Ringbuffer
    // Wir müssen mindestens PREAMBLE_LEN + MSG_BYTES Bytes haben
    int totalBits = s_bitPos;
    if (totalBits < (PREAMBLE_LEN + MSG_BYTES) * 8) return false;

    // Linearen Buffer aus Ringbuffer aufbauen (letzte ~400 Bits)
    int checkBytes = min(totalBits / 8, BIT_BUF_BYTES);
    uint8_t linear[BIT_BUF_BYTES];
    for (int i = 0; i < checkBytes; i++) {
        linear[i] = s_bitBuf[(s_bitPos / 8 - checkBytes + i) % BIT_BUF_BYTES];
    }

    // Preamble suchen
    for (int i = 0; i <= checkBytes - PREAMBLE_LEN - MSG_BYTES; i++) {
        bool match = true;
        for (int j = 0; j < PREAMBLE_LEN; j++) {
            if (linear[i+j] != PREAMBLE[j]) { match = false; break; }
        }
        if (!match) continue;

        // Preamble gefunden – Nachricht extrahieren
        memcpy(msg, &linear[i + PREAMBLE_LEN], MSG_BYTES);
        return true;
    }
    return false;
}

static bool bresserDecode(const uint8_t* rawMsg, char* jsonOut, size_t jsonLen) {
    uint8_t msg[MSG_BYTES];
    memcpy(msg, rawMsg, MSG_BYTES);

    // Sensor-Typ (vor Whitening-Entfernung)
    if (msg[21] == 0x00) return false;  // Sanity check

    int s_type   = msg[6] >> 4;
    int nstartup = (msg[6] & 0x08) >> 3;

    // Whitening entfernen
    for (int i = 0; i < MSG_BYTES; i++) msg[i] ^= 0xAA;

    // LFSR-16 Digest prüfen
    uint16_t chk    = ((uint16_t)msg[0] << 8) | msg[1];
    uint16_t digest = lfsr_digest16(&msg[2], 23, 0x8810, 0xBA95);
    if ((chk ^ digest) != 0x6DF1) {
        Serial.printf("[Bresser] Digest fail: %04X vs %04X\n", chk, digest ^ chk);
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
        // Wetterstation (7-in-1, 3-in-1, 8-in-1)
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

        doc["model"]          = (s_type == 12) ? "Bresser-3in1" : "Bresser-7in1";
        doc["temperature_C"]  = serialized(String(temp_c, 1));
        doc["humidity"]       = hum;
        doc["rain_mm"]        = serialized(String(rain_raw * 0.1f, 1));
        if (s_type != 12) {
            doc["wind_max_m_s"]  = serialized(String(wgst_raw * 0.1f, 1));
            doc["wind_avg_m_s"]  = serialized(String(wavg_raw * 0.1f, 1));
            doc["wind_dir_deg"]  = wdir;
            doc["light_lux"]     = (float)lght_raw;
            doc["uvi"]           = serialized(String(uv_raw * 0.1f, 1));
        }
        if (s_type == 13 && (msg[23] >> 4) < 10) {
            float tglobe = (msg[22] >> 4) * 10 + (msg[22] & 0x0F) + (msg[23] >> 4) * 0.1f;
            doc["temperature_globe_C"] = serialized(String(tglobe, 1));
        }

    } else if (s_type == 8) {
        // Air Quality PM2.5/PM10
        int pm25     = (msg[10] & 0x0F) * 1000 + (msg[11] >> 4) * 100
                     + (msg[11] & 0x0F) * 10   + (msg[12] >> 4);
        int pm10     = (msg[12] & 0x0F) * 1000 + (msg[13] >> 4) * 100
                     + (msg[13] & 0x0F) * 10   + (msg[14] >> 4);
        bool pm25_ok = (msg[10] & 0x0F) != 0x0F;
        bool pm10_ok = (msg[12] & 0x0F) != 0x0F;
        doc["model"]   = "Bresser-AirPM";
        if (pm25_ok) doc["pm2_5_ug_m3"]  = pm25;
        if (pm10_ok) doc["pm10_0_ug_m3"] = pm10;

    } else if (s_type == 10) {
        // CO2
        int co2    = ((msg[4] & 0xF0) >> 4) * 1000 + (msg[4] & 0x0F) * 100
                   + ((msg[5] & 0xF0) >> 4) * 10   + (msg[5] & 0x0F);
        bool co2ok = (msg[5] & 0x0F) != 0x0F;
        doc["model"] = "Bresser-CO2";
        if (co2ok) doc["co2_ppm"] = co2;

    } else if (s_type == 11) {
        // HCHO/VOC
        int hcho    = ((msg[4] & 0xF0) >> 4) * 1000 + (msg[4] & 0x0F) * 100
                    + ((msg[5] & 0xF0) >> 4) * 10   + (msg[5] & 0x0F);
        int voc     = msg[22] & 0x0F;
        bool hchook = (msg[5] & 0x0F) != 0x0F;
        bool vocok  = voc != 0x0F;
        doc["model"] = "Bresser-HCHOVOC";
        if (hchook) doc["hcho_ppb"]  = hcho;
        if (vocok)  doc["voc_level"] = voc;

    } else {
        Serial.printf("[Bresser] Unbekannter Sensor-Typ: %d\n", s_type);
        return false;
    }

    serializeJson(doc, jsonOut, jsonLen);
    return true;
}

// ─── ISR: Bit-Sampling via GDO0 ──────────────────────────────────────────────
// Der CC1101 gibt im Async-RX-Mode den demodulierten Bitstrom auf GDO0 aus.
// Wir sampeln mit einem Timer alle 124µs (= 1/8065 bps).

static hw_timer_t*     s_timer   = nullptr;
static volatile bool   s_newBit  = false;
static volatile uint8_t s_curBit = 0;

void IRAM_ATTR bresserTimerISR() {
    s_curBit = digitalRead(BRESSER_GDO0_PIN);
    s_newBit = true;
}

// ─── Zustandsmaschine für Byte-Empfang ───────────────────────────────────────

static constexpr int TIMER_US = 124;  // 1 Bit-Periode

static enum { IDLE, SYNCING, RECEIVING } s_state = IDLE;
static uint8_t  s_rxByte   = 0;
static int      s_rxBitCnt = 0;
static uint8_t  s_rxBuf[PREAMBLE_LEN + MSG_BYTES + 4];
static int      s_rxByteCnt = 0;

static bool s_frameReady  = false;
static uint8_t s_frameMsg[MSG_BYTES];

// ─── Öffentliche API ─────────────────────────────────────────────────────────

void bresserInit(PubSubClient& client) {
    s_client = &client;

    s_hspi.begin(BRESSER_SCK_PIN, BRESSER_MISO_PIN, BRESSER_MOSI_PIN, BRESSER_CS_PIN);
    pinMode(BRESSER_CS_PIN, OUTPUT);
    digitalWrite(BRESSER_CS_PIN, HIGH);
    delay(10);

    if (!cc1101Reset()) {
        Serial.println("[Bresser] CC1101 nicht gefunden!");
        return;
    }
    cc1101Configure868();

    // Hardware-Timer für Bit-Sampling (124µs Periode)
    s_timer = timerBegin(1, 80, true);  // Timer 1, 80MHz/80=1MHz, count-up
    timerAttachInterrupt(s_timer, &bresserTimerISR, true);
    timerAlarmWrite(s_timer, TIMER_US, true);  // Alarm alle 124µs
    timerAlarmEnable(s_timer);

    Serial.println("[Bresser] RX-Timer aktiv (124µs Sampling)");
}

void bresserLoop() {
    if (!s_newBit) return;
    uint8_t bit = s_curBit;
    s_newBit = false;

    pushBit(bit);

    // Nachricht suchen und dekodieren
    uint8_t msg[MSG_BYTES];
    if (!findAndExtract(msg)) return;

    // Prüfen ob Nachricht identisch mit letzter (Debounce)
    static uint8_t lastMsg[MSG_BYTES] = {};
    static uint32_t lastTime = 0;
    if (memcmp(msg, lastMsg, MSG_BYTES) == 0 && (millis() - lastTime) < 60000) return;
    memcpy(lastMsg, msg, MSG_BYTES);
    lastTime = millis();

    char json[384];
    if (!bresserDecode(msg, json, sizeof(json))) return;

    Serial.printf("[Bresser] → %s : %s\n", BRESSER_MQTT_TOPIC, json);
    if (s_client && s_client->connected()) {
        s_client->publish(BRESSER_MQTT_TOPIC, json, false);
    }

    // Bit-Buffer zurücksetzen nach erfolgreichem Decode
    s_bitPos = 0;
    memset(s_bitBuf, 0, sizeof(s_bitBuf));
}
