#include "bresser_handler.h"
#include "config.h"
#include <SPI.h>
#include <RadioLib.h>
#include <ArduinoJson.h>
#include <algorithm>

// ─── CC1101 (RadioLib) ───────────────────────────────────────────────────────
// Eigener SPI-Bus (HSPI), 433MHz-CC1101 (rf_handler) läuft auf VSPI.
// GDO0 ist hier NICHT mehr ein bit-gebangter Rohdatenausgang, sondern die
// Paket-Empfangen-IRQ-Leitung von RadioLib (fixedPacketLengthMode + FIFO).
static SPIClass      s_hspi(HSPI);
static Module        s_mod(BRESSER_CS_PIN, BRESSER_GDO0_PIN, RADIOLIB_NC, RADIOLIB_NC, s_hspi);
static CC1101        s_radio(&s_mod);

static PubSubClient* s_client = nullptr;
static bool          s_ready  = false;

// Gesamte Paketlänge im CC1101 Fixed-Length-Modus. Aufbau (siehe Konfiguration
// unten): [0]=0xD4 (letztes Sync-Byte, landet als erstes FIFO-Byte im Payload,
// siehe Kommentar bei setSyncWord), [1..25]=eigentliche 25 Bresser-7in1-Bytes,
// [26]=1 Byte Reserve.
static constexpr uint8_t MSG_BUF_SIZE = 27;
static constexpr int     MSG_BYTES    = 25;

// -----------------------------------------------------------------------------
// Frequency Scan
// -----------------------------------------------------------------------------

static constexpr float kFreqList[] = {
    868.220f, 868.225f, 868.230f, 868.235f,
    868.240f, 868.245f, 868.250f, 868.255f,
    868.260f, 868.265f, 868.270f, 868.275f,
    868.280f, 868.285f, 868.290f, 868.295f,
    868.300f, 868.305f, 868.310f, 868.315f,
    868.320f, 868.325f, 868.330f, 868.335f,
    868.340f, 868.345f, 868.350f, 868.355f,
    868.360f
};

static constexpr uint8_t kFreqCount =
    sizeof(kFreqList) / sizeof(kFreqList[0]);

static constexpr uint32_t SCAN_TIME_MS = 605000;

static uint8_t currentFreq = 0;
static uint32_t scanStart = 0;

// Statistik
static uint32_t irqCount = 0;
static uint32_t readOk = 0;
static uint32_t readError = 0;
static uint32_t digestOk = 0;
static uint32_t digestFail = 0;

static float rssiSum = 0;
static float rssiMin = 999;
static float rssiMax = -999;

// Für Median
static float rssiValues[256];
static uint16_t rssiCount = 0;

static float lqiSum = 0;
static uint8_t lqiMin = 255;
static uint8_t lqiMax = 0;
static uint8_t lqiValues[256];
static uint16_t lqiCount = 0;

// ─── ISR: Paket vollständig empfangen ────────────────────────────────────────

static volatile bool s_receivedFlag = false;

void IRAM_ATTR bresserPacketISR() {
    s_receivedFlag = true;
}

// RadioLib's CC1101::getRSSI() liefert im Paket-Modus NUR den gecachten
// RSSI-Wert des letzten erfolgreich gelesenen Pakets (intern "rawRSSI",
// initialisiert mit 0 → ergibt exakt -74.0 dBm). Das ist also KEIN Live-Wert
// des Rauschgrunds, sondern bleibt eingefroren, bis wieder ein Paket komplett
// empfangen wurde. Für einen echten Live-Wert (auch ohne Paket, z.B. für den
// periodischen Idle-Log) lesen wir das RSSI-Statusregister direkt – das wird
// vom CC1101 während RX kontinuierlich aktualisiert, unabhängig von
// Paketempfang. Braucht -DRADIOLIB_LOW_LEVEL=1 (bereits in platformio.ini).
static float bresserLiveRSSI() {
    uint8_t raw = s_radio.SPIreadRegister(RADIOLIB_CC1101_REG_RSSI);
    if (raw >= 128) return (((float)raw - 256.0f) / 2.0f) - 74.0f;
    return ((float)raw / 2.0f) - 74.0f;
}

// ─── Bresser 7-in-1 Decoder (unverändert – Vorlage: rtl_433 bresser_7in1.c) ──

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

static bool bresserDecode(const uint8_t* rawMsg, float rssi, char* jsonOut, size_t jsonLen) {
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
    doc["rssi"]       = serialized(String(rssi, 1));

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

static void resetDebugStats()
{
    irqCount = 0;
    readOk = 0;
    readError = 0;
    digestOk = 0;
    digestFail = 0;

    rssiSum = 0;
    rssiMin = 999;
    rssiMax = -999;
    rssiCount = 0;

    lqiSum = 0;
    lqiMin = 255;
    lqiMax = 0;
    lqiCount = 0;

    scanStart = millis();
}

static float calcMedianRSSI()
{
    if(rssiCount == 0)
        return 0;

    float tmp[256];

    memcpy(tmp, rssiValues, sizeof(float) * rssiCount);

    std::sort(tmp, tmp + rssiCount);

    if(rssiCount & 1)
        return tmp[rssiCount / 2];

    return (tmp[rssiCount/2-1] + tmp[rssiCount/2]) / 2.0f;
}

static float calcMedianLQI()
{
    if (lqiCount == 0)
        return 0;

    uint8_t tmp[256];
    memcpy(tmp, lqiValues, sizeof(uint8_t) * lqiCount);
    std::sort(tmp, tmp + lqiCount);

    if (lqiCount & 1)
        return tmp[lqiCount / 2];

    return (tmp[lqiCount/2-1] + tmp[lqiCount/2]) / 2.0f;
}

static void publishDebugStats()
{
    StaticJsonDocument<512> doc;

    doc["freq"] = kFreqList[currentFreq];
    doc["runtime_s"] = (millis() - scanStart) / 1000;

    doc["irq"] = irqCount;
    doc["read_ok"] = readOk;
    doc["read_error"] = readError;
    doc["digest_ok"] = digestOk;
    doc["digest_fail"] = digestFail;

    if(readOk)
    {
        doc["avg_rssi"] = rssiSum / readOk;
        doc["median_rssi"] = calcMedianRSSI();
        doc["min_rssi"] = rssiMin;
        doc["max_rssi"] = rssiMax;

        doc["avg_lqi"] = lqiSum / readOk;
        doc["median_lqi"] = calcMedianLQI();
        doc["min_lqi"] = lqiMin;
        doc["max_lqi"] = lqiMax;
    }

    char json[512];
    serializeJson(doc, json);

    Serial.println(json);

    if(s_client && s_client->connected())
        s_client->publish("weather/bresserDebug", json, false);
}

// ─── Öffentliche API ─────────────────────────────────────────────────────────

void bresserInit(PubSubClient& client) {
    s_client = &client;

    s_hspi.begin(BRESSER_SCK_PIN, BRESSER_MISO_PIN, BRESSER_MOSI_PIN, BRESSER_CS_PIN);

    // Konfiguration 1:1 übernommen von matthias-bs/BresserWeatherSensorReceiver
    // (aktiv gepflegtes, nachweislich funktionierendes RadioLib+CC1101-Setup für
    // exakt diesen Sensor). Quelle für die "krummen" Werte:
    // https://github.com/RFD-FHEM/RFFHEM/issues/607#issuecomment-830818445
    // Freq: 868.300 MHz, Bandbreite: 203kHz (Messung), DataRate: 8207.32 Baud
    ConfigFSK_t cfg;
    cfg.frequency          = BRESSER_FREQ_MHZ + (BRESSER_FREQ_OFFSET_KHZ / 1000.0); // MHz
    cfg.bitRate             = 8.21f;             // kBaud
    cfg.frequencyDeviation  = 57.136417f;        // kHz
    cfg.power               = 10;                // dBm (RX-only, irrelevant)
    cfg.preambleLength      = 32;                // Bit
    cfg.receiverBandwidth   = 203.0f;             // kHz — empirisch bester RSSI-Wert (siehe unten)

    //cfg.frequency = kFreqList[currentFreq]; // TODO add offset to frequency sweep
    int16_t state = s_radio.begin(cfg);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[Bresser] CC1101 Init fehlgeschlagen (Code %d) - SPI/Verkabelung prüfen\n", state);
        return;
    }

    // TI CC1101 Errata (Abschnitt 5.3): bei Datenraten < 250 kBaud müssen
    // TEST2/TEST1/TEST0 von den POR-Defaults (0x88/0x31/0x09) abweichend
    // gesetzt werden. FREND1 ist ein bekannter TI-Tuning-Wert für das
    // // RX-Frontend (LNA/Mixer-Bias).
    s_radio.SPIwriteRegister(RADIOLIB_CC1101_REG_AGCCTRL2, 0xC7);
    s_radio.SPIwriteRegister(RADIOLIB_CC1101_REG_AGCCTRL1, 0x00);
    s_radio.SPIwriteRegister(RADIOLIB_CC1101_REG_AGCCTRL0, 0xB2);
    s_radio.SPIwriteRegister(RADIOLIB_CC1101_REG_FOCCFG,   0x16);
    s_radio.SPIwriteRegister(RADIOLIB_CC1101_REG_BSCFG,    0x1C);
    s_radio.SPIwriteRegister(RADIOLIB_CC1101_REG_FREND1,   0x56);
    s_radio.SPIwriteRegister(RADIOLIB_CC1101_REG_TEST2,    0x81);
    s_radio.SPIwriteRegister(RADIOLIB_CC1101_REG_TEST1,    0x35);
    s_radio.SPIwriteRegister(RADIOLIB_CC1101_REG_TEST0,    0x09);
    s_radio.SPIwriteRegister(RADIOLIB_CC1101_REG_FSCTRL1,  0x06);

    // WICHTIG: Der TEST2/TEST1/TEST0/FREND1-Errata-Fix, der beim 433MHz-CC1101
    // (Board mit "v2.0"-Aufdruck) den entscheidenden Unterschied gemacht hat,
    // wurde hier testweise ebenfalls angewendet – und hat den RSSI empirisch
    // messbar VERSCHLECHTERT (203kHz Bandbreite: -77dBm ohne Fix vs. -92 bis
    // -104dBm mit Fix). Das 868MHz-Board hat keinen Revisions-Aufdruck (anders
    // als das 433er-Board) und ist vermutlich eine andere CC1101-Silizium-
    // Revision – TI hat die "optimalen" Register-Werte zwischen Revisionen
    // tatsächlich geändert, ein Preset ist also nicht zwangsläufig zwischen
    // Chargen/Revisionen übertragbar. Deshalb hier bewusst NICHT gesetzt und
    // stattdessen auf RadioLibs eigene begin()-Kalibrierung verlassen.

    // Fixed-Length-Paketmodus statt CC1101-Rohbitstream + Software-Präambelsuche:
    // Der CC1101 macht die Bit-/Byte-Synchronisation und Präambel-/Sync-Erkennung
    // jetzt komplett selbst und legt fertige Bytes ins FIFO.
    s_radio.fixedPacketLengthMode(MSG_BUF_SIZE);

    // Bresser nutzt eine eigene LFSR-Prüfsumme statt der CC1101-Hardware-CRC
    // (CRC-16/CCITT) – deren Filter würde sonst jedes Paket verwerfen.
    s_radio.setCrcFiltering(false);

    // Eigentliches Sync-Wort ist 0x2D 0xD4, das letzte Präambel-Byte ist 0xAA.
    // Der CC1101 erlaubt aber keine 40-Bit-Präambel, deshalb: 32-Bit-Präambel
    // (s.o.) + Sync auf "AA 2D" (letztes Präambel-Byte + erstes echtes
    // Sync-Byte). Das zweite echte Sync-Byte (0xD4) taucht dadurch als erstes
    // Byte im empfangenen Paket auf – wird unten verifiziert und übersprungen.
    // Sync-Wort-Erkennung mit 1 Bit Fehlertoleranz statt bit-exaktem Match
    // (maxErrBits=0 → 0). Bei schwachem/wechselhaftem Signal (siehe RSSI-Log)
    // erhöht das die Trefferquote spürbar, auf Kosten einer minimal höheren
    // Falscherkennungsrate (die wird ohnehin über den Digest in bresserDecode()
    // abgefangen).
    s_radio.setSyncWord(0xAA, 0x2D, 2, false);

    s_radio.setPacketReceivedAction(bresserPacketISR);

    state = s_radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[Bresser] startReceive() fehlgeschlagen (Code %d)\n", state);
        return;
    }
    resetDebugStats();

    s_ready = true;
    Serial.printf("[Bresser] CC1101 bereit @ %.3f MHz FSK (RadioLib) | Ver=0x%02X\n", BRESSER_FREQ_MHZ, s_radio.getChipVersion());

    Serial.printf("PartNum=0x%02X, Version=0x%02X, FSCAL3=0x%02X, FSCAL2=0x%02X, FSCAL1=0x%02X, FSCAL0=0x%02X, FREQ2=0x%02X, FREQ1=0x%02X, FREQ0=0x%02X\n",
        s_radio.SPIgetRegValue(RADIOLIB_CC1101_REG_PARTNUM), s_radio.SPIgetRegValue(RADIOLIB_CC1101_REG_VERSION), s_radio.SPIgetRegValue(RADIOLIB_CC1101_REG_FSCAL3),
        s_radio.SPIgetRegValue(RADIOLIB_CC1101_REG_FSCAL2), s_radio.SPIgetRegValue(RADIOLIB_CC1101_REG_FSCAL1), s_radio.SPIgetRegValue(RADIOLIB_CC1101_REG_FSCAL0),
        s_radio.SPIgetRegValue(RADIOLIB_CC1101_REG_FREQ2), s_radio.SPIgetRegValue(RADIOLIB_CC1101_REG_FREQ1), s_radio.SPIgetRegValue(RADIOLIB_CC1101_REG_FREQ0));
}

void bresserLoop() {
    // Periodische RSSI-Ausgabe (auch ohne Paket) – echter Live-Wert direkt aus
    // dem Statusregister (s. bresserLiveRSSI()), NICHT der paket-gecachte
    // s_radio.getRSSI()-Wert, der ohne Paketempfang bei 0/-74.0dBm einfriert.
    static uint32_t lastRssiLog = 0;
    uint32_t nowMs = millis();
    if (s_ready && (nowMs - lastRssiLog) > 5000) {
        lastRssiLog = nowMs;
        Serial.printf("[Bresser] RSSI (live): %.1f dBm\n", bresserLiveRSSI());
    }

    if(s_ready && millis() - scanStart >= SCAN_TIME_MS)
    {
        publishDebugStats();

        currentFreq++;

        if(currentFreq >= kFreqCount)
            currentFreq = 0;

        s_radio.standby();

        int16_t state = s_radio.setFrequency(kFreqList[currentFreq]);

        if(state != RADIOLIB_ERR_NONE)
            Serial.printf("setFrequency Fehler %d\n", state);

        s_radio.startReceive();

        resetDebugStats();

        Serial.printf("\n==============================\n");
        Serial.printf("Neue Frequenz %.3f MHz\n", kFreqList[currentFreq]);
        Serial.printf("==============================\n");
    }

    if (!s_ready || !s_receivedFlag) return;
    s_receivedFlag = false;
    irqCount++;

    Serial.println("[Bresser] IRQ (GDO0) ausgelöst – lese FIFO...");

    uint8_t recvData[MSG_BUF_SIZE];
    int16_t state = s_radio.readData(recvData, MSG_BUF_SIZE);

    // Live-Werte statt paket-gecachtem getRSSI(): funktioniert unabhängig
    // davon, ob readData() gleich erfolgreich ist oder nicht, und ist nicht
    // von einem eventuell veralteten Cache-Wert aus einem früheren Paket
    // verfälscht.
    float   rssi = bresserLiveRSSI();
    uint8_t lqi  = s_radio.getLQI();

    rssiSum += rssi;
    if(rssi < rssiMin) rssiMin = rssi;
    if(rssi > rssiMax) rssiMax = rssi;
    if(rssiCount < 256) rssiValues[rssiCount++] = rssi;

    lqiSum += lqi;
    if(lqi < lqiMin) lqiMin = lqi;
    if(lqi > lqiMax) lqiMax = lqi;
    if(lqiCount < 256) lqiValues[lqiCount++] = lqi;

    s_radio.startReceive();  // sofort wieder in RX, unabhängig vom Ergebnis

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[Bresser] readData() Fehler: %d | RSSI: %.1f dBm | LQI: %d\n", state, rssi, lqi);
        readError++;
        return;
    }
    readOk++;

    Serial.printf("[Bresser] RX %d Bytes | RSSI: %.1f dBm | LQI: %d | Byte[0]=0x%02X | Rohdaten: ",
                  MSG_BUF_SIZE, rssi, lqi, recvData[0]);
    for (uint8_t i = 0; i < MSG_BUF_SIZE; i++) Serial.printf("%02X ", recvData[i]);
    Serial.println();

    // Siehe setSyncWord()-Kommentar in bresserInit(): das erste Byte im FIFO
    // ist das zweite echte Sync-Byte (0xD4) und keine Nutzdaten.
    if (recvData[0] != 0xD4) {
        Serial.println("[Bresser] Byte[0] != 0xD4 – verworfen");
        return;
    }

    const uint8_t* msg = &recvData[1];  // 25 Bytes Nutzdaten

    static uint8_t  lastMsg[MSG_BYTES] = {};
    static uint32_t lastTime = 0;
    if (memcmp(msg, lastMsg, MSG_BYTES) == 0 && (nowMs - lastTime) < BRESSER_RX_DEBOUNCE_MS) return;

    char json[384];
    if (!bresserDecode(msg, rssi, json, sizeof(json))) {
        digestFail++;
        return;
    }
    digestOk++;

    memcpy(lastMsg, msg, MSG_BYTES);
    lastTime = nowMs;

    Serial.printf("[Bresser] → %s\n", json);
    if (s_client && s_client->connected()) {
        s_client->publish(BRESSER_MQTT_TOPIC, json, false);
    }
}
