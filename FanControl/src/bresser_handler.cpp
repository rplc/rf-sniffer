#include "bresser_handler.h"
#include "config.h"
#include <SPI.h>
#include <RadioLib.h>
#include <ArduinoJson.h>

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

// ─── ISR: Paket vollständig empfangen ────────────────────────────────────────

static volatile bool s_receivedFlag = false;

void IRAM_ATTR bresserPacketISR() {
    s_receivedFlag = true;
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
    cfg.frequency          = BRESSER_FREQ_MHZ; // 868.3 MHz
    cfg.bitRate             = 8.21f;             // kBaud
    cfg.frequencyDeviation  = 57.136417f;        // kHz
    cfg.power               = 10;                // dBm (RX-only, irrelevant)
    cfg.preambleLength      = 32;                // Bit
    cfg.receiverBandwidth   = 270.0f;             // kHz

    int16_t state = s_radio.begin(cfg);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[Bresser] CC1101 Init fehlgeschlagen (Code %d) - SPI/Verkabelung prüfen\n", state);
        return;
    }

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
    s_radio.setSyncWord(0xAA, 0x2D, 0, false);

    s_radio.setPacketReceivedAction(bresserPacketISR);

    state = s_radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[Bresser] startReceive() fehlgeschlagen (Code %d)\n", state);
        return;
    }

    s_ready = true;
    Serial.printf("[Bresser] CC1101 bereit @ %.3f MHz FSK (RadioLib)\n", BRESSER_FREQ_MHZ);
}

void bresserLoop() {
    if (!s_ready || !s_receivedFlag) return;
    s_receivedFlag = false;

    uint8_t recvData[MSG_BUF_SIZE];
    int16_t state = s_radio.readData(recvData, MSG_BUF_SIZE);
    s_radio.startReceive();  // sofort wieder in RX, unabhängig vom Ergebnis

    if (state != RADIOLIB_ERR_NONE) return;

    // Siehe setSyncWord()-Kommentar in bresserInit(): das erste Byte im FIFO
    // ist das zweite echte Sync-Byte (0xD4) und keine Nutzdaten.
    if (recvData[0] != 0xD4) return;

    const uint8_t* msg = &recvData[1];  // 25 Bytes Nutzdaten

    static uint8_t  lastMsg[MSG_BYTES] = {};
    static uint32_t lastTime = 0;
    uint32_t now = millis();
    if (memcmp(msg, lastMsg, MSG_BYTES) == 0 && (now - lastTime) < BRESSER_RX_DEBOUNCE_MS) return;

    char json[384];
    if (!bresserDecode(msg, json, sizeof(json))) return;

    memcpy(lastMsg, msg, MSG_BYTES);
    lastTime = now;

    Serial.printf("[Bresser] → %s\n", json);
    if (s_client && s_client->connected()) {
        s_client->publish(BRESSER_MQTT_TOPIC, json, false);
    }
}
