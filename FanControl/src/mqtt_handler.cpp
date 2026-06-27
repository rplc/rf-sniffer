#include "mqtt_handler.h"
#include "rf_sender.h"
#include "config.h"
#include <ArduinoJson.h>

// ─── Remote-ID Lookup ────────────────────────────────────────────────────────

struct FanConfig {
    const char* topic;
    uint8_t     id[2];
};

static const FanConfig FANS[] = {
    { MQTT_TOPIC_WZ_TISCH, REMOTE_ID_WZ_TISCH },
    { MQTT_TOPIC_WZ_COUCH, REMOTE_ID_WZ_COUCH },
    { MQTT_TOPIC_SZ,       REMOTE_ID_SZ        },
};
static constexpr uint8_t FAN_COUNT = sizeof(FANS) / sizeof(FANS[0]);

// ─── Dimmer-Hold-State ────────────────────────────────────────────────────────
// Wird im loop() abgearbeitet, damit MQTT nicht blockiert wird.

static bool     s_dimmerActive  = false;
static uint32_t s_dimmerNext    = 0;
static uint8_t  s_dimmerFanIdx  = 0;

void dimmerTick() {
    if (!s_dimmerActive) return;
    if (millis() >= s_dimmerNext) {
        rfSend(FANS[s_dimmerFanIdx].id, CMD_DIMMER, 1);
        s_dimmerNext = millis() + RF_DIMMER_INTERVAL_MS;
    }
}

// ─── Kommando-String → CMD-Byte ──────────────────────────────────────────────

// Erwartet JSON-Payload:
// {
//   "cmd": "speed",      // Pflichtfeld
//   "value": 3,          // optional, z.B. für speed (1-6), timer (1/4/8)
//   "dimmer_hold": true  // optional, nur für Dimmer
// }
//
// Mögliche cmd-Werte:
//   "all_off"      → alles aus (Licht + Ventilator)
//   "fan_off"      → nur Ventilator aus
//   "speed"        → value: 1-6
//   "natural_wind" → Naturwind-Modus
//   "light"        → Licht Toggle (an/aus)
//   "timer"        → value: 1, 4 oder 8 (Stunden)
//   "color_temp"   → Color-Temp-Cycle
//   "dimmer"       → dimmer_hold: true = starten, false = stoppen
//   "rotation"     → Sommer/Winter Toggle

static uint8_t resolveCmd(const JsonDocument& doc, bool& isDimmer, bool& dimmerStop) {
    isDimmer   = false;
    dimmerStop = false;

    const char* cmd = doc["cmd"] | "";

    if (strcmp(cmd, "all_off") == 0)      return CMD_ALL_OFF;
    if (strcmp(cmd, "fan_off") == 0)      return CMD_FAN_OFF;
    if (strcmp(cmd, "natural_wind") == 0) return CMD_NAT_WIND;
    if (strcmp(cmd, "light") == 0)        return CMD_LIGHT;
    if (strcmp(cmd, "color_temp") == 0)   return CMD_COLOR_TEMP;
    if (strcmp(cmd, "rotation") == 0)     return CMD_ROTATION;

    if (strcmp(cmd, "speed") == 0) {
        int spd = doc["value"] | 0;
        switch (spd) {
            case 1: return CMD_SPEED_1;
            case 2: return CMD_SPEED_2;
            case 3: return CMD_SPEED_3;
            case 4: return CMD_SPEED_4;
            case 5: return CMD_SPEED_5;
            case 6: return CMD_SPEED_6;
            default:
                Serial.printf("[MQTT] Ungültige Geschwindigkeit: %d\n", spd);
                return 0;
        }
    }

    if (strcmp(cmd, "timer") == 0) {
        int h = doc["value"] | 0;
        switch (h) {
            case 1: return CMD_TIMER_1H;
            case 4: return CMD_TIMER_4H;
            case 8: return CMD_TIMER_8H;
            default:
                Serial.printf("[MQTT] Ungültiger Timer-Wert: %d\n", h);
                return 0;
        }
    }

    if (strcmp(cmd, "dimmer") == 0) {
        isDimmer = true;
        bool hold = doc["dimmer_hold"] | false;
        if (!hold) dimmerStop = true;
        return CMD_DIMMER;
    }

    Serial.printf("[MQTT] Unbekanntes Kommando: %s\n", cmd);
    return 0;
}

// ─── MQTT Callback ───────────────────────────────────────────────────────────

static void onMessage(const char* topic, uint8_t* payload, unsigned int length) {
    // Fan-Konfiguration anhand Topic ermitteln
    const FanConfig* fan = nullptr;
    uint8_t fanIdx = 0;
    for (uint8_t i = 0; i < FAN_COUNT; i++) {
        if (strcmp(topic, FANS[i].topic) == 0) {
            fan    = &FANS[i];
            fanIdx = i;
            break;
        }
    }
    if (!fan) {
        Serial.printf("[MQTT] Unbekanntes Topic: %s\n", topic);
        return;
    }

    // JSON parsen
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) {
        Serial.printf("[MQTT] JSON-Fehler: %s\n", err.c_str());
        return;
    }

    Serial.printf("[MQTT] Topic=%s Payload=%.*s\n", topic, length, (char*)payload);

    bool isDimmer, dimmerStop;
    uint8_t cmd = resolveCmd(doc, isDimmer, dimmerStop);
    if (cmd == 0 && !isDimmer) return;

    if (isDimmer) {
        if (dimmerStop) {
            s_dimmerActive = false;
            Serial.println("[RF] Dimmer gestoppt");
        } else {
            s_dimmerActive = true;
            s_dimmerFanIdx = fanIdx;
            s_dimmerNext   = millis();
            Serial.println("[RF] Dimmer gestartet (Hold)");
        }
        return;
    }

    // Alle anderen Kommandos: Dimmer-Hold stoppen falls aktiv
    s_dimmerActive = false;
    rfSend(fan->id, cmd, RF_REPEAT_COUNT);
}

// ─── Verbindungsaufbau ───────────────────────────────────────────────────────

static PubSubClient* s_client = nullptr;

static void reconnect() {
    while (!s_client->connected()) {
        Serial.print("[MQTT] Verbinde...");
        String clientId = "fan-ctrl-" + String(ESP.getChipId(), HEX);
        bool ok = (strlen(MQTT_USER) > 0)
                ? s_client->connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)
                : s_client->connect(clientId.c_str());

        if (ok) {
            Serial.println(" verbunden");
            for (uint8_t i = 0; i < FAN_COUNT; i++) {
                s_client->subscribe(FANS[i].topic);
                Serial.printf("[MQTT] Abonniert: %s\n", FANS[i].topic);
            }
        } else {
            Serial.printf(" fehlgeschlagen (rc=%d), retry in 5s\n", s_client->state());
            delay(5000);
        }
    }
}

// ─── Öffentliche API ─────────────────────────────────────────────────────────

void mqttInit(PubSubClient& client) {
    s_client = &client;
    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setCallback(onMessage);
}

void mqttLoop(PubSubClient& client) {
    if (!client.connected()) reconnect();
    client.loop();
    dimmerTick();
}
