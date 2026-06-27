#include "mqtt_handler.h"
#include "rf_sender.h"
#include "rf_receiver.h"
#include "config.h"
#include <ArduinoJson.h>

// ─── Fan-Konfiguration ───────────────────────────────────────────────────────

struct FanConfig {
    const char* topic;        // MQTT-Topic für Kommandos (eingehend)
    uint8_t     id[2];
};

static const FanConfig FANS[] = {
    { MQTT_TOPIC_WZ_TISCH, REMOTE_ID_WZ_TISCH },
    { MQTT_TOPIC_WZ_COUCH, REMOTE_ID_WZ_COUCH },
    { MQTT_TOPIC_SZ,       REMOTE_ID_SZ       },
};
static constexpr uint8_t FAN_COUNT = sizeof(FANS) / sizeof(FANS[0]);

// ─── CMD-Byte → MQTT-Payload Reverse-Lookup ──────────────────────────────────

static bool cmdToPayload(uint8_t cmd, char* buf, size_t bufLen) {
    // Gibt JSON-String zurück, den ioBroker direkt verarbeiten kann
    const char* cmdStr = nullptr;
    int         value  = -1;

    switch (cmd) {
        case CMD_ALL_OFF:     cmdStr = "all_off";      break;
        case CMD_FAN_OFF:     cmdStr = "fan_off";      break;
        case CMD_SPEED_1:     cmdStr = "speed"; value = 1; break;
        case CMD_SPEED_2:     cmdStr = "speed"; value = 2; break;
        case CMD_SPEED_3:     cmdStr = "speed"; value = 3; break;
        case CMD_SPEED_4:     cmdStr = "speed"; value = 4; break;
        case CMD_SPEED_5:     cmdStr = "speed"; value = 5; break;
        case CMD_SPEED_6:     cmdStr = "speed"; value = 6; break;
        case CMD_NAT_WIND:    cmdStr = "natural_wind"; break;
        case CMD_LIGHT:       cmdStr = "light";        break;
        case CMD_TIMER_1H:    cmdStr = "timer"; value = 1; break;
        case CMD_TIMER_4H:    cmdStr = "timer"; value = 4; break;
        case CMD_TIMER_8H:    cmdStr = "timer"; value = 8; break;
        case CMD_COLOR_TEMP:  cmdStr = "color_temp";  break;
        case CMD_DIMMER:      cmdStr = "dimmer"; value = 0; break;  // Hold-Start
        case CMD_ROTATION:    cmdStr = "rotation";    break;
        default:
            Serial.printf("[RX] Unbekannter CMD: 0x%02X\n", cmd);
            return false;
    }

    StaticJsonDocument<64> doc;
    doc["cmd"] = cmdStr;
    if (value >= 0) doc["value"] = value;
    serializeJson(doc, buf, bufLen);
    return true;
}

// ─── RF-Frame empfangen → MQTT publish ───────────────────────────────────────

static PubSubClient* s_client = nullptr;

static void onRfFrame(const uint8_t remoteId[2], uint8_t cmd) {
    // Remote-ID → Fan-Konfiguration
    const FanConfig* fan = nullptr;
    for (uint8_t i = 0; i < FAN_COUNT; i++) {
        if (FANS[i].id[0] == remoteId[0] && FANS[i].id[1] == remoteId[1]) {
            fan = &FANS[i];
            break;
        }
    }
    if (!fan) {
        Serial.printf("[RX] Unbekannte Remote-ID: %02X%02X CMD=%02X\n",
                      remoteId[0], remoteId[1], cmd);
        return;
    }

    char payload[64];
    if (!cmdToPayload(cmd, payload, sizeof(payload))) return;

    Serial.printf("[RX] Remote %02X%02X → %s : %s\n",
                  remoteId[0], remoteId[1], fan->topic, payload);

    if (s_client && s_client->connected()) {
        s_client->publish(fan->topic, payload, /*retain=*/false);
    }
}

// ─── Dimmer-Hold-State ────────────────────────────────────────────────────────

static bool     s_dimmerActive = false;
static uint32_t s_dimmerNext   = 0;
static uint8_t  s_dimmerFanIdx = 0;

static void dimmerTick() {
    if (!s_dimmerActive) return;
    if (millis() >= s_dimmerNext) {
        rfSend(FANS[s_dimmerFanIdx].id, CMD_DIMMER, 1);
        s_dimmerNext = millis() + RF_DIMMER_INTERVAL_MS;
    }
}

// ─── CMD-String → CMD-Byte ───────────────────────────────────────────────────

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
        switch (doc["value"] | 0) {
            case 1: return CMD_SPEED_1;
            case 2: return CMD_SPEED_2;
            case 3: return CMD_SPEED_3;
            case 4: return CMD_SPEED_4;
            case 5: return CMD_SPEED_5;
            case 6: return CMD_SPEED_6;
            default:
                Serial.printf("[MQTT] Ungültige Geschwindigkeit: %d\n", (int)(doc["value"] | 0));
                return 0;
        }
    }

    if (strcmp(cmd, "timer") == 0) {
        switch (doc["value"] | 0) {
            case 1: return CMD_TIMER_1H;
            case 4: return CMD_TIMER_4H;
            case 8: return CMD_TIMER_8H;
            default:
                Serial.printf("[MQTT] Ungültiger Timer-Wert: %d\n", (int)(doc["value"] | 0));
                return 0;
        }
    }

    if (strcmp(cmd, "dimmer") == 0) {
        isDimmer = true;
        if (!(doc["dimmer_hold"] | false)) dimmerStop = true;
        return CMD_DIMMER;
    }

    Serial.printf("[MQTT] Unbekanntes Kommando: %s\n", cmd);
    return 0;
}

// ─── MQTT Callback (eingehende Kommandos von ioBroker) ───────────────────────

static void onMqttMessage(const char* topic, uint8_t* payload, unsigned int length) {
    const FanConfig* fan    = nullptr;
    uint8_t          fanIdx = 0;
    for (uint8_t i = 0; i < FAN_COUNT; i++) {
        if (strcmp(topic, FANS[i].topic) == 0) {
            fan    = &FANS[i];
            fanIdx = i;
            break;
        }
    }
    if (!fan) return;

    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, payload, length)) {
        Serial.println("[MQTT] JSON-Fehler");
        return;
    }

    Serial.printf("[MQTT] %s → %.*s\n", topic, length, (char*)payload);

    bool isDimmer, dimmerStop;
    uint8_t cmd = resolveCmd(doc, isDimmer, dimmerStop);

    if (isDimmer) {
        if (dimmerStop) {
            s_dimmerActive = false;
            Serial.println("[RF] Dimmer gestoppt");
        } else {
            s_dimmerActive = true;
            s_dimmerFanIdx = fanIdx;
            s_dimmerNext   = millis();
            Serial.println("[RF] Dimmer gestartet");
        }
        return;
    }

    if (cmd == 0) return;
    s_dimmerActive = false;
    rfSend(fan->id, cmd, RF_REPEAT_COUNT);
}

// ─── Verbindungsaufbau ───────────────────────────────────────────────────────

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
    client.setCallback(onMqttMessage);
    rfReceiverInit(onRfFrame);   // RX-Modus starten und Callback registrieren
}

void mqttLoop(PubSubClient& client) {
    if (!client.connected()) reconnect();
    client.loop();
    rfReceiverLoop();
    dimmerTick();
}
