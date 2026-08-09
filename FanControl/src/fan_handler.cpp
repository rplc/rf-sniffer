#include "fan_handler.h"
#include "rf_handler.h"
#include "config.h"
#include <ArduinoJson.h>
#include <rom/ets_sys.h>

static PubSubClient* s_client = nullptr;

// ─── Fan-Konfiguration ────────────────────────────────────────────────────────

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

struct QueueItem {
    uint8_t fanIdx;
    uint8_t cmd;
};
static constexpr uint8_t FAN_QUEUE_SIZE = 8;
static QueueItem s_queue[FAN_QUEUE_SIZE];
static uint8_t   s_qHead  = 0;
static uint8_t   s_qTail  = 0;
static uint8_t   s_qCount = 0;

static bool queuePush(uint8_t fanIdx, uint8_t cmd) {
    if (s_qCount >= FAN_QUEUE_SIZE) {
        Serial.println("[FAN] Queue voll - Kommando verworfen");
        return false;
    }
    s_queue[s_qTail] = { fanIdx, cmd };
    s_qTail = (s_qTail + 1) % FAN_QUEUE_SIZE;
    s_qCount++;
    return true;
}

static bool queuePop(QueueItem& out) {
    if (s_qCount == 0) return false;
    out = s_queue[s_qHead];
    s_qHead = (s_qHead + 1) % FAN_QUEUE_SIZE;
    s_qCount--;
    return true;
}

// ─── Fan TX ──────────────────────────────────────────────────────────────────

static void IRAM_ATTR sendFrame(const uint8_t id[2], uint8_t cmd) {
    const uint8_t payload[4] = { id[0], id[1], cmd, (uint8_t)(~cmd) };

    digitalWrite(CC1101_GDO0_PIN, HIGH); ets_delay_us(FAN_HEADER_HIGH);
    digitalWrite(CC1101_GDO0_PIN, LOW);  ets_delay_us(FAN_HEADER_LOW);

    for (int b = 0; b < 4; b++) {
        for (int bit = 7; bit >= 0; bit--) {
            bool v = (payload[b] >> bit) & 0x01;
            digitalWrite(CC1101_GDO0_PIN, HIGH); ets_delay_us(FAN_PULSE);
            digitalWrite(CC1101_GDO0_PIN, LOW);  ets_delay_us(v ? FAN_BIT_1 : FAN_BIT_0);
        }
    }
    digitalWrite(CC1101_GDO0_PIN, HIGH); ets_delay_us(FAN_PULSE);
    digitalWrite(CC1101_GDO0_PIN, LOW);  ets_delay_us(FAN_FRAME_GAP);
}

static void fanSend(const uint8_t id[2], uint8_t cmd) {
    rfEnterTx();
    for (uint8_t i = 0; i < FAN_REPEAT_COUNT; i++) {
        noInterrupts();
        sendFrame(id, cmd);
        interrupts();
    }
    digitalWrite(CC1101_GDO0_PIN, LOW);
    Serial.printf("[FAN] TX: ID=%02X%02X CMD=0x%02X\n", id[0], id[1], cmd);
    rfEnterRx();
}

// ─── CMD-Byte → MQTT-Payload ──────────────────────────────────────────────────

static bool cmdToPayload(uint8_t cmd, float rssi, char* buf, size_t bufLen) {
    StaticJsonDocument<128> doc;
    switch (cmd) {
        case FAN_CMD_ALL_OFF:    doc["cmd"] = "all_off";                break;
        case FAN_CMD_FAN_OFF:    doc["cmd"] = "fan_off";                break;
        case FAN_CMD_SPEED_1:    doc["cmd"] = "speed"; doc["value"] = 1; break;
        case FAN_CMD_SPEED_2:    doc["cmd"] = "speed"; doc["value"] = 2; break;
        case FAN_CMD_SPEED_3:    doc["cmd"] = "speed"; doc["value"] = 3; break;
        case FAN_CMD_SPEED_4:    doc["cmd"] = "speed"; doc["value"] = 4; break;
        case FAN_CMD_SPEED_5:    doc["cmd"] = "speed"; doc["value"] = 5; break;
        case FAN_CMD_SPEED_6:    doc["cmd"] = "speed"; doc["value"] = 6; break;
        case FAN_CMD_NAT_WIND:   doc["cmd"] = "natural_wind";           break;
        case FAN_CMD_LIGHT:      doc["cmd"] = "light";                  break;
        case FAN_CMD_TIMER_1H:   doc["cmd"] = "timer"; doc["value"] = 1; break;
        case FAN_CMD_TIMER_4H:   doc["cmd"] = "timer"; doc["value"] = 4; break;
        case FAN_CMD_TIMER_8H:   doc["cmd"] = "timer"; doc["value"] = 8; break;
        case FAN_CMD_COLOR_TEMP: doc["cmd"] = "color_temp";             break;
        case FAN_CMD_DIMMER:     doc["cmd"] = "dimmer";                 break;
        case FAN_CMD_ROTATION:   doc["cmd"] = "rotation";               break;
        default:
            Serial.printf("[FAN] Unbekannter CMD: 0x%02X\n", cmd);
            return false;
    }
    doc["ts"]   = millis();
    doc["rssi"] = serialized(String(rssi, 1));
    serializeJson(doc, buf, bufLen);
    return true;
}

// ─── RX-Callback ─────────────────────────────────────────────────────────────

void fanOnRfFrame(const uint8_t remoteId[2], uint8_t cmd) {
    const FanConfig* fan = nullptr;
    for (uint8_t i = 0; i < FAN_COUNT; i++) {
        if (FANS[i].id[0] == remoteId[0] && FANS[i].id[1] == remoteId[1]) {
            fan = &FANS[i];
            break;
        }
    }
    if (!fan) {
        Serial.printf("[FAN] Unbekannte Remote-ID: %02X%02X\n",
                      remoteId[0], remoteId[1]);
        return;
    }

    char payload[128];
    if (!cmdToPayload(cmd, rfGetRSSI(), payload, sizeof(payload))) return;

    Serial.printf("[FAN] RX → MQTT: %s : %s\n", fan->topic, payload);
    if (s_client && s_client->connected()) {
        s_client->publish(fan->topic, payload, false);
    }
}

// ─── Dimmer-Hold ─────────────────────────────────────────────────────────────

static bool     s_dimmerActive = false;
static uint32_t s_dimmerNext   = 0;
static uint8_t  s_dimmerFanIdx = 0;

void fanLoop() {
    if (s_dimmerActive) {
        if (millis() >= s_dimmerNext) {
            fanSend(FANS[s_dimmerFanIdx].id, FAN_CMD_DIMMER);
            s_dimmerNext = millis() + FAN_DIMMER_INTERVAL_MS;
        }
        return;
    }

    QueueItem item;
    if (queuePop(item)) {
        fanSend(FANS[item.fanIdx].id, item.cmd);
    }
}

// ─── CMD-String → CMD-Byte ───────────────────────────────────────────────────

static uint8_t resolveCmd(const JsonDocument& doc, bool& isDimmer, bool& dimmerStop) {
    isDimmer   = false;
    dimmerStop = false;
    const char* cmd = doc["cmd"] | "";

    if (strcmp(cmd, "all_off")      == 0) return FAN_CMD_ALL_OFF;
    if (strcmp(cmd, "fan_off")      == 0) return FAN_CMD_FAN_OFF;
    if (strcmp(cmd, "natural_wind") == 0) return FAN_CMD_NAT_WIND;
    if (strcmp(cmd, "light")        == 0) return FAN_CMD_LIGHT;
    if (strcmp(cmd, "color_temp")   == 0) return FAN_CMD_COLOR_TEMP;
    if (strcmp(cmd, "rotation")     == 0) return FAN_CMD_ROTATION;

    if (strcmp(cmd, "speed") == 0) {
        switch (doc["value"] | 0) {
            case 1: return FAN_CMD_SPEED_1;
            case 2: return FAN_CMD_SPEED_2;
            case 3: return FAN_CMD_SPEED_3;
            case 4: return FAN_CMD_SPEED_4;
            case 5: return FAN_CMD_SPEED_5;
            case 6: return FAN_CMD_SPEED_6;
            default: return 0;
        }
    }
    if (strcmp(cmd, "timer") == 0) {
        switch (doc["value"] | 0) {
            case 1: return FAN_CMD_TIMER_1H;
            case 4: return FAN_CMD_TIMER_4H;
            case 8: return FAN_CMD_TIMER_8H;
            default: return 0;
        }
    }
    if (strcmp(cmd, "dimmer") == 0) {
        isDimmer = true;
        if (!(doc["dimmer_hold"] | false)) dimmerStop = true;
        return FAN_CMD_DIMMER;
    }

    Serial.printf("[FAN] Unbekanntes CMD: %s\n", cmd);
    return 0;
}

// ─── MQTT eingehend ───────────────────────────────────────────────────────────

void fanOnMqttMessage(const char* topic, const uint8_t* payload, unsigned int length) {
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

    char payloadStr[256] = {};
    memcpy(payloadStr, payload, min((unsigned)length, (unsigned)sizeof(payloadStr)-1));
    Serial.printf("[FAN] MQTT: %s → %s\n", topic, payloadStr);

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, payloadStr)) {
        Serial.println("[FAN] JSON-Fehler");
        return;
    }

    bool isDimmer, dimmerStop;
    uint8_t cmd = resolveCmd(doc, isDimmer, dimmerStop);

    if (isDimmer) {
        s_dimmerActive = !dimmerStop;
        s_dimmerFanIdx = fanIdx;
        if (!dimmerStop) s_dimmerNext = millis();
        return;
    }

    if (cmd == 0) return;
    s_dimmerActive = false;
    queuePush(fanIdx, cmd);
}

void fanInit(PubSubClient& client) {
    s_client = &client;
}
