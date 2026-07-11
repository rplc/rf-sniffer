#include "lamp_handler.h"
#include "rf_handler.h"
#include "config.h"
#include <ArduinoJson.h>
#include <rom/ets_sys.h>

static PubSubClient* s_client = nullptr;

// ─── Lamp TX ─────────────────────────────────────────────────────────────────

static void IRAM_ATTR lampSendFrame(uint32_t code) {
    // Sync
    digitalWrite(CC1101_GDO0_PIN, HIGH); ets_delay_us(LAMP_TX_SHORT);
    digitalWrite(CC1101_GDO0_PIN, LOW);  ets_delay_us(LAMP_TX_SYNC_LOW);

    // 24 Bits MSB-first
    for (int i = 23; i >= 0; i--) {
        bool bit = (code >> i) & 0x1;
        if (bit) {
            digitalWrite(CC1101_GDO0_PIN, HIGH); ets_delay_us(LAMP_TX_LONG);
            digitalWrite(CC1101_GDO0_PIN, LOW);  ets_delay_us(LAMP_TX_SHORT);
        } else {
            digitalWrite(CC1101_GDO0_PIN, HIGH); ets_delay_us(LAMP_TX_SHORT);
            digitalWrite(CC1101_GDO0_PIN, LOW);  ets_delay_us(LAMP_TX_LONG);
        }
    }

    // Abschluss-Sync
    digitalWrite(CC1101_GDO0_PIN, HIGH); ets_delay_us(LAMP_TX_SHORT);
    digitalWrite(CC1101_GDO0_PIN, LOW);  ets_delay_us(LAMP_TX_SYNC_LOW);
}

static void lampSend(uint32_t code) {
    rfEnterTx();
    for (uint8_t i = 0; i < LAMP_REPEAT_COUNT; i++) {
        noInterrupts();
        lampSendFrame(code);
        interrupts();
    }
    digitalWrite(CC1101_GDO0_PIN, LOW);
    Serial.printf("[LAMP] TX: code=%u\n", code);
    rfEnterRx();
}

// ─── Code-Tabelle ─────────────────────────────────────────────────────────────

struct LampCode {
    uint32_t    code;
    const char* topic;
    const char* cmd;
};

static const LampCode LAMP_CODES[] = {
    { LAMP_CEILING_ONOFF,       LAMP_TOPIC_CEILING, "onoff"       },
    { LAMP_CEILING_CCT,         LAMP_TOPIC_CEILING, "cct"         },
    { LAMP_CEILING_BRIGHT_UP,   LAMP_TOPIC_CEILING, "bright_up"   },
    { LAMP_CEILING_BRIGHT_DOWN, LAMP_TOPIC_CEILING, "bright_down" },
    { LAMP_READING_ONOFF,       LAMP_TOPIC_READING, "onoff"       },
    { LAMP_READING_CCT,         LAMP_TOPIC_READING, "cct"         },
    { LAMP_READING_BRIGHT_UP,   LAMP_TOPIC_READING, "bright_up"   },
    { LAMP_READING_BRIGHT_DOWN, LAMP_TOPIC_READING, "bright_down" },
};
static constexpr uint8_t LAMP_CODE_COUNT = sizeof(LAMP_CODES) / sizeof(LAMP_CODES[0]);

// ─── CMD → Code ───────────────────────────────────────────────────────────────

static uint32_t resolveCode(const char* topic, const char* cmd) {
    bool isCeiling = (strstr(topic, "ceiling") != nullptr);
    if (strcmp(cmd, "onoff")       == 0) return isCeiling ? LAMP_CEILING_ONOFF       : LAMP_READING_ONOFF;
    if (strcmp(cmd, "cct")         == 0) return isCeiling ? LAMP_CEILING_CCT         : LAMP_READING_CCT;
    if (strcmp(cmd, "bright_up")   == 0) return isCeiling ? LAMP_CEILING_BRIGHT_UP   : LAMP_READING_BRIGHT_UP;
    if (strcmp(cmd, "bright_down") == 0) return isCeiling ? LAMP_CEILING_BRIGHT_DOWN : LAMP_READING_BRIGHT_DOWN;
    return 0;
}

// ─── RX-Callback ──────────────────────────────────────────────────────────────

void lampOnRfFrame(uint32_t code) {
    for (uint8_t i = 0; i < LAMP_CODE_COUNT; i++) {
        if (LAMP_CODES[i].code != code) continue;

        char payload[80];
        StaticJsonDocument<80> doc;
        doc["cmd"] = LAMP_CODES[i].cmd;
        doc["ts"]  = millis();
        serializeJson(doc, payload, sizeof(payload));

        Serial.printf("[LAMP] RX → MQTT: %s : %s\n", LAMP_CODES[i].topic, payload);
        if (s_client && s_client->connected()) {
            s_client->publish(LAMP_CODES[i].topic, payload, false);
        }
        return;
    }
    Serial.printf("[LAMP] RX: unbekannter Code %u\n", code);
}

// ─── MQTT eingehend ────────────────────────────────────────────────────────────

void lampOnMqttMessage(const char* topic, const uint8_t* payload, unsigned int length) {
    char payloadStr[80] = {};
    memcpy(payloadStr, payload, min((unsigned)length, (unsigned)sizeof(payloadStr)-1));
    Serial.printf("[LAMP] MQTT: %s → %s\n", topic, payloadStr);

    StaticJsonDocument<80> doc;
    if (deserializeJson(doc, payloadStr)) {
        Serial.println("[LAMP] JSON-Fehler");
        return;
    }

    const char* cmd = doc["cmd"] | "";
    if (!cmd || strlen(cmd) == 0) return;

    uint32_t code = resolveCode(topic, cmd);
    if (code == 0) {
        Serial.printf("[LAMP] Unbekanntes CMD: %s\n", cmd);
        return;
    }
    lampSend(code);
}

void lampInit(PubSubClient& client) { s_client = &client; }
void lampLoop() {}
