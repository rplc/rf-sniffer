#include "mqtt_handler.h"
#include "fan_handler.h"
#include "lamp_handler.h"
#include "binary_clock_handler.h"
#include "rf_handler.h"
#include "config.h"

// ─── Topics ───────────────────────────────────────────────────────────────────

static const char* FAN_TOPICS[] = {
    MQTT_TOPIC_WZ_TISCH,
    MQTT_TOPIC_WZ_COUCH,
    MQTT_TOPIC_SZ,
};
static const char* LAMP_TOPICS[] = {
    LAMP_TOPIC_CEILING,
    LAMP_TOPIC_READING,
};

// ─── Retained-Message-Filter ──────────────────────────────────────────────────

static uint32_t s_ignoreUntil = 0;

// ─── MQTT Callback ───────────────────────────────────────────────────────────

static void onMessage(const char* topic, uint8_t* payload, unsigned int length) {
    if (millis() < s_ignoreUntil) {
        Serial.printf("[MQTT] Ignoriert (retained): %s\n", topic);
        return;
    }

    // Fan-Topics
    for (const char* t : FAN_TOPICS) {
        if (strcmp(topic, t) == 0) {
            fanOnMqttMessage(topic, payload, length);
            return;
        }
    }
    // Lamp-Topics
    for (const char* t : LAMP_TOPICS) {
        if (strcmp(topic, t) == 0) {
            lampOnMqttMessage(topic, payload, length);
            return;
        }
    }

    if (strcmp(topic, BINARY_CLOCK_MQTT_TOPIC) == 0) {
        binaryClockOnMqttMessage(topic, payload, length);
        return;
    }
    Serial.printf("[MQTT] Unbekanntes Topic: %s\n", topic);
}

// ─── Verbindungsaufbau ────────────────────────────────────────────────────────

static PubSubClient* s_client = nullptr;

static void reconnect() {
    while (!s_client->connected()) {
        Serial.print("[MQTT] Verbinde...");
        String clientId = "fan-ctrl-" + String(ESP.getEfuseMac(), HEX);
        bool ok = (strlen(MQTT_USER) > 0)
                ? s_client->connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)
                : s_client->connect(clientId.c_str());

        if (ok) {
            Serial.println(" verbunden");
            // Retained-Filter vor dem Subscribe setzen -
            // retained Messages werden beim ersten client.loop() nach Subscribe
            // zugestellt, s_ignoreUntil muss also bereits aktiv sein
            s_ignoreUntil = millis() + 2000;
            Serial.println("[MQTT] Ignoriere retained Messages für 1500ms");
            for (const char* t : FAN_TOPICS) {
                s_client->subscribe(t);
                Serial.printf("[MQTT] Abonniert: %s\n", t);
            }
            for (const char* t : LAMP_TOPICS) {
                s_client->subscribe(t);
                Serial.printf("[MQTT] Abonniert: %s\n", t);
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
    client.setServer(MQTT_BROKER, 1883);
    client.setCallback(onMessage);
    if (!rfInit(fanOnRfFrame, lampOnRfFrame)) {
        Serial.println("[BOOT] CC1101 Fehler - Neustart in 10s");
        delay(10000);
        ESP.restart();
    }
    fanInit(client);
    lampInit(client);
}

void mqttLoop(PubSubClient& client) {
    if (!client.connected()) reconnect();
    client.loop();
}
