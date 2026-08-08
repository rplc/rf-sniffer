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

// Non-blocking: nur alle RECONNECT_INTERVAL_MS einen Versuch starten und
// sofort zurückkehren, egal ob er klappt oder nicht. So blockiert ein
// hängender/fehlschlagender MQTT-Connect nie den restlichen loop() -
// WiFi.status() und alle anderen *Loop()-Aufrufe laufen weiter.
static const uint32_t RECONNECT_INTERVAL_MS = 5000;
static uint32_t s_lastAttempt = 0;

static void reconnect() {
    if (s_client->connected()) return;

    if (millis() - s_lastAttempt < RECONNECT_INTERVAL_MS) return;

    s_lastAttempt = millis();

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
        Serial.println("[MQTT] Ignoriere retained Messages für 2s");
        for (const char* t : FAN_TOPICS) {
            s_client->subscribe(t);
            Serial.printf("[MQTT] Abonniert: %s\n", t);
        }
        for (const char* t : LAMP_TOPICS) {
            s_client->subscribe(t);
            Serial.printf("[MQTT] Abonniert: %s\n", t);
        }

        s_client->subscribe(BINARY_CLOCK_MQTT_TOPIC);
        Serial.printf("[MQTT] Abonniert: %s\n", BINARY_CLOCK_MQTT_TOPIC);
    } else {
        Serial.printf(" fehlgeschlagen (rc=%d), naechster Versuch in %lus\n",
                       s_client->state(), RECONNECT_INTERVAL_MS / 1000);
    }
}

// ─── Öffentliche API ─────────────────────────────────────────────────────────

void mqttInit(PubSubClient& client) {
    s_client = &client;
    client.setServer(MQTT_BROKER, 1883);
    client.setCallback(onMessage);
    client.setSocketTimeout(3);   // Sekunden - verhindert langes Hängen bei TCP-Connect-Problemen
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

bool mqttConnected() {
    return s_client != nullptr && s_client->connected();
}
