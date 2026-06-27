#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "rf_sender.h"
#include "mqtt_handler.h"

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// ─── WiFi ─────────────────────────────────────────────────────────────────────

static void wifiConnect() {
    Serial.printf("\n[WiFi] Verbinde mit %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print('.');
    }
    Serial.printf("\n[WiFi] Verbunden - IP: %s\n", WiFi.localIP().toString().c_str());
}

// ─── Setup / Loop ─────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== Fan RF Controller ===");

    wifiConnect();

    if (!rfSenderInit()) {
        Serial.println("[BOOT] CC1101-Fehler - Neustart in 10s");
        delay(10000);
        ESP.restart();
    }

    mqttInit(mqttClient);
}

void loop() {
    // WiFi-Reconnect
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Verbindung verloren - reconnect...");
        wifiConnect();
    }

    mqttLoop(mqttClient);
}
