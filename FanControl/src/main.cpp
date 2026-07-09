#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include "rf_handler.h"
#include "mqtt_handler.h"
#include "OTA.h"
#include "fan_handler.h"
#include "lamp_handler.h"
#include "bresser_handler.h"

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
OTASetup ota;

static void wifiConnect() {
    Serial.printf("\n[WiFi] Verbinde mit %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print('.');
    }
    Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== RF Controller (ESP32) ===");

    wifiConnect();

    mqttInit(mqttClient);
    bresserInit(mqttClient);
    ota.init();
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Verbindung verloren – reconnect...");
        wifiConnect();
    }

    mqttLoop(mqttClient);   // connect + client.loop()
    rfLoop();               // Fan+Lamp RX dekodieren → Callbacks
    fanLoop();              // Dimmer-Tick
    lampLoop();             // (Erweiterungspunkt)
    bresserLoop();          // Bresser 868MHz RX
    ota.handle();
}
