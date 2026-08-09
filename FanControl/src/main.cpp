#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "rf_handler.h"
#include "mqtt_handler.h"
#include "OTA.h"
#include "fan_handler.h"
#include "lamp_handler.h"
#include "bresser_handler.h"
#include "binary_clock_handler.h"

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
OTASetup ota;

// ─── Watchdog ──────────────────────────────────────────────────────────────
// Letzte Absicherung/Power Cycle; Hängt loop() irgendwo fest (i.e.
// esp_task_wdt_reset wird nicht gecalled), greift der WDT und der ESP startet neu.
static const uint32_t WDT_TIMEOUT_S = 60;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;

static bool wifiConnect() {
    Serial.printf("\n[WiFi] Verbinde mit %s\n", WIFI_SSID);

    WiFi.disconnect(true, true);   // sauber trennen, alten State verwerfen
    delay(100);
    WiFi.mode(WIFI_STA);

    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
        delay(500);
        Serial.print('.');
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] Timeout - kein Connect");
        return false;
    }

    Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== RF Controller (ESP32) ===");

    esp_task_wdt_init(WDT_TIMEOUT_S, true);   // true = Panic -> Neustart statt nur Log
    esp_task_wdt_add(NULL);

    wifiConnect();

    mqttInit(mqttClient);
    // Auf dem 868.3MHz Band ist bei mir ein massiver Störer, weswegen mit der Kombination
    // aus Antenne & Modul nichts empfangen werden kann. Der SDR von rtl_433 ist da deutlich
    // empfindlicher und besser im Empfang als das selbst gebastelte Zeug hier -> bei rtl_433
    // bleiben. Alternativen: RF Chips vom Typ RFM95 oder SX1276 mit deutlich besserer/größerer
    // Antenne als die 868 Helix Antenne, inkl. Groundplane.
    // bresserInit(mqttClient);
    ota.init();
    binaryClockInit();
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Verbindung verloren - reconnect...");
        wifiConnect();
    }

    mqttLoop(mqttClient);   // connect (non-blocking, alle 5s ein Versuch) + client.loop()
    rfLoop();               // Fan+Lamp RX dekodieren → Callbacks
    fanLoop();              // Dimmer-Tick
    lampLoop();             // (Erweiterungspunkt)
    // bresserLoop();          // Bresser 868MHz RX
    ota.handle();
    binaryClockLoop();

    // Watchdog nur füttern, wenn wirklich alles läuft - hängt irgendwo etwas
    // fest (WiFi-Treiber im kaputten Zustand, MQTT-Stack, o.ä.), läuft der WDT
    // nach WDT_TIMEOUT_S ab und der ESP macht einen Power-Cycle.
    if (WiFi.status() == WL_CONNECTED && mqttConnected()) {
        esp_task_wdt_reset();
    }
}
