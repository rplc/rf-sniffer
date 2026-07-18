#include "binary_clock_handler.h"
#include "config.h"
#include <ArduinoJson.h>
#include <NeoPixelBus.h>

// Do NOT use FastLED. FastLED nutzt RMT, RMT ist ISR getrieben und sehr Timing
// empfindlich und kommt durch WLAN, MQTT, CC1101 Interrupts durcheinander. Das
// resultiert in falsche LEDs an, falsche Farben, ...
// NeoPixelBus nutzt I2S (DMA Puffer) der komplette Frame Daten am Stück
// rausschickt, somit deutlich unempfindlicher.
NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1Ws2812xMethod> strip(BINARY_CLOCK_LED_COUNT, BINARY_CLOCK_PIN);

static uint8_t clockHours = 0;
static uint8_t clockMinutes = 0;

static bool clockOn = false;
static bool alertActive = false;

static uint32_t lastAlertBlink = 0;
static bool alertBlinkState = false;

static RgbColor hexColor(uint32_t hex) {
    return RgbColor((hex >> 16) & 0xFF, (hex >> 8) & 0xFF, hex & 0xFF);
}

void displayNumber(uint8_t value, uint8_t ledOffset) {
  if (value & 1) strip.SetPixelColor(ledOffset,   hexColor(0x190e03));
  if (value & 2) strip.SetPixelColor(ledOffset+1, hexColor(0x2d1a05));
  if (value & 4) strip.SetPixelColor(ledOffset+2, hexColor(0x422607));
  if (value & 8) strip.SetPixelColor(ledOffset+3, hexColor(0x502d09));
}

void updateDisplay() {
    if (!clockOn) {
        strip.ClearTo(RgbColor(0, 0, 0));
        strip.Show();
        return;
    }

    strip.ClearTo(RgbColor(0, 0, 0));

    if (alertBlinkState) {
        RgbColor alertColor = hexColor(0x420807);
        for (uint16_t i = 0; i < BINARY_CLOCK_LED_COUNT; i++) {
            strip.SetPixelColor(i, alertColor);
        }
    }

    uint8_t hour10 = clockHours / 10;
    uint8_t hour1  = clockHours % 10;

    uint8_t min10 = clockMinutes / 10;
    uint8_t min1  = clockMinutes % 10;

    displayNumber(hour10, 0);
    displayNumber(hour1, 2);
    displayNumber(min10, 6);
    displayNumber(min1, 9);

    Serial.printf("[Clock] updateDisplay: %02u:%02u on=%d alert=%d blink=%d\n",
                  clockHours, clockMinutes, clockOn, alertActive, alertBlinkState);

    strip.Show();
}

void binaryClockLoop() {
    if (!clockOn || !alertActive) {
        return;
    }

    uint32_t now = millis();

    if (now - lastAlertBlink >= 1000) {
        lastAlertBlink = now;

        alertBlinkState = !alertBlinkState;

        updateDisplay();
    }
}

void binaryClockOnMqttMessage(const char* topic, const uint8_t* payload, unsigned int length) {
    // MQTT payload: {"hours":"06","minutes":"52","on":true,"alert":false}
    StaticJsonDocument<256> doc;

    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
        return;
    }

    clockHours = atoi(doc["hours"] | "00");
    clockMinutes = atoi(doc["minutes"] | "00");

    bool currentlyAltering = alertActive;
    alertActive = doc["alert"] | false;
    clockOn = doc["on"] | false;

    if (alertActive && !currentlyAltering) {
        // now alerting -> reset everything
        lastAlertBlink = millis();
        alertBlinkState = false;
    } else if (!alertActive) {
        // alert is off/was turned off
        alertBlinkState = false;
    }

    updateDisplay();
}

void binaryClockInit() {
    strip.Begin();
    strip.ClearTo(RgbColor(0, 0, 0));
    strip.Show();
}
