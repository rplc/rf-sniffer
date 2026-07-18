#include "binary_clock_handler.h"
#include "config.h"
#include <ArduinoJson.h>
#include <FastLED.h>

CRGB leds[BINARY_CLOCK_LED_COUNT];

static uint8_t clockHours = 0;
static uint8_t clockMinutes = 0;

static bool clockOn = false;
static bool alertActive = false;

static uint32_t lastAlertBlink = 0;
static bool alertBlinkState = false;

void displayNumber(uint8_t value, uint8_t ledOffset) {
  if (value & 1) leds[ledOffset]   = CRGB(0x190e03);
  if (value & 2) leds[ledOffset+1] = CRGB(0x2d1a05);
  if (value & 4) leds[ledOffset+2] = CRGB(0x422607);
  if (value & 8) leds[ledOffset+3] = CRGB(0x502d09);
}

void updateDisplay() {
    if (!clockOn) {
        FastLED.clear();
        FastLED.show();
        return;
    }

    FastLED.clear();

    if (alertBlinkState) {
        fill_solid(leds, BINARY_CLOCK_LED_COUNT, CRGB(0x420807));
    }

    uint8_t hour10 = clockHours / 10;
    uint8_t hour1  = clockHours % 10;

    uint8_t min10 = clockMinutes / 10;
    uint8_t min1  = clockMinutes % 10;

    displayNumber(hour10, 0);
    displayNumber(hour1, 2);
    displayNumber(min10, 6);
    displayNumber(min1, 9);

    FastLED.show();
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
    
    Serial.printf("[CLOCK] MQTT received (%02d:%02d, clockOn: %s, alert: %s)\n", clockHours, clockMinutes, clockOn ? "true" : "false", alertActive ? "true" : "false");

    updateDisplay();
}

void binaryClockInit() {
    FastLED.addLeds<WS2812B, BINARY_CLOCK_PIN, GRB>(leds, BINARY_CLOCK_LED_COUNT);

    FastLED.clear();
    FastLED.show();
}
