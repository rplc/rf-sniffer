#include "binary_clock_handler.h"
#include "config.h"
#include <ArduinoJson.h>
#include <FastLED.h>

CRGB leds[BINARY_CLOCK_LED_COUNT];

void binaryClockLoop() {
    // TODO alert (blink them red)
}

void binaryClockOnMqttMessage(const char* topic, const uint8_t* payload, unsigned int length) {
    // TODO display time
    //   alert flag
    //   display on/off
    // {"hours":"06","minutes":"52","on":true,"alert":false}
}

void binaryClockInit() {
    FastLED.addLeds<WS2812B, BINARY_CLOCK_PIN, GRB>(leds, BINARY_CLOCK_LED_COUNT);
}
