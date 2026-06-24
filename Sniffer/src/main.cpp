#include <ELECHOUSE_CC1101_SRC_DRV.h>

#define GDO0_PIN D2

volatile uint16_t pulses[256];
volatile uint8_t pulseCount = 0;
volatile uint32_t lastMicros = 0;
volatile bool frameReady = false;

void ICACHE_RAM_ATTR isr() {
    uint32_t now = micros();

    if (lastMicros == 0) {
        lastMicros = now;
        return;
    }

    uint32_t dur = now - lastMicros;
    lastMicros = now;

    // Müll raus
    if (dur < 150) return;
    if (dur > 100000) {
        pulseCount = 0;
        return;
    }

    // Frame-Gap erkannt
    if (dur > 10000 && pulseCount > 20) {
        frameReady = true;
        return;
    }

    if (pulseCount < 256) {
        pulses[pulseCount++] = dur;
    }
}

void setup() {
    Serial.begin(115200);

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(303.875);
    ELECHOUSE_cc1101.setModulation(2);
    ELECHOUSE_cc1101.setRxBW(270.0);
    ELECHOUSE_cc1101.setDRate(4.8);
    ELECHOUSE_cc1101.SetRx();

    pinMode(GDO0_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(GDO0_PIN), isr, CHANGE);

    Serial.println("Ready");
}

void loop() {
    if (frameReady) {
        noInterrupts();

        Serial.println("FRAME:");

        for (int i = 0; i < pulseCount; i++) {
            if (pulses[i] < 560) Serial.print("S ");
            else if (pulses[i] < 900) Serial.print("M ");
            else Serial.print("L ");
            Serial.print(" ");
        }

        Serial.println();
        Serial.println("---");

        pulseCount = 0;
        frameReady = false;

        interrupts();
    }
}

void sendRaw(uint16_t *buf, int len) {
    ELECHOUSE_cc1101.SetTx();

    for (int i = 0; i < len; i++) {
        digitalWrite(GDO0_PIN, HIGH);
        delayMicroseconds(buf[i]);

        i++;
        if (i >= len) break;

        digitalWrite(GDO0_PIN, LOW);
        delayMicroseconds(buf[i]);
    }

    ELECHOUSE_cc1101.SetRx();
}