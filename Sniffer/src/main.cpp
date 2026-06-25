#include <ELECHOUSE_CC1101_SRC_DRV.h>

#define GDO0_PIN D2

// === Doppelpuffer ===
volatile unsigned int captureBuffer[512];
volatile int captureCount = 0;
volatile bool captureDone = false;
volatile unsigned long lastChange = 0;

unsigned int printBuffer[512];
int printCount = 0;

void ICACHE_RAM_ATTR signalChange() {
  unsigned long now = micros();

  if (lastChange == 0) {
    lastChange = now;
    return;
  }

  unsigned int duration = (unsigned int)(now - lastChange);
  lastChange = now;

  // Sehr kurze Pulse ignorieren (Rauschen / Bouncing)
  if (duration < 100) return;

  if (!captureDone) {
    if (captureCount < 512) {
      captureBuffer[captureCount++] = duration;
    }
    // Paketende: großes Gap oder Puffer voll
    if (duration > 10000 || captureCount >= 511) {
      captureDone = true;
    }
  }
}

void setup() {
  Serial.begin(921600);

  if (!ELECHOUSE_cc1101.getCC1101()) {
    Serial.println("CC1101 nicht gefunden!");
    while (1);
  }

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setMHZ(303.875);
  ELECHOUSE_cc1101.setModulation(2); // OOK
  ELECHOUSE_cc1101.setRxBW(200.0);
  ELECHOUSE_cc1101.SetRx();

  pinMode(GDO0_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GDO0_PIN), signalChange, CHANGE);

  Serial.println("Sniffer bereit...");
}

void loop() {
  if (captureDone) {
    // Puffer so schnell wie möglich umkopieren, dann Interrupts wieder frei
    noInterrupts();
    printCount = captureCount;
    memcpy(printBuffer, (const void*)captureBuffer, captureCount * sizeof(unsigned int));
    captureCount = 0;
    captureDone = false;
    lastChange = 0;
    interrupts();

    // Jetzt außerhalb von noInterrupts() ausgeben
    Serial.println("=== SIGNAL ===");
    for (int i = 0; i < printCount; i++) {
      Serial.print(printBuffer[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.print("Pulse: ");
    Serial.println(printCount);
    Serial.println("=============");
  }

  yield(); // ESP8266 Watchdog bedienen
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