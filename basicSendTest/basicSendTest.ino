void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== PURE GPIO TX TEST ===");

    // GDO0-Pin direkt als Output — CC1101 komplett ignoriert
    pinMode(22, OUTPUT);
    digitalWrite(22, LOW);
    delay(1000);

    Serial.println("Sende 10x Speed1 Frame (purer GPIO, kein CC1101)...");
    for (int rep = 0; rep < 10; rep++) {
        // Header
        digitalWrite(22, HIGH); delayMicroseconds(4500);
        digitalWrite(22, LOW);  delayMicroseconds(560);

        // Payload: ID=0x17, 0x1D, CMD=0x10, ~CMD=0xEF
        uint8_t payload[] = { 0x17, 0x1D, 0x10, 0xEF };
        for (int b = 0; b < 4; b++) {
            for (int bit = 7; bit >= 0; bit--) {
                bool v = (payload[b] >> bit) & 0x01;
                digitalWrite(22, HIGH);
                delayMicroseconds(v ? 1680 : 560);
                digitalWrite(22, LOW);
                delayMicroseconds(560);
            }
        }
        // Gap
        delayMicroseconds(20000);
        delay(300);
    }
    Serial.println("Fertig.");
}

void loop() {}