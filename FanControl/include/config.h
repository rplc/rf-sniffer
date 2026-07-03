#pragma once

// MQTT Topics – je Topic = ein Ventilator
#define MQTT_TOPIC_WZ_TISCH  "fan/wz-tisch"   // Remote A
#define MQTT_TOPIC_WZ_COUCH  "fan/wz-couch"   // Remote B
#define MQTT_TOPIC_SZ        "fan/sz"         // Remote C

// ─── Remote-IDs ───────────────────────
// Format: { ID_High, ID_Low }
#define REMOTE_ID_WZ_TISCH  { 0xC9, 0x22 }   // Remote A (WZ Tisch)
#define REMOTE_ID_WZ_COUCH  { 0x17, 0x1D }   // Remote B (WZ Couch)
#define REMOTE_ID_SZ        { 0x9A, 0xA7 }   // Remote C (Schlafzimmer)

// ─── CC1101 Pins (Wemos D1 Mini) ─────────────────────────────────────────────
// MOSI=D7, MISO=D6, SCK=D5 – Hardware-SPI, fest verdrahtet
#define CC1101_CS_PIN    D8   // CSN / SS
#define CC1101_GDO0_PIN  D2   // GDO0 – TX-Ausgang und RX-Eingang

// ─── RF-Timing (µs, gemittelt aus neuen Captures) ────────────────────────────
// Frequenz:   433.92 MHz
// Modulation: OOK/ASK
// Encoding:   PWM – kurzer HIGH=0, langer HIGH=1, fester LOW-Separator
//
// Gemessene Werte (n>250 Pulse je Kategorie):
//   Header HIGH:   avg=4489  → 4500
//   Header LOW:    avg=566   → 560
//   Bit-0 HIGH:    avg=559   → 560
//   Bit-1 HIGH:    avg=1679  → 1680
//   Separator LOW: avg=565   → 560  (gleich wie Header LOW)
//   Frame-Gap:     ~20190    → 20000 (konservativ kürzer für TX)
//
// Toleranzfenster für RX-Dekodierung:
//   Bit-0: 350–900 µs
//   Bit-1: 900–2200 µs
//   Header: 3500–5500 µs

#define RF_FREQ_MHZ      433.92

#define RF_HEADER_HIGH   4500
#define RF_HEADER_LOW     560
#define RF_BIT_SEP        560    // LOW-Puls zwischen Bits (= Header LOW)
#define RF_BIT_ONE       1680    // HIGH-Puls für Bit=1
#define RF_BIT_ZERO       560    // HIGH-Puls für Bit=0
#define RF_FRAME_GAP    20000    // Pause nach letztem Bit
#define RF_REPEAT_COUNT     3    // Frames pro Kommando

#define RF_DIMMER_INTERVAL_MS  70   // Wiederholrate beim Dimmen (Hold)

// ─── Kommando-Bytes B2 (verifiziert) ─────────────────────────────────────────
// B3 = ~B2 wird automatisch berechnet.
#define CMD_ALL_OFF     0x40
#define CMD_FAN_OFF     0x08
#define CMD_SPEED_1     0x10
#define CMD_SPEED_2     0x90
#define CMD_SPEED_3     0x48
#define CMD_SPEED_4     0xC8
#define CMD_SPEED_5     0x88
#define CMD_SPEED_6     0x60
#define CMD_NAT_WIND    0x98
#define CMD_LIGHT       0xC0   // Toggle
#define CMD_TIMER_1H    0x28
#define CMD_TIMER_4H    0xA8
#define CMD_TIMER_8H    0xFF
#define CMD_COLOR_TEMP  0x7C   // Cycle
#define CMD_DIMMER      0x4D   // Hold-Repeat
#define CMD_ROTATION    0x74   // Sommer/Winter Toggle
