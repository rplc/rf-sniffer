#pragma once

#define MQTT_TOPIC_WZ_TISCH  "fan/wz-tisch"
#define MQTT_TOPIC_WZ_COUCH  "fan/wz-couch"
#define MQTT_TOPIC_SZ        "fan/sz"

// ─── Remote-IDs ──────────────────────────────────────────────────────────────
#define REMOTE_ID_WZ_TISCH  { 0xC9, 0x22 }   // Remote A – WZ Tisch
#define REMOTE_ID_WZ_COUCH  { 0x17, 0x1D }   // Remote B – WZ Couch
#define REMOTE_ID_SZ        { 0x9A, 0xA7 }   // Remote C – Schlafzimmer

// ─── CC1101 Pins (ESP32) ─────────────────────────────────────────────────────
#define CC1101_SCK_PIN   18
#define CC1101_MISO_PIN  19
#define CC1101_MOSI_PIN  23
#define CC1101_CS_PIN    21
#define CC1101_GDO0_PIN  22   // TX-Ausgang (Datenleitung) + RX-Eingang

// ─── RF-Timing (µs, verifiziert mit Pegel-Sniffer) ───────────────────────────
// Protokoll: PWM mit Bit-Wert im LOW-Puls
//   Header HIGH: ~8120µs
//   Header LOW:  ~4490µs
//   HIGH-Marker: ~565µs  (konstant, zwischen jedem Bit)
//   LOW Bit-0:   ~565µs
//   LOW Bit-1:  ~1680µs
//   Frame-Gap:  ~20000µs

#define RF_FREQ_MHZ      433.90   // Kalibriert via SDR-Sweep

#define RF_HEADER_HIGH   8120
#define RF_HEADER_LOW    4490
#define RF_PULSE          560    // konstanter HIGH-Marker zwischen Bits
#define RF_BIT_0          560    // LOW-Dauer für Bit 0
#define RF_BIT_1         1680    // LOW-Dauer für Bit 1
#define RF_FRAME_GAP    20000
#define RF_REPEAT_COUNT     3

#define RF_DIMMER_INTERVAL_MS  70

// ─── Kommando-Bytes B2 (verifiziert, Checksumme B3=~B2) ──────────────────────
#define CMD_ALL_OFF     0x40
#define CMD_FAN_OFF     0x08
#define CMD_SPEED_1     0x10
#define CMD_SPEED_2     0x90
#define CMD_SPEED_3     0x48
#define CMD_SPEED_4     0xC8
#define CMD_SPEED_5     0x88
#define CMD_SPEED_6     0x60
#define CMD_NAT_WIND    0x98
#define CMD_LIGHT       0xC0
#define CMD_TIMER_1H    0x28
#define CMD_TIMER_4H    0xA8
#define CMD_TIMER_8H    0xFF
#define CMD_COLOR_TEMP  0x7C
#define CMD_DIMMER      0x4D
#define CMD_ROTATION    0x74
