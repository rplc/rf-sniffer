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
#define CC1101_GDO0_PIN  22   // TX-Ausgang + RX-Eingang (Fan + Lamp)

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
#define FAN_CMD_ALL_OFF     0x40
#define FAN_CMD_FAN_OFF     0x08
#define FAN_CMD_SPEED_1     0x10
#define FAN_CMD_SPEED_2     0x90
#define FAN_CMD_SPEED_3     0x48
#define FAN_CMD_SPEED_4     0xC8
#define FAN_CMD_SPEED_5     0x88
#define FAN_CMD_SPEED_6     0x60
#define FAN_CMD_NAT_WIND    0x98
#define FAN_CMD_LIGHT       0xC0
#define FAN_CMD_TIMER_1H    0x28
#define FAN_CMD_TIMER_4H    0xA8
#define FAN_CMD_TIMER_8H    0xFF
#define FAN_CMD_COLOR_TEMP  0x7C
#define FAN_CMD_DIMMER      0x4D
#define FAN_CMD_ROTATION    0x74

// ─── Stehlampe (24-Bit PWM, Protokoll 2 kompatibel) ─────────────────────────
#define LAMP_TOPIC_CEILING   "lamp/ceiling"   // Deckenstrahler
#define LAMP_TOPIC_READING   "lamp/reading"   // Leselampe

// Lamp-Codes (24-Bit, Pulse Length 248µs)
#define LAMP_CEILING_ONOFF    4835521
#define LAMP_CEILING_CCT      4835529
#define LAMP_CEILING_BRIGHT_UP   4835524
#define LAMP_CEILING_BRIGHT_DOWN 4835528

#define LAMP_READING_ONOFF    4835535
#define LAMP_READING_CCT      4835530
#define LAMP_READING_BRIGHT_UP   4835525
#define LAMP_READING_BRIGHT_DOWN 4835522

#define LAMP_PULSE_LENGTH     248
#define LAMP_REPEAT_COUNT       3
