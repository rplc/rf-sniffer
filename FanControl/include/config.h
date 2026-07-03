#pragma once

#define MQTT_TOPIC_WZ_TISCH  "fan/wz-tisch"
#define MQTT_TOPIC_WZ_COUCH  "fan/wz-couch"
#define MQTT_TOPIC_SZ        "fan/sz"

// ─── Remote-IDs ──────────────────────────────────────────────────────────────
#define REMOTE_ID_WZ_TISCH  { 0xC9, 0x22 }
#define REMOTE_ID_WZ_COUCH  { 0x17, 0x1D }
#define REMOTE_ID_SZ        { 0x9A, 0xA7 }

// ─── CC1101 + ESP32 Pins ─────────────────────────────────────────────────────
// SPI (Hardware VSPI)
#define CC1101_SCK_PIN   18
#define CC1101_MISO_PIN  19
#define CC1101_MOSI_PIN  23
#define CC1101_CS_PIN    21
#define CC1101_GDO0_PIN  22

// ─── RF-Timing (µs) ──────────────────────────────────────────────────────────
#define RF_FREQ_MHZ      433.92

#define RF_HEADER_HIGH   4500
#define RF_HEADER_LOW     560
#define RF_BIT_SEP        560
#define RF_BIT_ONE       1680
#define RF_BIT_ZERO       560
#define RF_FRAME_GAP    20000
#define RF_REPEAT_COUNT     3

#define RF_DIMMER_INTERVAL_MS  70

// ─── Kommando-Bytes B2 ───────────────────────────────────────────────────────
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
