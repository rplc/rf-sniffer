#pragma once

// ─── WiFi ────────────────────────────────────────────────────────────────────
#define WIFI_SSID     "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"

// ─── MQTT ────────────────────────────────────────────────────────────────────
#define MQTT_BROKER   "192.168.1.100"
#define MQTT_PORT     1883
#define MQTT_USER     ""          // leer lassen wenn kein Auth
#define MQTT_PASSWORD ""

// MQTT Topics – je Topic = ein Ventilator
#define MQTT_TOPIC_WZ_TISCH  "fan/wz-tisch"   // Remote A (WZ Tisch)
#define MQTT_TOPIC_WZ_COUCH  "fan/wz-couch"   // Remote B (WZ Couch)
#define MQTT_TOPIC_SZ        "fan/sz"          // Remote C (Schlafzimmer)

// ─── Remote-IDs (gesnifft, 2 Bytes pro Remote) ───────────────────────────────
// Format: { ID_High, ID_Low }
#define REMOTE_ID_WZ_TISCH  { 0xC9, 0x22 }   // Remote A
#define REMOTE_ID_WZ_COUCH  { 0x17, 0x1D }   // Remote B
#define REMOTE_ID_SZ        { 0x9A, 0xA7 }   // Remote C

// ─── CC1101 SPI-Pins (Wemos D1 Mini) ─────────────────────────────────────────
// MOSI=D7, MISO=D6, SCK=D5 – Hardware-SPI, fest verdrahtet
#define CC1101_CS_PIN    D8   // CSN / SS
#define CC1101_GDO0_PIN  D2   // GDO0 (TX-fertig Interrupt, optional)

// ─── RF-Timing (µs, aus Captures gemessen) ───────────────────────────────────
#define RF_FREQ_MHZ      303.875
#define RF_HEADER_HIGH   4520
#define RF_HEADER_LOW     540
#define RF_BIT_SEP        540   // LOW-Puls zwischen Bits
#define RF_BIT_ONE       1710   // HIGH-Puls für Bit=1
#define RF_BIT_ZERO       580   // HIGH-Puls für Bit=0
#define RF_FRAME_GAP    20300   // Pause nach Frame (µs)
#define RF_REPEAT_COUNT     3   // Frames pro Kommando
#define RF_DIMMER_INTERVAL_MS 70  // Intervall beim Dimmen (Hold-Betrieb)

// ─── Kommando-Bytes (B2) ─────────────────────────────────────────────────────
// B3 = ~B2 wird automatisch berechnet
#define CMD_ALL_OFF     0x40
#define CMD_FAN_OFF     0x08
#define CMD_SPEED_1     0x10
#define CMD_SPEED_2     0x90
#define CMD_SPEED_3     0x48
#define CMD_SPEED_4     0xC8
#define CMD_SPEED_5     0x88
#define CMD_SPEED_6     0x60
#define CMD_NAT_WIND    0x98
#define CMD_LIGHT       0xC0   // Toggle (On wenn aus, Off wenn an)
#define CMD_TIMER_1H    0x28
#define CMD_TIMER_4H    0xA8
#define CMD_TIMER_8H    0xFF
#define CMD_COLOR_TEMP  0x7C   // Cycle: warm → weiß → kalt
#define CMD_DIMMER      0x4D   // Hold-Repeat
#define CMD_ROTATION    0x74   // Sommer/Winter Toggle
