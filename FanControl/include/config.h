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
#define CC1101_GDO0_PIN  22

// ─── CC1101 Frequenz ─────────────────────────────────────────────────────────
#define RF_FREQ_MHZ      433.90

// ─── Fan RF-Timing (µs) ──────────────────────────────────────────────────────
// Protokoll: PWM mit Bit-Wert im LOW-Puls
// TX-Werte (direkt gesendet) = RX-Toleranzmitten
#define FAN_HEADER_HIGH   8120   // Header HIGH-Puls
#define FAN_HEADER_LOW    4490   // Header LOW-Puls (nach Header)
#define FAN_PULSE          560   // konstanter HIGH-Marker zwischen Bits
#define FAN_BIT_0          560   // LOW für Bit 0
#define FAN_BIT_1         1680   // LOW für Bit 1
#define FAN_FRAME_GAP    20000   // Frame-Trenner LOW
#define FAN_REPEAT_COUNT     3   // Frames pro Kommando

// RX-Toleranzfenster (±40% um TX-Werte)
#define FAN_RX_HEADER_MIN   5000   // Header HIGH muss > 5ms sein
#define FAN_RX_HEADER_MAX  12000   // Header HIGH muss < 12ms sein
#define FAN_RX_HDR_LOW_MAX 10000   // Header LOW ignorieren (3-10ms)
#define FAN_RX_BIT_MIN       200   // Bit LOW muss > 200µs sein
#define FAN_RX_BIT_MAX      3000   // Bit LOW muss < 3ms sein
#define FAN_RX_BIT_ONE       900   // LOW >= 900µs → Bit 1
#define FAN_RX_GAP_MIN     10000   // LOW > 10ms → Frame-Ende

#define FAN_RX_DEBOUNCE_MS   800   // Gleicher CMD innerhalb 800ms ignorieren
#define FAN_DIMMER_INTERVAL_MS 70  // Dimmer-Wiederholrate

// ─── Fan Kommando-Bytes B2 ───────────────────────────────────────────────────
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

// ─── Lamp MQTT-Topics ────────────────────────────────────────────────────────
#define LAMP_TOPIC_CEILING   "lamp/ceiling"
#define LAMP_TOPIC_READING   "lamp/reading"

// ─── Lamp RF-Codes (24-Bit) ──────────────────────────────────────────────────
#define LAMP_CEILING_ONOFF       4835521
#define LAMP_CEILING_CCT         4835529
#define LAMP_CEILING_BRIGHT_UP   4835524
#define LAMP_CEILING_BRIGHT_DOWN 4835528
#define LAMP_READING_ONOFF       4835535
#define LAMP_READING_CCT         4835530
#define LAMP_READING_BRIGHT_UP   4835525
#define LAMP_READING_BRIGHT_DOWN 4835522

// ─── Lamp RF-Timing (µs) ─────────────────────────────────────────────────────
// Protokoll 2 kompatibel: Bit-Wert im HIGH-Puls
// TX-Werte mit ets_delay_us Overhead-Kompensation:
#define LAMP_TX_SHORT       228   // 1x Basis-Puls TX (nominal 248µs)
#define LAMP_TX_LONG        724   // 3x Basis-Puls TX (nominal 744µs)
#define LAMP_TX_SYNC_LOW   7688   // 31x Basis-Puls (Sync / Frame-Trenner)

// RX-Toleranzfenster:
#define LAMP_RX_SYNC_MIN   5000   // LOW > 5ms → Sync erkannt
#define LAMP_RX_HIGH_MIN    100   // HIGH > 100µs → kein Rauschen
#define LAMP_RX_HIGH_MAX   1500   // HIGH < 1.5ms → gültiges Bit
#define LAMP_RX_BIT_ONE     450   // HIGH >= 450µs → Bit 1 (nominal 744µs)

#define LAMP_REPEAT_COUNT     3
#define LAMP_RX_DEBOUNCE_MS 800   // Gleicher Code innerhalb 800ms ignorieren

// ─── Zweiter CC1101: Bresser 7-in-1 Wetterstation (868 MHz) ──────────────────
// Eigener SPI-Bus (HSPI) um Konflikt mit dem 433MHz CC1101 (VSPI) zu vermeiden
#define BRESSER_SCK_PIN   14
#define BRESSER_MISO_PIN  12
#define BRESSER_MOSI_PIN  13
#define BRESSER_CS_PIN    15
#define BRESSER_GDO0_PIN  4    // Datenausgang (RX)

#define BRESSER_FREQ_MHZ  868.35
#define BRESSER_MQTT_TOPIC "weather/bresser7in1"
