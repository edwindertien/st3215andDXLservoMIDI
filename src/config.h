#pragma once
#include <Arduino.h>

// =============================================================================
// HARDWARE CONFIGURATION
// Board, encoder and display are selected via platformio.ini build_flags:
//   -D BOARD_PICO_GROVE       or  -D BOARD_XIAO_EXPANSION
//   -D ENCODER_M5_SINGLE      or  -D ENCODER_M5_8UNIT
//   -D DISPLAY_SH1107         or  -D DISPLAY_SSD1306
//
// Do NOT define them here — use the [env:xxx] sections in platformio.ini.
// That way switching target is just a one-click environment change in VS Code.
// =============================================================================
#if !defined(BOARD_PICO_GROVE) && !defined(BOARD_XIAO_EXPANSION)
  #error "No board selected. Set BOARD_PICO_GROVE or BOARD_XIAO_EXPANSION in platformio.ini build_flags."
#endif

#if defined(BOARD_PICO_GROVE)
// ---- Pico + Grove Shield ----
// Servo bus: UART0 on GP0/GP1
// OLED:      I2C0  on GP4/GP5  (Wire)
// Encoder:   I2C1  on GP6/GP7  (Wire1)
// USB Host:  GP16/GP17
namespace HW {
  static constexpr uint8_t  SERVO_TX_PIN   = 0;
  static constexpr uint8_t  SERVO_RX_PIN   = 1;
  static constexpr uint32_t SERVO_BAUD     = 1000000UL;

  static constexpr uint8_t  OLED_SDA_PIN   = 4;
  static constexpr uint8_t  OLED_SCL_PIN   = 5;
  static constexpr uint8_t  OLED_ADDR      = 0x3C;
  static constexpr uint16_t OLED_W         = 64;
  static constexpr uint16_t OLED_H         = 128;
  static constexpr int      OLED_ROTATION  = 1;       // SH1107: 1=portrait

  static constexpr uint8_t  ENC_SDA_PIN    = 6;
  static constexpr uint8_t  ENC_SCL_PIN    = 7;
  static constexpr uint8_t  ENC_ADDR       = 0x40;    // single encoder
  static constexpr uint8_t  ENC_8_ADDR     = 0x41;    // 8-encoder unit
  static constexpr uint32_t ENC_I2C_HZ     = 200000UL;

  static constexpr uint8_t  USB_HOST_DP_PIN = 16;
  static constexpr uint8_t  USB_HOST_DM_PIN = 17;

  // Buzzer / RGB LED — not present on Pico Grove config
  static constexpr int8_t   BUZZER_PIN      = -1;     // -1 = not present
  static constexpr int8_t   LED_R_PIN       = -1;
  static constexpr int8_t   LED_G_PIN       = -1;
  static constexpr int8_t   LED_B_PIN       = -1;
  static constexpr bool     LED_ACTIVE_LOW  = false;
}

#elif defined(BOARD_XIAO_EXPANSION)
// ---- XIAO RP2040 + Seeed Expansion Board ----
//
// XIAO RP2040 GPIO map (Earle Philhower Arduino core):
//   D0=GP26, D1=GP27, D2=GP28, D3=GP29
//   D4=GP6 (I2C SDA), D5=GP7 (I2C SCL)
//   D6=GP0 (UART0 TX), D7=GP1 (UART0 RX)
//   D8=GP2 (SPI SCK),  D9=GP4 (SPI MISO), D10=GP3 (SPI MOSI)
//
// Expansion board UART port labelled RX2/TX2 → UART0 (GP0/GP1)
// Expansion board I2C → GP6/GP7 (same bus for OLED + Encoder, Wire)
// Built-in OLED SSD1306 128x64 on I2C (0x3C)
// Buzzer: expansion board passive buzzer on GP? — confirmed pin 7 in Seeed examples
// RGB LED on XIAO RP2040: GP17=RED, GP16=GREEN, GP25=BLUE, active LOW
// USB host: GP26/GP27 (D0/D1) — GP16/GP17 occupied by RGB LED
namespace HW {
  static constexpr uint8_t  SERVO_TX_PIN   = 0;       // GP0  = D6
  static constexpr uint8_t  SERVO_RX_PIN   = 1;       // GP1  = D7
  static constexpr uint32_t SERVO_BAUD     = 1000000UL;

  // OLED and Encoder share the single exposed I2C bus (Wire = GP6/GP7)
  static constexpr uint8_t  OLED_SDA_PIN   = 6;       // GP6  = D4
  static constexpr uint8_t  OLED_SCL_PIN   = 7;       // GP7  = D5
  static constexpr uint8_t  OLED_ADDR      = 0x3C;
  static constexpr uint16_t OLED_W         = 128;
  static constexpr uint16_t OLED_H         = 64;
  static constexpr int      OLED_ROTATION  = 0;       // SSD1306: 0=normal

  // Encoder is on the same I2C bus as the OLED
  static constexpr uint8_t  ENC_SDA_PIN    = 6;       // same as OLED
  static constexpr uint8_t  ENC_SCL_PIN    = 7;
  static constexpr uint8_t  ENC_ADDR       = 0x40;    // single encoder
  static constexpr uint8_t  ENC_8_ADDR     = 0x41;    // 8-encoder unit
  static constexpr uint32_t ENC_I2C_HZ     = 400000UL;

  // USB host: use D0/D1 = GP26/GP27 (GP16/GP17 are RGB LED)
  static constexpr uint8_t  USB_HOST_DP_PIN = 26;
  static constexpr uint8_t  USB_HOST_DM_PIN = 27;

  // Expansion board passive buzzer pin
  // Seeed expansion board schematic: buzzer on D6 of the expansion headers
  // which maps to GP3 on XIAO RP2040
  static constexpr int8_t   BUZZER_PIN      = 3;      // GP3 = D10

  // XIAO RP2040 built-in RGB LED, active LOW
  static constexpr int8_t   LED_R_PIN       = 17;     // GP17
  static constexpr int8_t   LED_G_PIN       = 16;     // GP16
  static constexpr int8_t   LED_B_PIN       = 25;     // GP25
  static constexpr bool     LED_ACTIVE_LOW  = true;
}

#else
  #error "No board configuration selected — uncomment one BOARD_xxx define in config.h"
#endif

// =============================================================================
// STATUS LED HELPERS
// Inline helpers so the rest of the code doesn't need to know active polarity.
// =============================================================================
namespace StatusLed {
  inline void begin() {
    if (HW::LED_R_PIN >= 0) { pinMode(HW::LED_R_PIN, OUTPUT); }
    if (HW::LED_G_PIN >= 0) { pinMode(HW::LED_G_PIN, OUTPUT); }
    if (HW::LED_B_PIN >= 0) { pinMode(HW::LED_B_PIN, OUTPUT); }
    // Start with all off
    if (HW::LED_R_PIN >= 0) digitalWrite(HW::LED_R_PIN, HW::LED_ACTIVE_LOW ? HIGH : LOW);
    if (HW::LED_G_PIN >= 0) digitalWrite(HW::LED_G_PIN, HW::LED_ACTIVE_LOW ? HIGH : LOW);
    if (HW::LED_B_PIN >= 0) digitalWrite(HW::LED_B_PIN, HW::LED_ACTIVE_LOW ? HIGH : LOW);
  }

  // Set individual channels: true = ON
  inline void set(bool r, bool g, bool b) {
    auto lvl = [](bool on) {
      return (bool)(HW::LED_ACTIVE_LOW ? !on : on) ? HIGH : LOW;
    };
    if (HW::LED_R_PIN >= 0) digitalWrite(HW::LED_R_PIN, lvl(r));
    if (HW::LED_G_PIN >= 0) digitalWrite(HW::LED_G_PIN, lvl(g));
    if (HW::LED_B_PIN >= 0) digitalWrite(HW::LED_B_PIN, lvl(b));
  }

  inline void off()   { set(false, false, false); }
  inline void green() { set(false, true,  false); }
  inline void amber() { set(true,  true,  false); }
  inline void red()   { set(true,  false, false); }
  inline void blue()  { set(false, false, true ); }
  inline void white() { set(true,  true,  true ); }
}

// =============================================================================
// BUZZER HELPERS
// =============================================================================
namespace Buzzer {
  inline void begin() {
    if (HW::BUZZER_PIN >= 0) {
      pinMode(HW::BUZZER_PIN, OUTPUT);
      digitalWrite(HW::BUZZER_PIN, LOW);
    }
  }

  // Blocking beep: freq in Hz, duration in ms
  inline void beep(uint16_t freqHz = 2000, uint16_t durationMs = 80) {
    if (HW::BUZZER_PIN < 0) return;
    uint32_t period = 1000000UL / freqHz;  // microseconds
    uint32_t cycles = (uint32_t)durationMs * 1000UL / period;
    for (uint32_t i = 0; i < cycles; ++i) {
      digitalWrite(HW::BUZZER_PIN, HIGH);
      delayMicroseconds(period / 2);
      digitalWrite(HW::BUZZER_PIN, LOW);
      delayMicroseconds(period / 2);
    }
  }

  // Two-tone scan-complete chime
  inline void scanDone() {
    beep(1200, 60);
    delay(40);
    beep(2400, 80);
  }

  // Short error blip
  inline void error() {
    beep(400, 200);
  }
}

// =============================================================================
// ST3215 / servo bus constants (unchanged)
// =============================================================================
namespace ST3215 {
  static constexpr int POS_MIN = 0;
  static constexpr int POS_MAX = 4095;
  static constexpr int POS_MID = 2047;

  static constexpr uint16_t DEFAULT_SPEED = 800;
  static constexpr uint8_t  DEFAULT_ACC   = 30;

  static constexpr int MAX_SCAN_IDS = 32;
}

namespace UI {
  static constexpr uint16_t FEEDBACK_INTERVAL_MS = 100;
  static constexpr uint16_t UI_REFRESH_MS         = 50;
  static constexpr int      POSITION_STEP         = 8;
}
