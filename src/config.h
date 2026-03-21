#pragma once
#include <Arduino.h>

namespace HW {
  static constexpr uint8_t SERVO_TX_PIN = 0;
  static constexpr uint8_t SERVO_RX_PIN = 1;
  static constexpr uint32_t SERVO_BAUD = 1000000UL;

  static constexpr uint8_t OLED_SDA_PIN = 4;
  static constexpr uint8_t OLED_SCL_PIN = 5;
  static constexpr uint8_t OLED_ADDR    = 0x3C;
  static constexpr uint16_t OLED_W      = 64;
  static constexpr uint16_t OLED_H      = 128;

  static constexpr uint8_t ENC_SDA_PIN  = 6;
  static constexpr uint8_t ENC_SCL_PIN  = 7;
  static constexpr uint8_t ENC_ADDR     = 0x40;
  static constexpr uint32_t ENC_I2C_HZ  = 200000UL;
}

namespace ST3215 {
  static constexpr int POS_MIN = 0;
  static constexpr int POS_MAX = 4095;
  static constexpr int POS_MID = 2047;

  static constexpr uint16_t DEFAULT_SPEED = 800;
  static constexpr uint8_t DEFAULT_ACC = 30;

  static constexpr int MAX_SCAN_IDS = 32;
}

namespace UI {
  static constexpr uint16_t FEEDBACK_INTERVAL_MS = 100;
  static constexpr uint16_t UI_REFRESH_MS = 50;
  static constexpr int POSITION_STEP = 8;
}