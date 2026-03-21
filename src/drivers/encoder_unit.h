#pragma once
#include <Arduino.h>
#include <Wire.h>

class UnitEncoder {
public:
  UnitEncoder(TwoWire& wire, uint8_t addr);

  bool begin(uint8_t sdaPin, uint8_t sclPin, uint32_t freqHz);

  void update();

  int readSteps();

  bool wasShortPressed();
  bool wasLongPressed();

  bool isPressed() const;
  void clearEvents();

  void setLedColor(uint8_t index, uint32_t color);

private:
  bool writeBytes(uint8_t reg, const uint8_t* buffer, uint8_t length);
  bool readBytes(uint8_t reg, uint8_t* buffer, uint8_t length);

  int16_t readRawCount();
  bool readButtonRaw();

  TwoWire& _wire;
  uint8_t _addr;

  int16_t _lastRawCount = 0;
  int _stepAccumulator = 0;

  bool _lastRawButton = false;
  bool _stableButton = false;
  unsigned long _lastDebounceMs = 0;

  unsigned long _pressStartMs = 0;
  bool _longPressFired = false;
  bool _waitForReleaseAfterLong = false;

  bool _shortPressEvent = false;
  bool _longPressEvent = false;

  static constexpr uint8_t MODE_REG    = 0x00;
  static constexpr uint8_t ENCODER_REG = 0x10;
  static constexpr uint8_t BUTTON_REG  = 0x20;
  static constexpr uint8_t RGB_LED_REG = 0x30;

  static constexpr unsigned long DEBOUNCE_MS  = 30;
  static constexpr unsigned long LONGPRESS_MS = 700;
  static constexpr unsigned long EVENT_GAP_MS = 120;

  static constexpr int COUNTS_PER_STEP = 2;

  unsigned long _lastEventMs = 0;
};