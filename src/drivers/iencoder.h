#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// IEncoder — pure abstract interface for rotary encoder input devices.
//
// Implemented by:
//   UnitEncoder   — M5Stack single encoder unit (STM32F030, I2C 0x40)
//   Encoder8Unit  — M5Stack 8-encoder unit     (STM32F030, I2C 0x41)
//
// App holds an IEncoder* so it works with either hardware without #ifdefs
// in the application logic.
// ---------------------------------------------------------------------------

class IEncoder {
public:
  virtual ~IEncoder() = default;

  // Call every loop() iteration to update internal state.
  virtual void update() = 0;

  // Delta steps since last call. Positive = clockwise.
  virtual int readSteps() = 0;

  // True once after a short press (button released within long-press timeout).
  virtual bool wasShortPressed() = 0;

  // True once after a long press (button held past threshold).
  virtual bool wasLongPressed() = 0;

  // Instantaneous debounced button state.
  virtual bool isPressed() const = 0;

  // Clear all accumulated steps and pending press events.
  virtual void clearEvents() = 0;

  // Set LED on channel ch to the given 0xRRGGBB colour.
  // For single-encoder unit, ch is always 0 (or ignored).
  virtual void setLedColor(uint8_t ch, uint32_t rgb24) = 0;
};
