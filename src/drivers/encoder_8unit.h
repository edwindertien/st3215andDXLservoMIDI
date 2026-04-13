#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "iencoder.h"

// ---------------------------------------------------------------------------
// Encoder8Unit — M5Stack 8-Encoder Unit (STM32F030, I2C default 0x41)
//
// Register map (from official M5Stack UNIT_8ENCODER.h):
//   0x00 + ch*4  : int32_t encoder count, channel 0..7 (LE)
//   0x20 + ch*4  : int32_t increment (delta since last read, reset-on-read)
//   0x40 + ch    : uint8_t reset counter (write 1 to reset channel count)
//   0x50 + ch    : uint8_t button state, 1=pressed, channel 0..7
//   0x60         : uint8_t switch state (the physical toggle switch)
//   0x70 + ch*3  : uint8_t[3] RGB LED color for channel 0..7  (R, G, B)
//   0xFE         : uint8_t firmware version
//   0xFF         : uint8_t I2C address (writable to change)
//
// NOTE: The official M5Stack library uses Wire.begin(sda, scl, hz) which is
// ESP32-specific and doesn't compile on RP2040/AVR.  This driver uses only
// the standard TwoWire API and is fully portable.
//
// Usage (same interface as UnitEncoder):
//   Wire.setSDA(HW::ENC_SDA_PIN); Wire.setSCL(HW::ENC_SCL_PIN); Wire.begin();
//   Encoder8Unit enc(Wire, HW::ENC_8_ADDR);
//   enc.begin();
//   // In loop:
//   enc.update();
//   int delta = enc.readSteps();       // delta from encoder 0 (navigation)
//   bool sp   = enc.wasShortPressed(); // button 0
//   bool lp   = enc.wasLongPressed();  // button 0 held
// ---------------------------------------------------------------------------

class Encoder8Unit : public IEncoder {
public:
  explicit Encoder8Unit(TwoWire& wire, uint8_t addr = 0x41)
    : _wire(wire), _addr(addr) {}

  // Called every loop() iteration to update internal state.
  bool begin();
  void update();

  int  readSteps();
  bool wasShortPressed();
  bool wasLongPressed();
  bool isPressed() const { return _stableButton; }
  void clearEvents();

  int32_t readCount(uint8_t ch);
  bool    isButtonPressed(uint8_t ch);
  bool    switchState();

  void setLed(uint8_t ch, uint8_t r, uint8_t g, uint8_t b);
  void setAllLeds(uint8_t r, uint8_t g, uint8_t b);
  void setLedColor(uint8_t ch, uint32_t rgb24);

private:
  bool    writeReg(uint8_t reg, const uint8_t* buf, uint8_t len);
  bool    readReg (uint8_t reg, uint8_t* buf, uint8_t len);
  int32_t readCount0();
  int32_t readIncrement0();   // delta since last read (reset-on-read by firmware)

  TwoWire& _wire;
  uint8_t  _addr;

  // Channel 0 navigation state
  int32_t       _lastCount    = 0;
  int           _stepAccum    = 0;

  bool          _lastButton   = false;
  bool          _stableButton = false;
  unsigned long _debounceMs   = 0;
  unsigned long _pressStartMs = 0;
  bool          _longFired    = false;
  bool          _waitRelease  = false;
  bool          _shortEvent   = false;
  bool          _longEvent    = false;
  unsigned long _lastEventMs  = 0;

  static constexpr uint8_t  REG_ENCODER_BASE   = 0x00; // +ch*4  int32 abs count
  static constexpr uint8_t  REG_INCREMENT_BASE = 0x20; // +ch*4  int32 delta (reset-on-read)
  static constexpr uint8_t  REG_RESET_COUNTER  = 0x40; // +ch    write 1 to reset encoder count
  static constexpr uint8_t  REG_BUTTON_BASE    = 0x50; // +ch    1=pressed  ← was 0x40, WRONG
  static constexpr uint8_t  REG_SWITCH         = 0x60; //        toggle switch state
  static constexpr uint8_t  REG_LED_BASE       = 0x70; // +ch*3  R,G,B
  static constexpr uint8_t  REG_FW_VER         = 0xFE;
  static constexpr uint8_t  REG_I2C_ADDR       = 0xFF;

  static constexpr unsigned long DEBOUNCE_MS  = 20;
  static constexpr unsigned long LONGPRESS_MS = 700;
  static constexpr unsigned long EVENT_GAP_MS = 120;
  static constexpr int           COUNTS_PER_STEP = 2;
};
