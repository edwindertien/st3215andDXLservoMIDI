#include "encoder_8unit.h"

// ---------------------------------------------------------------------------
// I2C helpers — portable TwoWire only, no ESP32-specific begin()
// ---------------------------------------------------------------------------

bool Encoder8Unit::writeReg(uint8_t reg, const uint8_t* buf, uint8_t len) {
  _wire.beginTransmission(_addr);
  _wire.write(reg);
  for (uint8_t i = 0; i < len; ++i) _wire.write(buf[i]);
  return _wire.endTransmission() == 0;
}

bool Encoder8Unit::readReg(uint8_t reg, uint8_t* buf, uint8_t len) {
  _wire.beginTransmission(_addr);
  _wire.write(reg);
  // Match the single encoder driver and official M5Stack library:
  // endTransmission() with no argument = default stop behaviour on this core.
  if (_wire.endTransmission() != 0) return false;
  if ((uint8_t)_wire.requestFrom((int)_addr, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; ++i)
    buf[i] = _wire.available() ? _wire.read() : 0;
  return true;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool Encoder8Unit::begin() {
  _wire.beginTransmission(_addr);
  if (_wire.endTransmission() != 0) return false;

  _lastCount    = readCount0();
  _lastButton   = false;
  _stableButton = false;
  _debounceMs   = millis();
  return true;
}

// ---------------------------------------------------------------------------
// Navigation interface (channel 0)
// ---------------------------------------------------------------------------

int32_t Encoder8Unit::readCount0() {
  uint8_t buf[4] = {0,0,0,0};
  if (!readReg(REG_ENCODER_BASE, buf, 4)) return _lastCount;
  return (int32_t)((uint32_t)buf[0] |
                   ((uint32_t)buf[1] << 8) |
                   ((uint32_t)buf[2] << 16) |
                   ((uint32_t)buf[3] << 24));
}

// Read the increment register (delta since last read, reset-on-read)
// This is more reliable than diffing absolute counts since it can't overflow
// between polls and we can't miss steps.
int32_t Encoder8Unit::readIncrement0() {
  uint8_t buf[4] = {0,0,0,0};
  uint8_t reg = REG_INCREMENT_BASE; // channel 0 increment
  _wire.beginTransmission(_addr);
  _wire.write(reg);
  if (_wire.endTransmission(true) != 0) return 0;
  if ((uint8_t)_wire.requestFrom((int)_addr, 4) != 4) return 0;
  for (uint8_t i = 0; i < 4; ++i) buf[i] = _wire.available() ? _wire.read() : 0;
  return (int32_t)((uint32_t)buf[0] |
                   ((uint32_t)buf[1] << 8) |
                   ((uint32_t)buf[2] << 16) |
                   ((uint32_t)buf[3] << 24));
}

void Encoder8Unit::update() {
  unsigned long now = millis();

  // ---- Encoder steps: use INCREMENT register (delta since last read) ----
  _stepAccum += (int)readIncrement0();

  // ---- Button: read current state (register 0x50, ch0) ----
  // The STM32 firmware reports the raw GPIO pin state.
  // The push switch is wired active-low (pressed = GND), so:
  //   raw == 0  →  pressed
  //   raw != 0  →  not pressed
  // This matches the single encoder unit which also uses inverted logic
  // (see encoder_unit.cpp: `return data == 0`).
  uint8_t raw = 0;
  readReg(REG_BUTTON_BASE, &raw, 1);
  bool rawBtn = (raw == 0);

  // Track raw edge for debounce timer reset
  if (rawBtn != _lastButton) {
    _lastButton  = rawBtn;
    _debounceMs  = now;
  }

  // Still within debounce window — do nothing
  if ((now - _debounceMs) < DEBOUNCE_MS) return;

  // Stable state changed
  if (_stableButton != _lastButton) {
    _stableButton = _lastButton;

    if (_stableButton) {
      // Stable press — start timing
      _pressStartMs = now;
      _longFired    = false;
    } else {
      // Stable release
      if (_waitRelease) {
        _waitRelease = false;
      } else if (_pressStartMs != 0 && !_longFired &&
                 (now - _lastEventMs) > EVENT_GAP_MS) {
        _shortEvent  = true;
        _lastEventMs = now;
      }
      _pressStartMs = 0;
      _longFired    = false;
    }
  }

  // Long press fires while held
  if (_stableButton &&
      !_longFired &&
      !_waitRelease &&
      _pressStartMs != 0 &&
      (now - _pressStartMs) >= LONGPRESS_MS &&
      (now - _lastEventMs) > EVENT_GAP_MS) {
    _longEvent    = true;
    _longFired    = true;
    _waitRelease  = true;
    _lastEventMs  = now;
  }
}

int Encoder8Unit::readSteps() {
  // COUNTS_PER_STEP: empirically 2 counts per detent on 8-encoder unit
  int steps = _stepAccum / COUNTS_PER_STEP;
  _stepAccum -= steps * COUNTS_PER_STEP;
  return steps;
}

bool Encoder8Unit::wasShortPressed() {
  bool e = _shortEvent;
  _shortEvent = false;
  return e;
}

bool Encoder8Unit::wasLongPressed() {
  bool e = _longEvent;
  _longEvent = false;
  return e;
}

void Encoder8Unit::clearEvents() {
  _shortEvent   = false;
  _longEvent    = false;
  _stepAccum    = 0;
  // Reset button state so release after screen change can't fire a spurious
  // short press — BUT preserve _waitRelease if the button is still physically
  // held (long press navigated away while button is down).
  _pressStartMs = 0;
  _longFired    = false;
  _debounceMs   = millis();
  if (!_stableButton) {
    // Button already released — safe to fully reset
    _stableButton = false;
    _lastButton   = false;
    _waitRelease  = false;
  } else {
    // Button still held — keep _stableButton/_lastButton true so the
    // debounce state is consistent, but arm _waitRelease so the
    // upcoming release is swallowed and produces no short press.
    _waitRelease  = true;
  }
}

// ---------------------------------------------------------------------------
// Raw multi-channel access
// ---------------------------------------------------------------------------

int32_t Encoder8Unit::readCount(uint8_t ch) {
  if (ch > 7) return 0;
  uint8_t buf[4] = {0,0,0,0};
  readReg(REG_ENCODER_BASE + ch * 4, buf, 4);
  return (int32_t)((uint32_t)buf[0] |
                   ((uint32_t)buf[1] << 8) |
                   ((uint32_t)buf[2] << 16) |
                   ((uint32_t)buf[3] << 24));
}

bool Encoder8Unit::isButtonPressed(uint8_t ch) {
  if (ch > 7) return false;
  uint8_t v = 0;
  readReg(REG_BUTTON_BASE + ch, &v, 1);
  return v != 0;
}

bool Encoder8Unit::switchState() {
  uint8_t v = 0;
  readReg(REG_SWITCH, &v, 1);
  return v != 0;
}

// ---------------------------------------------------------------------------
// LED control
// ---------------------------------------------------------------------------

void Encoder8Unit::setLed(uint8_t ch, uint8_t r, uint8_t g, uint8_t b) {
  if (ch > 8) return; // ch 8 = switch LED
  uint8_t buf[3] = {r, g, b};
  writeReg(REG_LED_BASE + ch * 3, buf, 3);
}

void Encoder8Unit::setAllLeds(uint8_t r, uint8_t g, uint8_t b) {
  for (uint8_t ch = 0; ch <= 8; ++ch) setLed(ch, r, g, b);
}

void Encoder8Unit::setLedColor(uint8_t ch, uint32_t rgb24) {
  setLed(ch,
         (uint8_t)((rgb24 >> 16) & 0xFF),
         (uint8_t)((rgb24 >>  8) & 0xFF),
         (uint8_t)( rgb24        & 0xFF));
}
