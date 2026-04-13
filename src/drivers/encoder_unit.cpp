#include "encoder_unit.h"

UnitEncoder::UnitEncoder(TwoWire& wire, uint8_t addr)
    : _wire(wire), _addr(addr) {}

bool UnitEncoder::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t freqHz) {
  // Wire bus pins/clock are already configured by main.cpp setup() before
  // begin() is called.  We intentionally do NOT call setSDA/setSCL/begin/
  // setClock here — doing so would overwrite the bus configuration and break
  // any other peripheral (e.g. the OLED) sharing the same Wire instance.
  // Arguments kept for API compatibility.
  (void)sdaPin; (void)sclPin; (void)freqHz;

  _wire.beginTransmission(_addr);
  if (_wire.endTransmission() != 0) return false;

  uint8_t mode = 0;
  writeBytes(MODE_REG, &mode, 1);

  _lastRawCount = readRawCount();
  _lastRawButton = readButtonRaw();
  _stableButton = _lastRawButton;
  _lastDebounceMs = millis();
  _pressStartMs = 0;
  _lastEventMs = 0;
  _longPressFired = false;
  _waitForReleaseAfterLong = false;

  return true;
}

bool UnitEncoder::writeBytes(uint8_t reg, const uint8_t* buffer, uint8_t length) {
  _wire.beginTransmission(_addr);
  _wire.write(reg);
  for (uint8_t i = 0; i < length; ++i) {
    _wire.write(buffer[i]);
  }
  return _wire.endTransmission() == 0;
}

bool UnitEncoder::readBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
  _wire.beginTransmission(_addr);
  _wire.write(reg);
  if (_wire.endTransmission() != 0) return false;

  if (_wire.requestFrom((int)_addr, (int)length) != length) return false;

  for (uint8_t i = 0; i < length; ++i) {
    buffer[i] = _wire.available() ? _wire.read() : 0;
  }
  return true;
}

int16_t UnitEncoder::readRawCount() {
  uint8_t data[2] = {0, 0};
  if (!readBytes(ENCODER_REG, data, 2)) return _lastRawCount;
  return (int16_t)(data[0] | (data[1] << 8));
}

bool UnitEncoder::readButtonRaw() {
  uint8_t data = 0;
  if (!readBytes(BUTTON_REG, &data, 1)) return _stableButton;

  // Your module uses inverted logic
  return data == 0;
}

void UnitEncoder::update() {
  _shortPressEvent = false;
  _longPressEvent = false;

  // Encoder movement
  int16_t raw = readRawCount();
  int16_t delta = raw - _lastRawCount;
  _lastRawCount = raw;
  _stepAccumulator += delta;

  // Button debounce
  bool rawButton = readButtonRaw();

  if (rawButton != _lastRawButton) {
    _lastRawButton = rawButton;
    _lastDebounceMs = millis();
  }

  if ((millis() - _lastDebounceMs) < DEBOUNCE_MS) {
    return;
  }

  if (_stableButton != _lastRawButton) {
    _stableButton = _lastRawButton;

    if (_stableButton) {
      // stable press started
      _pressStartMs = millis();
      _longPressFired = false;
    } else {
      // stable release
      if (_waitForReleaseAfterLong) {
        _waitForReleaseAfterLong = false;
      } else if (_pressStartMs != 0 && !_longPressFired &&
                 (millis() - _lastEventMs) > EVENT_GAP_MS) {
        _shortPressEvent = true;
        _lastEventMs = millis();
      }

      _pressStartMs = 0;
      _longPressFired = false;
    }
  }

  // Fire long press immediately once threshold is reached
  if (_stableButton &&
      !_longPressFired &&
      !_waitForReleaseAfterLong &&
      _pressStartMs != 0 &&
      (millis() - _pressStartMs) >= LONGPRESS_MS &&
      (millis() - _lastEventMs) > EVENT_GAP_MS) {
    _longPressEvent = true;
    _longPressFired = true;
    _waitForReleaseAfterLong = true;
    _lastEventMs = millis();
  }
}

int UnitEncoder::readSteps() {
  int steps = 0;

  while (_stepAccumulator >= COUNTS_PER_STEP) {
    _stepAccumulator -= COUNTS_PER_STEP;
    steps++;
  }

  while (_stepAccumulator <= -COUNTS_PER_STEP) {
    _stepAccumulator += COUNTS_PER_STEP;
    steps--;
  }

  return steps;
}

bool UnitEncoder::wasShortPressed() {
  bool e = _shortPressEvent;
  _shortPressEvent = false;
  return e;
}

bool UnitEncoder::wasLongPressed() {
  bool e = _longPressEvent;
  _longPressEvent = false;
  return e;
}

bool UnitEncoder::isPressed() const {
  return _stableButton;
}

void UnitEncoder::clearEvents() {
  _shortPressEvent = false;
  _longPressEvent  = false;
  _stepAccumulator = 0;
  _pressStartMs           = 0;
  _longPressFired         = false;
  _lastDebounceMs         = millis();
  if (!_stableButton) {
    _stableButton            = false;
    _lastRawButton           = false;
    _waitForReleaseAfterLong = false;
  } else {
    // Button still held after long press — swallow the upcoming release
    _waitForReleaseAfterLong = true;
  }
}

void UnitEncoder::setLedColor(uint8_t index, uint32_t color) {
  uint8_t data[4];
  data[0] = index;
  data[1] = (color >> 16) & 0xFF;
  data[2] = (color >> 8) & 0xFF;
  data[3] = color & 0xFF;
  writeBytes(RGB_LED_REG, data, 4);
}