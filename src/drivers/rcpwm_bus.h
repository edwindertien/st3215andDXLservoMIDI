#pragma once
#include <Arduino.h>
#include "iservo_bus.h"
#include "../config.h"

// ---------------------------------------------------------------------------
// RCPWMBus — IServoBus implementation for standard RC hobby servos.
//
// Outputs 50 Hz PWM on up to RCPWM_CH_COUNT GPIO pins.
// Pulse width: 1000 µs (pos=0) … 1500 µs (centre) … 2000 µs (pos=4095)
//
// No serial bus, no scan, no EEPROM.  All 6 channels are always "present".
// Servo IDs are 1-6 corresponding to RCPWM_PINS[0-5].
//
// PWM maths (RP2040 @ 125 MHz):
//   divider  = 64
//   wrap     = 39062   →  f = 125e6 / (64 × 39063) ≈ 50.00 Hz
//   1 µs     = 125e6 / 64 / 1e6 = 1.953125 counts
//   800 µs   = 1562 counts  (min)
//   1500 µs  = 2930 counts  (centre)
//   2200 µs  = 4297 counts  (max)
// ---------------------------------------------------------------------------

static constexpr int     RCPWM_CH_COUNT   = 6;
static constexpr uint16_t RCPWM_PWM_DIV   = 64;
static constexpr uint32_t RCPWM_PWM_WRAP  = 39062;
static constexpr uint16_t RCPWM_CNT_MIN   = 1562;  // 800 µs
static constexpr uint16_t RCPWM_CNT_CTR   = 2930;  // 1500 µs
static constexpr uint16_t RCPWM_CNT_MAX   = 4297;  // 2200 µs

class RCPWMBus : public IServoBus {
public:
  // IServoBus interface
  void begin(HardwareSerial& /*serial*/, uint32_t /*baud*/) override;
  void setBaud(uint32_t /*baud*/) override {}
  uint32_t currentBaud() const override { return 0; }

  // All 6 channels always present — scan fills ids[0..5] = {1,2,3,4,5,6}
  bool ping(uint8_t id) override { return id >= 1 && id <= RCPWM_CH_COUNT; }
  int  scan(uint8_t* ids, int maxIds, int& lastPingId) override;

  bool setPosition(uint8_t id, int pos, uint16_t speed, uint8_t acc) override;
  int  readPosition(uint8_t id) override;   // returns last commanded position

  bool torqueEnable(uint8_t id, bool en) override;

  // Status reads — RC servos have no feedback
  bool readVoltage(uint8_t /*id*/, int& mv) override          { mv = -1; return false; }
  bool readTemperature(uint8_t /*id*/, int& t) override       { t  = -1; return false; }
  bool readStatus(uint8_t /*id*/, int& s) override            { s  =  0; return true;  }
  bool readLoad(uint8_t /*id*/, int& l) override              { l  =  0; return false; }
  bool readCurrent(uint8_t /*id*/, int& c) override           { c  =  0; return false; }

  // Config — nothing to configure on RC servos
  bool loadConfig(uint8_t id, uint8_t& outId,
                  int& outMin, int& outMax,
                  int& outTorqueLimit, int& outCenterOffset,
                  int& outMode, int& outBaudIndex) override;

  bool saveId(uint8_t, uint8_t) override              { return false; }
  bool saveMinMax(uint8_t, int, int) override          { return false; }
  bool saveTorqueLimit(uint8_t, int) override          { return false; }
  bool saveCenterOffset(uint8_t, int) override         { return false; }
  bool saveMode(uint8_t, int) override                 { return false; }
  bool saveBaud(uint8_t, int) override                 { return false; }

  const char* protocolName() const override { return "RC PWM"; }
  int posMin() const override { return 800; }   // microseconds
  int posMax() const override { return 2200; }  // microseconds

private:
  int      _pos[RCPWM_CH_COUNT];    // last commanded position per channel
  bool     _enabled[RCPWM_CH_COUNT]; // torque enable state

  // Convert servo ID (1-6) to channel index (0-5)
  int chIdx(uint8_t id) const { return (id >= 1 && id <= RCPWM_CH_COUNT) ? id - 1 : -1; }

  // Set PWM level for a channel (0 = disable pulse)
  void setPulse(int ch, uint16_t counts);

  // Position → PWM counts. Position is now expressed directly in
  // microseconds (800-2200), matching posMin()/posMax().
  static uint16_t usToCounts(int us);
};

extern RCPWMBus rcpwmBus;