#include "rcpwm_bus.h"
#include <hardware/pwm.h>
#include <hardware/gpio.h>
#include "../config.h"

RCPWMBus rcpwmBus;

void RCPWMBus::begin(HardwareSerial& /*serial*/, uint32_t /*baud*/) {
  for (int ch = 0; ch < RCPWM_CH_COUNT; ++ch) {
    _pos[ch]     = 1500;   // centre, in microseconds
    _enabled[ch] = false;

    uint8_t pin = HW::RCPWM_PINS[ch];

    // Configure GPIO as PWM output
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Configure the PWM slice for this pin
    uint slice = pwm_gpio_to_slice_num(pin);
    uint chan  = pwm_gpio_to_channel(pin);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, RCPWM_PWM_DIV);
    pwm_config_set_wrap(&cfg, RCPWM_PWM_WRAP);
    pwm_init(slice, &cfg, true);

    // Start with pulse disabled (level = 0)
    pwm_set_chan_level(slice, chan, 0);
  }
}

int RCPWMBus::scan(uint8_t* ids, int maxIds, int& lastPingId) {
  // All 6 channels are always present — no bus communication needed.
  int count = 0;
  for (int ch = 0; ch < RCPWM_CH_COUNT && count < maxIds; ++ch) {
    ids[count++] = (uint8_t)(ch + 1); // IDs 1-6
  }
  lastPingId = RCPWM_CH_COUNT;
  return count;
}

uint16_t RCPWMBus::usToCounts(int us) {
  // Map microseconds (800-2200) directly to PWM counts (1562-4297)
  if (us < 800)  us = 800;
  if (us > 2200) us = 2200;
  // counts = 1.953125 * us  (125MHz / 64 / 1e6)
  return (uint16_t)(((uint32_t)us * 125) / 64);
}

void RCPWMBus::setPulse(int ch, uint16_t counts) {
  uint8_t pin   = HW::RCPWM_PINS[ch];
  uint    slice = pwm_gpio_to_slice_num(pin);
  uint    chan  = pwm_gpio_to_channel(pin);
  pwm_set_chan_level(slice, chan, counts);
}

bool RCPWMBus::setPosition(uint8_t id, int pos, uint16_t /*speed*/, uint8_t /*acc*/) {
  int ch = chIdx(id);
  if (ch < 0) return false;
  if (pos < 800)  pos = 800;
  if (pos > 2200) pos = 2200;
  _pos[ch] = pos;
  if (_enabled[ch]) setPulse(ch, usToCounts(pos));
  return true;
}

int RCPWMBus::readPosition(uint8_t id) {
  // RC servos have no position feedback — return last commanded position.
  int ch = chIdx(id);
  return (ch >= 0) ? _pos[ch] : -1;
}

bool RCPWMBus::torqueEnable(uint8_t id, bool en) {
  int ch = chIdx(id);
  if (ch < 0) return false;
  _enabled[ch] = en;
  if (en) {
    setPulse(ch, usToCounts(_pos[ch]));
  } else {
    setPulse(ch, 0); // zero level = no pulse = servo releases
  }
  return true;
}

bool RCPWMBus::loadConfig(uint8_t id, uint8_t& outId,
                           int& outMin, int& outMax,
                           int& outTorqueLimit, int& outCenterOffset,
                           int& outMode, int& outBaudIndex) {
  if (chIdx(id) < 0) return false;
  outId           = id;
  outMin          = 800;
  outMax          = 2200;
  outTorqueLimit  = 1000;
  outCenterOffset = 0;
  outMode         = 0;
  outBaudIndex    = 0;
  return true;
}