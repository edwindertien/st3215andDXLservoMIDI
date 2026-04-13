#pragma once
#include <Arduino.h>
#include <SCServo.h>
#include "iservo_bus.h"

// ---------------------------------------------------------------------------
// ST3215Bus — concrete IServoBus implementation for Waveshare ST3215 / STS
// series serial bus servos (SCServo / SMS_STS protocol).
// ---------------------------------------------------------------------------
class ST3215Bus : public IServoBus {
public:
  void begin(HardwareSerial& serial, uint32_t baud = 1000000UL) override;
  void setBaud(uint32_t baud) override;
  uint32_t currentBaud() const override { return _baud; }

  bool ping(uint8_t id) override;
  int  scan(uint8_t* ids, int maxIds, int& lastPingId) override;

  bool setPosition(uint8_t id, int pos, uint16_t speed, uint8_t acc) override;
  int  readPosition(uint8_t id) override;
  bool torqueEnable(uint8_t id, bool en) override;

  bool readVoltage(uint8_t id, int& mv) override;
  bool readTemperature(uint8_t id, int& tempC) override;
  bool readStatus(uint8_t id, int& statusByte) override;
  bool readLoad(uint8_t id, int& loadPct) override;
  bool readCurrent(uint8_t id, int& currentMa) override;

  bool loadConfig(uint8_t id, uint8_t& outId, int& outMin, int& outMax,
                  int& outTorqueLimit, int& outCenterOffset,
                  int& outMode, int& outBaudIndex) override;
  bool saveId(uint8_t currentId, uint8_t newId) override;
  bool saveMinMax(uint8_t id, int minV, int maxV) override;
  bool saveTorqueLimit(uint8_t id, int limit) override;
  bool saveCenterOffset(uint8_t id, int offset) override;
  bool saveMode(uint8_t id, int mode) override;
  bool saveBaud(uint8_t id, int baudIndex) override;

  const char* protocolName() const override { return "ST3215 / STS"; }
  int posMin() const override { return 0; }
  int posMax() const override { return 4095; }

private:
  SMS_STS          _servo;
  HardwareSerial*  _serial = nullptr;
  uint32_t         _baud   = 1000000UL;
};

// ---------------------------------------------------------------------------
// Global instance — used by main.cpp and injected into App via IServoBus*.
// When more protocols are added, main.cpp will select which concrete object
// to pass to App based on the stored protocol selection.
// ---------------------------------------------------------------------------
extern ST3215Bus st3215Bus;