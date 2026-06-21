#pragma once
#include <Arduino.h>
#include <SCServo.h>
#include "iservo_bus.h"

// ---------------------------------------------------------------------------
// EchoSMS_STS — SMS_STS subclass that overrides wFlushSCS() to drain the
// TX echo produced by auto-direction (no DE/RE pin) TTL/RS485 adapters.
//
// The SCServo library calls wFlushSCS() immediately after transmitting each
// packet and before reading the servo response. The base implementation is
// empty. We override it to drain all bytes that arrived during transmission
// (the echo) so that checkHead() / readSCS() then see only the real response.
//
// Drain window: wait up to 3 byte-times at the current baud, reading until
// the UART goes quiet. This is always shorter than the servo's return delay
// (~500us for ST3215) so the real response is never consumed.
// ---------------------------------------------------------------------------
class EchoSMS_STS : public SMS_STS {
public:
  uint32_t _baud  = 1000000UL; // kept in sync with Serial1 baud by ST3215Bus
  int8_t   _dePin = -1;        // RS485 direction pin; -1 = auto-direction (echo drain)

protected:
  // Called by SCServo library before every TX packet.
  // Assert DE HIGH so the RS485 driver enables its transmitter.
  // With auto-direction adapter (_dePin=-1) this is a no-op.
  int writeSCS(unsigned char* nDat, int nLen) override {
    if (_dePin >= 0) digitalWrite(_dePin, HIGH);
    return SMS_STS::writeSCS(nDat, nLen);
  }
  int writeSCS(unsigned char bDat) override {
    if (_dePin >= 0) digitalWrite(_dePin, HIGH);
    return SMS_STS::writeSCS(bDat);
  }

  // Called by SCServo library immediately after every TX packet.
  // With a wired DE pin: flush(), release DE, then return (no echo to drain).
  // With auto-direction adapter: flush(), wait for echo to arrive, drain it.
  void wFlushSCS() override {
    if (!pSerial) return;
    pSerial->flush(); // wait for TX shift register to empty

    if (_dePin >= 0) {
      // Wired direction control: add a short guard then release DE.
      // No echo to drain — the RS485 driver blocked RX during TX.
      uint32_t guardUs = (2UL * 10UL * 1000000UL) / _baud;
      delayMicroseconds(guardUs);
      digitalWrite(_dePin, LOW);
    } else {
      // Auto-direction adapter: echo arrives on RX; drain it.
      // Bounded drain — do NOT use drain-until-empty which would
      // consume the servo response arriving ~50µs after the echo.
      uint32_t waitUs = (2UL * 10UL * 1000000UL) / _baud;
      delayMicroseconds(waitUs);
      while (pSerial->read() != -1) {}
    }
  }
};

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

  const char* protocolName() const override {
    return _scsMode ? "SC09 / SCS" : "ST3215 / STS";
  }
  int posMin() const override { return 0; }
  int posMax() const override { return _scsMode ? 1023 : 4095; }

  // Diagnostic helper — direct register byte read, bypasses any app-level logic.
  int rawReadByte(uint8_t id, uint8_t reg) { return _servo.readByte(id, reg); }
  int rawWriteByte(uint8_t id, uint8_t reg, uint8_t val) { return _servo.writeByte(id, reg, val); }

  // Switch between STS (ST3215, default) and SCS (SC09) sub-protocol.
  // Affects EEPROM lock register address and position range only.
  void setScsMode(bool scs); // implementation in servo_bus.cpp — also sets _servo.End
  bool getScsMode() const   { return _scsMode; }

private:
  EchoSMS_STS      _servo;
  HardwareSerial*  _serial  = nullptr;
  uint32_t         _baud    = 1000000UL;
  bool             _scsMode = false;  // false=STS (ST3215), true=SCS (SC09)
};

// ---------------------------------------------------------------------------
// Global instance — used by main.cpp and injected into App via IServoBus*.
// ---------------------------------------------------------------------------
extern ST3215Bus st3215Bus;