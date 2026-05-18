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
  uint32_t _baud = 1000000UL; // kept in sync with Serial1 baud by ST3215Bus

protected:
  void wFlushSCS() override {
    if (!pSerial) return;
    // Wait for UART TX to physically complete. write() is non-blocking;
    // flush() blocks until the shift register sends its last bit.
    // By the time flush() returns, ALL echo bytes have arrived in the
    // RX FIFO (echo propagation = ~1µs, well within flush() settling).
    pSerial->flush();
    // Wait 2 byte-times as a margin, then drain the echo.
    // Do NOT use drain-until-empty — that would consume the servo response
    // which arrives ~50µs after the echo ends.
    // 2 byte-times at _baud is always enough margin and always ends before
    // the servo starts responding.
    uint32_t waitUs = (2UL * 10UL * 1000000UL) / _baud;
    delayMicroseconds(waitUs);
    while (pSerial->read() != -1) {} // discard echo bytes
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

  const char* protocolName() const override { return "ST3215 / STS"; }
  int posMin() const override { return 0; }
  int posMax() const override { return 4095; }

private:
  EchoSMS_STS      _servo;  // echo-aware subclass of SMS_STS
  HardwareSerial*  _serial = nullptr;
  uint32_t         _baud   = 1000000UL;
};

// ---------------------------------------------------------------------------
// Global instance — used by main.cpp and injected into App via IServoBus*.
// ---------------------------------------------------------------------------
extern ST3215Bus st3215Bus;