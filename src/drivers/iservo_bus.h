#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// IServoBus — pure abstract interface for all servo bus implementations.
//
// App holds a pointer to this interface. Concrete implementations
// (ST3215Bus, and future Dxl1Bus, Dxl2Bus, AK60Bus) are selected at
// runtime from the "Select Protocol" menu.
//
// Coordinate system used throughout:
//   pos      — integer position in raw encoder counts (0..posMax)
//   posMax   — implementation-specific maximum (e.g. 4095 for ST3215)
//   speed    — unsigned, 0..speedMax (implementation-specific)
//   acc      — unsigned, 0..accMax (implementation-specific)
// ---------------------------------------------------------------------------

class IServoBus {
public:
  virtual ~IServoBus() = default;

  // --- Lifecycle ---
  // Called once from App::begin() (or on protocol switch). Implementations
  // should configure the serial port and set any protocol-specific timeouts.
  virtual void begin(HardwareSerial& serial, uint32_t baud) = 0;

  // Change baud rate (used during Scan All to try different rates).
  virtual void setBaud(uint32_t baud) = 0;
  virtual uint32_t currentBaud() const = 0;

  // --- Discovery ---
  virtual bool ping(uint8_t id) = 0;

  // Scan IDs 0..maxId, fill ids[], return count found.
  // lastPingId updated during scan for progress display.
  virtual int  scan(uint8_t* ids, int maxIds, int& lastPingId) = 0;

  // --- Motion ---
  virtual bool setPosition(uint8_t id, int pos, uint16_t speed, uint8_t acc) = 0;
  virtual int  readPosition(uint8_t id) = 0;  // -1 on failure
  virtual bool torqueEnable(uint8_t id, bool en) = 0;

  // --- Telemetry ---
  // Returns false if the read failed; leaves the out-parameter unchanged.
  virtual bool readVoltage(uint8_t id, int& mv)     = 0;
  virtual bool readTemperature(uint8_t id, int& tempC) = 0;
  virtual bool readStatus(uint8_t id, int& statusByte) = 0;
  virtual bool readLoad(uint8_t id, int& loadPct)    = 0;
  virtual bool readCurrent(uint8_t id, int& currentMa) = 0;

  // --- EPROM Configuration ---
  virtual bool loadConfig(uint8_t id,
                          uint8_t& outId,
                          int& outMin, int& outMax,
                          int& outTorqueLimit,
                          int& outCenterOffset,
                          int& outMode,
                          int& outBaudIndex) = 0;

  virtual bool saveId(uint8_t currentId, uint8_t newId) = 0;
  virtual bool saveMinMax(uint8_t id, int minV, int maxV) = 0;
  virtual bool saveTorqueLimit(uint8_t id, int limit) = 0;
  virtual bool saveCenterOffset(uint8_t id, int offset) = 0;
  virtual bool saveMode(uint8_t id, int mode) = 0;
  virtual bool saveBaud(uint8_t id, int baudIndex) = 0;

  // --- Protocol metadata ---
  // Human-readable name shown in the Select Protocol menu.
  virtual const char* protocolName() const = 0;

  // Position range for this protocol (used by MIDI scaling and UI).
  virtual int posMin() const = 0;
  virtual int posMax() const = 0;
  virtual int posMid() const { return (posMin() + posMax()) / 2; }
};