#pragma once
#include <Arduino.h>
#include "iservo_bus.h"

// ---------------------------------------------------------------------------
// Dxl1Bus — IServoBus implementation for Dynamixel Protocol 1.0
//
// Targets the MX-28R / MX-64R (RS-485) series at factory firmware.
// All register addresses and packet framing from the official Robotis
// MX-28T/R/AT/AR and MX-64T/R/AT/AR Protocol 1.0 control tables.
//
// Packet format:
//   TX (Instruction): 0xFF 0xFF ID LEN INS [PARAMS...] CHECKSUM
//   RX (Status):      0xFF 0xFF ID LEN ERR [PARAMS...] CHECKSUM
//   CHECKSUM = ~(ID + LEN + INS/ERR + SUM(PARAMS)) & 0xFF
//
// Half-duplex RS-485: a direction-enable (DE) pin may be needed.
// If dePin == -1, the adapter auto-switches (e.g. USB-RS485 dongles
// with automatic direction control).
//
// Default baud: 57600 (Dynamixel factory default for MX series).
// ---------------------------------------------------------------------------

// Actual servo baud rates per register formula: baud = 2000000 / (reg + 1)
// Note: these differ from "standard" values — "57600" is actually 57142 baud (reg=34).
//       "115200" is actually 117647 baud (reg=16). Using actual values prevents UART errors.
static constexpr int    DXL1_BAUD_COUNT = 8;
static constexpr uint32_t DXL1_BAUD_TABLE[DXL1_BAUD_COUNT] = {
  9615, 19230, 57142, 100000, 117647, 200000, 250000, 1000000
};
static constexpr int DXL1_DEFAULT_BAUD_INDEX = 2; // 57142 (~57600, reg=34)

class Dxl1Bus : public IServoBus {
public:
  // dePin: GPIO for RS-485 direction enable (-1 = auto / not needed)
  explicit Dxl1Bus(int dePin = -1) : _dePin(dePin) {}

  void begin(HardwareSerial& serial, uint32_t baud = 57600) override;
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

  const char* protocolName() const override { return "DXL MX (P1.0)"; }
  int posMin() const override { return 0; }
  int posMax() const override { return 4095; }

  // Expose baud table so app can iterate for Scan All
  static constexpr int          baudCount()              { return DXL1_BAUD_COUNT; }
  static constexpr uint32_t     baudAt(int i)            { return DXL1_BAUD_TABLE[i]; }
  static constexpr int          defaultBaudIndex()       { return DXL1_DEFAULT_BAUD_INDEX; }

private:
  // ---- Low-level packet I/O ----
  void     txBegin();
  void     txEnd();
  void     flushRx();

  int      sendInstruction(uint8_t id, uint8_t ins,
                           const uint8_t* params, uint8_t paramLen);

  // Drain exactly byteCount bytes from RX — used to swallow the TX echo that
  // auto-direction RS485 adapters produce at lower baud rates.
  void     drainEcho(uint8_t byteCount);

  // Read a status packet. Returns parameter byte count, or -1 on timeout/error.
  int      recvStatus(uint8_t id, uint8_t* buf, uint8_t bufLen, uint8_t& errByte);

  // Convenience wrappers
  int      readByte(uint8_t id, uint8_t addr);
  int      readWord(uint8_t id, uint8_t addr);
  bool     writeByte(uint8_t id, uint8_t addr, uint8_t val);
  bool     writeWord(uint8_t id, uint8_t addr, uint16_t val);

  // EEPROM write with mandatory 5ms settle delay
  bool     eepromWriteByte(uint8_t id, uint8_t addr, uint8_t val);
  bool     eepromWriteWord(uint8_t id, uint8_t addr, uint16_t val);

  static uint8_t checksum(uint8_t id, uint8_t len,
                          uint8_t ins, const uint8_t* p, uint8_t pLen);

  HardwareSerial* _serial  = nullptr;
  int             _dePin   = -1;
  uint32_t        _baud    = 57600;

  static constexpr uint32_t RX_TIMEOUT_US   = 20000; // 20 ms status packet wait
  static constexpr uint32_t ECHO_TIMEOUT_US =  5000; // 5 ms to receive our own echo
  static constexpr uint32_t FLUSH_US        =  1000; // 1 ms fixed flush window

  // ---- DXL Protocol 1.0 instructions ----
  static constexpr uint8_t INS_PING  = 0x01;
  static constexpr uint8_t INS_READ  = 0x02;
  static constexpr uint8_t INS_WRITE = 0x03;

  // ---- MX series Protocol 1.0 control table (EEPROM area) ----
  static constexpr uint8_t ADDR_MODEL_NUMBER_L = 0;
  static constexpr uint8_t ADDR_ID             = 3;
  static constexpr uint8_t ADDR_BAUD_RATE      = 4;
  static constexpr uint8_t ADDR_CW_ANGLE_LIMIT_L  = 6;
  static constexpr uint8_t ADDR_CCW_ANGLE_LIMIT_L = 8;
  static constexpr uint8_t ADDR_TORQUE_LIMIT_L    = 14; // Max Torque in EEPROM
  static constexpr uint8_t ADDR_DRIVE_MODE        = 10; // bit0: 0=joint,1=wheel
  static constexpr uint8_t ADDR_ALARM_SHUTDOWN    = 18;

  // ---- MX series Protocol 1.0 control table (RAM area) ----
  static constexpr uint8_t ADDR_TORQUE_ENABLE     = 24;
  static constexpr uint8_t ADDR_TORQUE_LIMIT_RAM  = 34; // runtime torque limit
  static constexpr uint8_t ADDR_GOAL_POSITION_L   = 30;
  static constexpr uint8_t ADDR_MOVING_SPEED_L    = 32;
  static constexpr uint8_t ADDR_PRESENT_POSITION_L = 36;
  static constexpr uint8_t ADDR_PRESENT_SPEED_L   = 38;
  static constexpr uint8_t ADDR_PRESENT_LOAD_L    = 40;
  static constexpr uint8_t ADDR_PRESENT_VOLTAGE   = 42;
  static constexpr uint8_t ADDR_PRESENT_TEMP      = 43;
  static constexpr uint8_t ADDR_PRESENT_CURRENT_L = 68; // MX-28/64 only
  static constexpr uint8_t ADDR_ALARM_LED         = 17;
  static constexpr uint8_t ADDR_MOVING            = 46;
};

extern Dxl1Bus dxl1Bus;