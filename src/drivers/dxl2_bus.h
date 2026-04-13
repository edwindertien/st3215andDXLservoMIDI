#pragma once
#include <Arduino.h>
#include "iservo_bus.h"

// ---------------------------------------------------------------------------
// Dxl2Bus — IServoBus implementation for Dynamixel Protocol 2.0
//
// Targets XM / XH / XD series (RS-485) and X330 series.
// Packet format:
//   TX: 0xFF 0xFF 0xFD 0x00  ID  LEN_L LEN_H  INS  [PARAMS...]  CRC_L CRC_H
//   RX: 0xFF 0xFF 0xFD 0x00  ID  LEN_L LEN_H  0x55 ERR [PARAMS...] CRC_L CRC_H
//   LEN = number of bytes from INS to last CRC byte inclusive
//   CRC: CRC-16 IBM (poly 0x8005) over all bytes from ID through last PARAM
//
// XM430-W350 / XM540-W150 control table addresses used here.
// ---------------------------------------------------------------------------

// DXL2 baud table — register value 0..7 → actual baud
// Control table register value is the INDEX into this table.
static constexpr int DXL2_BAUD_COUNT = 8;
static constexpr uint32_t DXL2_BAUD_TABLE[DXL2_BAUD_COUNT] = {
  9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000
};
static constexpr int DXL2_DEFAULT_BAUD_INDEX = 1; // 57600 (XH/XM series factory default)

class Dxl2Bus : public IServoBus {
public:
  explicit Dxl2Bus(int dePin = -1) : _dePin(dePin) {}

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

  const char* protocolName() const override { return "DXL X (P2.0)"; }
  int posMin() const override { return 0; }
  int posMax() const override { return 4095; }

  static constexpr int      baudCount()        { return DXL2_BAUD_COUNT; }
  static constexpr uint32_t baudAt(int i)      { return DXL2_BAUD_TABLE[i]; }
  static constexpr int      defaultBaudIndex() { return DXL2_DEFAULT_BAUD_INDEX; }

private:
  void    txBegin();
  void    txEnd();
  void    flushRx();

  // Send a P2.0 instruction packet; returns total bytes written to wire
  int     sendInstruction(uint8_t id, uint8_t ins,
                          const uint8_t* params, uint16_t paramLen);

  // Discard TX echo using baud-derived time window (same logic as Dxl1Bus)
  void    drainEcho(uint8_t byteCount);

  // Receive a P2.0 status packet; returns param byte count or -1 on error
  int     recvStatus(uint8_t id, uint8_t* buf, uint8_t bufLen, uint8_t& errByte);

  // Convenience read/write
  int     readByte(uint8_t id, uint16_t addr);
  int     readWord(uint8_t id, uint16_t addr);
  int32_t readDword(uint8_t id, uint16_t addr);
  bool    writeByte(uint8_t id, uint16_t addr, uint8_t val);
  bool    writeWord(uint8_t id, uint16_t addr, uint16_t val);
  bool    writeDword(uint8_t id, uint16_t addr, uint32_t val);

  // EEPROM write with settle delay
  bool    eepromWriteByte(uint8_t id, uint16_t addr, uint8_t val);
  bool    eepromWriteWord(uint8_t id, uint16_t addr, uint16_t val);
  bool    eepromWriteDword(uint8_t id, uint16_t addr, uint32_t val);

  // Read a status packet from any ID (for broadcast ping responses)
  int     recvStatusAny(uint8_t* buf, uint8_t bufLen, uint8_t& errByte);
  int     pingBroadcast(uint8_t* ids, int maxIds);

  static uint16_t crc16(const uint8_t* data, uint16_t len);

  HardwareSerial* _serial   = nullptr;
  int             _dePin    = -1;
  uint32_t        _baud     = 57600;
  uint8_t         _lastRxId  = 0;  // ID of last received status packet
  uint8_t         _lastTxLen = 0;  // byte count of last sent packet (for echo skip)

  static constexpr uint32_t RX_TIMEOUT_US       = 20000; // 20 ms per packet
  static constexpr uint32_t FLUSH_US            =  1000; // 1 ms flush
  static constexpr uint32_t BROADCAST_TIMEOUT_MS=   200; // 200 ms to collect broadcast responses

  // Protocol 2.0 instructions
  static constexpr uint8_t INS_PING   = 0x01;
  static constexpr uint8_t INS_READ   = 0x02;
  static constexpr uint8_t INS_WRITE  = 0x03;
  static constexpr uint8_t INS_STATUS = 0x55;

  // XM series control table — EEPROM area
  static constexpr uint16_t ADDR_MODEL_NUMBER    =   0; // 2 bytes
  static constexpr uint16_t ADDR_ID              =   7; // 1 byte
  static constexpr uint16_t ADDR_BAUD_RATE       =   8; // 1 byte (index 0-7)
  static constexpr uint16_t ADDR_DRIVE_MODE      =  10; // 1 byte
  static constexpr uint16_t ADDR_OP_MODE         =  11; // 1 byte (3=pos, 1=vel, 16=ext-pos)
  static constexpr uint16_t ADDR_HOMING_OFFSET   =  20; // 4 bytes signed
  static constexpr uint16_t ADDR_TEMP_LIMIT      =  31; // 1 byte
  static constexpr uint16_t ADDR_MAX_VOLT_LIMIT  =  32; // 2 bytes (unit: 0.1V)
  static constexpr uint16_t ADDR_MIN_VOLT_LIMIT  =  34; // 2 bytes
  static constexpr uint16_t ADDR_PWM_LIMIT       =  36; // 2 bytes (0-885)
  static constexpr uint16_t ADDR_CURRENT_LIMIT   =  38; // 2 bytes (unit: 2.69mA)
  static constexpr uint16_t ADDR_ACCEL_LIMIT     =  40; // 4 bytes
  static constexpr uint16_t ADDR_VEL_LIMIT       =  44; // 4 bytes
  static constexpr uint16_t ADDR_MAX_POS_LIMIT   =  48; // 4 bytes
  static constexpr uint16_t ADDR_MIN_POS_LIMIT   =  52; // 4 bytes

  // XM series control table — RAM area
  static constexpr uint16_t ADDR_TORQUE_ENABLE   =  64; // 1 byte
  static constexpr uint16_t ADDR_GOAL_PWM        = 100; // 2 bytes
  static constexpr uint16_t ADDR_GOAL_CURRENT    = 102; // 2 bytes
  static constexpr uint16_t ADDR_GOAL_VELOCITY   = 104; // 4 bytes
  static constexpr uint16_t ADDR_PROFILE_ACCEL   = 108; // 4 bytes
  static constexpr uint16_t ADDR_PROFILE_VEL     = 112; // 4 bytes
  static constexpr uint16_t ADDR_GOAL_POSITION   = 116; // 4 bytes
  static constexpr uint16_t ADDR_PRESENT_PWM     = 124; // 2 bytes
  static constexpr uint16_t ADDR_PRESENT_CURRENT = 126; // 2 bytes (unit: 2.69mA)
  static constexpr uint16_t ADDR_PRESENT_VELOCITY= 128; // 4 bytes
  static constexpr uint16_t ADDR_PRESENT_POSITION= 132; // 4 bytes
  static constexpr uint16_t ADDR_PRESENT_VOLTAGE = 144; // 2 bytes (unit: 0.1V)
  static constexpr uint16_t ADDR_PRESENT_TEMP    = 146; // 1 byte
  static constexpr uint16_t ADDR_HARDWARE_ERROR  = 70;  // 1 byte
public:
  // Debug: raw byte dump to serial terminal
  void rawPingDump(uint8_t id, Print& out);
  void rawBroadcastDump(Print& out);
};

extern Dxl2Bus dxl2Bus;