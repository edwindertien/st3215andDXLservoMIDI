#include "servo_bus.h"
#include "../config.h"

// Global ST3215Bus instance — selected by default, injected into App
ST3215Bus st3215Bus;

void ST3215Bus::begin(HardwareSerial& serial, uint32_t baud) {
  _serial = &serial;
  _baud   = baud;
  _servo.pSerial   = &serial;
  _servo.IOTimeOut = 5;
  _servo._baud     = baud;
  _servo._dePin    = (int8_t)HW::SERVO_DE_PIN;
  _servo.End       = _scsMode ? 1 : 0; // 0=little-endian (STS), 1=big-endian (SCS/SC09)
  if (HW::SERVO_DE_PIN >= 0) {
    pinMode(HW::SERVO_DE_PIN, OUTPUT);
    digitalWrite(HW::SERVO_DE_PIN, LOW); // default to RX mode
  }
  _serial->end();
  _serial->begin(baud);
  delay(5);
}

void ST3215Bus::setBaud(uint32_t baud) {
  if (!_serial) return;
  _baud        = baud;
  _servo._baud = baud;
  _serial->end();
  _serial->begin(baud);
  delay(5);
}

void ST3215Bus::setScsMode(bool scs) {
  _scsMode   = scs;
  _servo.End = scs ? 1 : 0; // switch word byte order immediately
}

bool ST3215Bus::ping(uint8_t id) {
  return _servo.Ping(id) == id;
}

int ST3215Bus::scan(uint8_t* ids, int maxIds, int& lastPingId) {
  int count = 0;
  for (int id = 0; id <= 253; ++id) {
    lastPingId = id;
    if (ping((uint8_t)id) && count < maxIds)
      ids[count++] = (uint8_t)id;
    delay(2);
  }
  return count;
}

bool ST3215Bus::setPosition(uint8_t id, int pos, uint16_t speed, uint8_t acc) {
  if (_scsMode) {
    // SC09: position is 10-bit (0-1023). Clamp and use WritePosEx.
    // WritePosEx on SCSCL ignores ACC (forces 0) which is fine for SC09.
    int p = pos > 1023 ? 1023 : (pos < 0 ? 0 : pos);
    return _servo.WritePosEx((uint8_t)id, (int16_t)p, speed, acc) >= 0;
  }
  return _servo.WritePosEx(id, pos, speed, acc) >= 0;
}

int ST3215Bus::readPosition(uint8_t id) {
  // readWord uses _servo.End for byte order: 0=STS (little-endian), 1=SCS (big-endian)
  // SMS_STS::ReadPos also applies a sign-bit mask which is wrong for SC09,
  // so we use readWord directly for both protocols.
  int v = _servo.readWord(id, 56); // SMS_STS_PRESENT_POSITION_L = SCSCL_PRESENT_POSITION_L = 56
  int maxPos = _scsMode ? 1023 : 4095;
  return (v >= 0 && v <= maxPos) ? v : -1;
}

bool ST3215Bus::torqueEnable(uint8_t id, bool en) {
  return _servo.EnableTorque(id, en ? 1 : 0) >= 0;
}

bool ST3215Bus::readVoltage(uint8_t id, int& mv) {
  int v = _servo.ReadVoltage(id);
  if (v < 0) return false;
  mv = v;
  return true;
}

bool ST3215Bus::readTemperature(uint8_t id, int& tempC) {
  int v = _servo.ReadTemper(id);
  if (v < 0) return false;
  tempC = v;
  return true;
}

bool ST3215Bus::readStatus(uint8_t id, int& statusByte) {
  int v = _servo.readByte(id, 65);
  if (v < 0) return false;
  statusByte = v;
  return true;
}

bool ST3215Bus::readLoad(uint8_t id, int& loadPct) {
  int v = _servo.readWord(id, SMS_STS_PRESENT_LOAD_L);
  if (v < 0) return false;
  int magnitude = v & 0x3FF;
  int sign      = (v >> 10) & 1;
  loadPct = (sign ? -1 : 1) * (magnitude * 100 / 1000);
  return true;
}

bool ST3215Bus::readCurrent(uint8_t id, int& currentMa) {
  int v = _servo.readWord(id, SMS_STS_PRESENT_CURRENT_L);
  if (v < 0) return false;
  currentMa = v * 65 / 10;
  return true;
}

bool ST3215Bus::loadConfig(uint8_t id, uint8_t& outId, int& outMin, int& outMax,
                            int& outTorqueLimit, int& outCenterOffset,
                            int& outMode, int& outBaudIndex) {
  if (!ping(id)) return false;
  outId = id;

  if (_scsMode) {
    // SC09 / SCS protocol — use direct readWord to avoid SMS_STS post-processing.
    // Registers 9/11 are min/max same as STS, but readWord returns raw little-endian.
    int minV = _servo.readWord(id, 9);   // SCSCL_MIN_ANGLE_LIMIT_L
    int maxV = _servo.readWord(id, 11);  // SCSCL_MAX_ANGLE_LIMIT_L
    int baud = _servo.readByte(id, 6);   // SCSCL_BAUD_RATE
    if (minV < 0 || maxV < 0) return false;
    outMin         = minV & 0x3FF; // 10-bit clamp
    outMax         = maxV & 0x3FF;
    outTorqueLimit = 1000;         // SC09 has no torque limit register
    outCenterOffset = 0;           // SC09 has no center offset register
    outMode        = 0;            // SC09 mode via angle limits, not a register
    outBaudIndex   = (baud >= 0 && baud < 8) ? baud : 0;
    return true;
  }

  int minV  = _servo.readWord(id, SMS_STS_MIN_ANGLE_LIMIT_L);
  int maxV  = _servo.readWord(id, SMS_STS_MAX_ANGLE_LIMIT_L);
  int torq  = _servo.readWord(id, SMS_STS_TORQUE_LIMIT_L);
  int ofs   = _servo.readWord(id, SMS_STS_OFS_L);
  int mode  = _servo.readByte(id, SMS_STS_MODE);
  int baud  = _servo.readByte(id, SMS_STS_BAUD_RATE);
  if (minV < 0 || maxV < 0) return false;
  outMin          = minV;
  outMax          = maxV;
  outTorqueLimit  = (torq >= 0) ? torq : 1000;
  outMode         = (mode >= 0) ? (mode & 0x03) : 0;
  outBaudIndex    = (baud >= 0 && baud < 8) ? baud : 0;
  if (ofs >= 0) {
    if (ofs > 2047) ofs -= 4096;
    outCenterOffset = ofs;
  } else {
    outCenterOffset = 0;
  }
  return true;
}

// Lock register address differs between STS (ST3215) and SCS (SC09):
//   SMS_STS_LOCK = 55  (0x37) — STS protocol
//   SCSCL_LOCK   = 48  (0x30) — SCS protocol
// Using wrong address leaves EEPROM locked → writes go to SRAM only → lost on power cycle.
static constexpr uint8_t STS_LOCK_REG = 55;
static constexpr uint8_t SCS_LOCK_REG = 48;

// Inline helpers so every save* function gets the right register
#define LOCK_REG  (_scsMode ? SCS_LOCK_REG : STS_LOCK_REG)
#define UNLOCK(id) _servo.writeByte((id), LOCK_REG, 0)
#define RELOCK(id) _servo.writeByte((id), LOCK_REG, 1)

bool ST3215Bus::saveId(uint8_t currentId, uint8_t newId) {
  UNLOCK(currentId);
  int ok = _servo.writeByte(currentId, SMS_STS_ID, newId);
  RELOCK(currentId);
  return ok >= 0;
}

bool ST3215Bus::saveMinMax(uint8_t id, int minV, int maxV) {
  UNLOCK(id);
  int a = _servo.writeWord(id, SMS_STS_MIN_ANGLE_LIMIT_L, minV);
  int b = _servo.writeWord(id, SMS_STS_MAX_ANGLE_LIMIT_L, maxV);
  RELOCK(id);
  return (a >= 0 && b >= 0);
}

bool ST3215Bus::saveTorqueLimit(uint8_t id, int limit) {
  if (_scsMode) return true; // SC09 has no torque limit register — silently skip
  UNLOCK(id);
  int ok = _servo.writeWord(id, SMS_STS_TORQUE_LIMIT_L, (uint16_t)limit);
  RELOCK(id);
  return ok >= 0;
}

bool ST3215Bus::saveCenterOffset(uint8_t id, int offset) {
  uint16_t wire = (offset >= 0) ? (uint16_t)offset : (uint16_t)(offset + 4096);
  UNLOCK(id);
  int ok = _servo.writeWord(id, SMS_STS_OFS_L, wire);
  RELOCK(id);
  return ok >= 0;
}

bool ST3215Bus::saveMode(uint8_t id, int mode) {
  if (_scsMode) return true; // SC09 has no mode register — silently skip
  UNLOCK(id);
  int ok = _servo.writeByte(id, SMS_STS_MODE, (uint8_t)(mode & 0x03));
  RELOCK(id);
  return ok >= 0;
}

bool ST3215Bus::saveBaud(uint8_t id, int baudIndex) {
  if (baudIndex < 0 || baudIndex >= 8) return false;
  UNLOCK(id);
  int ok = _servo.writeByte(id, SMS_STS_BAUD_RATE, (uint8_t)baudIndex);
  RELOCK(id);
  return ok >= 0;
}

#undef LOCK_REG
#undef UNLOCK
#undef RELOCK