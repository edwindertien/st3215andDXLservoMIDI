#include "servo_bus.h"

// Global ST3215Bus instance — selected by default, injected into App
ST3215Bus st3215Bus;

void ST3215Bus::begin(HardwareSerial& serial, uint32_t baud) {
  _serial = &serial;
  _baud   = baud;
  _servo.pSerial  = &serial;
  _servo.IOTimeOut = 20;
  // Claim Serial1 at the correct baud. All three buses share the same Serial1
  // object; whichever calls begin() last wins. end() first forces a full UART
  // reinitialisation on RP2040 (begin() alone is silently ignored if the port
  // is already open at a different baud rate).
  _serial->end();
  _serial->begin(baud);
  delay(5);
}

void ST3215Bus::setBaud(uint32_t baud) {
  if (!_serial) return;
  _baud = baud;
  _serial->end();   // force full UART reinit — same fix as dxl1_bus / dxl2_bus
  _serial->begin(baud);
  delay(5);
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
  return _servo.WritePosEx(id, pos, speed, acc) >= 0;
}

int ST3215Bus::readPosition(uint8_t id) {
  int v = _servo.ReadPos(id);
  return (v >= 0 && v <= 4095) ? v : -1;
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

bool ST3215Bus::saveId(uint8_t currentId, uint8_t newId) {
  _servo.unLockEprom(currentId);
  int ok = _servo.writeByte(currentId, SMS_STS_ID, newId);
  _servo.LockEprom(currentId);
  return ok >= 0;
}

bool ST3215Bus::saveMinMax(uint8_t id, int minV, int maxV) {
  _servo.unLockEprom(id);
  int a = _servo.writeWord(id, SMS_STS_MIN_ANGLE_LIMIT_L, minV);
  int b = _servo.writeWord(id, SMS_STS_MAX_ANGLE_LIMIT_L, maxV);
  _servo.LockEprom(id);
  return (a >= 0 && b >= 0);
}

bool ST3215Bus::saveTorqueLimit(uint8_t id, int limit) {
  _servo.unLockEprom(id);
  int ok = _servo.writeWord(id, SMS_STS_TORQUE_LIMIT_L, (uint16_t)limit);
  _servo.LockEprom(id);
  return ok >= 0;
}

bool ST3215Bus::saveCenterOffset(uint8_t id, int offset) {
  uint16_t wire = (offset >= 0) ? (uint16_t)offset : (uint16_t)(offset + 4096);
  _servo.unLockEprom(id);
  int ok = _servo.writeWord(id, SMS_STS_OFS_L, wire);
  _servo.LockEprom(id);
  return ok >= 0;
}

bool ST3215Bus::saveMode(uint8_t id, int mode) {
  _servo.unLockEprom(id);
  int ok = _servo.writeByte(id, SMS_STS_MODE, (uint8_t)(mode & 0x03));
  _servo.LockEprom(id);
  return ok >= 0;
}

bool ST3215Bus::saveBaud(uint8_t id, int baudIndex) {
  if (baudIndex < 0 || baudIndex >= 8) return false;
  _servo.unLockEprom(id);
  int ok = _servo.writeByte(id, SMS_STS_BAUD_RATE, (uint8_t)baudIndex);
  _servo.LockEprom(id);
  return ok >= 0;
}


// #include "servo_bus.h"

// // Global ST3215Bus instance — selected by default, injected into App
// ST3215Bus st3215Bus;

// void ST3215Bus::begin(HardwareSerial& serial, uint32_t baud) {
//   _serial = &serial;
//   _baud   = baud;
//   _servo.pSerial  = &serial;
//   _servo.IOTimeOut = 20;
// }

// void ST3215Bus::setBaud(uint32_t baud) {
//   if (!_serial) return;
//   _baud = baud;
//   _serial->begin(baud);
//   delay(5);
// }

// bool ST3215Bus::ping(uint8_t id) {
//   return _servo.Ping(id) == id;
// }

// int ST3215Bus::scan(uint8_t* ids, int maxIds, int& lastPingId) {
//   int count = 0;
//   for (int id = 0; id <= 253; ++id) {
//     lastPingId = id;
//     if (ping((uint8_t)id) && count < maxIds)
//       ids[count++] = (uint8_t)id;
//     delay(2);
//   }
//   return count;
// }

// bool ST3215Bus::setPosition(uint8_t id, int pos, uint16_t speed, uint8_t acc) {
//   return _servo.WritePosEx(id, pos, speed, acc) >= 0;
// }

// int ST3215Bus::readPosition(uint8_t id) {
//   int v = _servo.ReadPos(id);
//   return (v >= 0 && v <= 4095) ? v : -1;
// }

// bool ST3215Bus::torqueEnable(uint8_t id, bool en) {
//   return _servo.EnableTorque(id, en ? 1 : 0) >= 0;
// }

// bool ST3215Bus::readVoltage(uint8_t id, int& mv) {
//   int v = _servo.ReadVoltage(id);
//   if (v < 0) return false;
//   mv = v;
//   return true;
// }

// bool ST3215Bus::readTemperature(uint8_t id, int& tempC) {
//   int v = _servo.ReadTemper(id);
//   if (v < 0) return false;
//   tempC = v;
//   return true;
// }

// bool ST3215Bus::readStatus(uint8_t id, int& statusByte) {
//   int v = _servo.readByte(id, 65);
//   if (v < 0) return false;
//   statusByte = v;
//   return true;
// }

// bool ST3215Bus::readLoad(uint8_t id, int& loadPct) {
//   int v = _servo.readWord(id, SMS_STS_PRESENT_LOAD_L);
//   if (v < 0) return false;
//   int magnitude = v & 0x3FF;
//   int sign      = (v >> 10) & 1;
//   loadPct = (sign ? -1 : 1) * (magnitude * 100 / 1000);
//   return true;
// }

// bool ST3215Bus::readCurrent(uint8_t id, int& currentMa) {
//   int v = _servo.readWord(id, SMS_STS_PRESENT_CURRENT_L);
//   if (v < 0) return false;
//   currentMa = v * 65 / 10;
//   return true;
// }

// bool ST3215Bus::loadConfig(uint8_t id, uint8_t& outId, int& outMin, int& outMax,
//                             int& outTorqueLimit, int& outCenterOffset,
//                             int& outMode, int& outBaudIndex) {
//   if (!ping(id)) return false;
//   outId = id;
//   int minV  = _servo.readWord(id, SMS_STS_MIN_ANGLE_LIMIT_L);
//   int maxV  = _servo.readWord(id, SMS_STS_MAX_ANGLE_LIMIT_L);
//   int torq  = _servo.readWord(id, SMS_STS_TORQUE_LIMIT_L);
//   int ofs   = _servo.readWord(id, SMS_STS_OFS_L);
//   int mode  = _servo.readByte(id, SMS_STS_MODE);
//   int baud  = _servo.readByte(id, SMS_STS_BAUD_RATE);
//   if (minV < 0 || maxV < 0) return false;
//   outMin          = minV;
//   outMax          = maxV;
//   outTorqueLimit  = (torq >= 0) ? torq : 1000;
//   outMode         = (mode >= 0) ? (mode & 0x03) : 0;
//   outBaudIndex    = (baud >= 0 && baud < 8) ? baud : 0;
//   if (ofs >= 0) {
//     if (ofs > 2047) ofs -= 4096;
//     outCenterOffset = ofs;
//   } else {
//     outCenterOffset = 0;
//   }
//   return true;
// }

// bool ST3215Bus::saveId(uint8_t currentId, uint8_t newId) {
//   _servo.unLockEprom(currentId);
//   int ok = _servo.writeByte(currentId, SMS_STS_ID, newId);
//   _servo.LockEprom(currentId);
//   return ok >= 0;
// }

// bool ST3215Bus::saveMinMax(uint8_t id, int minV, int maxV) {
//   _servo.unLockEprom(id);
//   int a = _servo.writeWord(id, SMS_STS_MIN_ANGLE_LIMIT_L, minV);
//   int b = _servo.writeWord(id, SMS_STS_MAX_ANGLE_LIMIT_L, maxV);
//   _servo.LockEprom(id);
//   return (a >= 0 && b >= 0);
// }

// bool ST3215Bus::saveTorqueLimit(uint8_t id, int limit) {
//   _servo.unLockEprom(id);
//   int ok = _servo.writeWord(id, SMS_STS_TORQUE_LIMIT_L, (uint16_t)limit);
//   _servo.LockEprom(id);
//   return ok >= 0;
// }

// bool ST3215Bus::saveCenterOffset(uint8_t id, int offset) {
//   uint16_t wire = (offset >= 0) ? (uint16_t)offset : (uint16_t)(offset + 4096);
//   _servo.unLockEprom(id);
//   int ok = _servo.writeWord(id, SMS_STS_OFS_L, wire);
//   _servo.LockEprom(id);
//   return ok >= 0;
// }

// bool ST3215Bus::saveMode(uint8_t id, int mode) {
//   _servo.unLockEprom(id);
//   int ok = _servo.writeByte(id, SMS_STS_MODE, (uint8_t)(mode & 0x03));
//   _servo.LockEprom(id);
//   return ok >= 0;
// }

// bool ST3215Bus::saveBaud(uint8_t id, int baudIndex) {
//   if (baudIndex < 0 || baudIndex >= 8) return false;
//   _servo.unLockEprom(id);
//   int ok = _servo.writeByte(id, SMS_STS_BAUD_RATE, (uint8_t)baudIndex);
//   _servo.LockEprom(id);
//   return ok >= 0;
// }
