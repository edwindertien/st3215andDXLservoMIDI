#include "dxl1_bus.h"

// Global instance
Dxl1Bus dxl1Bus;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void Dxl1Bus::begin(HardwareSerial& serial, uint32_t baud) {
  _serial = &serial;
  _baud   = baud;
  _serial->begin(baud);
  if (_dePin >= 0) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW); // receive mode
  }
  delay(5);
}

void Dxl1Bus::setBaud(uint32_t baud) {
  if (!_serial) return;
  _baud = baud;
  _serial->begin(baud);
  delay(5);
}

// ---------------------------------------------------------------------------
// Direction control
// ---------------------------------------------------------------------------

void Dxl1Bus::txBegin() {
  if (_dePin >= 0) digitalWrite(_dePin, HIGH);
}

void Dxl1Bus::txEnd() {
  if (!_serial) return;
  _serial->flush(); // wait for TX complete
  if (_dePin >= 0) {
    delayMicroseconds(10); // guard time after last bit
    digitalWrite(_dePin, LOW);
  }
}

void Dxl1Bus::flushRx() {
  if (!_serial) return;
  unsigned long t = micros();
  while (micros() - t < 2000) {
    if (_serial->available()) {
      _serial->read();
      t = micros(); // reset on each byte received
    }
  }
}

// ---------------------------------------------------------------------------
// Checksum
// ---------------------------------------------------------------------------

uint8_t Dxl1Bus::checksum(uint8_t id, uint8_t len, uint8_t ins,
                           const uint8_t* p, uint8_t pLen) {
  uint16_t sum = id + len + ins;
  for (uint8_t i = 0; i < pLen; ++i) sum += p[i];
  return (uint8_t)(~sum & 0xFF);
}

// ---------------------------------------------------------------------------
// Packet transmit
// ---------------------------------------------------------------------------

bool Dxl1Bus::sendInstruction(uint8_t id, uint8_t ins,
                               const uint8_t* params, uint8_t paramLen) {
  if (!_serial) return false;
  uint8_t len = paramLen + 2; // INS + PARAMS + CHECKSUM — length field is LEN
  // Actually length field = number of remaining bytes after LEN = INS + PARAMS + CS
  // = paramLen + 2
  uint8_t cs = checksum(id, len, ins, params, paramLen);

  flushRx(); // clear any stale bytes before transmitting

  txBegin();
  _serial->write(0xFF);
  _serial->write(0xFF);
  _serial->write(id);
  _serial->write(len);
  _serial->write(ins);
  for (uint8_t i = 0; i < paramLen; ++i) _serial->write(params[i]);
  _serial->write(cs);
  txEnd();
  return true;
}

// ---------------------------------------------------------------------------
// Packet receive
// ---------------------------------------------------------------------------

int Dxl1Bus::recvStatus(uint8_t id, uint8_t* buf, uint8_t bufLen,
                          uint8_t& errByte) {
  if (!_serial) return -1;

  // State machine: wait for 0xFF 0xFF, then read header bytes
  enum State { WAIT_FF1, WAIT_FF2, READ_ID, READ_LEN, READ_ERR, READ_PARAMS, READ_CS };
  State state = WAIT_FF1;

  uint8_t rxId = 0, rxLen = 0, rxErr = 0, rxCs = 0;
  uint8_t paramCount = 0, paramExpected = 0;
  uint8_t localBuf[32];

  unsigned long start = micros();
  while (micros() - start < RX_TIMEOUT_US) {
    if (!_serial->available()) continue;
    uint8_t b = (uint8_t)_serial->read();

    switch (state) {
      case WAIT_FF1: if (b == 0xFF) state = WAIT_FF2; break;
      case WAIT_FF2: state = (b == 0xFF) ? READ_ID : WAIT_FF1; break;
      case READ_ID:
        rxId = b;
        // Ignore echo of our own transmission on half-duplex buses
        if (rxId == id) state = READ_LEN;
        else state = WAIT_FF1;
        break;
      case READ_LEN:
        rxLen = b;
        // LEN = number of remaining bytes (ERR + PARAMS + CS)
        // So paramExpected = LEN - 2
        if (rxLen < 2) { state = WAIT_FF1; break; }
        paramExpected = rxLen - 2;
        state = READ_ERR;
        break;
      case READ_ERR:
        rxErr = b;
        paramCount = 0;
        state = (paramExpected > 0) ? READ_PARAMS : READ_CS;
        break;
      case READ_PARAMS:
        if (paramCount < sizeof(localBuf)) localBuf[paramCount] = b;
        if (++paramCount >= paramExpected) state = READ_CS;
        break;
      case READ_CS:
        rxCs = b;
        // Verify checksum
        {
          uint8_t cs = checksum(rxId, rxLen, rxErr, localBuf, paramExpected);
          if (cs != rxCs) return -1;
        }
        // Copy params to caller buffer
        {
          uint8_t toCopy = (paramCount < bufLen) ? paramCount : bufLen;
          for (uint8_t i = 0; i < toCopy; ++i) buf[i] = localBuf[i];
        }
        errByte = rxErr;
        return (int)paramCount;
    }
  }
  return -1; // timeout
}

// ---------------------------------------------------------------------------
// Read/write convenience wrappers
// ---------------------------------------------------------------------------

int Dxl1Bus::readByte(uint8_t id, uint8_t addr) {
  uint8_t params[2] = { addr, 1 };
  sendInstruction(id, INS_READ, params, 2);
  uint8_t buf[4]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  if (n < 1 || err != 0) return -1;
  return buf[0];
}

int Dxl1Bus::readWord(uint8_t id, uint8_t addr) {
  uint8_t params[2] = { addr, 2 };
  sendInstruction(id, INS_READ, params, 2);
  uint8_t buf[4]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  if (n < 2 || err != 0) return -1;
  return (int)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

bool Dxl1Bus::writeByte(uint8_t id, uint8_t addr, uint8_t val) {
  uint8_t params[2] = { addr, val };
  sendInstruction(id, INS_WRITE, params, 2);
  uint8_t buf[4]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  return (n >= 0 && err == 0);
}

bool Dxl1Bus::writeWord(uint8_t id, uint8_t addr, uint16_t val) {
  uint8_t params[3] = { addr, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
  sendInstruction(id, INS_WRITE, params, 3);
  uint8_t buf[4]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  return (n >= 0 && err == 0);
}

// ---------------------------------------------------------------------------
// IServoBus — Discovery
// ---------------------------------------------------------------------------

bool Dxl1Bus::ping(uint8_t id) {
  sendInstruction(id, INS_PING, nullptr, 0);
  uint8_t buf[4]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  return (n >= 0 && err == 0);
}

int Dxl1Bus::scan(uint8_t* ids, int maxIds, int& lastPingId) {
  int count = 0;
  for (int id = 0; id <= 253; ++id) {
    lastPingId = id;
    if (ping((uint8_t)id) && count < maxIds)
      ids[count++] = (uint8_t)id;
    delay(2);
  }
  return count;
}

// ---------------------------------------------------------------------------
// IServoBus — Motion
// ---------------------------------------------------------------------------

bool Dxl1Bus::setPosition(uint8_t id, int pos, uint16_t speed, uint8_t acc) {
  // MX Protocol 1.0: write Goal Position (2 bytes) + Moving Speed (2 bytes)
  // at address 30 in one WRITE instruction (4 consecutive bytes).
  // acc is ignored — MX P1.0 has no acceleration register via standard protocol.
  (void)acc;
  pos = (pos < 0) ? 0 : (pos > 4095) ? 4095 : pos;
  uint16_t spd = (speed > 1023) ? 1023 : speed;
  uint8_t params[5] = {
    ADDR_GOAL_POSITION_L,
    (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
    (uint8_t)(spd & 0xFF), (uint8_t)(spd >> 8)
  };
  sendInstruction(id, INS_WRITE, params, 5);
  uint8_t buf[4]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  return (n >= 0 && err == 0);
}

int Dxl1Bus::readPosition(uint8_t id) {
  int v = readWord(id, ADDR_PRESENT_POSITION_L);
  if (v < 0 || v > 4095) return -1;
  return v;
}

bool Dxl1Bus::torqueEnable(uint8_t id, bool en) {
  return writeByte(id, ADDR_TORQUE_ENABLE, en ? 1 : 0);
}

// ---------------------------------------------------------------------------
// IServoBus — Telemetry
// ---------------------------------------------------------------------------

bool Dxl1Bus::readVoltage(uint8_t id, int& mv) {
  // Present Voltage: value is 10× actual voltage in volts
  // e.g. 100 = 10.0 V → return mV
  int v = readByte(id, ADDR_PRESENT_VOLTAGE);
  if (v < 0) return false;
  mv = v * 100; // 10× V → mV (v=100 → 10V → 10000 mV)
  return true;
}

bool Dxl1Bus::readTemperature(uint8_t id, int& tempC) {
  int v = readByte(id, ADDR_PRESENT_TEMP);
  if (v < 0) return false;
  tempC = v;
  return true;
}

bool Dxl1Bus::readStatus(uint8_t id, int& statusByte) {
  // MX P1.0: Alarm Shutdown (addr 18) holds the shutdown condition bits,
  // but the active error byte comes from the status packet ERR field.
  // We send a dummy PING and capture the error byte as the "status".
  sendInstruction(id, INS_PING, nullptr, 0);
  uint8_t buf[4]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  if (n < 0) return false;
  // MX error byte bit layout (Protocol 1.0):
  // Bit 0: Input Voltage Error
  // Bit 1: Angle Limit Error
  // Bit 2: Overheating Error
  // Bit 3: Range Error
  // Bit 4: Checksum Error
  // Bit 5: Overload Error
  // Bit 6: Instruction Error
  // Map to our UI's expected fault bit positions (same as ST3215):
  // UI bit 0=Voltage, 1=Sensor(→RangeErr), 2=Temp, 3=Current(→none), 4=Angle, 5=Overload
  uint8_t mapped = 0;
  if (err & 0x01) mapped |= (1 << 0); // Voltage
  if (err & 0x08) mapped |= (1 << 1); // Range → Sensor slot
  if (err & 0x04) mapped |= (1 << 2); // Overheating
  if (err & 0x02) mapped |= (1 << 4); // Angle Limit
  if (err & 0x20) mapped |= (1 << 5); // Overload
  statusByte = mapped;
  return true;
}

bool Dxl1Bus::readLoad(uint8_t id, int& loadPct) {
  // Present Load: 0..2047. Bit10=direction (0=CCW, 1=CW), bits0-9=magnitude 0..1023
  int v = readWord(id, ADDR_PRESENT_LOAD_L);
  if (v < 0) return false;
  int magnitude = v & 0x3FF;
  int direction = (v >> 10) & 1;
  loadPct = (direction ? -1 : 1) * (magnitude * 100 / 1023);
  return true;
}

bool Dxl1Bus::readCurrent(uint8_t id, int& currentMa) {
  // Present Current at addr 68 (MX-28/64 only, 2 bytes).
  // Unit: 4.5 mA per count (centered at 2048 = 0 A)
  int v = readWord(id, ADDR_PRESENT_CURRENT_L);
  if (v < 0) { currentMa = -1; return false; }
  int signed_v = v - 2048;
  currentMa = abs(signed_v) * 45 / 10; // 4.5 mA/count
  return true;
}

// ---------------------------------------------------------------------------
// IServoBus — Configuration
// ---------------------------------------------------------------------------

bool Dxl1Bus::loadConfig(uint8_t id, uint8_t& outId, int& outMin, int& outMax,
                          int& outTorqueLimit, int& outCenterOffset,
                          int& outMode, int& outBaudIndex) {
  if (!ping(id)) return false;
  outId = id;

  int cw  = readWord(id, ADDR_CW_ANGLE_LIMIT_L);
  int ccw = readWord(id, ADDR_CCW_ANGLE_LIMIT_L);
  int torq = readWord(id, ADDR_TORQUE_LIMIT_L);
  int baud = readByte(id, ADDR_BAUD_RATE);

  if (cw < 0 || ccw < 0) return false;
  outMin = cw;
  outMax = ccw;
  outTorqueLimit  = (torq >= 0) ? torq : 1023;
  outCenterOffset = 0; // MX P1.0 has multi-turn offset but not a simple signed offset
  // Mode: if both limits are 0, it's wheel mode
  outMode = (cw == 0 && ccw == 0) ? 1 : 0;

  // Convert DXL baud byte to nearest index in our DXL1_BAUD_TABLE
  // DXL baud formula: Baud = 2000000 / (byte + 1)
  // Common values: 1→57600(≈), 3→500000, 34→57600, 207→9600 etc.
  // Simpler: just match against table
  uint32_t realBaud = (baud >= 0) ? 2000000UL / ((uint32_t)baud + 1) : 57600;
  outBaudIndex = DXL1_DEFAULT_BAUD_INDEX; // default to 57600
  for (int i = 0; i < DXL1_BAUD_COUNT; ++i) {
    uint32_t diff = (realBaud > DXL1_BAUD_TABLE[i])
                    ? realBaud - DXL1_BAUD_TABLE[i]
                    : DXL1_BAUD_TABLE[i] - realBaud;
    uint32_t dBest = (DXL1_BAUD_TABLE[outBaudIndex] > realBaud)
                     ? DXL1_BAUD_TABLE[outBaudIndex] - realBaud
                     : realBaud - DXL1_BAUD_TABLE[outBaudIndex];
    if (diff < dBest) outBaudIndex = i;
  }
  return true;
}

bool Dxl1Bus::saveId(uint8_t currentId, uint8_t newId) {
  // EEPROM writes require torque off first
  torqueEnable(currentId, false);
  bool ok = writeByte(currentId, ADDR_ID, newId);
  return ok;
}

bool Dxl1Bus::saveMinMax(uint8_t id, int minV, int maxV) {
  torqueEnable(id, false);
  bool a = writeWord(id, ADDR_CW_ANGLE_LIMIT_L,  (uint16_t)minV);
  bool b = writeWord(id, ADDR_CCW_ANGLE_LIMIT_L, (uint16_t)maxV);
  return a && b;
}

bool Dxl1Bus::saveTorqueLimit(uint8_t id, int limit) {
  // Write both the EEPROM max torque (addr 14) and RAM runtime limit (addr 34)
  torqueEnable(id, false);
  bool a = writeWord(id, ADDR_TORQUE_LIMIT_L,   (uint16_t)limit);
  bool b = writeWord(id, ADDR_TORQUE_LIMIT_RAM, (uint16_t)limit);
  return a && b;
}

bool Dxl1Bus::saveCenterOffset(uint8_t id, int offset) {
  // MX P1.0 does not have a simple center offset register at the same
  // address as ST3215. The multi-turn offset (addr 20) is different in meaning.
  // We silently accept and ignore rather than writing a wrong value.
  (void)id; (void)offset;
  return true;
}

bool Dxl1Bus::saveMode(uint8_t id, int mode) {
  // Mode 0 (joint): set CW=0, CCW=4095
  // Mode 1 (wheel): set both CW=0, CCW=0 (endless turn mode)
  torqueEnable(id, false);
  if (mode == 0) {
    bool a = writeWord(id, ADDR_CW_ANGLE_LIMIT_L,  0);
    bool b = writeWord(id, ADDR_CCW_ANGLE_LIMIT_L, 4095);
    return a && b;
  } else {
    bool a = writeWord(id, ADDR_CW_ANGLE_LIMIT_L,  0);
    bool b = writeWord(id, ADDR_CCW_ANGLE_LIMIT_L, 0);
    return a && b;
  }
}

bool Dxl1Bus::saveBaud(uint8_t id, int baudIndex) {
  if (baudIndex < 0 || baudIndex >= DXL1_BAUD_COUNT) return false;
  // Convert baud rate to DXL byte value: byte = (2000000 / baud) - 1
  uint32_t baud = DXL1_BAUD_TABLE[baudIndex];
  uint8_t dxlBaud = (uint8_t)((2000000UL / baud) - 1);
  torqueEnable(id, false);
  return writeByte(id, ADDR_BAUD_RATE, dxlBaud);
}
