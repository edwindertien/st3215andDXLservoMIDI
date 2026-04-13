#include "dxl1_bus.h"
#include "../config.h"

// Global instance
Dxl1Bus dxl1Bus;

// ---------------------------------------------------------------------------
// TX/RX debug blink — Green = TX sent, Red = RX received
// ---------------------------------------------------------------------------
static inline void blinkTx() {
#if defined(BOARD_XIAO_EXPANSION)
  if (HW::LED_G_PIN >= 0) {
    digitalWrite(HW::LED_G_PIN, LOW);
    delayMicroseconds(300);
    digitalWrite(HW::LED_G_PIN, HIGH);
  }
#endif
}
static inline void blinkRx() {
#if defined(BOARD_XIAO_EXPANSION)
  if (HW::LED_R_PIN >= 0) {
    digitalWrite(HW::LED_R_PIN, LOW);
    delayMicroseconds(300);
    digitalWrite(HW::LED_R_PIN, HIGH);
  }
#endif
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void Dxl1Bus::begin(HardwareSerial& serial, uint32_t baud) {
  _serial = &serial;
  _baud   = baud;
  _serial->end();
  _serial->begin(baud);
  if (_dePin >= 0) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
  }
  delay(5);
}

void Dxl1Bus::setBaud(uint32_t baud) {
  if (!_serial) return;
  _baud = baud;
  _serial->end();   // force full UART reinitialisation on RP2040
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
  _serial->flush();
  // Post-TX guard — same reasoning as Dxl2Bus::txEnd().
  // Ensures the HW shift register is fully drained and the auto-direction
  // adapter has switched back to RX before we start listening for a response.
  if (_baud > 0) {
    uint32_t guardUs = (3UL * 10UL * 1000000UL) / _baud;
    if (guardUs < 20) guardUs = 20;
    delayMicroseconds(guardUs);
  }
  if (_dePin >= 0) digitalWrite(_dePin, LOW);
}

void Dxl1Bus::flushRx() {
  if (!_serial) return;
  // Fixed-duration drain — never blocks longer than FLUSH_US
  unsigned long t = micros();
  while (micros() - t < FLUSH_US) {
    if (_serial->available()) _serial->read();
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

int Dxl1Bus::sendInstruction(uint8_t id, uint8_t ins,
                              const uint8_t* params, uint8_t paramLen) {
  if (!_serial) return 0;
  uint8_t len = paramLen + 2;
  uint8_t cs  = checksum(id, len, ins, params, paramLen);

  flushRx(); // clear stale bytes before sending

  txBegin();
  _serial->write(0xFF);
  _serial->write(0xFF);
  _serial->write(id);
  _serial->write(len);
  _serial->write(ins);
  for (uint8_t i = 0; i < paramLen; ++i) _serial->write(params[i]);
  _serial->write(cs);
  txEnd();

  blinkTx();
  return 6 + paramLen; // 2 header + id + len + ins + params + cs
}

// ---------------------------------------------------------------------------
// Packet receive
// We accept a status packet from ANY servo ID — the checksum fully validates
// the packet.  Filtering on ID here would cause the parser to reset to
// WAIT_FF1 on any stale byte or echo where rxId != expected, then burn the
// entire 20 ms timeout before returning -1.  The id argument is kept for
// API compatibility but is no longer used for filtering.
// ---------------------------------------------------------------------------

int Dxl1Bus::recvStatus(uint8_t id, uint8_t* buf, uint8_t bufLen,
                         uint8_t& errByte) {
  if (!_serial) return -1;
  (void)id; // accept response from any servo; checksum ensures integrity

  enum State { WAIT_FF1, WAIT_FF2, READ_ID, READ_LEN,
               READ_ERR, READ_PARAMS, READ_CS };
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
        rxId  = b;
        state = READ_LEN; // accept any ID — checksum validates later
        break;
      case READ_LEN:
        rxLen = b;
        if (rxLen < 2) { state = WAIT_FF1; break; }
        paramExpected = rxLen - 2;
        state = READ_ERR;
        break;
      case READ_ERR:
        rxErr      = b;
        paramCount = 0;
        state      = (paramExpected > 0) ? READ_PARAMS : READ_CS;
        break;
      case READ_PARAMS:
        if (paramCount < sizeof(localBuf)) localBuf[paramCount] = b;
        if (++paramCount >= paramExpected) state = READ_CS;
        break;
      case READ_CS:
        rxCs = b;
        {
          uint8_t cs = checksum(rxId, rxLen, rxErr, localBuf, paramExpected);
          if (cs != rxCs) { state = WAIT_FF1; break; } // bad CS — keep hunting
        }
        {
          uint8_t toCopy = (paramCount < bufLen) ? paramCount : bufLen;
          for (uint8_t i = 0; i < toCopy; ++i) buf[i] = localBuf[i];
        }
        errByte = rxErr;
        blinkRx();
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
  // At STATUS_RETURN_LEVEL=1 (MX factory default) the servo does NOT reply to
  // WRITE instructions — waiting for a status packet would block 20ms every call.
  // We only wait if STATUS_RETURN_LEVEL=2 (all packets).  Since we can't know
  // the level without reading it, we use a short probe: if a byte arrives within
  // 3ms we parse it; otherwise we treat it as success (level 1 behaviour).
  unsigned long t = micros();
  while (micros() - t < 3000) {
    if (_serial->available()) {
      uint8_t buf[4]; uint8_t err = 0;
      int n = recvStatus(id, buf, sizeof(buf), err);
      return (n < 0) || (err == 0);
    }
  }
  return true; // no response = STATUS_RETURN_LEVEL 1, treat as success
}

bool Dxl1Bus::writeWord(uint8_t id, uint8_t addr, uint16_t val) {
  uint8_t params[3] = { addr, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
  sendInstruction(id, INS_WRITE, params, 3);
  unsigned long t = micros();
  while (micros() - t < 3000) {
    if (_serial->available()) {
      uint8_t buf[4]; uint8_t err = 0;
      int n = recvStatus(id, buf, sizeof(buf), err);
      return (n < 0) || (err == 0);
    }
  }
  return true;
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
  (void)acc;  // MX-28 P1.0 has no acceleration register in RAM — ignored
  pos = (pos < 0) ? 0 : (pos > 4095) ? 4095 : pos;
  uint16_t spd = (speed > 1023) ? 1023 : (speed == 0 ? 0 : speed);

  // Sync write: GOAL_POSITION_L(30) + MOVING_SPEED_L(32) — 4 bytes contiguous
  // This is a WRITE instruction, no status reply at STATUS_RETURN_LEVEL=1.
  uint8_t params[5] = {
    ADDR_GOAL_POSITION_L,
    (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
    (uint8_t)(spd & 0xFF), (uint8_t)(spd >> 8)
  };
  sendInstruction(id, INS_WRITE, params, 5);
  // Don't wait for status reply — at STATUS_RETURN_LEVEL=1 there is none,
  // and blocking 20ms here makes live control unusable.
  return true;
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
  int v = readByte(id, ADDR_PRESENT_VOLTAGE);
  if (v < 0) return false;
  mv = v * 100;
  return true;
}

bool Dxl1Bus::readTemperature(uint8_t id, int& tempC) {
  int v = readByte(id, ADDR_PRESENT_TEMP);
  if (v < 0) return false;
  tempC = v;
  return true;
}

bool Dxl1Bus::readStatus(uint8_t id, int& statusByte) {
  sendInstruction(id, INS_PING, nullptr, 0);
  uint8_t buf[4]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  if (n < 0) return false;
  uint8_t mapped = 0;
  if (err & 0x01) mapped |= (1 << 0);
  if (err & 0x08) mapped |= (1 << 1);
  if (err & 0x04) mapped |= (1 << 2);
  if (err & 0x02) mapped |= (1 << 4);
  if (err & 0x20) mapped |= (1 << 5);
  statusByte = mapped;
  return true;
}

bool Dxl1Bus::readLoad(uint8_t id, int& loadPct) {
  int v = readWord(id, ADDR_PRESENT_LOAD_L);
  if (v < 0) return false;
  int magnitude = v & 0x3FF;
  int direction = (v >> 10) & 1;
  loadPct = (direction ? -1 : 1) * (magnitude * 100 / 1023);
  return true;
}

bool Dxl1Bus::readCurrent(uint8_t id, int& currentMa) {
  int v = readWord(id, ADDR_PRESENT_CURRENT_L);
  if (v < 0) { currentMa = -1; return false; }
  int signed_v = v - 2048;
  currentMa = abs(signed_v) * 45 / 10;
  return true;
}

// ---------------------------------------------------------------------------
// IServoBus — Configuration load
// ---------------------------------------------------------------------------

bool Dxl1Bus::loadConfig(uint8_t id, uint8_t& outId, int& outMin, int& outMax,
                          int& outTorqueLimit, int& outCenterOffset,
                          int& outMode, int& outBaudIndex) {
  if (!ping(id)) return false;
  outId = id;

  int cw        = readWord(id, ADDR_CW_ANGLE_LIMIT_L);
  int ccw       = readWord(id, ADDR_CCW_ANGLE_LIMIT_L);
  int torq      = readWord(id, ADDR_TORQUE_LIMIT_L);
  int baud      = readByte(id, ADDR_BAUD_RATE);
  int driveMode = readByte(id, ADDR_DRIVE_MODE); // reg 10: bit0=0→joint, bit0=1→wheel

  if (cw < 0 || ccw < 0) return false;
  outMin          = cw;
  outMax          = ccw;
  outTorqueLimit  = (torq >= 0) ? torq : 1023;
  outCenterOffset = 0;

  // Wheel mode: ADDR_DRIVE_MODE bit0=1, OR both limits are 0 (older firmware)
  if (driveMode >= 0) {
    outMode = (driveMode & 0x01) ? 1 : 0;
  } else {
    outMode = (cw == 0 && ccw == 0) ? 1 : 0; // fallback if reg unreadable
  }

  uint32_t realBaud = (baud >= 0) ? 2000000UL / ((uint32_t)baud + 1) : 57600;
  outBaudIndex = DXL1_DEFAULT_BAUD_INDEX;
  for (int i = 0; i < DXL1_BAUD_COUNT; ++i) {
    uint32_t diff  = (realBaud > DXL1_BAUD_TABLE[i])
                     ? realBaud - DXL1_BAUD_TABLE[i]
                     : DXL1_BAUD_TABLE[i] - realBaud;
    uint32_t dBest = (DXL1_BAUD_TABLE[outBaudIndex] > realBaud)
                     ? DXL1_BAUD_TABLE[outBaudIndex] - realBaud
                     : realBaud - DXL1_BAUD_TABLE[outBaudIndex];
    if (diff < dBest) outBaudIndex = i;
  }
  return true;
}

// ---------------------------------------------------------------------------
// EEPROM write helpers — 5 ms settle per write
// ---------------------------------------------------------------------------

bool Dxl1Bus::eepromWriteByte(uint8_t id, uint8_t addr, uint8_t val) {
  if (!writeByte(id, addr, val)) return false;
  delay(5);
  return true;
}

bool Dxl1Bus::eepromWriteWord(uint8_t id, uint8_t addr, uint16_t val) {
  if (!writeWord(id, addr, val)) return false;
  delay(5);
  return true;
}

// ---------------------------------------------------------------------------
// IServoBus — Configuration save
// Torque off is called ONCE at the start (in saveMinMax, the first call).
// ---------------------------------------------------------------------------

bool Dxl1Bus::saveId(uint8_t currentId, uint8_t newId) {
  torqueEnable(currentId, false);
  delay(10);
  return eepromWriteByte(currentId, ADDR_ID, newId);
}

bool Dxl1Bus::saveMinMax(uint8_t id, int minV, int maxV) {
  torqueEnable(id, false); // first call in batch — disable torque once
  delay(10);
  bool a = eepromWriteWord(id, ADDR_CW_ANGLE_LIMIT_L,  (uint16_t)minV);
  bool b = eepromWriteWord(id, ADDR_CCW_ANGLE_LIMIT_L, (uint16_t)maxV);
  return a && b;
}

bool Dxl1Bus::saveTorqueLimit(uint8_t id, int limit) {
  // Torque already off from saveMinMax
  bool a = eepromWriteWord(id, ADDR_TORQUE_LIMIT_L,  (uint16_t)limit);
  bool b = writeWord(id, ADDR_TORQUE_LIMIT_RAM, (uint16_t)limit); // RAM, no delay needed
  return a && b;
}

bool Dxl1Bus::saveCenterOffset(uint8_t id, int offset) {
  (void)id; (void)offset;
  return true;
}

bool Dxl1Bus::saveMode(uint8_t id, int mode) {
  if (mode != 1) return true; // joint: limits already set by saveMinMax
  bool a = eepromWriteWord(id, ADDR_CW_ANGLE_LIMIT_L,  0);
  bool b = eepromWriteWord(id, ADDR_CCW_ANGLE_LIMIT_L, 0);
  return a && b;
}

bool Dxl1Bus::saveBaud(uint8_t id, int baudIndex) {
  if (baudIndex < 0 || baudIndex >= DXL1_BAUD_COUNT) return false;
  uint32_t baud    = DXL1_BAUD_TABLE[baudIndex];
  uint8_t  dxlBaud = (uint8_t)((2000000UL / baud) - 1);
  return eepromWriteByte(id, ADDR_BAUD_RATE, dxlBaud);
}
// #include "dxl1_bus.h"
// #include "../config.h"

// // Global instance
// Dxl1Bus dxl1Bus;

// // ---------------------------------------------------------------------------
// // TX/RX debug blink — Green = TX sent, Red = RX received
// // ---------------------------------------------------------------------------
// static inline void blinkTx() {
// #if defined(BOARD_XIAO_EXPANSION)
//   if (HW::LED_G_PIN >= 0) {
//     digitalWrite(HW::LED_G_PIN, LOW);
//     delayMicroseconds(300);
//     digitalWrite(HW::LED_G_PIN, HIGH);
//   }
// #endif
// }
// static inline void blinkRx() {
// #if defined(BOARD_XIAO_EXPANSION)
//   if (HW::LED_R_PIN >= 0) {
//     digitalWrite(HW::LED_R_PIN, LOW);
//     delayMicroseconds(300);
//     digitalWrite(HW::LED_R_PIN, HIGH);
//   }
// #endif
// }

// // ---------------------------------------------------------------------------
// // Lifecycle
// // ---------------------------------------------------------------------------

// void Dxl1Bus::begin(HardwareSerial& serial, uint32_t baud) {
//   _serial = &serial;
//   _baud   = baud;
//   _serial->begin(baud);
//   if (_dePin >= 0) {
//     pinMode(_dePin, OUTPUT);
//     digitalWrite(_dePin, LOW);
//   }
//   delay(5);
// }

// void Dxl1Bus::setBaud(uint32_t baud) {
//   if (!_serial) return;
//   _baud = baud;
//   _serial->begin(baud);
//   delay(5);
// }

// // ---------------------------------------------------------------------------
// // Direction control
// // ---------------------------------------------------------------------------

// void Dxl1Bus::txBegin() {
//   if (_dePin >= 0) digitalWrite(_dePin, HIGH);
// }

// void Dxl1Bus::txEnd() {
//   if (!_serial) return;
//   _serial->flush();
//   if (_dePin >= 0) {
//     delayMicroseconds(20);
//     digitalWrite(_dePin, LOW);
//   }
// }

// void Dxl1Bus::flushRx() {
//   if (!_serial) return;
//   // Fixed-duration drain — never blocks longer than FLUSH_US
//   unsigned long t = micros();
//   while (micros() - t < FLUSH_US) {
//     if (_serial->available()) _serial->read();
//   }
// }

// // ---------------------------------------------------------------------------
// // Checksum
// // ---------------------------------------------------------------------------

// uint8_t Dxl1Bus::checksum(uint8_t id, uint8_t len, uint8_t ins,
//                            const uint8_t* p, uint8_t pLen) {
//   uint16_t sum = id + len + ins;
//   for (uint8_t i = 0; i < pLen; ++i) sum += p[i];
//   return (uint8_t)(~sum & 0xFF);
// }

// // ---------------------------------------------------------------------------
// // Packet transmit
// // Returns total bytes written to wire; caller passes this to drainEcho().
// // ---------------------------------------------------------------------------

// int Dxl1Bus::sendInstruction(uint8_t id, uint8_t ins,
//                               const uint8_t* params, uint8_t paramLen) {
//   if (!_serial) return 0;
//   uint8_t len = paramLen + 2;
//   uint8_t cs  = checksum(id, len, ins, params, paramLen);

//   flushRx(); // clear stale bytes before sending

//   txBegin();
//   _serial->write(0xFF);
//   _serial->write(0xFF);
//   _serial->write(id);
//   _serial->write(len);
//   _serial->write(ins);
//   for (uint8_t i = 0; i < paramLen; ++i) _serial->write(params[i]);
//   _serial->write(cs);
//   txEnd(); // calls _serial->flush() — blocks until all bytes physically sent

//   blinkTx();
//   return 6 + paramLen; // 2 header + id + len + ins + params + cs
// }

// // ---------------------------------------------------------------------------
// // drainEcho — discard the TX echo from auto-direction RS485 adapters.
// //
// // Auto-direction adapters loop TX back to RX because their direction-switching
// // circuit samples the bus while still driving it.  The echo arrives
// // simultaneously with (and ends just after) our own TX packet.
// //
// // The drain window is derived from the baud rate:
// //   window = (byteCount * 10 bits * 1000000us/s) / baud  +  200us margin
// // This is just long enough to receive the echo at the current baud, and ends
// // well before the servo's response can arrive (which needs a full round-trip
// // of at least one more TX-packet time).
// //
// // At 57142 baud, 6-byte packet: window = 1050 + 200 = ~1250us  — echo arrives, response safe
// // At 1Mbaud,   6-byte packet: window =   60 + 200 =  ~260us  — no echo, window ends before response
// // ---------------------------------------------------------------------------

// void Dxl1Bus::drainEcho(uint8_t byteCount) {
//   if (!_serial || _baud == 0) return;
//   // Time for byteCount bytes at current baud (10 bits per byte) + 200us margin
//   uint32_t windowUs = ((uint32_t)byteCount * 10UL * 1000000UL) / _baud + 200UL;
//   uint8_t drained = 0;
//   unsigned long start = micros();
//   while (micros() - start < windowUs) {
//     if (_serial->available()) {
//       _serial->read();
//       ++drained;
//       // If we've received exactly byteCount bytes, the echo is complete — stop early
//       if (drained >= byteCount) break;
//     }
//   }
// }

// // ---------------------------------------------------------------------------
// // Packet receive
// // We accept a status packet from ANY servo ID — the checksum fully validates
// // the packet.  Filtering on ID here would cause the parser to reset to
// // WAIT_FF1 on any stale byte or echo where rxId != expected, then burn the
// // entire 20 ms timeout before returning -1.  The id argument is kept for
// // API compatibility but is no longer used for filtering.
// // ---------------------------------------------------------------------------

// int Dxl1Bus::recvStatus(uint8_t id, uint8_t* buf, uint8_t bufLen,
//                          uint8_t& errByte) {
//   if (!_serial) return -1;
//   (void)id; // accept response from any servo; checksum ensures integrity

//   enum State { WAIT_FF1, WAIT_FF2, READ_ID, READ_LEN,
//                READ_ERR, READ_PARAMS, READ_CS };
//   State state = WAIT_FF1;

//   uint8_t rxId = 0, rxLen = 0, rxErr = 0, rxCs = 0;
//   uint8_t paramCount = 0, paramExpected = 0;
//   uint8_t localBuf[32];

//   unsigned long start = micros();
//   while (micros() - start < RX_TIMEOUT_US) {
//     if (!_serial->available()) continue;
//     uint8_t b = (uint8_t)_serial->read();

//     switch (state) {
//       case WAIT_FF1: if (b == 0xFF) state = WAIT_FF2; break;
//       case WAIT_FF2: state = (b == 0xFF) ? READ_ID : WAIT_FF1; break;
//       case READ_ID:
//         rxId  = b;
//         state = READ_LEN; // accept any ID — checksum validates later
//         break;
//       case READ_LEN:
//         rxLen = b;
//         if (rxLen < 2) { state = WAIT_FF1; break; }
//         paramExpected = rxLen - 2;
//         state = READ_ERR;
//         break;
//       case READ_ERR:
//         rxErr      = b;
//         paramCount = 0;
//         state      = (paramExpected > 0) ? READ_PARAMS : READ_CS;
//         break;
//       case READ_PARAMS:
//         if (paramCount < sizeof(localBuf)) localBuf[paramCount] = b;
//         if (++paramCount >= paramExpected) state = READ_CS;
//         break;
//       case READ_CS:
//         rxCs = b;
//         {
//           uint8_t cs = checksum(rxId, rxLen, rxErr, localBuf, paramExpected);
//           if (cs != rxCs) { state = WAIT_FF1; break; } // bad CS — keep hunting
//         }
//         {
//           uint8_t toCopy = (paramCount < bufLen) ? paramCount : bufLen;
//           for (uint8_t i = 0; i < toCopy; ++i) buf[i] = localBuf[i];
//         }
//         errByte = rxErr;
//         blinkRx();
//         return (int)paramCount;
//     }
//   }
//   return -1; // timeout
// }

// // ---------------------------------------------------------------------------
// // Read/write convenience wrappers
// // ---------------------------------------------------------------------------

// int Dxl1Bus::readByte(uint8_t id, uint8_t addr) {
//   uint8_t params[2] = { addr, 1 };
//   int txLen = sendInstruction(id, INS_READ, params, 2);
//   drainEcho(txLen);
//   uint8_t buf[4]; uint8_t err;
//   int n = recvStatus(id, buf, sizeof(buf), err);
//   if (n < 1 || err != 0) return -1;
//   return buf[0];
// }

// int Dxl1Bus::readWord(uint8_t id, uint8_t addr) {
//   uint8_t params[2] = { addr, 2 };
//   int txLen = sendInstruction(id, INS_READ, params, 2);
//   drainEcho(txLen);
//   uint8_t buf[4]; uint8_t err;
//   int n = recvStatus(id, buf, sizeof(buf), err);
//   if (n < 2 || err != 0) return -1;
//   return (int)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
// }

// bool Dxl1Bus::writeByte(uint8_t id, uint8_t addr, uint8_t val) {
//   uint8_t params[2] = { addr, val };
//   int txLen = sendInstruction(id, INS_WRITE, params, 2);
//   drainEcho(txLen); // swallow echo; at STATUS_RETURN_LEVEL=1 nothing follows
//   unsigned long t = micros();
//   while (micros() - t < 3000) {
//     if (_serial->available()) {
//       uint8_t buf[4]; uint8_t err = 0;
//       int n = recvStatus(id, buf, sizeof(buf), err);
//       return (n < 0) || (err == 0);
//     }
//   }
//   return true;
// }

// bool Dxl1Bus::writeWord(uint8_t id, uint8_t addr, uint16_t val) {
//   uint8_t params[3] = { addr, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
//   int txLen = sendInstruction(id, INS_WRITE, params, 3);
//   drainEcho(txLen);
//   unsigned long t = micros();
//   while (micros() - t < 3000) {
//     if (_serial->available()) {
//       uint8_t buf[4]; uint8_t err = 0;
//       int n = recvStatus(id, buf, sizeof(buf), err);
//       return (n < 0) || (err == 0);
//     }
//   }
//   return true;
// }

// // ---------------------------------------------------------------------------
// // IServoBus — Discovery
// // ---------------------------------------------------------------------------

// bool Dxl1Bus::ping(uint8_t id) {
//   int txLen = sendInstruction(id, INS_PING, nullptr, 0);
//   drainEcho(txLen);
//   uint8_t buf[4]; uint8_t err;
//   int n = recvStatus(id, buf, sizeof(buf), err);
//   return (n >= 0 && err == 0);
// }

// int Dxl1Bus::scan(uint8_t* ids, int maxIds, int& lastPingId) {
//   int count = 0;
//   for (int id = 0; id <= 253; ++id) {
//     lastPingId = id;
//     if (ping((uint8_t)id) && count < maxIds)
//       ids[count++] = (uint8_t)id;
//     delay(2);
//   }
//   return count;
// }

// // ---------------------------------------------------------------------------
// // IServoBus — Motion
// // ---------------------------------------------------------------------------

// bool Dxl1Bus::setPosition(uint8_t id, int pos, uint16_t speed, uint8_t acc) {
//   (void)acc;
//   pos = (pos < 0) ? 0 : (pos > 4095) ? 4095 : pos;
//   uint16_t spd = (speed > 1023) ? 1023 : (speed == 0 ? 0 : speed);

//   // WRITE instruction; no status reply at STATUS_RETURN_LEVEL=1.
//   uint8_t params[5] = {
//     ADDR_GOAL_POSITION_L,
//     (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
//     (uint8_t)(spd & 0xFF), (uint8_t)(spd >> 8)
//   };
//   int txLen = sendInstruction(id, INS_WRITE, params, 5);
//   drainEcho(txLen); // keep RX clean for next readPosition call
//   return true;
// }

// int Dxl1Bus::readPosition(uint8_t id) {
//   int v = readWord(id, ADDR_PRESENT_POSITION_L);
//   if (v < 0 || v > 4095) return -1;
//   return v;
// }

// bool Dxl1Bus::torqueEnable(uint8_t id, bool en) {
//   return writeByte(id, ADDR_TORQUE_ENABLE, en ? 1 : 0);
// }

// // ---------------------------------------------------------------------------
// // IServoBus — Telemetry
// // ---------------------------------------------------------------------------

// bool Dxl1Bus::readVoltage(uint8_t id, int& mv) {
//   int v = readByte(id, ADDR_PRESENT_VOLTAGE);
//   if (v < 0) return false;
//   mv = v * 100;
//   return true;
// }

// bool Dxl1Bus::readTemperature(uint8_t id, int& tempC) {
//   int v = readByte(id, ADDR_PRESENT_TEMP);
//   if (v < 0) return false;
//   tempC = v;
//   return true;
// }

// bool Dxl1Bus::readStatus(uint8_t id, int& statusByte) {
//   int txLen = sendInstruction(id, INS_PING, nullptr, 0);
//   drainEcho(txLen);
//   uint8_t buf[4]; uint8_t err;
//   int n = recvStatus(id, buf, sizeof(buf), err);
//   if (n < 0) return false;
//   uint8_t mapped = 0;
//   if (err & 0x01) mapped |= (1 << 0);
//   if (err & 0x08) mapped |= (1 << 1);
//   if (err & 0x04) mapped |= (1 << 2);
//   if (err & 0x02) mapped |= (1 << 4);
//   if (err & 0x20) mapped |= (1 << 5);
//   statusByte = mapped;
//   return true;
// }

// bool Dxl1Bus::readLoad(uint8_t id, int& loadPct) {
//   int v = readWord(id, ADDR_PRESENT_LOAD_L);
//   if (v < 0) return false;
//   int magnitude = v & 0x3FF;
//   int direction = (v >> 10) & 1;
//   loadPct = (direction ? -1 : 1) * (magnitude * 100 / 1023);
//   return true;
// }

// bool Dxl1Bus::readCurrent(uint8_t id, int& currentMa) {
//   int v = readWord(id, ADDR_PRESENT_CURRENT_L);
//   if (v < 0) { currentMa = -1; return false; }
//   int signed_v = v - 2048;
//   currentMa = abs(signed_v) * 45 / 10;
//   return true;
// }

// // ---------------------------------------------------------------------------
// // IServoBus — Configuration load
// // ---------------------------------------------------------------------------

// bool Dxl1Bus::loadConfig(uint8_t id, uint8_t& outId, int& outMin, int& outMax,
//                           int& outTorqueLimit, int& outCenterOffset,
//                           int& outMode, int& outBaudIndex) {
//   if (!ping(id)) return false;
//   outId = id;

//   int cw        = readWord(id, ADDR_CW_ANGLE_LIMIT_L);
//   int ccw       = readWord(id, ADDR_CCW_ANGLE_LIMIT_L);
//   int torq      = readWord(id, ADDR_TORQUE_LIMIT_L);
//   int baud      = readByte(id, ADDR_BAUD_RATE);
//   int driveMode = readByte(id, ADDR_DRIVE_MODE); // reg 10: bit0=0→joint, bit0=1→wheel

//   if (cw < 0 || ccw < 0) return false;
//   outMin          = cw;
//   outMax          = ccw;
//   outTorqueLimit  = (torq >= 0) ? torq : 1023;
//   outCenterOffset = 0;

//   // Wheel mode: ADDR_DRIVE_MODE bit0=1, OR both limits are 0 (older firmware)
//   if (driveMode >= 0) {
//     outMode = (driveMode & 0x01) ? 1 : 0;
//   } else {
//     outMode = (cw == 0 && ccw == 0) ? 1 : 0; // fallback if reg unreadable
//   }

//   uint32_t realBaud = (baud >= 0) ? 2000000UL / ((uint32_t)baud + 1) : 57600;
//   outBaudIndex = DXL1_DEFAULT_BAUD_INDEX;
//   for (int i = 0; i < DXL1_BAUD_COUNT; ++i) {
//     uint32_t diff  = (realBaud > DXL1_BAUD_TABLE[i])
//                      ? realBaud - DXL1_BAUD_TABLE[i]
//                      : DXL1_BAUD_TABLE[i] - realBaud;
//     uint32_t dBest = (DXL1_BAUD_TABLE[outBaudIndex] > realBaud)
//                      ? DXL1_BAUD_TABLE[outBaudIndex] - realBaud
//                      : realBaud - DXL1_BAUD_TABLE[outBaudIndex];
//     if (diff < dBest) outBaudIndex = i;
//   }
//   return true;
// }

// // ---------------------------------------------------------------------------
// // EEPROM write helpers — 5 ms settle per write
// // ---------------------------------------------------------------------------

// // ---------------------------------------------------------------------------
// // EEPROM write helpers
// // The MX-28 needs ~5ms for EEPROM to complete, but at STATUS_RETURN_LEVEL=1
// // we have no acknowledgement to wait for. Use a generous 20ms settle to ensure
// // the previous write has finished before sending the next one — this is what
// // makes back-to-back writes (e.g. CW limit then CCW limit) reliable.
// // ---------------------------------------------------------------------------

// bool Dxl1Bus::eepromWriteByte(uint8_t id, uint8_t addr, uint8_t val) {
//   writeByte(id, addr, val);
//   delay(20); // generous EEPROM settle — no ack at STATUS_RETURN_LEVEL=1
//   return true;
// }

// bool Dxl1Bus::eepromWriteWord(uint8_t id, uint8_t addr, uint16_t val) {
//   writeWord(id, addr, val);
//   delay(20); // generous EEPROM settle
//   return true;
// }

// // ---------------------------------------------------------------------------
// // IServoBus — Configuration save
// // Torque off is called ONCE at the start (in saveMinMax, the first call).
// // ---------------------------------------------------------------------------

// bool Dxl1Bus::saveId(uint8_t currentId, uint8_t newId) {
//   torqueEnable(currentId, false);
//   delay(10);
//   return eepromWriteByte(currentId, ADDR_ID, newId);
// }

// bool Dxl1Bus::saveMinMax(uint8_t id, int minV, int maxV) {
//   torqueEnable(id, false); // first call in batch — disable torque once
//   delay(20);               // let torque-off settle before EEPROM access
//   eepromWriteWord(id, ADDR_CW_ANGLE_LIMIT_L,  (uint16_t)minV);
//   eepromWriteWord(id, ADDR_CCW_ANGLE_LIMIT_L, (uint16_t)maxV);
//   return true;
// }

// bool Dxl1Bus::saveTorqueLimit(uint8_t id, int limit) {
//   // Torque already off from saveMinMax
//   bool a = eepromWriteWord(id, ADDR_TORQUE_LIMIT_L,  (uint16_t)limit);
//   bool b = writeWord(id, ADDR_TORQUE_LIMIT_RAM, (uint16_t)limit); // RAM, no delay needed
//   return a && b;
// }

// bool Dxl1Bus::saveCenterOffset(uint8_t id, int offset) {
//   (void)id; (void)offset;
//   return true;
// }

// bool Dxl1Bus::saveMode(uint8_t id, int mode) {
//   if (mode != 1) return true; // joint: limits already set by saveMinMax
//   bool a = eepromWriteWord(id, ADDR_CW_ANGLE_LIMIT_L,  0);
//   bool b = eepromWriteWord(id, ADDR_CCW_ANGLE_LIMIT_L, 0);
//   return a && b;
// }

// bool Dxl1Bus::saveBaud(uint8_t id, int baudIndex) {
//   if (baudIndex < 0 || baudIndex >= DXL1_BAUD_COUNT) return false;
//   uint32_t baud    = DXL1_BAUD_TABLE[baudIndex];
//   uint8_t  dxlBaud = (uint8_t)((2000000UL / baud) - 1);
//   return eepromWriteByte(id, ADDR_BAUD_RATE, dxlBaud);
// }