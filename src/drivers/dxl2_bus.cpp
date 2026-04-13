#include "dxl2_bus.h"
#include "../config.h"

Dxl2Bus dxl2Bus;

// ---------------------------------------------------------------------------
// TX/RX blink — Green=TX, Red=RX (same as DXL1)
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
// CRC-16 IBM (poly 0x8005, init 0x0000) — as specified by Robotis P2.0 docs
// ---------------------------------------------------------------------------
static const uint16_t CRC_TABLE[256] = {
  0x0000,0x8005,0x800F,0x000A,0x801B,0x001E,0x0014,0x8011,
  0x8033,0x0036,0x003C,0x8039,0x0028,0x802D,0x8027,0x0022,
  0x8063,0x0066,0x006C,0x8069,0x0078,0x807D,0x8077,0x0072,
  0x0050,0x8055,0x805F,0x005A,0x804B,0x004E,0x0044,0x8041,
  0x80C3,0x00C6,0x00CC,0x80C9,0x00D8,0x80DD,0x80D7,0x00D2,
  0x00F0,0x80F5,0x80FF,0x00FA,0x80EB,0x00EE,0x00E4,0x80E1,
  0x00A0,0x80A5,0x80AF,0x00AA,0x80BB,0x00BE,0x00B4,0x80B1,
  0x8093,0x0096,0x009C,0x8099,0x0088,0x808D,0x8087,0x0082,
  0x8183,0x0186,0x018C,0x8189,0x0198,0x819D,0x8197,0x0192,
  0x01B0,0x81B5,0x81BF,0x01BA,0x81AB,0x01AE,0x01A4,0x81A1,
  0x01E0,0x81E5,0x81EF,0x01EA,0x81FB,0x01FE,0x01F4,0x81F1,
  0x81D3,0x01D6,0x01DC,0x81D9,0x01C8,0x81CD,0x81C7,0x01C2,
  0x0140,0x8145,0x814F,0x014A,0x815B,0x015E,0x0154,0x8151,
  0x8173,0x0176,0x017C,0x8179,0x0168,0x816D,0x8167,0x0162,
  0x8123,0x0126,0x012C,0x8129,0x0138,0x813D,0x8137,0x0132,
  0x0110,0x8115,0x811F,0x011A,0x810B,0x010E,0x0104,0x8101,
  0x8303,0x0306,0x030C,0x8309,0x0318,0x831D,0x8317,0x0312,
  0x0330,0x8335,0x833F,0x033A,0x832B,0x032E,0x0324,0x8321,
  0x0360,0x8365,0x836F,0x036A,0x837B,0x037E,0x0374,0x8371,
  0x8353,0x0356,0x035C,0x8359,0x0348,0x834D,0x8347,0x0342,
  0x03C0,0x83C5,0x83CF,0x03CA,0x83DB,0x03DE,0x03D4,0x83D1,
  0x83F3,0x03F6,0x03FC,0x83F9,0x03E8,0x83ED,0x83E7,0x03E2,
  0x83A3,0x03A6,0x03AC,0x83A9,0x03B8,0x83BD,0x83B7,0x03B2,
  0x0390,0x8395,0x839F,0x039A,0x838B,0x038E,0x0384,0x8381,
  0x0280,0x8285,0x828F,0x028A,0x829B,0x029E,0x0294,0x8291,
  0x82B3,0x02B6,0x02BC,0x82B9,0x02A8,0x82AD,0x82A7,0x02A2,
  // indices 208-255: corrected from Robotis official protocol.c
  0x82E3,0x02E6,0x02EC,0x82E9,0x02F8,0x82FD,0x82F7,0x02F2,
  0x02D0,0x82D5,0x82DF,0x02DA,0x82CB,0x02CE,0x02C4,0x82C1,
  0x8243,0x0246,0x024C,0x8249,0x0258,0x825D,0x8257,0x0252,
  0x0270,0x8275,0x827F,0x027A,0x826B,0x026E,0x0264,0x8261,
  0x0220,0x8225,0x822F,0x022A,0x823B,0x023E,0x0234,0x8231,
  0x8213,0x0216,0x021C,0x8219,0x0208,0x820D,0x8207,0x0202
};

uint16_t Dxl2Bus::crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0;
  for (uint16_t i = 0; i < len; ++i)
    crc = (crc << 8) ^ CRC_TABLE[((crc >> 8) ^ data[i]) & 0xFF];
  return crc;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void Dxl2Bus::begin(HardwareSerial& serial, uint32_t baud) {
  _serial = &serial;
  _baud   = baud;
  _serial->end();
  _serial->begin(baud);
  if (_dePin >= 0) { pinMode(_dePin, OUTPUT); digitalWrite(_dePin, LOW); }
  delay(5);
}

void Dxl2Bus::setBaud(uint32_t baud) {
  if (!_serial) return;
  _baud = baud;
  _serial->end();
  _serial->begin(baud);
  // Flush any stale bytes that were in the HW RX FIFO before end().
  // end() resets the SW ring buffer but the HW FIFO is re-read on begin().
  // Wait 2 byte-times at the new baud, then drain.
  uint32_t drainUs = (2UL * 10UL * 1000000UL) / baud + 200UL;
  delayMicroseconds(drainUs);
  while (_serial->available()) _serial->read();
  delay(2); // settle
}

// ---------------------------------------------------------------------------
// Direction control / flush
// ---------------------------------------------------------------------------

void Dxl2Bus::txBegin() {
  if (_dePin >= 0) digitalWrite(_dePin, HIGH);
}

void Dxl2Bus::txEnd() {
  if (!_serial) return;
  _serial->flush();
  // Post-TX guard: wait for the shift register to fully drain, then give the
  // auto-direction adapter time to switch back to RX.
  // flush() only guarantees the SW FIFO is empty — the HW shift register may
  // still be clocking out the last byte.  At 1Mbaud that's ~10us; at 57600 ~174us.
  // We wait 3 byte-times to cover both the shift register and the adapter's
  // direction-switching circuit (which triggers on TX pin going idle).
  // dePin adapters get an explicit LOW signal; auto-direction adapters rely on
  // TX going idle, which the guard delay ensures has actually happened.
  if (_baud > 0) {
    uint32_t guardUs = (3UL * 10UL * 1000000UL) / _baud; // 3 byte-times
    if (guardUs < 20) guardUs = 20; // minimum 20us regardless
    delayMicroseconds(guardUs);
  }
  if (_dePin >= 0) digitalWrite(_dePin, LOW);
}

void Dxl2Bus::flushRx() {
  if (!_serial) return;
  unsigned long t = micros();
  while (micros() - t < FLUSH_US)
    if (_serial->available()) _serial->read();
}

// ---------------------------------------------------------------------------
// Packet transmit
// P2.0 packet: FF FF FD 00  ID  LEN_L LEN_H  INS  [PARAMS...]  CRC_L CRC_H
// LEN = 1(INS) + paramLen + 2(CRC)
// CRC covers ALL bytes from 0xFF through last PARAM byte (entire packet minus CRC)
// ---------------------------------------------------------------------------

int Dxl2Bus::sendInstruction(uint8_t id, uint8_t ins,
                              const uint8_t* params, uint16_t paramLen) {
  if (!_serial) return 0;
  uint16_t len = paramLen + 3; // INS + PARAMS + 2 CRC bytes

  // Build full packet (minus CRC) for CRC calculation
  // Per Robotis spec: CRC covers everything from 0xFF 0xFF 0xFD 0x00 ... through last param
  uint8_t crcBuf[256];
  uint8_t ci = 0;
  crcBuf[ci++] = 0xFF;
  crcBuf[ci++] = 0xFF;
  crcBuf[ci++] = 0xFD;
  crcBuf[ci++] = 0x00;
  crcBuf[ci++] = id;
  crcBuf[ci++] = (uint8_t)(len & 0xFF);
  crcBuf[ci++] = (uint8_t)(len >> 8);
  crcBuf[ci++] = ins;
  for (uint16_t i = 0; i < paramLen && ci < sizeof(crcBuf)-2; ++i)
    crcBuf[ci++] = params[i];
  uint16_t crc = crc16(crcBuf, ci);

  flushRx();
  txBegin();
  for (uint8_t i = 0; i < ci; ++i) _serial->write(crcBuf[i]);
  _serial->write((uint8_t)(crc & 0xFF));
  _serial->write((uint8_t)(crc >> 8));
  _lastTxLen = ci + 2; // total bytes sent: packet + 2 CRC bytes
  txEnd();

  blinkTx();
  return (int)_lastTxLen;
}

// ---------------------------------------------------------------------------
// drainEcho — discard TX echo from auto-direction RS485 adapters.
// Window = (byteCount * 10bits * 1e6us) / baud + 200us.
// Ends before the servo response can arrive, safe at any baud rate.
// At 57600:  10-byte ping tx = 1736us, window = 1936us — echo fits, response safe.
// At 1Mbaud: 10-byte ping tx =  100us, window =  300us — no echo, window ends first.
// ---------------------------------------------------------------------------

void Dxl2Bus::drainEcho(uint8_t byteCount) {
  if (!_serial || _baud == 0) return;
  uint32_t windowUs = ((uint32_t)byteCount * 10UL * 1000000UL) / _baud + 200UL;
  uint8_t drained = 0;
  unsigned long start = micros();
  while (micros() - start < windowUs) {
    if (_serial->available()) {
      _serial->read();
      if (++drained >= byteCount) break;
    }
  }
}

// ---------------------------------------------------------------------------
// Packet receive
// P2.0 status: FF FF FD 00  ID  LEN_L LEN_H  0x55  ERR  [PARAMS...]  CRC_L CRC_H
// CRC covers ALL bytes from 0xFF through last PARAM (entire packet minus CRC)
// ---------------------------------------------------------------------------

int Dxl2Bus::recvStatus(uint8_t id, uint8_t* buf, uint8_t bufLen, uint8_t& errByte) {
  if (!_serial) return -1;
  // Echo must be drained by the caller before invoking recvStatus.

  enum State { H1, H2, H3, H4_RSRV, S_ID, LEN_L, LEN_H,
               INS, ERR, PARAMS, CRC_L, CRC_H };
  State state = H1;

  uint8_t rxId = 0, rxErr = 0, rxCrcL = 0;
  uint16_t rxLen = 0;
  uint16_t paramExpected = 0, paramCount = 0;
  uint8_t localBuf[64];

  // Accumulate all bytes for CRC (header through last param)
  uint8_t crcBuf[128];
  uint8_t ci = 0;

  unsigned long start = micros();
  while (micros() - start < RX_TIMEOUT_US) {
    if (!_serial->available()) continue;
    uint8_t b = (uint8_t)_serial->read();

    switch (state) {
      case H1:
        ci = 0;
        if (b == 0xFF) { crcBuf[ci++] = b; state = H2; }
        break;
      case H2:
        if (b == 0xFF) { crcBuf[ci++] = b; state = H3; }
        else { state = H1; }
        break;
      case H3:
        if (b == 0xFD) { crcBuf[ci++] = b; state = H4_RSRV; }
        else { state = H1; }
        break;
      case H4_RSRV:
        if (b == 0x00) { crcBuf[ci++] = b; state = S_ID; }
        else { state = H1; }
        break;
      case S_ID:
        rxId = b;
        if (rxId != id) { state = H1; break; }
        crcBuf[ci++] = b;
        state = LEN_L;
        break;
      case LEN_L:
        rxLen = b;
        crcBuf[ci++] = b;
        state = LEN_H;
        break;
      case LEN_H:
        rxLen |= ((uint16_t)b << 8);
        crcBuf[ci++] = b;
        if (rxLen < 4) { state = H1; break; }
        paramExpected = rxLen - 4; // LEN = INS(1)+ERR(1)+params+CRC(2)
        state = INS;
        break;
      case INS:
        crcBuf[ci++] = b;
        state = (b == INS_STATUS) ? ERR : H1;
        break;
      case ERR:
        rxErr = b;
        crcBuf[ci++] = b;
        paramCount = 0;
        state = (paramExpected > 0) ? PARAMS : CRC_L;
        break;
      case PARAMS:
        if (paramCount < sizeof(localBuf)) localBuf[paramCount] = b;
        if (ci < sizeof(crcBuf)) crcBuf[ci++] = b;
        if (++paramCount >= paramExpected) state = CRC_L;
        break;
      case CRC_L:
        rxCrcL = b; // don't add to crcBuf — CRC not included in CRC calculation
        state = CRC_H;
        break;
      case CRC_H: {
        uint16_t rxCrc   = (uint16_t)rxCrcL | ((uint16_t)b << 8);
        uint16_t calcCrc = crc16(crcBuf, ci);
        if (rxCrc != calcCrc) return -1;
        uint8_t toCopy = (paramCount < bufLen) ? paramCount : bufLen;
        for (uint8_t i = 0; i < toCopy; ++i) buf[i] = localBuf[i];
        errByte = rxErr;
        blinkRx();
        return (int)paramCount;
      }
    }
  }
  return -1; // timeout
}

// ---------------------------------------------------------------------------
// Read/write helpers
// ---------------------------------------------------------------------------

// Like recvStatus but accepts a status packet from any servo ID.
// Stores the received ID in _lastRxId.
int Dxl2Bus::recvStatusAny(uint8_t* buf, uint8_t bufLen, uint8_t& errByte) {
  if (!_serial) return -1;
  // Echo must be drained by the caller before invoking recvStatusAny.

  enum State { H1, H2, H3, H4_RSRV, S_ID, LEN_L, LEN_H,
               INS, ERR, PARAMS, CRC_L, CRC_H };
  State state = H1;

  uint8_t rxId = 0, rxErr = 0, rxCrcL = 0;
  uint16_t rxLen = 0, paramExpected = 0, paramCount = 0;
  uint8_t localBuf[64];
  uint8_t crcBuf[128];
  uint8_t ci = 0;

  unsigned long start = micros();
  while (micros() - start < RX_TIMEOUT_US) {
    if (!_serial->available()) continue;
    uint8_t b = (uint8_t)_serial->read();
    switch (state) {
      case H1: ci=0; if(b==0xFF){crcBuf[ci++]=b;state=H2;} break;
      case H2: if(b==0xFF){crcBuf[ci++]=b;state=H3;}else{state=H1;} break;
      case H3: if(b==0xFD){crcBuf[ci++]=b;state=H4_RSRV;}else{state=H1;} break;
      case H4_RSRV: if(b==0x00){crcBuf[ci++]=b;state=S_ID;}else{state=H1;} break;
      case S_ID: rxId=b; crcBuf[ci++]=b; state=LEN_L; break;
      case LEN_L: rxLen=b; crcBuf[ci++]=b; state=LEN_H; break;
      case LEN_H:
        rxLen|=((uint16_t)b<<8); crcBuf[ci++]=b;
        if(rxLen<4){state=H1;break;}
        paramExpected=rxLen-4; state=INS; break;
      case INS: crcBuf[ci++]=b; state=(b==INS_STATUS)?ERR:H1; break;
      case ERR: rxErr=b; crcBuf[ci++]=b; paramCount=0;
                state=(paramExpected>0)?PARAMS:CRC_L; break;
      case PARAMS:
        if(paramCount<sizeof(localBuf)) localBuf[paramCount]=b;
        if(ci<sizeof(crcBuf)) crcBuf[ci++]=b;
        if(++paramCount>=paramExpected) state=CRC_L; break;
      case CRC_L: rxCrcL=b; state=CRC_H; break;
      case CRC_H: {
        uint16_t rxCrc=(uint16_t)rxCrcL|((uint16_t)b<<8);
        uint16_t calcCrc=crc16(crcBuf,ci);
        if(rxCrc!=calcCrc) return -1;
        uint8_t toCopy=(paramCount<bufLen)?paramCount:bufLen;
        for(uint8_t i=0;i<toCopy;++i) buf[i]=localBuf[i];
        errByte=rxErr;
        _lastRxId=rxId;
        blinkRx();
        return (int)paramCount;
      }
    }
  }
  return -1;
}

int Dxl2Bus::readByte(uint8_t id, uint16_t addr) {
  uint8_t params[4] = {
    (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8),
    1, 0
  };
  int txLen = sendInstruction(id, INS_READ, params, 4);
  drainEcho(txLen);
  uint8_t buf[8]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  if (n < 1 || err != 0) return -1;
  return buf[0];
}

int Dxl2Bus::readWord(uint8_t id, uint16_t addr) {
  uint8_t params[4] = {
    (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8),
    2, 0
  };
  int txLen = sendInstruction(id, INS_READ, params, 4);
  drainEcho(txLen);
  uint8_t buf[8]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  if (n < 2 || err != 0) return -1;
  return (int)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

int32_t Dxl2Bus::readDword(uint8_t id, uint16_t addr) {
  uint8_t params[4] = {
    (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8),
    4, 0
  };
  int txLen = sendInstruction(id, INS_READ, params, 4);
  drainEcho(txLen);
  uint8_t buf[8]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  if (n < 4 || err != 0) return -1;
  return (int32_t)((uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24));
}

bool Dxl2Bus::writeByte(uint8_t id, uint16_t addr, uint8_t val) {
  uint8_t params[3] = { (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8), val };
  int txLen = sendInstruction(id, INS_WRITE, params, 3);
  drainEcho(txLen);
  unsigned long t = micros();
  while (micros() - t < 3000) {
    if (_serial->available()) {
      uint8_t buf[8]; uint8_t err = 0;
      int n = recvStatus(id, buf, sizeof(buf), err);
      return (n < 0) || (err == 0);
    }
  }
  return true; // no response = STATUS_RETURN_LEVEL 1, treat as success
}

bool Dxl2Bus::writeWord(uint8_t id, uint16_t addr, uint16_t val) {
  uint8_t params[4] = {
    (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8),
    (uint8_t)(val & 0xFF),  (uint8_t)(val >> 8)
  };
  int txLen = sendInstruction(id, INS_WRITE, params, 4);
  drainEcho(txLen);
  unsigned long t = micros();
  while (micros() - t < 3000) {
    if (_serial->available()) {
      uint8_t buf[8]; uint8_t err = 0;
      int n = recvStatus(id, buf, sizeof(buf), err);
      return (n < 0) || (err == 0);
    }
  }
  return true;
}

bool Dxl2Bus::writeDword(uint8_t id, uint16_t addr, uint32_t val) {
  uint8_t params[6] = {
    (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8),
    (uint8_t)(val & 0xFF),  (uint8_t)((val >> 8) & 0xFF),
    (uint8_t)((val >> 16) & 0xFF), (uint8_t)((val >> 24) & 0xFF)
  };
  int txLen = sendInstruction(id, INS_WRITE, params, 6);
  drainEcho(txLen);
  unsigned long t = micros();
  while (micros() - t < 3000) {
    if (_serial->available()) {
      uint8_t buf[8]; uint8_t err = 0;
      int n = recvStatus(id, buf, sizeof(buf), err);
      return (n < 0) || (err == 0);
    }
  }
  return true;
}

bool Dxl2Bus::eepromWriteByte(uint8_t id, uint16_t addr, uint8_t val) {
  writeByte(id, addr, val);
  delay(20);
  return true;
}
bool Dxl2Bus::eepromWriteWord(uint8_t id, uint16_t addr, uint16_t val) {
  writeWord(id, addr, val);
  delay(20);
  return true;
}
bool Dxl2Bus::eepromWriteDword(uint8_t id, uint16_t addr, uint32_t val) {
  writeDword(id, addr, val);
  delay(20);
  return true;
}

// ---------------------------------------------------------------------------
// IServoBus — Discovery
// ---------------------------------------------------------------------------

bool Dxl2Bus::ping(uint8_t id) {
  int txLen = sendInstruction(id, INS_PING, nullptr, 0);
  drainEcho(txLen);
  uint8_t buf[8]; uint8_t err;
  int n = recvStatus(id, buf, sizeof(buf), err);
  return (n >= 0 && err == 0);
}

// Broadcast ping: send to ID 0xFE, collect all responses.
// Returns number of servos found. Each responding servo sends model+firmware info.
// Per P2.0 spec, PING is one of the few instructions that gets status responses
// to the broadcast ID.
int Dxl2Bus::pingBroadcast(uint8_t* ids, int maxIds) {
  int txLen = sendInstruction(0xFE, INS_PING, nullptr, 0);
  drainEcho(txLen); // drain echo once, then collect multiple servo responses
  int count = 0;
  // Collect responses for up to BROADCAST_TIMEOUT_MS — each servo responds
  // with a small delay between packets (they use their ID as a backoff)
  unsigned long deadline = millis() + BROADCAST_TIMEOUT_MS;
  while (millis() < deadline && count < maxIds) {
    uint8_t buf[8]; uint8_t err = 0;
    int n = recvStatusAny(buf, sizeof(buf), err);
    if (n >= 0 && err == 0 && buf[0] != 0) {
      ids[count++] = _lastRxId;
    }
  }
  return count;
}

int Dxl2Bus::scan(uint8_t* ids, int maxIds, int& lastPingId) {
  // Try broadcast ping first — fast detection
  uint8_t bcIds[32];
  int bcCount = pingBroadcast(bcIds, 32);
  if (bcCount > 0) {
    // Found servos via broadcast — copy to output
    int n = (bcCount < maxIds) ? bcCount : maxIds;
    for (int i = 0; i < n; ++i) ids[i] = bcIds[i];
    lastPingId = 253; // mark as complete
    return n;
  }
  // Broadcast found nothing — fall back to brute-force sweep
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

bool Dxl2Bus::setPosition(uint8_t id, int pos, uint16_t speed, uint8_t acc) {
  // Profile velocity (speed): DXL2 unit is 0.229 RPM
  // We map our 0-4095 speed range linearly to 0-1023 rev/min
  // Profile accel: unit 214.577 rev/min²; map acc 0-254 to 0-100 profile units
  uint32_t profVel  = (uint32_t)speed * 1023 / 4095;
  uint32_t profAcc  = (uint32_t)acc;
  writeDword(id, ADDR_PROFILE_ACCEL, profAcc);
  writeDword(id, ADDR_PROFILE_VEL,   profVel);
  // Goal position: 4 bytes
  return writeDword(id, ADDR_GOAL_POSITION, (uint32_t)(uint16_t)pos);
}

int Dxl2Bus::readPosition(uint8_t id) {
  int32_t v = readDword(id, ADDR_PRESENT_POSITION);
  if (v < 0) return -1;
  // Clamp to 0-4095 for display (extended position can be larger)
  if (v > 4095) v = 4095;
  if (v < 0)    v = 0;
  return (int)v;
}

bool Dxl2Bus::torqueEnable(uint8_t id, bool en) {
  return writeByte(id, ADDR_TORQUE_ENABLE, en ? 1 : 0);
}

// ---------------------------------------------------------------------------
// IServoBus — Telemetry
// ---------------------------------------------------------------------------

bool Dxl2Bus::readVoltage(uint8_t id, int& mv) {
  // Present voltage: unit 0.1V, so multiply by 100 to get mV
  int v = readWord(id, ADDR_PRESENT_VOLTAGE);
  if (v < 0) return false;
  mv = v * 100;
  return true;
}

bool Dxl2Bus::readTemperature(uint8_t id, int& tempC) {
  int v = readByte(id, ADDR_PRESENT_TEMP);
  if (v < 0) return false;
  tempC = v;
  return true;
}

bool Dxl2Bus::readStatus(uint8_t id, int& statusByte) {
  // Hardware error status byte at addr 70
  int v = readByte(id, ADDR_HARDWARE_ERROR);
  if (v < 0) return false;
  // DXL2 hardware error bits:
  // Bit 0: Input Voltage Error
  // Bit 2: Overheating Error
  // Bit 3: Motor Encoder Error
  // Bit 4: Electrical Shock Error
  // Bit 5: Overload Error
  // Map to our UI bit layout: 0=Volt, 1=Sensor, 2=Temp, 3=Current, 4=Angle, 5=Overload
  uint8_t mapped = 0;
  if (v & 0x01) mapped |= (1 << 0); // Voltage
  if (v & 0x08) mapped |= (1 << 1); // Encoder → Sensor slot
  if (v & 0x04) mapped |= (1 << 2); // Overheating
  if (v & 0x10) mapped |= (1 << 3); // Electrical shock → Current slot
  if (v & 0x20) mapped |= (1 << 5); // Overload
  statusByte = mapped;
  return true;
}

bool Dxl2Bus::readLoad(uint8_t id, int& loadPct) {
  // Present PWM: -885..885 (signed 16-bit), proportional to load
  int v = readWord(id, ADDR_PRESENT_PWM);
  if (v < 0) return false;
  int16_t sv = (int16_t)(uint16_t)v;
  loadPct = (int)sv * 100 / 885;
  return true;
}

bool Dxl2Bus::readCurrent(uint8_t id, int& currentMa) {
  // Present current: signed 16-bit, unit 2.69 mA
  int v = readWord(id, ADDR_PRESENT_CURRENT);
  if (v < 0) { currentMa = -1; return false; }
  int16_t sv = (int16_t)(uint16_t)v;
  currentMa = abs((int)sv) * 269 / 100; // 2.69 mA/unit
  return true;
}

// ---------------------------------------------------------------------------
// IServoBus — Configuration load
// ---------------------------------------------------------------------------

bool Dxl2Bus::loadConfig(uint8_t id, uint8_t& outId, int& outMin, int& outMax,
                          int& outTorqueLimit, int& outCenterOffset,
                          int& outMode, int& outBaudIndex) {
  if (!ping(id)) return false;
  outId = id;

  int32_t minPos = readDword(id, ADDR_MIN_POS_LIMIT);
  int32_t maxPos = readDword(id, ADDR_MAX_POS_LIMIT);
  int     opMode = readByte(id, ADDR_OP_MODE);
  int     baud   = readByte(id, ADDR_BAUD_RATE);
  int32_t offset = readDword(id, ADDR_HOMING_OFFSET);
  int     curLim = readWord(id, ADDR_CURRENT_LIMIT);

  if (minPos < 0 || maxPos < 0) return false;

  outMin          = (int)minPos;
  outMax          = (int)maxPos;
  // Current limit: unit 2.69mA; map to 0-1000 range (1000 = full / CURRENT_LIMIT default ~1193)
  outTorqueLimit  = (curLim >= 0) ? (int)((long)curLim * 1000 / 1193) : 1000;
  outCenterOffset = (offset >= 0) ? (int)offset : 0;
  // Operating mode: 3=position, 1=velocity(≈wheel), 4=ext-pos, 16=multi-turn
  outMode         = (opMode == 1 || opMode == 0) ? 1 : 0; // 1=wheel/vel, 0=joint/pos
  // Baud: register value IS the table index directly
  outBaudIndex    = (baud >= 0 && baud < DXL2_BAUD_COUNT) ? baud : DXL2_DEFAULT_BAUD_INDEX;
  return true;
}

// ---------------------------------------------------------------------------
// IServoBus — Configuration save
// ---------------------------------------------------------------------------

bool Dxl2Bus::saveId(uint8_t currentId, uint8_t newId) {
  torqueEnable(currentId, false);
  delay(10);
  return eepromWriteByte(currentId, ADDR_ID, newId);
}

bool Dxl2Bus::saveMinMax(uint8_t id, int minV, int maxV) {
  torqueEnable(id, false);
  delay(10);
  bool a = eepromWriteDword(id, ADDR_MIN_POS_LIMIT, (uint32_t)(int32_t)minV);
  bool b = eepromWriteDword(id, ADDR_MAX_POS_LIMIT, (uint32_t)(int32_t)maxV);
  return a && b;
}

bool Dxl2Bus::saveTorqueLimit(uint8_t id, int limit) {
  // Map 0-1000 back to current limit unit (2.69 mA); default max = 1193 units
  uint16_t curLim = (uint16_t)((long)limit * 1193 / 1000);
  return eepromWriteWord(id, ADDR_CURRENT_LIMIT, curLim);
}

bool Dxl2Bus::saveCenterOffset(uint8_t id, int offset) {
  return eepromWriteDword(id, ADDR_HOMING_OFFSET, (uint32_t)(int32_t)offset);
}

bool Dxl2Bus::saveMode(uint8_t id, int mode) {
  // mode 0 = joint/position → op mode 3 (Position Control)
  // mode 1 = wheel/velocity → op mode 1 (Velocity Control)
  uint8_t opMode = (mode == 1) ? 1 : 3;
  return eepromWriteByte(id, ADDR_OP_MODE, opMode);
}

bool Dxl2Bus::saveBaud(uint8_t id, int baudIndex) {
  if (baudIndex < 0 || baudIndex >= DXL2_BAUD_COUNT) return false;
  // DXL2: baud register value IS the table index (0-7)
  return eepromWriteByte(id, ADDR_BAUD_RATE, (uint8_t)baudIndex);
}

// ---------------------------------------------------------------------------
// Debug: send a raw PING and print every received byte to a Print stream.
// Bypasses all packet parsing — shows exactly what the servo sends back.
// Call from serial terminal with 'p' command in main.cpp loop().
// ---------------------------------------------------------------------------
void Dxl2Bus::rawPingDump(uint8_t id, Print& out) {
  if (!_serial) { out.println(F("ERR: no serial")); return; }

  // Build ping packet
  uint8_t pkt[16];
  uint8_t pi = 0;
  uint16_t len = 3;
  pkt[pi++] = 0xFF; pkt[pi++] = 0xFF;
  pkt[pi++] = 0xFD; pkt[pi++] = 0x00;
  pkt[pi++] = id;
  pkt[pi++] = (uint8_t)(len & 0xFF);
  pkt[pi++] = (uint8_t)(len >> 8);
  pkt[pi++] = INS_PING;
  uint16_t crc = crc16(pkt, pi);
  pkt[pi++] = (uint8_t)(crc & 0xFF);
  pkt[pi++] = (uint8_t)(crc >> 8);
  uint8_t txLen = pi; // 10 bytes

  out.print(F("TX: "));
  for (uint8_t i = 0; i < txLen; ++i) {
    if (pkt[i] < 0x10) out.print('0');
    out.print(pkt[i], HEX); out.print(' ');
  }
  out.println();

  // Drain anything in RX buffer — timed drain to catch any stale bytes
  // that may still be arriving after a recent setBaud() call.
  // A fixed 5ms window is generous enough for any baud rate.
  { unsigned long t = millis();
    while (millis() - t < 5) { if (_serial->available()) _serial->read(); } }

  // Send packet
  txBegin();
  for (uint8_t i = 0; i < txLen; ++i) _serial->write(pkt[i]);
  // Wait for all bytes to physically leave the shift register
  // At 57600 baud: 10 bytes * (1/57600) * 10 bits = 1.736ms
  // flush() waits for FIFO empty but NOT shift register empty,
  // so add a guard to ensure the adapter has switched back to RX.
  _serial->flush();
  delayMicroseconds(300); // extra guard for adapter RX direction switch
  txEnd();

  // Buffer ALL bytes with microsecond timestamps first — do NOT print inside
  // the receive loop.  At 1Mbaud, 14 response bytes arrive in 140us.  Each
  // Serial.print() at 115200 baud blocks for ~80us, so printing inside the
  // loop causes us to miss bytes.  Capture everything first, then print.
  static const int RXBUF = 64;
  uint8_t  rxBuf[RXBUF];
  uint32_t rxUs[RXBUF];
  int total = 0;

  unsigned long t0us = micros();
  unsigned long t0ms = millis();
  while (millis() - t0ms < 100 && total < RXBUF) {
    if (_serial->available()) {
      rxBuf[total] = (uint8_t)_serial->read();
      rxUs[total]  = (uint32_t)(micros() - t0us);
      ++total;
    }
  }

  out.println(F("All RX bytes (format: [t_us] HH):"));
  for (int i = 0; i < total; ++i) {
    out.print('['); out.print(rxUs[i]); out.print(F("us] "));
    if (rxBuf[i] < 0x10) out.print('0');
    out.print(rxBuf[i], HEX);
    out.print(' ');
  }
  if (total == 0) {
    out.println(F("(nothing at all received in 100ms)"));
    out.println(F("Check: power to servo? RS485 wiring? Servo baud matches?"));
  } else {
    out.println();
    out.print(F("Total: ")); out.print(total); out.println(F(" bytes"));
    out.println(F("Expected: ~4 garbled echo bytes at <100us, then 14 response bytes at ~500-700us"));
  }
}


void Dxl2Bus::rawBroadcastDump(Print& out) {
  if (!_serial) { out.println(F("ERR: no serial")); return; }

  // Send broadcast ping
  uint8_t crcBuf[16];
  uint8_t ci = 0;
  uint16_t len = 3;
  crcBuf[ci++] = 0xFF; crcBuf[ci++] = 0xFF;
  crcBuf[ci++] = 0xFD; crcBuf[ci++] = 0x00;
  crcBuf[ci++] = 0xFE; // broadcast ID
  crcBuf[ci++] = (uint8_t)(len & 0xFF);
  crcBuf[ci++] = (uint8_t)(len >> 8);
  crcBuf[ci++] = INS_PING;
  uint16_t crc = crc16(crcBuf, ci);
  uint8_t crcL = crc & 0xFF, crcH = crc >> 8;

  out.print(F("TX broadcast: "));
  for (uint8_t i = 0; i < ci; ++i) {
    if (crcBuf[i] < 0x10) out.print('0');
    out.print(crcBuf[i], HEX); out.print(' ');
  }
  if (crcL < 0x10) out.print('0'); out.print(crcL, HEX); out.print(' ');
  if (crcH < 0x10) out.print('0'); out.print(crcH, HEX);
  out.println();

  flushRx();
  txBegin();
  for (uint8_t i = 0; i < ci; ++i) _serial->write(crcBuf[i]);
  _serial->write(crcL);
  _serial->write(crcH);
  txEnd();

  // Collect raw bytes for 500ms
  out.print(F("RX raw (500ms): "));
  unsigned long t = millis();
  int count = 0;
  while (millis() - t < 500) {
    if (_serial->available()) {
      uint8_t b = (uint8_t)_serial->read();
      if (b < 0x10) out.print('0');
      out.print(b, HEX); out.print(' ');
      ++count;
    }
  }
  if (count == 0) out.print(F("(nothing)"));
  out.println();
  out.print(F("Total: ")); out.println(count);
}