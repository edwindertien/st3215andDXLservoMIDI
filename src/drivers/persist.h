#pragma once
#include <Arduino.h>
#include "../model/servo_model.h"
#include "../config.h"

static constexpr uint32_t PERSIST_MAGIC   = 0x53543332;
static constexpr uint8_t  PERSIST_VERSION = 8;  // bumped: added DXL2 protocol
static constexpr char     PERSIST_PATH[]  = "/config.bin";

struct PersistedMidiBinding {
  uint8_t servoId  = 0xFF;
  uint8_t cc       = MIDI_CC_NONE;
  uint8_t channel  = 1;
  bool    inverted = false;
  uint8_t smoothing = 0;
};

struct PersistentConfig {
  uint32_t magic         = PERSIST_MAGIC;
  uint8_t  version       = PERSIST_VERSION;
  BusProtocol protocol   = BusProtocol::ST3215;  // selected servo protocol
  uint8_t  servoCount    = 0;
  uint8_t  ids[ST3215::MAX_SCAN_IDS] = {0};
  uint8_t  activeIndex   = 0;
  uint8_t  scanBaudIndex = 0;
  bool     torqueEnabled = true;
  uint16_t speed         = ST3215::DEFAULT_SPEED;
  uint8_t  acc           = ST3215::DEFAULT_ACC;
  uint8_t  midiChannel   = 1;
  uint8_t  midiCount     = 0;
  PersistedMidiBinding midiBindings[MIDI_MAX_SERVOS];
  // Global CC mappings
  uint8_t  speedCC       = MIDI_CC_NONE;
  uint8_t  speedChannel  = 1;
  uint8_t  accCC         = MIDI_CC_NONE;
  uint8_t  accChannel    = 1;
  uint8_t  smoothCC      = MIDI_CC_NONE;
  uint8_t  smoothChannel = 1;
};

struct PersistDiag {
  bool     mounted          = false;
  uint32_t totalBytes       = 0;
  uint32_t usedBytes        = 0;
  bool     fileExists       = false;
  uint32_t fileSize         = 0;
  uint32_t magic            = 0;
  uint8_t  version          = 0;
  uint8_t  savedServoCount  = 0;
  uint8_t  savedMidiCount   = 0;
  uint32_t expectedSize     = 0;
  uint32_t expectedMagic    = 0;
  uint8_t  expectedVersion  = 0;
};

class Persist {
public:
  bool begin();
  bool load(PersistentConfig& dst);
  bool save(const PersistentConfig& cfg);
  PersistDiag diagnose();
  bool isMounted() const { return _mounted; }
private:
  bool _mounted = false;
};

extern Persist persist;