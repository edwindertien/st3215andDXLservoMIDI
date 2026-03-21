#pragma once
#include <Arduino.h>

enum class ScreenId : uint8_t {
  Home, Scan, SelectServo, LiveControl, ServoInfo,
  ConfigMenu, ConfirmSave, SaveResult,
  ScanBaudSelect, ScanAllBaud, ModeWarn,
  MidiSetup, MidiRun,
  PersistDiag,
  SelectProtocol   // choose between ST3215, DXL1, etc.
};

static constexpr int BAUD_TABLE_COUNT = 8;
static constexpr uint32_t BAUD_TABLE[BAUD_TABLE_COUNT] = {
  1000000UL, 500000UL, 250000UL, 128000UL,
  115200UL,   76800UL,  57600UL,  38400UL
};

// Dynamixel Protocol 1.0 baud table (MX series)
static constexpr int DXL1_BAUD_CNT = 8;
static constexpr uint32_t DXL1_BAUDS[DXL1_BAUD_CNT] = {
  9600, 19200, 57600, 115200, 200000, 250000, 400000, 1000000
};
static constexpr int DXL1_DEFAULT_BAUD_IDX = 2; // 57600

// Active bus protocol — persisted and used to select IServoBus* at boot
enum class BusProtocol : uint8_t {
  ST3215 = 0,
  DXL1   = 1,   // Dynamixel Protocol 1.0 (MX series RS-485)
  // DXL2, AK60 reserved for future phases
};
static constexpr int BUS_PROTOCOL_COUNT = 2;
static constexpr const char* BUS_PROTOCOL_NAMES[BUS_PROTOCOL_COUNT] = {
  "ST3215 / STS",
  "DXL MX (P1.0)"
};

enum class MenuItemType : uint8_t { Action, Toggle, Integer, Choice, Back };

enum class ServoParam : uint8_t {
  None, Position, Speed, Acceleration,
  TorqueEnable, TorqueLimit, ServoId, BaudRate,
  MinLimit, MaxLimit, CenterOffset, Mode
};

struct ServoConfigBuffer {
  bool    loaded       = false;
  bool    dirty        = false;
  uint8_t servoId      = 1;
  int     baudIndex    = 0;
  int     minLimit     = 0;
  int     maxLimit     = 4095;
  int     centerOffset = 0;
  int     torqueLimit  = 1000;
  int     mode         = 0;
};

// ---------------------------------------------------------------------------
// MIDI
// ---------------------------------------------------------------------------
static constexpr uint8_t  MIDI_CC_NONE        = 255;
static constexpr int      MIDI_MAX_SERVOS      = 32;
static constexpr uint32_t MIDI_TX_INTERVAL_MS  = 25;
static constexpr uint32_t MIDI_RX_INTERVAL_MS  = 25;
static constexpr int      MIDI_LOG_SIZE        = 4;

// Sentinel servoId values for global parameter bindings
// Real servo IDs are 0..253; 0xFC–0xFE are reserved
static constexpr uint8_t MIDI_GLOBAL_SPEED  = 0xFE;
static constexpr uint8_t MIDI_GLOBAL_ACC    = 0xFD;
static constexpr uint8_t MIDI_GLOBAL_SMOOTH = 0xFC; // global smoothing amount

// MIDI message types for the monitor log
enum class MidiMsgType : uint8_t {
  CC = 0, NoteOn, NoteOff, PitchBend, ProgramChange, AfterTouch, Other
};

struct MidiServoBinding {
  uint8_t servoId   = 0xFF;
  uint8_t cc        = MIDI_CC_NONE;
  uint8_t channel   = 1;
  bool    inverted  = false;   // if true: CC 0→maxPos, CC 127→minPos
  uint8_t smoothing = 0;       // 0=no filter, 127=max (IIR alpha = (128-s)/128)
  float   smoothPos = -1.0f;   // filtered target position (-1 = uninitialised)
  int8_t  lastSent  = -1;
  int8_t  lastRecv  = -1;
  int8_t  pendingCC = -1;
  uint8_t txFlash   = 0;
  uint8_t rxFlash   = 0;
  int     minLimit  = 0;
  int     maxLimit  = 4095;
};

// Extended MIDI monitor log entry — covers CC, notes, pitch bend, etc.
struct MidiLogEntry {
  MidiMsgType type    = MidiMsgType::CC;
  uint8_t channel     = 0;
  uint8_t byte1       = 0;   // CC#, note number, program number, etc.
  uint8_t byte2       = 0;   // CC value, note velocity, aftertouch value, etc.
  int16_t int14       = 0;   // for pitch bend (-8192..+8191)
  bool    valid       = false;
};

struct MidiState {
  bool    active       = false;
  bool    panic        = false;
  uint8_t channel      = 1;
  // Per-servo bindings (0..bindingCount-1) followed by global param bindings:
  //   [bindingCount]   = Speed     (servoId = MIDI_GLOBAL_SPEED)
  //   [bindingCount+1] = Acc       (servoId = MIDI_GLOBAL_ACC)
  //   [bindingCount+2] = Smoothing (servoId = MIDI_GLOBAL_SMOOTH)
  // totalBindings = bindingCount + 3
  MidiServoBinding bindings[MIDI_MAX_SERVOS + 3];
  uint8_t bindingCount  = 0;
  int     setupCursor   = 0;
  int     runCursor     = 0;
  bool    editingCC     = false;
  bool    editingCh     = false;
  bool    editingInv    = false;  // editing invert flag
  bool    editingSmooth = false;  // editing smoothing value
  bool    showMonitor   = false;
  uint8_t  txServoIndex = 0;
  uint32_t txNextMs     = 0;
  uint32_t rxNextMs     = 0;
  MidiLogEntry log[MIDI_LOG_SIZE];
  uint8_t      logHead  = 0;
};

// Total editable bindings: servo rows + Speed + Acc + Smoothing
inline int midiTotalBindings(const MidiState& m) {
  return (int)m.bindingCount + 3;
}
