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
// DXL1 (MX series) baud table — values are the ACTUAL baud rates the servo
// runs at, derived from the register formula: baud = 2000000 / (reg + 1)
// These differ from "standard" values — e.g. the servo's "57600" is actually
// 57142 baud (reg=34). Using the correct value avoids UART framing errors.
// Index: 0=~9600  1=~19200  2=~57142  3=100000  4=~117647  5=200000  6=250000  7=1000000
static constexpr int DXL1_BAUD_CNT = 8;
static constexpr uint32_t DXL1_BAUDS[DXL1_BAUD_CNT] = {
  9615, 19230, 57142, 100000, 117647, 200000, 250000, 1000000
};
static constexpr int DXL1_DEFAULT_BAUD_IDX = 2; // 57142 (~57600, reg=34)

// Dynamixel Protocol 2.0 baud table (XM/XH/XD series)
static constexpr int DXL2_BAUD_CNT = 8;
static constexpr uint32_t DXL2_BAUDS[DXL2_BAUD_CNT] = {
  9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000
};
static constexpr int DXL2_DEFAULT_BAUD_IDX = 1; // 57600 (XH/XM series factory default)

// Active bus protocol — persisted and used to select IServoBus* at boot
enum class BusProtocol : uint8_t {
  ST3215 = 0,
  DXL1   = 1,   // Dynamixel Protocol 1.0 (MX series RS-485)
  DXL2   = 2,   // Dynamixel Protocol 2.0 (XM/XH/XD series RS-485)
};
static constexpr int BUS_PROTOCOL_COUNT = 3;
static constexpr const char* BUS_PROTOCOL_NAMES[BUS_PROTOCOL_COUNT] = {
  "ST3215 / STS",
  "DXL MX (P1.0)",
  "DXL X  (P2.0)"
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

// ---------------------------------------------------------------------------
// USB Host input bindings
// Maps a USB host device event (axis, CC, button) to a servo or global param.
// Shares the same sink semantics as MidiServoBinding — each binding maps
// one source value (0-127 or scaled axis) to one servo or global slot.
// ---------------------------------------------------------------------------

// Source type for a UsbHostBinding
enum class UsbHostSrcType : uint8_t {
  MidiCC     = 0,  // CC from USB host MIDI device
  MidiNote   = 1,  // Note-on velocity from USB host MIDI device
  HidAxis    = 2,  // Signed axis from HID gamepad/SpaceMouse (-32768..32767)
  HidButton  = 3,  // Button from HID gamepad (0 or 1 → maps to min or max pos)
};

static constexpr int USB_HOST_MAX_BINDINGS = 32;

struct UsbHostBinding {
  // Source
  UsbHostSrcType srcType   = UsbHostSrcType::HidAxis;
  uint8_t        devIndex  = 0;    // which host device slot (0-based)
  uint8_t        srcIndex  = 0;    // axis index, CC#, or note#
  uint8_t        srcChannel = 1;   // MIDI channel (1-16); ignored for HID

  // Sink — same sentinel scheme as MidiServoBinding
  uint8_t  servoId   = 0xFF;       // servo ID, or MIDI_GLOBAL_* sentinel
  bool     inverted  = false;
  uint8_t  smoothing = 0;
  float    smoothPos = -1.0f;

  // Axis scaling: raw axis range → 0..127 before applying ccToPos
  int16_t  axisMin   = -32768;     // raw axis value → CC 0
  int16_t  axisMax   =  32767;     // raw axis value → CC 127
  int16_t  deadzone  = 512;        // ±deadzone around zero → no movement

  // Runtime state (not persisted)
  int8_t   lastRecv  = -1;
  int8_t   pendingVal = -1;        // -1 = nothing pending
  uint8_t  rxFlash   = 0;

  int      minLimit  = 0;
  int      maxLimit  = 4095;
};

// Extend MidiState to include host bindings and host-connected status
// NOTE: This is added as a separate parallel list, not replacing MidiServoBinding.
// The host bindings are processed in tickHostInput() alongside tickMidi().
struct UsbHostState {
  bool            active         = false; // enabled via MIDI Run or standalone
  UsbHostBinding  bindings[USB_HOST_MAX_BINDINGS];
  uint8_t         bindingCount   = 0;
  bool            devicePresent  = false;
  char            deviceName[12] = "---";
  uint32_t        rxNextMs       = 0;
};