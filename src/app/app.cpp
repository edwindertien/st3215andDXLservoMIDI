#include "app.h"
#include "../drivers/persist.h"
#include "../drivers/dxl1_bus.h"
#include <stdio.h>
#include <string.h>

// Defined in main.cpp — returns the concrete IServoBus* for a given protocol
extern IServoBus* busForProtocol(BusProtocol p);

namespace {

const char* HOME_ITEMS[] = {
    "Scan Bus",
    "Select Servo",
    "Live Control",
    "Servo Info",
    "Configure",
    "Save Changes",
    "MIDI Mode",
    "Flash Diag",
    "Protocol"       // select ST3215 / DXL MX / etc.
};
constexpr int HOME_COUNT = sizeof(HOME_ITEMS) / sizeof(HOME_ITEMS[0]);

constexpr int SCAN_BAUD_ITEM_COUNT = BAUD_TABLE_COUNT + 1;
constexpr int CONFIG_ITEM_COUNT    = 7;

// Flash duration in render() cycles for MIDI activity indicators
constexpr uint8_t MIDI_FLASH_FRAMES = 8;

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

} // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

App::App(AppState& state, UnitEncoder& encoder, OledUi& ui,
         IServoBus* bus, MidiEngine& midi)
    : _app(state), _encoder(encoder), _ui(ui), _bus(bus), _midi(midi) {}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void App::begin() {
  // Try to restore persisted state; fall back to a fresh bus scan
  loadPersistedState();

  _app.screen    = ScreenId::Home;
  _app.menuIndex = 0;
  _app.editing   = false;
}

void App::tick() {
  handleInput();
  updateFeedback();
  if (_app.midi.active) tickMidi();

  // Lazy persist write — only when something changed, not every frame
  if (_persistDirty) {
    savePersistedState();
    _persistDirty = false;
  }

  render();
  _midi.tick();
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------

void App::markDirty() {
  _persistDirty = true;
}

void App::loadPersistedState() {
  PersistentConfig cfg;
  bool loaded = persist.load(cfg);

  if (loaded) {
    // Restore protocol first so activeBaudTable() works correctly
    _app.protocol = cfg.protocol;
    setBus(busForProtocol(_app.protocol));
  }

  if (loaded && cfg.servoCount > 0) {
    // Restore servo list without scanning
    _app.servoCount    = cfg.servoCount;
    _app.activeIndex   = cfg.activeIndex;
    _app.scanBaudIndex = cfg.scanBaudIndex;
    for (uint8_t i = 0; i < cfg.servoCount; ++i)
      _app.ids[i] = cfg.ids[i];

    // Restore motion params
    _app.torqueEnabled = cfg.torqueEnabled;
    _app.speed         = cfg.speed;
    _app.acc           = cfg.acc;

    // Restore MIDI global channel
    _app.midi.channel = cfg.midiChannel;

    if (_app.oledOk)
      _ui.splash("Restored", String(cfg.servoCount) + " servo(s)", "Short=scan");

    // Give the servos time to power up, then load runtime state
    delay(300);
    { int cnt; const uint32_t* tbl = activeBaudTable(cnt);
      _bus->setBaud(tbl[clampInt(_app.scanBaudIndex, 0, cnt - 1)]); }

    if (hasActiveServo()) {
      loadActiveServoRuntime();
      loadStagedConfigFromActive();

      // Apply stored speed/acc/torque to the active servo
      if (_app.torqueEnabled)
        _bus->setPosition(activeServoId(), _app.actualPos >= 0 ? _app.actualPos : _bus->posMid(),
                         _app.speed, _app.acc);
      _bus->torqueEnable(activeServoId(), _app.torqueEnabled);
    }

    // Restore MIDI bindings
    _app.midi.bindingCount = 0;
    for (uint8_t i = 0; i < cfg.midiCount && i < MIDI_MAX_SERVOS; ++i) {
      MidiServoBinding b;
      b.servoId   = cfg.midiBindings[i].servoId;
      b.cc        = cfg.midiBindings[i].cc;
      b.channel   = cfg.midiBindings[i].channel;
      b.inverted  = cfg.midiBindings[i].inverted;
      b.smoothing = cfg.midiBindings[i].smoothing;
      b.smoothPos = -1.0f;
      b.lastSent = -1; b.lastRecv = -1; b.pendingCC = -1;
      b.txFlash  = 0;  b.rxFlash  = 0;
      uint8_t dummyId; int dummyTorq, dummyOfs, dummyMode, dummyBaud;
      int rMin = 0, rMax = 4095;
      _bus->loadConfig(b.servoId, dummyId, rMin, rMax,
                      dummyTorq, dummyOfs, dummyMode, dummyBaud);
      b.minLimit = rMin;
      b.maxLimit = rMax;
      _app.midi.bindings[_app.midi.bindingCount++] = b;
    }

    // Restore global Speed / Acc / Smooth CC bindings
    {
      auto& spd = _app.midi.bindings[_app.midi.bindingCount];
      spd = {}; spd.servoId = MIDI_GLOBAL_SPEED;
      spd.cc = cfg.speedCC; spd.channel = cfg.speedChannel;

      auto& acc = _app.midi.bindings[_app.midi.bindingCount + 1];
      acc = {}; acc.servoId = MIDI_GLOBAL_ACC;
      acc.cc = cfg.accCC; acc.channel = cfg.accChannel;

      auto& smt = _app.midi.bindings[_app.midi.bindingCount + 2];
      smt = {}; smt.servoId = MIDI_GLOBAL_SMOOTH;
      smt.cc = cfg.smoothCC; smt.channel = cfg.smoothChannel;
    }
  } else {
    // No valid saved state — scan the bus as before
    if (_app.oledOk) _ui.splash("Scanning bus...");
    scanBusWithUi();
    loadActiveServoRuntime();
    loadStagedConfigFromActive();
  }
}

void App::savePersistedState() {
  PersistentConfig cfg;

  cfg.servoCount    = _app.servoCount;
  cfg.activeIndex   = (uint8_t)_app.activeIndex;
  cfg.scanBaudIndex = (uint8_t)_app.scanBaudIndex;
  for (uint8_t i = 0; i < _app.servoCount; ++i)
    cfg.ids[i] = _app.ids[i];

  cfg.protocol      = _app.protocol;
  cfg.torqueEnabled = _app.torqueEnabled;
  cfg.speed         = (uint16_t)_app.speed;
  cfg.acc           = (uint8_t)_app.acc;

  cfg.midiChannel = _app.midi.channel;
  cfg.midiCount   = _app.midi.bindingCount;
  for (uint8_t i = 0; i < _app.midi.bindingCount; ++i) {
    cfg.midiBindings[i].servoId   = _app.midi.bindings[i].servoId;
    cfg.midiBindings[i].cc        = _app.midi.bindings[i].cc;
    cfg.midiBindings[i].channel   = _app.midi.bindings[i].channel;
    cfg.midiBindings[i].inverted  = _app.midi.bindings[i].inverted;
    cfg.midiBindings[i].smoothing = _app.midi.bindings[i].smoothing;
  }
  {
    const MidiServoBinding& spd = _app.midi.bindings[_app.midi.bindingCount];
    cfg.speedCC      = spd.cc;
    cfg.speedChannel = spd.channel;
    const MidiServoBinding& acc = _app.midi.bindings[_app.midi.bindingCount + 1];
    cfg.accCC        = acc.cc;
    cfg.accChannel   = acc.channel;
    const MidiServoBinding& smt = _app.midi.bindings[_app.midi.bindingCount + 2];
    cfg.smoothCC      = smt.cc;
    cfg.smoothChannel = smt.channel;
  }

  persist.save(cfg);
}

// ---------------------------------------------------------------------------
// Protocol helpers
// ---------------------------------------------------------------------------

void App::setBus(IServoBus* bus) {
  _bus = bus;
  // Reset servo state — caller must trigger a scan after switching
  _app.servoCount  = 0;
  _app.activeIndex = 0;
  _app.targetPos   = _bus->posMid();
  _app.actualPos   = -1;
  _servoVoltageMv  = -1;
  _servoTempC      = -1;
  _servoOnline     = false;
}

// Returns the baud table for the active protocol
const uint32_t* App::activeBaudTable(int& count) const {
  switch (_app.protocol) {
    case BusProtocol::DXL1:
      count = DXL1_BAUD_CNT;
      return DXL1_BAUDS;
    case BusProtocol::ST3215:
    default:
      count = BAUD_TABLE_COUNT;
      return BAUD_TABLE;
  }
}

void App::enterScreen(ScreenId screen, int menuIndex, bool editing) {
  _app.screen    = screen;
  _app.menuIndex = menuIndex;
  _app.editing   = editing;
  _encoder.clearEvents();
}

// ---------------------------------------------------------------------------
// Servo helpers
// ---------------------------------------------------------------------------

bool App::hasActiveServo() const {
  return _app.servoCount > 0 &&
         _app.activeIndex >= 0 &&
         _app.activeIndex < _app.servoCount;
}

uint8_t App::activeServoId() const {
  if (!hasActiveServo()) return 0xFF;
  return _app.ids[_app.activeIndex];
}

// ---------------------------------------------------------------------------
// Scanning
// ---------------------------------------------------------------------------

void App::scanBus() {
  _app.lastPingId  = -1;
  _app.servoCount  = _bus->scan(_app.ids, ST3215::MAX_SCAN_IDS, _app.lastPingId);
  _app.activeIndex = 0;
  if (hasActiveServo()) {
    loadActiveServoRuntime();
    loadStagedConfigFromActive();
  } else {
    _app.targetPos  = _bus->posMid();
    _app.actualPos  = -1;
    _servoVoltageMv = -1;
    _servoTempC     = -1;
    _servoOnline    = false;
  }
}

void App::scanBusWithUi() {
  _app.lastPingId = -1;
  _app.servoCount = 0;
  _app.activeIndex = 0;

  int baudCnt; const uint32_t* baudTbl = activeBaudTable(baudCnt);
  uint32_t baud = baudTbl[clampInt(_app.scanBaudIndex, 0, baudCnt - 1)];
  _bus->setBaud(baud);

  if (_app.oledOk) _ui.drawScanProgress(baud, 0, 0, 253);

  unsigned long pressStart = 0;
  bool aborted = false;

  for (int id = 0; id <= 253; ++id) {
    _app.lastPingId = id;
    if (_bus->ping((uint8_t)id) && _app.servoCount < ST3215::MAX_SCAN_IDS) {
      _app.ids[_app.servoCount++] = (uint8_t)id;
    }
    if (_app.oledOk) _ui.drawScanProgress(baud, id, _app.servoCount, 253);

    // Poll encoder for long-press abort
    _encoder.update();
    if (_encoder.isPressed()) {
      if (pressStart == 0) pressStart = millis();
      if (millis() - pressStart >= 700UL) {
        aborted = true;
        _encoder.clearEvents();
        break;
      }
    } else {
      pressStart = 0;
    }

    delay(2);
  }

  if (aborted && _app.oledOk)
    _ui.splash("Scan stopped", String(_app.servoCount) + " found");

  _app.activeIndex = 0;
  if (hasActiveServo()) {
    loadActiveServoRuntime();
    loadStagedConfigFromActive();
  } else {
    _app.targetPos  = _bus->posMid();
    _app.actualPos  = -1;
    _servoVoltageMv = -1;
    _servoTempC     = -1;
    _servoOnline    = false;
  }
  markDirty();
}

// Returns true if the scan was aborted by long press
bool App::scanBusAtBaud(uint32_t baud) {
  _bus->setBaud(baud);
  _app.scanAllCurrentBaud = baud;
  if (_app.oledOk)
    _ui.drawScanAllProgress(baud, _app.scanAllBaudStep, BAUD_TABLE_COUNT,
                            0, _app.servoCount, 253);

  unsigned long pressStart = 0;

  for (int id = 0; id <= 253; ++id) {
    _app.lastPingId = id;
    if (_bus->ping((uint8_t)id)) {
      bool found = false;
      for (uint8_t k = 0; k < _app.servoCount; ++k) {
        if (_app.ids[k] == (uint8_t)id) { found = true; break; }
      }
      if (!found && _app.servoCount < ST3215::MAX_SCAN_IDS)
        _app.ids[_app.servoCount++] = (uint8_t)id;
    }
    if (_app.oledOk)
      _ui.drawScanAllProgress(baud, _app.scanAllBaudStep, BAUD_TABLE_COUNT,
                              id, _app.servoCount, 253);

    // Poll encoder for long-press abort
    _encoder.update();
    if (_encoder.isPressed()) {
      if (pressStart == 0) pressStart = millis();
      if (millis() - pressStart >= 700UL) {
        _encoder.clearEvents();
        return true; // aborted
      }
    } else {
      pressStart = 0;
    }

    delay(2);
  }
  return false; // completed normally
}

void App::scanAllBauds() {
  _app.servoCount  = 0;
  _app.activeIndex = 0;
  _app.lastPingId  = -1;

  { int cnt; const uint32_t* tbl = activeBaudTable(cnt);
    for (int bi = 0; bi < cnt; ++bi) {
      _app.scanAllBaudStep = bi;
      bool aborted = scanBusAtBaud(tbl[bi]);
      if (aborted) {
        if (_app.oledOk) _ui.splash("Scan stopped", String(_app.servoCount) + " found");
        break;
      }
    }
    _bus->setBaud(tbl[clampInt(_app.scanBaudIndex, 0, cnt - 1)]);
  }

  if (hasActiveServo()) {
    loadActiveServoRuntime();
    loadStagedConfigFromActive();
  } else {
    _app.targetPos  = _bus->posMid();
    _app.actualPos  = -1;
    _servoVoltageMv = -1;
    _servoTempC     = -1;
    _servoOnline    = false;
  }
  markDirty();
}

// ---------------------------------------------------------------------------
// Config helpers
// ---------------------------------------------------------------------------

void App::loadActiveServoRuntime() {
  if (!hasActiveServo()) return;
  uint8_t id = activeServoId();
  _app.actualPos = _bus->readPosition(id);
  if (_app.actualPos >= 0) _app.targetPos = _app.actualPos;
  _servoVoltageMv = -1;
  _servoTempC     = -1;
  _bus->readVoltage(id, _servoVoltageMv);
  _bus->readTemperature(id, _servoTempC);
  _servoOnline = _bus->ping(id);
}

void App::loadStagedConfigFromActive() {
  if (!hasActiveServo()) return;
  uint8_t currentId   = activeServoId();
  uint8_t loadedId    = currentId;
  int     loadedMin   = 0, loadedMax = 4095;
  int     loadedTorq  = 1000, loadedOfs = 0;
  int     loadedMode  = 0, loadedBaud = 0;

  bool ok = _bus->loadConfig(currentId, loadedId, loadedMin, loadedMax,
                             loadedTorq, loadedOfs, loadedMode, loadedBaud);
  _app.cfg.loaded       = true;
  _app.cfg.dirty        = false;
  _app.cfg.servoId      = loadedId;
  _app.cfg.minLimit     = ok ? loadedMin  : 0;
  _app.cfg.maxLimit     = ok ? loadedMax  : 4095;
  _app.cfg.torqueLimit  = ok ? loadedTorq : 1000;
  _app.cfg.centerOffset = ok ? loadedOfs  : 0;
  _app.cfg.mode         = ok ? loadedMode : 0;
  _app.cfg.baudIndex    = ok ? loadedBaud : 0;
}

void App::selectNextServo() {
  if (_app.servoCount == 0) return;
  _app.activeIndex = (_app.activeIndex + 1) % _app.servoCount;
  loadActiveServoRuntime();
  loadStagedConfigFromActive();
}

// ---------------------------------------------------------------------------
// MIDI helpers
// ---------------------------------------------------------------------------

// Build (or rebuild) the bindings array from detected servo IDs.
// Preserves existing CC/channel assignments when servo IDs haven't changed.
void App::rebuildMidiBindings() {
  MidiState& m = _app.midi;

  // Snapshot old bindings to preserve CC/channel assignments
  MidiServoBinding old[MIDI_MAX_SERVOS + 2];
  uint8_t oldTotal = (uint8_t)midiTotalBindings(m);
  for (uint8_t i = 0; i < oldTotal; ++i) old[i] = m.bindings[i];

  uint8_t newCount = 0;
  for (uint8_t s = 0; s < _app.servoCount && s < MIDI_MAX_SERVOS; ++s) {
    uint8_t sid = _app.ids[s];

    MidiServoBinding b;
    b.servoId   = sid;
    b.cc        = MIDI_CC_NONE;
    b.channel   = m.channel;
    b.lastSent  = -1; b.lastRecv = -1; b.pendingCC = -1;
    b.txFlash   = 0;  b.rxFlash  = 0;

    // Preserve previous CC/channel for this servo ID
    for (uint8_t k = 0; k < oldTotal; ++k) {
      if (old[k].servoId == sid) {
        b.cc      = old[k].cc;
        b.channel = old[k].channel;
        break;
      }
    }

    // Cache min/max limits
    if (hasActiveServo() && activeServoId() == sid) {
      b.minLimit = _app.cfg.minLimit;
      b.maxLimit = _app.cfg.maxLimit;
    } else {
      uint8_t dummyId; int dummyTorq, dummyOfs, dummyMode, dummyBaud;
      int rMin = 0, rMax = 4095;
      _bus->loadConfig(sid, dummyId, rMin, rMax,
                      dummyTorq, dummyOfs, dummyMode, dummyBaud);
      b.minLimit = rMin;
      b.maxLimit = rMax;
    }

    m.bindings[newCount++] = b;
  }

  m.bindingCount = newCount;

  // Always append Speed, Acc, and Smooth global slots
  auto restoreGlobal = [&](uint8_t sentinel) -> MidiServoBinding {
    MidiServoBinding b;
    b.servoId   = sentinel;
    b.cc        = MIDI_CC_NONE;
    b.channel   = m.channel;
    b.inverted  = false;
    b.smoothing = 0;
    b.smoothPos = -1.0f;
    b.lastSent  = -1; b.lastRecv = -1; b.pendingCC = -1;
    b.txFlash   = 0;  b.rxFlash  = 0;
    b.minLimit  = 0;  b.maxLimit = 4095;
    for (uint8_t k = 0; k < oldTotal; ++k) {
      if (old[k].servoId == sentinel) {
        b.cc      = old[k].cc;
        b.channel = old[k].channel;
        break;
      }
    }
    return b;
  };

  m.bindings[m.bindingCount]     = restoreGlobal(MIDI_GLOBAL_SPEED);
  m.bindings[m.bindingCount + 1] = restoreGlobal(MIDI_GLOBAL_ACC);
  m.bindings[m.bindingCount + 2] = restoreGlobal(MIDI_GLOBAL_SMOOTH);

  // Also restore per-servo inverted/smoothing from old bindings
  for (uint8_t s = 0; s < newCount; ++s) {
    for (uint8_t k = 0; k < oldTotal; ++k) {
      if (old[k].servoId == m.bindings[s].servoId) {
        m.bindings[s].inverted  = old[k].inverted;
        m.bindings[s].smoothing = old[k].smoothing;
        m.bindings[s].smoothPos = old[k].smoothPos;
        break;
      }
    }
  }

  m.setupCursor  = 0;
  m.runCursor    = 0;
  m.txServoIndex = 0;
  m.txNextMs     = 0;
  m.rxNextMs     = 0;
}

// Called from the MIDI engine callback — runs in interrupt/USB context,
// so we only write to pendingCC here and let tickMidi() apply it on a timer.
void App::onMidiCC(uint8_t channel, uint8_t cc, uint8_t value) {
  if (!_app.midi.active) return;
  MidiState& m = _app.midi;
  // Log is written by onMidiAny (also called for CC by the engine)
  // Buffer the latest value for the rate-limited RX flush
  // (covers both per-servo and global speed/acc/smooth bindings)
  int total = midiTotalBindings(m);
  for (int i = 0; i < total; ++i) {
    MidiServoBinding& b = m.bindings[i];
    if (b.cc == MIDI_CC_NONE || b.cc != cc || b.channel != channel) continue;
    b.pendingCC = (int8_t)value;
    b.lastRecv  = (int8_t)value;
    b.rxFlash   = MIDI_FLASH_FRAMES;
  }
}

// Logs any MIDI message to the monitor ring buffer.
// Always logs (even before Run mode) so monitor works in Setup too.
void App::onMidiAny(MidiMsgType type, uint8_t channel,
                    uint8_t byte1, uint8_t byte2, int16_t int14) {
  // Log unconditionally when MIDI is active OR when on any MIDI screen
  ScreenId scr = _app.screen;
  bool onMidiScreen = (scr == ScreenId::MidiRun || scr == ScreenId::MidiSetup);
  if (!_app.midi.active && !onMidiScreen) return;

  MidiState& m = _app.midi;
  MidiLogEntry& entry = m.log[m.logHead % MIDI_LOG_SIZE];
  entry.type    = type;
  entry.channel = channel;
  entry.byte1   = byte1;
  entry.byte2   = byte2;
  entry.int14   = int14;
  entry.valid   = true;
  m.logHead     = (m.logHead + 1) % MIDI_LOG_SIZE;
}

// tickMidi() is called every loop() when MIDI mode is active.
//
// TX path: one servo is polled per MIDI_TX_INTERVAL_MS window, round-robin.
//   With N servos the effective outgoing rate per servo is
//   1 / (N * MIDI_TX_INTERVAL_MS) — e.g. 4 servos @ 25ms = 10 Hz each,
//   which is well within the bus budget.
//
// RX path: all pending inbound CC values are flushed to the servo bus
//   once per MIDI_RX_INTERVAL_MS, discarding intermediate messages.
//   This decouples USB MIDI receive rate from servo bus write rate.
void App::tickMidi() {
  if (_app.midi.panic) {
    doMidiPanic();
    _app.midi.panic = false;
    return;
  }

  MidiState& m      = _app.midi;
  uint32_t   now    = millis();

  // ---- TX: poll one binding and send outgoing CC ----
  if (now >= m.txNextMs && midiTotalBindings(m) > 0) {
    m.txNextMs = now + MIDI_TX_INTERVAL_MS;
    int total = midiTotalBindings(m);

    uint8_t checked = 0;
    while (checked < (uint8_t)total) {
      if (m.txServoIndex >= (uint8_t)total) m.txServoIndex = 0;
      MidiServoBinding& b = m.bindings[m.txServoIndex];
      m.txServoIndex++;
      checked++;
      if (b.cc == MIDI_CC_NONE) continue;

      uint8_t ccVal = 64;
      if (b.servoId == MIDI_GLOBAL_SPEED) {
        ccVal = (uint8_t)map(clampInt(_app.speed, 0, 4095), 0, 4095, 0, 127);
      } else if (b.servoId == MIDI_GLOBAL_ACC) {
        ccVal = (uint8_t)map(clampInt(_app.acc, 0, 254), 0, 254, 0, 127);
      } else if (b.servoId == MIDI_GLOBAL_SMOOTH) {
        ccVal = (m.bindingCount > 0) ? m.bindings[0].smoothing : 0;
      } else {
        int pos = _bus->readPosition(b.servoId);
        if (pos < 0) break;
        ccVal = posToCC(pos, b.minLimit, b.maxLimit, b.inverted);
      }

      if ((int8_t)ccVal != b.lastSent) {
        _midi.sendCC(b.channel, b.cc, ccVal);
        b.lastSent = (int8_t)ccVal;
        b.txFlash  = MIDI_FLASH_FRAMES;
      }
      break;
    }
  }

  // ---- RX: flush pending inbound CC values ----
  if (now >= m.rxNextMs) {
    m.rxNextMs = now + MIDI_RX_INTERVAL_MS;
    int total  = midiTotalBindings(m);

    for (int i = 0; i < total; ++i) {
      MidiServoBinding& b = m.bindings[i];
      if (b.pendingCC < 0) continue;
      uint8_t val = (uint8_t)b.pendingCC;
      b.pendingCC = -1;

      if (b.servoId == MIDI_GLOBAL_SPEED) {
        _app.speed = (int)map((long)val, 0, 127, 0, 4095);
        markDirty();
        for (uint8_t s = 0; s < _app.servoCount; ++s)
          _bus->setPosition(_app.ids[s], _bus->readPosition(_app.ids[s]),
                           _app.speed, _app.acc);
      } else if (b.servoId == MIDI_GLOBAL_ACC) {
        _app.acc = (int)map((long)val, 0, 127, 0, 254);
        markDirty();
        for (uint8_t s = 0; s < _app.servoCount; ++s)
          _bus->setPosition(_app.ids[s], _bus->readPosition(_app.ids[s]),
                           _app.speed, _app.acc);
      } else if (b.servoId == MIDI_GLOBAL_SMOOTH) {
        // Apply smoothing value to all per-servo bindings
        for (uint8_t s = 0; s < m.bindingCount; ++s)
          m.bindings[s].smoothing = val;
      } else {
        // Compute raw target position
        int rawTarget = ccToPos(val, b.minLimit, b.maxLimit, b.inverted);

        // Apply IIR smoothing filter if enabled
        // alpha = (128 - smoothing) / 128.0f
        // smoothing=0 → alpha=1.0 (instant), smoothing=127 → alpha≈0.008 (very slow)
        int finalTarget;
        if (b.smoothing == 0) {
          finalTarget   = rawTarget;
          b.smoothPos   = (float)rawTarget;
        } else {
          if (b.smoothPos < 0.0f) b.smoothPos = (float)rawTarget; // initialise
          float alpha   = (128.0f - (float)b.smoothing) / 128.0f;
          b.smoothPos   = alpha * (float)rawTarget + (1.0f - alpha) * b.smoothPos;
          finalTarget   = (int)(b.smoothPos + 0.5f);
          finalTarget   = clampInt(finalTarget, b.minLimit, b.maxLimit);
        }

        _bus->setPosition(b.servoId, finalTarget, _app.speed, _app.acc);
      }
    }
  }

  // ---- Tick flash counters ----
  int total = midiTotalBindings(m);
  for (int i = 0; i < total; ++i) {
    if (m.bindings[i].txFlash > 0) --m.bindings[i].txFlash;
    if (m.bindings[i].rxFlash > 0) --m.bindings[i].rxFlash;
  }
}

void App::doMidiPanic() {
  _midi.panic();
  MidiState& m = _app.midi;
  int total    = midiTotalBindings(m);
  for (int i = 0; i < total; ++i) {
    m.bindings[i].lastSent = -1;
    m.bindings[i].lastRecv = -1;
    m.bindings[i].pendingCC = -1;
    m.bindings[i].txFlash  = 0;
    m.bindings[i].rxFlash  = 0;
  }
}

// Scale pos within [minL..maxL] to 0..127, optionally inverted
uint8_t App::posToCC(int pos, int minL, int maxL, bool inverted) {
  if (minL >= maxL) return 64;
  pos = clampInt(pos, minL, maxL);
  uint8_t v = (uint8_t)map(pos, minL, maxL, 0, 127);
  return inverted ? (uint8_t)(127 - v) : v;
}

// Scale CC 0..127 to position within [minL..maxL], optionally inverted
int App::ccToPos(uint8_t cc, int minL, int maxL, bool inverted) {
  if (minL >= maxL) return minL;
  uint8_t c = inverted ? (uint8_t)(127 - cc) : cc;
  return (int)map((long)c, 0, 127, minL, maxL);
}

// ---------------------------------------------------------------------------
// Input routing
// ---------------------------------------------------------------------------

void App::handleInput() {
  _encoder.update();
  int  delta      = _encoder.readSteps();
  bool shortPress = _encoder.wasShortPressed();
  bool longPress  = _encoder.wasLongPressed();

  switch (_app.screen) {
    case ScreenId::Home:           handleHomeInput(delta, shortPress, longPress);           break;
    case ScreenId::Scan:           handleScanInput(delta, shortPress, longPress);           break;
    case ScreenId::ScanBaudSelect: handleScanBaudSelectInput(delta, shortPress, longPress); break;
    case ScreenId::ScanAllBaud:    handleScanAllBaudInput(delta, shortPress, longPress);    break;
    case ScreenId::SelectServo:    handleSelectServoInput(delta, shortPress, longPress);    break;
    case ScreenId::LiveControl:    handleLiveControlInput(delta, shortPress, longPress);    break;
    case ScreenId::ServoInfo:      handleServoInfoInput(delta, shortPress, longPress);      break;
    case ScreenId::ConfigMenu:     handleConfigInput(delta, shortPress, longPress);         break;
    case ScreenId::ConfirmSave:    handleConfirmSaveInput(delta, shortPress, longPress);    break;
    case ScreenId::SaveResult:     handleSaveResultInput(delta, shortPress, longPress);     break;
    case ScreenId::ModeWarn:       handleModeWarnInput(delta, shortPress, longPress);       break;
    case ScreenId::MidiSetup:      handleMidiSetupInput(delta, shortPress, longPress);      break;
    case ScreenId::MidiRun:        handleMidiRunInput(delta, shortPress, longPress);        break;
    case ScreenId::PersistDiag:    handlePersistDiagInput(delta, shortPress, longPress);    break;
    case ScreenId::SelectProtocol: handleSelectProtocolInput(delta, shortPress, longPress); break;
    default: break;
  }
}

void App::handleHomeInput(int delta, bool shortPress, bool longPress) {
  (void)longPress;
  if (delta != 0)
    _app.menuIndex = clampInt(_app.menuIndex + delta, 0, HOME_COUNT - 1);
  if (!shortPress) return;

  switch (_app.menuIndex) {
    case 0: enterScreen(ScreenId::ScanBaudSelect, _app.scanBaudIndex, false); break;
    case 1: enterScreen(ScreenId::SelectServo, _app.activeIndex, false);       break;
    case 2: enterScreen(ScreenId::LiveControl, 1, false);                      break;
    case 3:
      _servoStatusByte = -1;
      _servoLoadPct    = 0;
      _servoCurrentMa  = -1;
      enterScreen(ScreenId::ServoInfo, 0, false);
      break;
    case 4: enterScreen(ScreenId::ConfigMenu, 0, false);                       break;
    case 5: enterScreen(ScreenId::ConfirmSave, 0, false);                      break;
    case 6:
      // Enter MIDI mode: rebuild bindings then go to Setup
      rebuildMidiBindings();
      _app.midi.active = false; // Setup first, Run starts from there
      enterScreen(ScreenId::MidiSetup, 0, false);
      break;
    case 7:
      enterScreen(ScreenId::PersistDiag, 0, false);
      break;
    case 8:
      enterScreen(ScreenId::SelectProtocol, (int)_app.protocol, false);
      break;
  }
}

void App::handleScanInput(int delta, bool shortPress, bool longPress) {
  (void)delta;
  if (shortPress || longPress) enterScreen(ScreenId::Home, 0, false);
}

void App::handleScanBaudSelectInput(int delta, bool shortPress, bool longPress) {
  if (longPress) { enterScreen(ScreenId::Home, 0, false); return; }
  if (delta != 0)
    { int _bc; activeBaudTable(_bc);
      _app.menuIndex = clampInt(_app.menuIndex + delta, 0, _bc); }
  if (!shortPress) return;

  { int _bc; activeBaudTable(_bc);
  if (_app.menuIndex == _bc) {
    _app.scanAllBaudStep = 0;
    enterScreen(ScreenId::ScanAllBaud, 0, false);
    scanAllBauds();
    enterScreen(ScreenId::Home, 0, false);
  } else {
    _app.scanBaudIndex = _app.menuIndex;
    enterScreen(ScreenId::Scan, 0, false);
    scanBusWithUi();
    enterScreen(ScreenId::Home, 0, false);
  }}
}

void App::handleScanAllBaudInput(int delta, bool shortPress, bool longPress) {
  (void)delta; (void)shortPress; (void)longPress;
}

void App::handleSelectServoInput(int delta, bool shortPress, bool longPress) {
  if (longPress) { enterScreen(ScreenId::Home, 0, false); return; }
  if (_app.servoCount <= 0) {
    if (shortPress) enterScreen(ScreenId::Home, 0, false);
    return;
  }
  if (delta != 0)
    _app.activeIndex = clampInt(_app.activeIndex + delta, 0, _app.servoCount - 1);
  if (shortPress) {
    loadActiveServoRuntime();
    loadStagedConfigFromActive();
    markDirty();
    enterScreen(ScreenId::Home, 0, false);
  }
}

void App::handleLiveControlInput(int delta, bool shortPress, bool longPress) {
  constexpr int LIVE_ITEM_COUNT = 4;
  if (longPress) {
    if (_app.editing) _app.editing = false;
    else enterScreen(ScreenId::Home, 0, false);
    return;
  }
  if (!_app.editing) {
    if (delta != 0)
      _app.menuIndex = clampInt(_app.menuIndex + delta, 0, LIVE_ITEM_COUNT - 1);
    if (shortPress) _app.editing = true;
    return;
  }
  if (!hasActiveServo()) { _app.editing = false; return; }
  uint8_t id = activeServoId();

  switch (_app.menuIndex) {
    case 0:
      if (delta != 0) {
        _app.torqueEnabled = !_app.torqueEnabled;
        _bus->torqueEnable(id, _app.torqueEnabled);
        markDirty();
      }
      break;
    case 1:
      if (delta != 0) {
        _app.targetPos = clampInt(_app.targetPos + delta * UI::POSITION_STEP,
                                  ST3215::POS_MIN, ST3215::POS_MAX);
        if (_app.torqueEnabled)
          _bus->setPosition(id, _app.targetPos, _app.speed, _app.acc);
      }
      break;
    case 2:
      if (delta != 0) {
        _app.speed = clampInt(_app.speed + delta * 10, 0, 4095);
        if (_app.torqueEnabled)
          _bus->setPosition(id, _app.targetPos, _app.speed, _app.acc);
        markDirty();
      }
      break;
    case 3:
      if (delta != 0) {
        _app.acc = clampInt(_app.acc + delta, 0, 254);
        if (_app.torqueEnabled)
          _bus->setPosition(id, _app.targetPos, _app.speed, _app.acc);
        markDirty();
      }
      break;
  }
  if (shortPress) _app.editing = false;
}

void App::handleServoInfoInput(int delta, bool shortPress, bool longPress) {
  (void)delta;
  if (longPress) {
    _app.menuIndex = 0; // reset page
    enterScreen(ScreenId::Home, 0, false);
  } else if (shortPress) {
    _app.menuIndex = (_app.menuIndex == 0) ? 1 : 0; // toggle page
  }
}

void App::handleConfigInput(int delta, bool shortPress, bool longPress) {
  static int editBackup = 0;
  if (longPress) {
    if (_app.editing) {
      switch (_app.menuIndex) {
        case 0: _app.cfg.servoId      = (uint8_t)editBackup; break;
        case 1: _app.cfg.minLimit     = editBackup; break;
        case 2: _app.cfg.maxLimit     = editBackup; break;
        case 3: _app.cfg.torqueLimit  = editBackup; break;
        case 4: _app.cfg.centerOffset = editBackup; break;
        case 5: _app.cfg.mode         = editBackup; break;
        case 6: _app.cfg.baudIndex    = editBackup; break;
      }
      _app.editing = false;
    } else {
      enterScreen(ScreenId::Home, 0, false);
    }
    return;
  }
  if (!_app.editing) {
    if (delta != 0)
      _app.menuIndex = clampInt(_app.menuIndex + delta, 0, CONFIG_ITEM_COUNT - 1);
    if (shortPress && hasActiveServo()) {
      switch (_app.menuIndex) {
        case 0: editBackup = _app.cfg.servoId;      _app.editing = true; break;
        case 1: editBackup = _app.cfg.minLimit;     _app.editing = true; break;
        case 2: editBackup = _app.cfg.maxLimit;     _app.editing = true; break;
        case 3: editBackup = _app.cfg.torqueLimit;  _app.editing = true; break;
        case 4: editBackup = _app.cfg.centerOffset; _app.editing = true; break;
        case 5:
          editBackup = _app.cfg.mode;
          if (_app.cfg.mode == 0) {
            _app.pendingMode = 1;
            enterScreen(ScreenId::ModeWarn, 0, false);
          } else {
            _app.cfg.mode  = 0;
            _app.cfg.dirty = true;
          }
          break;
        case 6: editBackup = _app.cfg.baudIndex; _app.editing = true; break;
      }
    }
    return;
  }
  bool changed = false;
  switch (_app.menuIndex) {
    case 0: { int v = clampInt((int)_app.cfg.servoId + delta, 0, 253);
              if (v != _app.cfg.servoId) { _app.cfg.servoId = (uint8_t)v; changed = true; } break; }
    case 1: { int v = clampInt(_app.cfg.minLimit + delta * 8, 0, 4095);
              if (v != _app.cfg.minLimit) { _app.cfg.minLimit = v; changed = true; } break; }
    case 2: { int v = clampInt(_app.cfg.maxLimit + delta * 8, 0, 4095);
              if (v != _app.cfg.maxLimit) { _app.cfg.maxLimit = v; changed = true; } break; }
    case 3: { int v = clampInt(_app.cfg.torqueLimit + delta * 10, 0, 1000);
              if (v != _app.cfg.torqueLimit) { _app.cfg.torqueLimit = v; changed = true; } break; }
    case 4: { int v = clampInt(_app.cfg.centerOffset + delta * 4, -2047, 2047);
              if (v != _app.cfg.centerOffset) { _app.cfg.centerOffset = v; changed = true; } break; }
    case 5: break; // handled via ModeWarn
    case 6: { int v = clampInt(_app.cfg.baudIndex + delta, 0, BAUD_TABLE_COUNT - 1);
              if (v != _app.cfg.baudIndex) { _app.cfg.baudIndex = v; changed = true; } break; }
  }
  if (changed) _app.cfg.dirty = true;
  if (shortPress) _app.editing = false;
}

void App::handleConfirmSaveInput(int delta, bool shortPress, bool longPress) {
  if (longPress) { enterScreen(ScreenId::Home, 0, false); return; }
  if (delta != 0) _app.menuIndex = clampInt(_app.menuIndex + delta, 0, 1);
  if (!shortPress) return;
  if (_app.menuIndex == 0) { enterScreen(ScreenId::Home, 0, false); return; }
  _app.saveOk = saveStagedConfig();
  enterScreen(ScreenId::SaveResult, 0, false);
}

void App::handleSaveResultInput(int delta, bool shortPress, bool longPress) {
  (void)delta;
  if (shortPress || longPress) enterScreen(ScreenId::Home, 0, false);
}

void App::handleModeWarnInput(int delta, bool shortPress, bool longPress) {
  if (longPress) { enterScreen(ScreenId::ConfigMenu, 5, false); return; }
  if (delta != 0) _app.menuIndex = clampInt(_app.menuIndex + delta, 0, 1);
  if (!shortPress) return;
  if (_app.menuIndex == 1) {
    _app.cfg.mode  = _app.pendingMode;
    _app.cfg.dirty = true;
  }
  enterScreen(ScreenId::ConfigMenu, 5, false);
}

// ---------------------------------------------------------------------------
// MIDI Setup input
// Rows 0..bindingCount-1 = per-servo binding rows
// Row bindingCount        = ">> Run MIDI <<"
//
// Short press on a servo row:
//   first press  → enter CC edit  (turn = change CC, short = confirm)
//   second press → enter Ch edit  (turn = change channel, short = confirm)
// Long press on a servo row while editing → cancel that edit
// Long press while not editing → back to Home
// Short press on Run row → start MIDI Run
// ---------------------------------------------------------------------------
void App::handleMidiSetupInput(int delta, bool shortPress, bool longPress) {
  MidiState& m   = _app.midi;
  // Rows: 0..totalBindings-1 = all bindings, last = Run row
  int totalRows  = midiTotalBindings(m) + 1;

  if (longPress) {
    if (m.editingCC || m.editingCh || m.editingInv || m.editingSmooth) {
      m.editingCC     = false;
      m.editingCh     = false;
      m.editingInv    = false;
      m.editingSmooth = false;
    } else {
      m.active = false;
      markDirty();
      enterScreen(ScreenId::Home, 0, false);
    }
    return;
  }

  // Navigation when not editing
  if (!m.editingCC && !m.editingCh && !m.editingInv && !m.editingSmooth) {
    if (delta != 0)
      m.setupCursor = clampInt(m.setupCursor + delta, 0, totalRows - 1);

    if (shortPress) {
      if (m.setupCursor == midiTotalBindings(m)) {
        // Run row
        m.active      = true;
        m.runCursor   = 0;
        m.showMonitor = false;
        markDirty();
        enterScreen(ScreenId::MidiRun, 0, false);
      } else {
        m.editingCC = true; // step 1: CC edit
      }
    }
    return;
  }

  // Step 1: CC editing
  if (m.editingCC) {
    if (delta != 0) {
      MidiServoBinding& b = m.bindings[m.setupCursor];
      int cur = (b.cc == MIDI_CC_NONE) ? 0 : (int)b.cc;
      b.cc = (uint8_t)clampInt(cur + delta, 0, 127);
    }
    if (shortPress) { m.editingCC = false; m.editingCh = true; }
    return;
  }

  // Step 2: Channel editing
  if (m.editingCh) {
    if (delta != 0) {
      MidiServoBinding& b = m.bindings[m.setupCursor];
      b.channel = (uint8_t)clampInt((int)b.channel + delta, 1, 16);
    }
    if (shortPress) {
      m.editingCh = false;
      // Global bindings (speed/acc/smooth) skip invert and smooth steps
      uint8_t sid = m.bindings[m.setupCursor].servoId;
      bool isGlobal = (sid == MIDI_GLOBAL_SPEED || sid == MIDI_GLOBAL_ACC ||
                       sid == MIDI_GLOBAL_SMOOTH);
      if (isGlobal) { markDirty(); }
      else          { m.editingInv = true; }
    }
    return;
  }

  // Step 3: Invert toggle (per-servo only)
  if (m.editingInv) {
    if (delta != 0) {
      m.bindings[m.setupCursor].inverted = !m.bindings[m.setupCursor].inverted;
    }
    if (shortPress) { m.editingInv = false; m.editingSmooth = true; }
    return;
  }

  // Step 4: Smoothing amount
  if (m.editingSmooth) {
    if (delta != 0) {
      MidiServoBinding& b = m.bindings[m.setupCursor];
      b.smoothing = (uint8_t)clampInt((int)b.smoothing + delta, 0, 127);
      b.smoothPos = -1.0f; // reset filter when smoothing changes
    }
    if (shortPress) { m.editingSmooth = false; markDirty(); }
    return;
  }
}

// ---------------------------------------------------------------------------
// MIDI Run input
// Rows 0..bindingCount-1 = per-servo activity rows
// Row bindingCount        = Panic row
//
// Short press on Panic row (or any row if it is selected) → trigger panic
// Long press → back to MidiSetup
// ---------------------------------------------------------------------------
void App::handleMidiRunInput(int delta, bool shortPress, bool longPress) {
  MidiState& m = _app.midi;
  // Rows: 0..totalBindings-1 = servo+global, totalBindings = Monitor, totalBindings+1 = Panic
  int totalRows = midiTotalBindings(m) + 2;

  if (longPress) {
    if (m.showMonitor) {
      m.showMonitor = false;
      return;
    }
    m.active = false;
    markDirty();
    enterScreen(ScreenId::MidiSetup, 0, false);
    return;
  }

  if (m.showMonitor) {
    // In full-screen monitor view: short press or turn goes back
    if (shortPress || delta != 0) m.showMonitor = false;
    return;
  }

  if (delta != 0)
    m.runCursor = clampInt(m.runCursor + delta, 0, totalRows - 1);

  if (shortPress) {
    int monitorRow = midiTotalBindings(m);
    int panicRow   = midiTotalBindings(m) + 1;
    if (m.runCursor == monitorRow) {
      m.showMonitor = true;            // enter full-screen monitor
    } else if (m.runCursor == panicRow) {
      m.panic = true;                  // trigger panic
    }
    // short press on servo rows does nothing
  }
}

// PersistDiag: any interaction returns to Home
void App::handlePersistDiagInput(int delta, bool shortPress, bool longPress) {
  (void)delta;
  if (shortPress || longPress) enterScreen(ScreenId::Home, 0, false);
}

// SelectProtocol: scroll through protocols, short press applies, long press cancels
void App::handleSelectProtocolInput(int delta, bool shortPress, bool longPress) {
  if (longPress) { enterScreen(ScreenId::Home, 0, false); return; }
  if (delta != 0)
    _app.menuIndex = clampInt(_app.menuIndex + delta, 0, BUS_PROTOCOL_COUNT - 1);
  if (shortPress) {
    BusProtocol chosen = (BusProtocol)_app.menuIndex;
    if (chosen != _app.protocol) {
      _app.protocol = chosen;
      // Hot-swap the bus pointer
      IServoBus* newBus = busForProtocol(chosen);
      setBus(newBus);
      markDirty();
      // Set default baud for new protocol and prompt a scan
      int cnt; const uint32_t* tbl = activeBaudTable(cnt);
      _app.scanBaudIndex = 0;
      // For DXL1 default is index 2 (57600)
      if (chosen == BusProtocol::DXL1) _app.scanBaudIndex = DXL1_DEFAULT_BAUD_IDX;
      _bus->setBaud(tbl[_app.scanBaudIndex]);
      if (_app.oledOk)
        _ui.splash("Protocol", _bus->protocolName(), "Scan needed");
      delay(1000);
    }
    enterScreen(ScreenId::Home, 0, false);
  }
}

// ---------------------------------------------------------------------------
// Feedback and rendering
// ---------------------------------------------------------------------------

void App::updateFeedback() {
  if (!hasActiveServo()) return;
  unsigned long now = millis();
  if (now - _app.lastFeedbackMs < UI::FEEDBACK_INTERVAL_MS) return;
  _app.lastFeedbackMs = now;
  uint8_t id = activeServoId();
  _app.actualPos = _bus->readPosition(id);
  _bus->readVoltage(id, _servoVoltageMv);
  _bus->readTemperature(id, _servoTempC);
  _bus->readStatus(id, _servoStatusByte);
  _bus->readLoad(id, _servoLoadPct);
  _bus->readCurrent(id, _servoCurrentMa);
  _servoOnline = _bus->ping(id);
}

void App::render() {
  if (!_app.oledOk) return;
  unsigned long now = millis();
  if (now - _app.lastUiMs < UI::UI_REFRESH_MS) return;
  _app.lastUiMs = now;

  switch (_app.screen) {
    case ScreenId::Home:
      _ui.drawMenu("ST3215 Tool", HOME_ITEMS, HOME_COUNT, _app.menuIndex, false,
                   hasActiveServo() ? "Short=enter" : "No servo");
      break;
    case ScreenId::Scan:
      _ui.drawScanProgress(BAUD_TABLE[_app.scanBaudIndex],
                           _app.lastPingId, _app.servoCount, 253);
      break;
    case ScreenId::ScanBaudSelect:
      _ui.drawScanBaudSelect(_app.menuIndex);
      break;
    case ScreenId::ScanAllBaud:
      _ui.drawScanAllProgress(_app.scanAllCurrentBaud,
                              _app.scanAllBaudStep, BAUD_TABLE_COUNT,
                              _app.lastPingId, _app.servoCount, 253);
      break;
    case ScreenId::SelectServo:
      _ui.drawSelectServo(_app.ids, _app.servoCount, _app.activeIndex);
      break;
    case ScreenId::LiveControl:
      _ui.drawLiveControl(activeServoId(),
                          _app.targetPos, _app.actualPos,
                          _app.torqueEnabled,
                          _app.cfg.minLimit, _app.cfg.maxLimit,
                          _app.speed, _app.acc,
                          _app.menuIndex, _app.editing);
      break;
    case ScreenId::ServoInfo:
      _ui.drawServoInfo(activeServoId(), _app.actualPos,
                        _servoVoltageMv, _servoTempC,
                        _servoOnline,
                        _servoStatusByte, _servoLoadPct, _servoCurrentMa,
                        _app.menuIndex);
      break;
    case ScreenId::ConfigMenu:
      _ui.drawConfigMenu(activeServoId(),
                         _app.cfg.servoId,
                         _app.cfg.minLimit, _app.cfg.maxLimit,
                         _app.cfg.torqueLimit, _app.cfg.centerOffset,
                         _app.cfg.mode, _app.cfg.baudIndex,
                         _app.cfg.dirty, _app.menuIndex, _app.editing);
      break;
    case ScreenId::ConfirmSave:
      _ui.drawConfirmSave(activeServoId(),
                          _app.cfg.servoId,
                          _app.cfg.minLimit, _app.cfg.maxLimit,
                          _app.cfg.torqueLimit, _app.cfg.centerOffset,
                          _app.cfg.mode, _app.cfg.baudIndex,
                          _app.cfg.dirty, _app.menuIndex);
      break;
    case ScreenId::SaveResult:
      _ui.drawSaveResult(_app.saveOk, _app.saveMessage);
      break;
    case ScreenId::ModeWarn:
      _ui.drawModeWarn(_app.menuIndex);
      break;
    case ScreenId::MidiSetup: {
      int editStep = 0;
      if      (_app.midi.editingCC)     editStep = 1;
      else if (_app.midi.editingCh)     editStep = 2;
      else if (_app.midi.editingInv)    editStep = 3;
      else if (_app.midi.editingSmooth) editStep = 4;
      _ui.drawMidiSetup(_app.midi.bindings, _app.midi.bindingCount,
                        _app.midi.setupCursor, editStep, _midi.isMounted());
      break;
    }
    case ScreenId::MidiRun:
      _ui.drawMidiRun(_app.midi.bindings, _app.midi.bindingCount,
                      _app.midi.runCursor,
                      _midi.isMounted(),
                      _app.midi.showMonitor,
                      _app.midi.log,
                      _app.midi.logHead);
      break;
    case ScreenId::PersistDiag:
      _ui.drawPersistDiag(persist.diagnose());
      break;
    case ScreenId::SelectProtocol:
      _ui.drawSelectProtocol((int)_app.protocol, _app.menuIndex);
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Validation and saving
// ---------------------------------------------------------------------------

bool App::validateConfig(char* msg, size_t msgLen) {
  if (!hasActiveServo()) { snprintf(msg, msgLen, "No active servo"); return false; }
  if (!_app.cfg.dirty)   { snprintf(msg, msgLen, "Nothing to save"); return false; }
  if (_app.cfg.minLimit >= _app.cfg.maxLimit) {
    snprintf(msg, msgLen, "Min must be < Max"); return false;
  }
  if (_app.cfg.servoId != activeServoId()) {
    for (uint8_t i = 0; i < _app.servoCount; ++i) {
      if (i == _app.activeIndex) continue;
      if (_app.ids[i] == _app.cfg.servoId) {
        snprintf(msg, msgLen, "ID already exists"); return false;
      }
    }
  }
  snprintf(msg, msgLen, "OK");
  return true;
}

bool App::saveStagedConfig() {
  char msg[32];
  if (!validateConfig(msg, sizeof(msg))) {
    strncpy(_app.saveMessage, msg, sizeof(_app.saveMessage));
    _app.saveMessage[sizeof(_app.saveMessage) - 1] = '\0';
    return false;
  }

  uint8_t oldId = activeServoId();
  uint8_t newId = _app.cfg.servoId;
  bool ok = true;

  if (newId != oldId) {
    ok = _bus->saveId(oldId, newId);
    if (!ok) {
      strncpy(_app.saveMessage, "ID write failed", sizeof(_app.saveMessage));
      _app.saveMessage[sizeof(_app.saveMessage)-1] = '\0';
      return false;
    }
    delay(50);
    _app.ids[_app.activeIndex] = newId;
    if (!_bus->ping(newId)) {
      strncpy(_app.saveMessage, "New ID not found", sizeof(_app.saveMessage));
      _app.saveMessage[sizeof(_app.saveMessage)-1] = '\0';
      return false;
    }
    oldId = newId;
  }

  auto failWith = [&](const char* s) -> bool {
    strncpy(_app.saveMessage, s, sizeof(_app.saveMessage));
    _app.saveMessage[sizeof(_app.saveMessage)-1] = '\0';
    return false;
  };

  if (!_bus->saveMinMax(oldId, _app.cfg.minLimit, _app.cfg.maxLimit))
    return failWith("Limit write failed");
  if (!_bus->saveTorqueLimit(oldId, _app.cfg.torqueLimit))
    return failWith("Torque write failed");
  if (!_bus->saveCenterOffset(oldId, _app.cfg.centerOffset))
    return failWith("Offset write failed");
  if (!_bus->saveMode(oldId, _app.cfg.mode))
    return failWith("Mode write failed");
  if (!_bus->saveBaud(oldId, _app.cfg.baudIndex))
    return failWith("Baud write failed");

  _app.cfg.dirty = false;
  if (_app.cfg.baudIndex != 0)
    strncpy(_app.saveMessage, "Saved! Rescan needed", sizeof(_app.saveMessage));
  else
    strncpy(_app.saveMessage, "Saved + locked", sizeof(_app.saveMessage));
  _app.saveMessage[sizeof(_app.saveMessage)-1] = '\0';

  loadActiveServoRuntime();
  loadStagedConfigFromActive();
  return true;
}
