#pragma once
#include "../app_state.h"
#include "../drivers/encoder_unit.h"
#include "../drivers/oled_ui.h"
#include "../drivers/iservo_bus.h"
#include "../drivers/midi_engine.h"
#include "../drivers/persist.h"

class App {
public:
  App(AppState& state, UnitEncoder& encoder, OledUi& ui,
      IServoBus* bus, MidiEngine& midi);

  void begin();
  void tick();
  void setBus(IServoBus* bus);  // hot-swap the bus pointer (on protocol change)

  // Called by the MIDI engine CC callback (static trampoline → this)
  void onMidiCC(uint8_t channel, uint8_t cc, uint8_t value);
  void onMidiAny(MidiMsgType type, uint8_t channel,
                 uint8_t byte1, uint8_t byte2, int16_t int14);

private:
  AppState&    _app;
  UnitEncoder& _encoder;
  OledUi&      _ui;
  IServoBus*   _bus;
  MidiEngine&  _midi;

  int  _servoVoltageMv  = -1;
  int  _servoTempC      = -1;
  bool _servoOnline     = false;
  int  _servoStatusByte = -1;  // raw status/fault byte (reg 65), -1 = unread
  int  _servoLoadPct    = 0;   // load in percent (signed, -100..+100)
  int  _servoCurrentMa  = -1;  // motor current in mA, -1 = unread
  bool _persistDirty    = false; // set when config needs writing to flash

  // Persistence
  void loadPersistedState();   // called in begin() — restore or scan fresh
  void savePersistedState();   // write PersistentConfig to LittleFS
  void markDirty();            // sets _persistDirty; save happens lazily in tick()

  void scanBus();
  void scanBusWithUi();
  bool scanBusAtBaud(uint32_t baud);  // returns true if aborted by long press
  void scanAllBauds();
  void loadActiveServoRuntime();
  void loadStagedConfigFromActive();

  bool hasActiveServo() const;
  uint8_t activeServoId() const;
  void selectNextServo();

  // MIDI helpers
  void rebuildMidiBindings();
  void tickMidi();
  void applyMidiCC(uint8_t channel, uint8_t cc, uint8_t value);
  void doMidiPanic();

  // Scaling helpers
  static uint8_t posToCC(int pos, int minLimit, int maxLimit, bool inverted = false);
  static int     ccToPos(uint8_t cc, int minLimit, int maxLimit, bool inverted = false);

  void handleInput();
  void handleHomeInput(int delta, bool shortPress, bool longPress);
  void handleScanInput(int delta, bool shortPress, bool longPress);
  void handleScanBaudSelectInput(int delta, bool shortPress, bool longPress);
  void handleScanAllBaudInput(int delta, bool shortPress, bool longPress);
  void handleSelectServoInput(int delta, bool shortPress, bool longPress);
  void handleLiveControlInput(int delta, bool shortPress, bool longPress);
  void handleServoInfoInput(int delta, bool shortPress, bool longPress);
  void handleConfigInput(int delta, bool shortPress, bool longPress);
  void handleConfirmSaveInput(int delta, bool shortPress, bool longPress);
  void handleSaveResultInput(int delta, bool shortPress, bool longPress);
  void handleModeWarnInput(int delta, bool shortPress, bool longPress);
  void handleMidiSetupInput(int delta, bool shortPress, bool longPress);
  void handleMidiRunInput(int delta, bool shortPress, bool longPress);
  void handlePersistDiagInput(int delta, bool shortPress, bool longPress);
  void handleSelectProtocolInput(int delta, bool shortPress, bool longPress);

  // Returns the baud table and count for the current protocol
  const uint32_t* activeBaudTable(int& count) const;

  void updateFeedback();
  void render();

  bool saveStagedConfig();
  bool validateConfig(char* msg, size_t msgLen);

  void enterScreen(ScreenId screen, int menuIndex = 0, bool editing = false);
};
