#pragma once
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "../model/servo_model.h"
// PersistDiag forward declared here to avoid circular include;
// persist.h is included in oled_ui.cpp
struct PersistDiag;

class OledUi {
public:
  OledUi(uint16_t w, uint16_t h, TwoWire* wire, uint8_t addr);

  bool begin();
  void clear();
  void splash(const String& l1, const String& l2 = "", const String& l3 = "");

  void drawMenu(const char* title,
                const char* const* items,
                int itemCount,
                int selected,
                bool editing = false,
                const char* footer = nullptr);

  void drawSelectServo(const uint8_t* ids,
                       int servoCount,
                       int selectedIndex);

  void drawLiveControl(uint8_t servoId,
                       int targetPos,
                       int actualPos,
                       bool torqueEnabled,
                       int minLimit,
                       int maxLimit,
                       int speed,
                       int acc,
                       int selected,
                       bool editing);

  void drawServoInfo(uint8_t servoId,
                     int actualPos,
                     int voltageRaw,
                     int tempC,
                     bool online,
                     int statusByte,
                     int loadPct,
                     int currentMa,
                     int page);        // 0=main info, 1=faults

  void drawConfigMenu(uint8_t servoId,
                      uint8_t stagedId,
                      int minLimit,
                      int maxLimit,
                      int torqueLimit,
                      int centerOffset,
                      int mode,
                      int baudIndex,
                      bool dirty,
                      int selected,
                      bool editing);

  void drawConfirmSave(uint8_t servoId,
                       uint8_t stagedId,
                       int minLimit,
                       int maxLimit,
                       int torqueLimit,
                       int centerOffset,
                       int mode,
                       int baudIndex,
                       bool dirty,
                       int selected);

  void drawSaveResult(bool ok, const char* message);

  void drawScanProgress(uint32_t baud,
                        int lastPingId,
                        int foundCount,
                        int totalIds);

  void drawScanBaudSelect(int selected);

  void drawScanAllProgress(uint32_t currentBaud,
                           int baudStep,
                           int baudTotal,
                           int lastPingId,
                           int foundCount,
                           int totalIds);

  void drawModeWarn(int selected);

  // Flash / LittleFS diagnostics
  void drawPersistDiag(const struct PersistDiag& diag);

  // Protocol selection screen
  void drawSelectProtocol(int activeProtocol, int cursor);

  // MIDI Setup: list of servo bindings, editable CC and channel per row
  // editStep: 0=none, 1=CC, 2=channel, 3=invert, 4=smooth
  void drawMidiSetup(const MidiServoBinding* bindings,
                     uint8_t count,
                     int selected,
                     int editStep,
                     bool usbMounted);

  // MIDI Run: live activity view with per-servo TX/RX indicators + panic row
  // When showMonitor=true, shows the MIDI monitor log instead of servo rows
  void drawMidiRun(const MidiServoBinding* bindings,
                   uint8_t count,
                   int selected,
                   bool usbMounted,
                   bool showMonitor,
                   const MidiLogEntry* log,
                   uint8_t logHead);

private:
  Adafruit_SH1107 _display;
  uint8_t _addr;
  int _rotation = 1;

  void header(const char* title);
  void headerWithRight(const char* title, const char* rightText);
  void footer(const char* text);
  void drawSelectionMarker(int rowY, bool editing);

  float posToDeg(int pos);
  void printDegreeSymbol();
  void printAngleFromPos(int pos);
  void printVoltageFromRaw(int raw);

  void drawPositionBar(int targetPos,
                       int actualPos,
                       int minLimit,
                       int maxLimit,
                       int x,
                       int y,
                       int w,
                       int h);
};
