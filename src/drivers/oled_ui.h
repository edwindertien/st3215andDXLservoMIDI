#pragma once
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include "../config.h"          // must come before display includes for DISPLAY_xxx
#include "../model/servo_model.h"

// Compile-time display backend selection driven by config.h
#if defined(DISPLAY_SSD1306)
  #include <Adafruit_SSD1306.h>
  // SSD1306: constructor is (width, height, wire*, reset_pin)
  #define _OLED_CLASS       Adafruit_SSD1306
  #define _OLED_COLOR_ON    SSD1306_WHITE
  #define _OLED_COLOR_OFF   SSD1306_BLACK
  #define _OLED_CTOR(w,h,wire)  Adafruit_SSD1306((w), (h), (wire), -1)
#else
  // Default: SH1107 (M5Stack 1.3" OLED, Pico Grove config)
  #include <Adafruit_SH110X.h>
  #define _OLED_CLASS       Adafruit_SH1107
  #define _OLED_COLOR_ON    SH110X_WHITE
  #define _OLED_COLOR_OFF   SH110X_BLACK
  #define _OLED_CTOR(w,h,wire)  Adafruit_SH1107((w), (h), (wire))
#endif

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
                      bool editing,
                      const uint32_t* baudTable,
                      int baudCount);

  void drawConfirmSave(uint8_t servoId,
                       uint8_t stagedId,
                       int minLimit,
                       int maxLimit,
                       int torqueLimit,
                       int centerOffset,
                       int mode,
                       int baudIndex,
                       bool dirty,
                       int selected,
                       const uint32_t* baudTable,
                       int baudCount);

  void drawSaveResult(bool ok, const char* message);

  void drawScanProgress(uint32_t baud,
                        int lastPingId,
                        int foundCount,
                        int totalIds);

  void drawScanBaudSelect(int selected, const uint32_t* baudTable, int baudCount);

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
  _OLED_CLASS _display;
  uint8_t _addr;
  int _rotation = HW::OLED_ROTATION;

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