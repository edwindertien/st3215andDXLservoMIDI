#include "oled_ui.h"
#include "persist.h"
#include <stdio.h>

OledUi::OledUi(uint16_t w, uint16_t h, TwoWire* wire, uint8_t addr)
    : _display(_OLED_CTOR(w, h, wire)), _addr(addr) {}

bool OledUi::begin() {
#if defined(DISPLAY_SSD1306)
  if (!_display.begin(SSD1306_SWITCHCAPVCC, _addr)) return false;
#else
  if (!_display.begin(_addr, true)) return false;
#endif
  _display.setRotation(_rotation);
  _display.clearDisplay();
  _display.setTextColor(_OLED_COLOR_ON);
  _display.setTextWrap(false);
  _display.cp437(true);
  _display.display();
  return true;
}

void OledUi::clear() {
  _display.clearDisplay();
  _display.display();
}

void OledUi::splash(const String& l1, const String& l2, const String& l3) {
  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setCursor(0, 0);
  _display.println(l1);
  if (l2.length()) _display.println(l2);
  if (l3.length()) _display.println(l3);
  _display.display();
}

void OledUi::header(const char* title) {
  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setCursor(0, 0);
  _display.println(title);
  _display.drawLine(0, 9, 127, 9, _OLED_COLOR_ON);
}

void OledUi::headerWithRight(const char* title, const char* rightText) {
  _display.clearDisplay();
  _display.setTextSize(1);

  _display.setCursor(0, 0);
  _display.print(title);

  if (rightText && rightText[0] != '\0') {
    int16_t x1, y1;
    uint16_t w, h;
    _display.getTextBounds(rightText, 0, 0, &x1, &y1, &w, &h);
    int x = 127 - (int)w;
    if (x < 0) x = 0;
    _display.setCursor(x, 0);
    _display.print(rightText);
  }

  _display.drawLine(0, 9, 127, 9, _OLED_COLOR_ON);
}

void OledUi::footer(const char* text) {
  if (!text) return;
  _display.drawLine(0, 55, 127, 55, _OLED_COLOR_ON);
  _display.setCursor(0, 57);
  _display.print(text);
}

void OledUi::drawSelectionMarker(int rowY, bool editing) {
  _display.setCursor(0, rowY);
  _display.print(editing ? "*" : ">");
}

float OledUi::posToDeg(int pos) {
  return 360.0f * ((float)pos / 4095.0f);
}

void OledUi::printDegreeSymbol() {
  _display.write((uint8_t)248);
}

void OledUi::printAngleFromPos(int pos) {
  _display.print(posToDeg(pos), 1);
  printDegreeSymbol();
}

void OledUi::printVoltageFromRaw(int raw) {
  if (raw < 0) {
    _display.print("---");
    return;
  }
  float volts = raw / 10.0f;
  _display.print(volts, 1);
  _display.print("V");
}

void OledUi::drawPositionBar(int targetPos,
                             int actualPos,
                             int minLimit,
                             int maxLimit,
                             int x,
                             int y,
                             int w,
                             int h) {
  if (minLimit < 0) minLimit = 0;
  if (maxLimit > 4095) maxLimit = 4095;
  if (minLimit > maxLimit) {
    minLimit = 0;
    maxLimit = 4095;
  }

  _display.drawRect(x, y, w, h, _OLED_COLOR_ON);

  int innerX0 = x + 1;
  int innerX1 = x + w - 2;
  int innerW  = innerX1 - innerX0 + 1;

  int minX = map(minLimit, 0, 4095, innerX0, innerX1);
  int maxX = map(maxLimit, 0, 4095, innerX0, innerX1);

  // Hatch unusable left region
  for (int xx = innerX0; xx < minX; ++xx) {
    if (((xx - innerX0) & 1) == 0) {
      _display.drawLine(xx, y + 1, xx, y + h - 2, _OLED_COLOR_ON);
    }
  }

  // Hatch unusable right region
  for (int xx = maxX + 1; xx <= innerX1; ++xx) {
    if (((xx - innerX0) & 1) == 0) {
      _display.drawLine(xx, y + 1, xx, y + h - 2, _OLED_COLOR_ON);
    }
  }

  // Endpoint markers
  _display.drawLine(minX, y + 1, minX, y + h - 2, _OLED_COLOR_ON);
  _display.drawLine(maxX, y + 1, maxX, y + h - 2, _OLED_COLOR_ON);

  // Target marker: thin line
  int tgtX = map(targetPos, 0, 4095, innerX0, innerX1);
  _display.drawLine(tgtX, y + 1, tgtX, y + h - 2, _OLED_COLOR_ON);

  // Actual marker: filled block
  if (actualPos >= 0) {
    int actX = map(actualPos, 0, 4095, innerX0, innerX1);
    int boxW = 3;
    int boxX = actX - (boxW / 2);
    if (boxX < innerX0) boxX = innerX0;
    if (boxX > innerX1 - boxW + 1) boxX = innerX1 - boxW + 1;
    _display.fillRect(boxX, y + 2, boxW, h - 4, _OLED_COLOR_ON);
  }
}

void OledUi::drawMenu(const char* title,
                      const char* const* items,
                      int itemCount,
                      int selected,
                      bool editing,
                      const char* footerText) {
  header(title);

  int start = 0;
  if (selected >= 4) start = selected - 3;

  for (int row = 0; row < 4; ++row) {
    int idx = start + row;
    if (idx >= itemCount) break;
    int y = 12 + row * 10;
    if (idx == selected) drawSelectionMarker(y, editing);
    _display.setCursor(10, y);
    _display.print(items[idx]);
  }

  footer(footerText);
  _display.display();
}

void OledUi::drawSelectServo(const uint8_t* ids,
                             int servoCount,
                             int selectedIndex) {
  header("Select Servo");

  if (servoCount <= 0) {
    _display.setCursor(0, 20);
    _display.print("No servos found");
    footer("Long=Back");
    _display.display();
    return;
  }

  int start = 0;
  if (selectedIndex >= 4) start = selectedIndex - 3;

  for (int row = 0; row < 4; ++row) {
    int idx = start + row;
    if (idx >= servoCount) break;
    int y = 12 + row * 10;
    if (idx == selectedIndex) drawSelectionMarker(y, false);
    _display.setCursor(10, y);
    _display.print("ID ");
    _display.print(ids[idx]);
  }

  footer("Short=Select Long=Back");
  _display.display();
}

void OledUi::drawLiveControl(uint8_t servoId,
                             int targetPos,
                             int actualPos,
                             bool torqueEnabled,
                             int minLimit,
                             int maxLimit,
                             int speed,
                             int acc,
                             int selected,
                             bool editing) {
  char right[16];
  snprintf(right, sizeof(right), "ID %u", servoId);
  headerWithRight("Live Control", right);

  // Layout (128x64, header=0-9, footer=55-63):
  //   y=11  [>] Trq:ON              180.0°   row0 (selectable), actual angle right
  //   y=22  [>] T:2047              180.0°   row1 (selectable), target angle right
  //   y=33  [>] Spd:800   [>]Acc:30          row2+3 share one line
  //   y=44  position bar (10px tall fits before footer at 55)
  //   y=55  footer

  const int yTrq = 11;
  const int yTgt = 22;
  const int ySpd = 33;
  const int yBar = 44;

  // --- Row 0: Torque toggle | actual angle (display-only) ---
  if (selected == 0) drawSelectionMarker(yTrq, editing);
  _display.setCursor(10, yTrq);
  _display.print("Trq:");
  _display.print(torqueEnabled ? "ON" : "OFF");

  if (actualPos >= 0) {
    char abuf[8];
    float adeg = 360.0f * ((float)actualPos / 4095.0f);
    snprintf(abuf, sizeof(abuf), "%.1f", adeg);
    // Each char is 6px wide at textSize=1; degree symbol counts as one char
    int strW = ((int)strlen(abuf) + 1) * 6;
    _display.setCursor(127 - strW, yTrq);
    _display.print(abuf);
    printDegreeSymbol();
  } else {
    _display.setCursor(110, yTrq);
    _display.print("---");
  }

  // --- Row 1: Target position | target angle ---
  if (selected == 1) drawSelectionMarker(yTgt, editing);
  _display.setCursor(10, yTgt);
  _display.print("T:");
  _display.print(targetPos);

  {
    char tbuf[8];
    float tdeg = 360.0f * ((float)targetPos / 4095.0f);
    snprintf(tbuf, sizeof(tbuf), "%.1f", tdeg);
    int strW = ((int)strlen(tbuf) + 1) * 6;
    _display.setCursor(127 - strW, yTgt);
    _display.print(tbuf);
    printDegreeSymbol();
  }

  // --- Row 2+3: Speed (left half) and Acceleration (right half) on one line ---
  if (selected == 2) drawSelectionMarker(ySpd, editing);
  _display.setCursor(10, ySpd);
  _display.print("Spd:");
  _display.print(speed);

  if (selected == 3) {
    _display.setCursor(66, ySpd);
    _display.print(editing ? "*" : ">");
  }
  _display.setCursor(74, ySpd);
  _display.print("Acc:");
  _display.print(acc);

  // --- Position bar ---
  drawPositionBar(targetPos, actualPos, minLimit, maxLimit, 0, yBar, 128, 9);

  footer(editing ? "Turn=chg Short=OK Long=cancel" : "Short=edit Long=back");
  _display.display();
}

void OledUi::drawServoInfo(uint8_t servoId,
                           int actualPos,
                           int voltageRaw,
                           int tempC,
                           bool online,
                           int statusByte,
                           int loadPct,
                           int currentMa,
                           int page) {
  char right[16];
  snprintf(right, sizeof(right), "ID %u", servoId);

  if (page == 0) {
    // ---- Page 1: runtime values ----
    headerWithRight("Servo Info", right);

    _display.setCursor(0, 12);
    _display.print("Online: ");
    _display.print(online ? "YES" : "NO");

    _display.setCursor(0, 22);
    _display.print("Pos: ");
    if (actualPos >= 0) {
      _display.print(actualPos);
      _display.print(" ");
      printAngleFromPos(actualPos);
    } else {
      _display.print("---");
    }

    _display.setCursor(0, 32);
    _display.print("Volt: ");
    printVoltageFromRaw(voltageRaw);

    _display.setCursor(0, 42);
    _display.print("Temp: ");
    if (tempC >= 0) {
      _display.print(tempC);
      printDegreeSymbol();
      _display.print("C");
    } else {
      _display.print("---");
    }

    footer("Short=faults Long=back");

  } else {
    // ---- Page 2: faults, load, current ----
    headerWithRight("Faults", right);

    // Load and current on first row
    _display.setCursor(0, 12);
    _display.print("Load:");
    _display.print(loadPct);
    _display.print("% Cur:");
    if (currentMa >= 0) { _display.print(currentMa); _display.print("mA"); }
    else                { _display.print("---"); }

    // Fault flags
    if (statusByte < 0) {
      _display.setCursor(0, 22);
      _display.print("Status: ---");
    } else if (statusByte == 0) {
      _display.setCursor(0, 22);
      _display.print("No faults");
      _display.setCursor(0, 32);
      _display.print("All OK");
    } else {
      // Decode bits — print only active faults, up to 3 rows
      const char* faultNames[6] = {
        "Voltage", "Sensor", "Overtemp",
        "Overcurrent", "Angle", "Overload"
      };
      int row = 0;
      for (int bit = 0; bit < 6 && row < 3; ++bit) {
        if (statusByte & (1 << bit)) {
          _display.setCursor(0, 22 + row * 10);
          _display.print("! ");
          _display.print(faultNames[bit]);
          row++;
        }
      }
    }

    footer("Short=info Long=back");
  }

  _display.display();
}

void OledUi::drawConfigMenu(uint8_t servoId,
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
                            int baudCount) {
  char right[20];
  snprintf(right, sizeof(right), "ID %u%s", servoId, dirty ? "*" : "");
  headerWithRight("Configure", right);

  // 7 items: 0=ID, 1=Min, 2=Max, 3=TrqLim, 4=Offset, 5=Mode, 6=Baud
  // Show 4 rows at a time, scroll with selection
  const char* labels[7] = { "NewID", "Min", "Max", "TrqLim", "Offset", "Mode", "Baud" };

  int start = 0;
  if (selected >= 4) start = selected - 3;

  const int y0   = 12;
  const int step = 10;

  for (int row = 0; row < 4; ++row) {
    int idx = start + row;
    if (idx >= 7) break;
    int y = y0 + row * step;

    if (idx == selected) drawSelectionMarker(y, editing);
    _display.setCursor(10, y);
    _display.print(labels[idx]);
    _display.print(":");

    switch (idx) {
      case 0: _display.print(stagedId);                               break;
      case 1: _display.print(minLimit);                               break;
      case 2: _display.print(maxLimit);                               break;
      case 3: _display.print(torqueLimit);                            break;
      case 4: _display.print(centerOffset);                           break;
      case 5: _display.print(mode == 0 ? "Servo" : "Wheel");         break;
      case 6: {
        int bi = (baudIndex >= 0 && baudIndex < baudCount) ? baudIndex : 0;
        uint32_t b = baudTable[bi];
        if (b >= 1000000UL)      { _display.print(b / 1000000UL); _display.print("M"); }
        else if (b >= 1000UL)    { _display.print(b / 1000UL);    _display.print("K"); }
        else                     { _display.print(b); }
        break;
      }
    }
  }

  footer(editing ? "Turn=chg Shrt=OK Lng=cancel" : "Shrt=edit Lng=back");
  _display.display();
}

void OledUi::drawConfirmSave(uint8_t servoId,
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
                             int baudCount) {
  char right[20];
  snprintf(right, sizeof(right), "ID %u", servoId);
  headerWithRight("Save Config?", right);

  // Compact layout — 7 values crammed into 3 rows (pairs/triples)
  _display.setCursor(0, 12);
  _display.print("ID:");  _display.print(stagedId);
  _display.print(" Mn:"); _display.print(minLimit);
  _display.print(" Mx:"); _display.print(maxLimit);

  _display.setCursor(0, 22);
  _display.print("TL:"); _display.print(torqueLimit);
  _display.print(" Ofs:"); _display.print(centerOffset);

  _display.setCursor(0, 32);
  _display.print("Md:"); _display.print(mode == 0 ? "Srv" : "Whl");
  {
    int bi = (baudIndex >= 0 && baudIndex < baudCount) ? baudIndex : 0;
    uint32_t b = baudTable[bi];
    _display.print(" Bd:");
    if (b >= 1000000UL)   { _display.print(b / 1000000UL); _display.print("M"); }
    else if (b >= 1000UL) { _display.print(b / 1000UL);    _display.print("K"); }
    else                  { _display.print(b); }
  }
  if (dirty) _display.print("*");

  // Yes / No buttons at y=44
  const int y = 44;
  if (selected == 0) {
    _display.fillRect(0, y - 1, 28, 9, _OLED_COLOR_ON);
    _display.setTextColor(_OLED_COLOR_OFF);
    _display.setCursor(4, y);  _display.print("No");
    _display.setTextColor(_OLED_COLOR_ON);
    _display.setCursor(40, y); _display.print("Yes");
  } else {
    _display.setCursor(4, y);  _display.print("No");
    _display.fillRect(36, y - 1, 32, 9, _OLED_COLOR_ON);
    _display.setTextColor(_OLED_COLOR_OFF);
    _display.setCursor(40, y); _display.print("Yes");
    _display.setTextColor(_OLED_COLOR_ON);
  }

  _display.display();
}

void OledUi::drawSaveResult(bool ok, const char* message) {
  header(ok ? "Save OK" : "Save Failed");
  _display.setCursor(0, 16);
  _display.println(message);
  footer("Short/Long=Back");
  _display.display();
}

void OledUi::drawScanProgress(uint32_t baud,
                              int lastPingId,
                              int foundCount,
                              int totalIds) {
  header("Scanning Bus");

  _display.setCursor(0, 14);
  _display.print("Baud: ");
  _display.println(baud);

  _display.setCursor(0, 24);
  _display.print("Poll ID: ");
  _display.println(lastPingId);

  _display.setCursor(0, 34);
  _display.print("Found: ");
  _display.println(foundCount);

  int progress = lastPingId;
  if (progress < 0) progress = 0;
  if (progress > totalIds) progress = totalIds;

  const int barX = 0;
  const int barY = 46;
  const int barW = 128;
  const int barH = 6;

  _display.drawRect(barX, barY, barW, barH, _OLED_COLOR_ON);
  int fillW = map(progress, 0, totalIds, 0, barW - 2);
  if (fillW > 0) {
    _display.fillRect(barX + 1, barY + 1, fillW, barH - 2, _OLED_COLOR_ON);
  }

  footer("Long=stop");
  _display.display();
}
// ---------------------------------------------------------------------------
// Baud-select screen: 8 named baud rates + "Scan All" at the bottom
// ---------------------------------------------------------------------------
void OledUi::drawScanBaudSelect(int selected, const uint32_t* baudTable, int baudCount) {
  header("Scan Baud Rate");

  int start = 0;
  if (selected >= 4) start = selected - 3;

  const int y0    = 12;
  const int step  = 10;
  const int total = baudCount + 1; // rates + Scan All

  for (int row = 0; row < 4; ++row) {
    int idx = start + row;
    if (idx >= total) break;
    int y = y0 + row * step;

    if (idx == selected) drawSelectionMarker(y, false);
    _display.setCursor(10, y);

    if (idx < baudCount) {
      uint32_t b = baudTable[idx];
      if (b >= 1000000UL) {
        _display.print(b / 1000000UL);
        _display.print(",000,000");
      } else {
        _display.print(b);
      }
    } else {
      _display.print(">> Scan All <<");
    }
  }

  footer("Short=scan Long=back");
  _display.display();
}

// ---------------------------------------------------------------------------
// Scan-all progress: shows which baud step we are on and ID scan progress
// ---------------------------------------------------------------------------
void OledUi::drawScanAllProgress(uint32_t currentBaud,
                                 int baudStep,
                                 int baudTotal,
                                 int lastPingId,
                                 int foundCount,
                                 int totalIds) {
  header("Scan All Bauds");

  // Baud step progress bar at top (which of the 8 baud rates)
  _display.setCursor(0, 12);
  _display.print("Baud ");
  _display.print(baudStep + 1);
  _display.print("/");
  _display.print(baudTotal);
  _display.print(": ");
  if (currentBaud >= 1000000UL) {
    _display.print(currentBaud / 1000000UL); _display.print("M");
  } else if (currentBaud >= 1000UL) {
    _display.print(currentBaud / 1000UL);    _display.print("K");
  } else {
    _display.print(currentBaud);
  }

  // Baud step bar
  {
    const int bx = 0, by = 22, bw = 128, bh = 5;
    _display.drawRect(bx, by, bw, bh, _OLED_COLOR_ON);
    int fill = map(baudStep, 0, baudTotal, 0, bw - 2);
    if (fill > 0) _display.fillRect(bx + 1, by + 1, fill, bh - 2, _OLED_COLOR_ON);
  }

  _display.setCursor(0, 30);
  _display.print("ID: ");
  _display.print(lastPingId < 0 ? 0 : lastPingId);
  _display.print("  Found: ");
  _display.print(foundCount);

  // ID scan progress bar
  {
    int progress = (lastPingId < 0) ? 0 : lastPingId;
    if (progress > totalIds) progress = totalIds;
    const int bx = 0, by = 40, bw = 128, bh = 5;
    _display.drawRect(bx, by, bw, bh, _OLED_COLOR_ON);
    int fill = map(progress, 0, totalIds, 0, bw - 2);
    if (fill > 0) _display.fillRect(bx + 1, by + 1, fill, bh - 2, _OLED_COLOR_ON);
  }

  footer("Long=stop");
  _display.display();
}

// ---------------------------------------------------------------------------
// Mode warning: shown before switching to wheel mode
// ---------------------------------------------------------------------------
void OledUi::drawModeWarn(int selected) {
  header("! Wheel Mode !");

  _display.setCursor(0, 12);
  _display.print("Wheel mode removes");
  _display.setCursor(0, 21);
  _display.print("position limits.");
  _display.setCursor(0, 30);
  _display.print("Hardware damage");
  _display.setCursor(0, 39);
  _display.print("may occur!");

  // No / Yes buttons at y=50
  const int y = 50;
  if (selected == 0) {
    _display.fillRect(0, y - 1, 28, 9, _OLED_COLOR_ON);
    _display.setTextColor(_OLED_COLOR_OFF);
    _display.setCursor(4, y);  _display.print("No");
    _display.setTextColor(_OLED_COLOR_ON);
    _display.setCursor(40, y); _display.print("Yes");
  } else {
    _display.setCursor(4, y);  _display.print("No");
    _display.fillRect(36, y - 1, 32, 9, _OLED_COLOR_ON);
    _display.setTextColor(_OLED_COLOR_OFF);
    _display.setCursor(40, y); _display.print("Yes");
    _display.setTextColor(_OLED_COLOR_ON);
  }

  _display.display();
}

// ---------------------------------------------------------------------------
// MIDI Setup screen
// Each row: [>] ID:1  CC:7  Ch:1
// Last virtual row = ">> Run / Start <<"
// If no servos: show message.
// ---------------------------------------------------------------------------
void OledUi::drawMidiSetup(const MidiServoBinding* bindings,
                            uint8_t count,
                            int selected,
                            int editStep,
                            bool usbMounted) {
  // editStep: 0=navigating, 1=CC, 2=channel, 3=invert, 4=smooth

  const int totalBindings = (int)count + 3; // servo + Spd + Acc + Smt
  const int TOTAL_ROWS    = totalBindings + 1;

  headerWithRight("MIDI Setup", usbMounted ? "USB" : "noUSB");

  if (count == 0) {
    _display.setCursor(0, 18); _display.print("No servos found.");
    _display.setCursor(0, 29); _display.print("Scan bus first.");
    footer("Long=back");
    _display.display();
    return;
  }

  int start = 0;
  if (selected >= 4) start = selected - 3;

  const int y0 = 12, step = 10;

  // Fixed pixel columns (textSize=1, 6px/char, display 128px wide):
  //  x=8  : label  3 chars right-aligned ID or Spd/Acc/Smt
  //  x=32 : C:nnn  CC number (max 3 digits = 127)
  //  x=62 : c:nn   channel  (max 2 digits = 16)
  //  x=90 : [I]    invert indicator (per-servo only)
  //  x=108: nnn    smooth value (per-servo only)

  for (int row = 0; row < 4; ++row) {
    int idx = start + row;
    if (idx >= TOTAL_ROWS) break;
    int y   = y0 + row * step;
    bool isSel = (idx == selected);

    if (idx == totalBindings) {
      if (isSel) drawSelectionMarker(y, false);
      _display.setCursor(10, y);
      _display.print(">Run<");
      continue;
    }

    const MidiServoBinding& b = bindings[idx];
    if (isSel) drawSelectionMarker(y, editStep != 0);

    bool isGlobal = (b.servoId == MIDI_GLOBAL_SPEED ||
                     b.servoId == MIDI_GLOBAL_ACC   ||
                     b.servoId == MIDI_GLOBAL_SMOOTH);

    // --- Label (x=8) ---
    _display.setCursor(8, y);
    if      (b.servoId == MIDI_GLOBAL_SPEED)  _display.print("Spd");
    else if (b.servoId == MIDI_GLOBAL_ACC)    _display.print("Acc");
    else if (b.servoId == MIDI_GLOBAL_SMOOTH) _display.print("Smt");
    else {
      if      (b.servoId <  10) _display.print("  ");
      else if (b.servoId < 100) _display.print(" ");
      _display.print(b.servoId);
    }

    // --- CC (x=32) ---
    _display.setCursor(32, y);
    if (isSel && editStep == 1) _display.print("[");
    else                        _display.print("C:");
    if (b.cc == MIDI_CC_NONE)   _display.print("--");
    else                        _display.print(b.cc);
    if (isSel && editStep == 1) _display.print("]");

    // --- Channel (x=62) ---
    _display.setCursor(62, y);
    if (isSel && editStep == 2) _display.print("[");
    else                        _display.print("c:");
    _display.print(b.channel);
    if (isSel && editStep == 2) _display.print("]");

    // --- Invert + Smooth (per-servo only) ---
    if (!isGlobal) {
      // Invert (x=90) — [I]/[-] when editing, I/- otherwise
      _display.setCursor(90, y);
      if (isSel && editStep == 3)
        _display.print(b.inverted ? "[I]" : "[-]");
      else
        _display.print(b.inverted ? " I " : " - ");

      // Smooth (x=108) — [nn]/nn when editing, -- when zero
      _display.setCursor(108, y);
      if (isSel && editStep == 4) _display.print("[");
      if (b.smoothing == 0)       _display.print("--");
      else                        _display.print(b.smoothing);
      if (isSel && editStep == 4) _display.print("]");
    }
  }

  switch (editStep) {
    case 1: footer("Turn=CC   Shrt=next"); break;
    case 2: footer("Turn=Ch   Shrt=next"); break;
    case 3: footer("Turn=Inv  Shrt=next"); break;
    case 4: footer("Turn=Smt  Shrt=done"); break;
    default:footer("Shrt=edit Lng=back");  break;
  }

  _display.display();
}

void OledUi::drawMidiRun(const MidiServoBinding* bindings,
                          uint8_t count,
                          int selected,
                          bool usbMounted,
                          bool showMonitor,
                          const MidiLogEntry* log,
                          uint8_t logHead) {

  if (showMonitor) {
    headerWithRight("MIDI Monitor", usbMounted ? "USB" : "!USB");
    const int y0 = 12, step = 11;
    for (int row = 0; row < MIDI_LOG_SIZE; ++row) {
      int idx = ((int)logHead - 1 - row + MIDI_LOG_SIZE) % MIDI_LOG_SIZE;
      const MidiLogEntry& e = log[idx];
      _display.setCursor(0, y0 + row * step);
      if (!e.valid) { _display.print("---"); continue; }
      switch (e.type) {
        case MidiMsgType::CC:
          _display.print("CC"); _display.print(e.byte1);
          _display.print(" Ch:"); _display.print(e.channel);
          _display.print(" ="); _display.print(e.byte2); break;
        case MidiMsgType::NoteOn:
          _display.print("NOn N:"); _display.print(e.byte1);
          _display.print(" v:"); _display.print(e.byte2);
          _display.print(" c:"); _display.print(e.channel); break;
        case MidiMsgType::NoteOff:
          _display.print("NOff N:"); _display.print(e.byte1);
          _display.print(" c:"); _display.print(e.channel); break;
        case MidiMsgType::PitchBend:
          _display.print("PB Ch:"); _display.print(e.channel);
          _display.print(" "); _display.print(e.int14); break;
        case MidiMsgType::ProgramChange:
          _display.print("PC Ch:"); _display.print(e.channel);
          _display.print(" P:"); _display.print(e.byte1); break;
        case MidiMsgType::AfterTouch:
          _display.print("AT Ch:"); _display.print(e.channel);
          _display.print(" ="); _display.print(e.byte1); break;
        default:
          _display.print("?? Ch:"); _display.print(e.channel); break;
      }
    }
    footer("Any key=back");
    _display.display();
    return;
  }

  // ---- Servo activity list ----
  // Rows: 0..totalBindings-1 = servos+globals (bindingCount+3), Monitor, Panic
  const int totalBindings = (int)count + 3;  // +Spd +Acc +Smt
  const int TOTAL_ROWS    = totalBindings + 2; // +Monitor +Panic

  headerWithRight("MIDI Run", usbMounted ? "USB" : "!USB");

  int start = 0;
  if (selected >= 4) start = selected - 3;

  const int y0   = 12;
  const int step = 10;

  for (int row = 0; row < 4; ++row) {
    int idx = start + row;
    if (idx >= TOTAL_ROWS) break;
    int y = y0 + row * step;
    bool isSelected = (idx == selected);

    if (idx == totalBindings) {
      // Monitor row
      if (isSelected) drawSelectionMarker(y, false);
      _display.setCursor(10, y);
      _display.print("MIDI Monitor");

    } else if (idx == totalBindings + 1) {
      // Panic row
      if (isSelected) {
        _display.fillRect(0, y - 1, 128, 10, _OLED_COLOR_ON);
        _display.setTextColor(_OLED_COLOR_OFF);
        _display.setCursor(10, y);
        _display.print("!! MIDI PANIC !!");
        _display.setTextColor(_OLED_COLOR_ON);
      } else {
        _display.setCursor(10, y);
        _display.print("!! MIDI PANIC !!");
      }

    } else {
      // Servo or global binding row
      const MidiServoBinding& b = bindings[idx];
      if (isSelected) drawSelectionMarker(y, false);

      _display.setCursor(10, y);
      // Label
      if (b.servoId == MIDI_GLOBAL_SPEED)       _display.print("Spd");
      else if (b.servoId == MIDI_GLOBAL_ACC)    _display.print("Acc");
      else if (b.servoId == MIDI_GLOBAL_SMOOTH) _display.print("Smt");
      else { _display.print("ID:"); _display.print(b.servoId); }

      if (b.cc == MIDI_CC_NONE) {
        _display.print(" CC:--");
      } else {
        _display.print(" C:");
        _display.print(b.cc);
        _display.print(b.rxFlash > 0 ? " \x11" : "  ");
        if (b.lastRecv >= 0) _display.print((int8_t)b.lastRecv);
        else                 _display.print("--");
        _display.print(b.txFlash > 0 ? "\x10" : " ");
        if (b.lastSent >= 0) _display.print((int8_t)b.lastSent);
        else                 _display.print("--");
      }
    }
  }

  footer("Short=select Long=setup");
  _display.display();
}

// ---------------------------------------------------------------------------
// Persist diagnostics screen
// ---------------------------------------------------------------------------
void OledUi::drawPersistDiag(const PersistDiag& d) {
  header("Flash Diag");

  const int y0   = 12;
  const int step = 10;

  // Row 0: mounted + filesystem size
  _display.setCursor(0, y0);
  _display.print("FS:");
  _display.print(d.mounted ? "OK" : "FAIL");
  if (d.mounted && d.totalBytes > 0) {
    _display.print(" ");
    _display.print(d.usedBytes / 1024);
    _display.print("/");
    _display.print(d.totalBytes / 1024);
    _display.print("K");
  }

  // Row 1: file exists + size vs expected
  _display.setCursor(0, y0 + step);
  _display.print("File:");
  if (!d.fileExists) {
    _display.print("MISSING");
  } else {
    _display.print(d.fileSize);
    _display.print("B exp:");
    _display.print(d.expectedSize);
    _display.print("B");
  }

  // Row 2: magic check
  _display.setCursor(0, y0 + step * 2);
  _display.print("Magic:");
  if (!d.fileExists) {
    _display.print("---");
  } else if (d.magic == d.expectedMagic) {
    _display.print("OK");
  } else {
    _display.print("BAD ");
    _display.print(d.magic, HEX);
  }

  // Row 3: version check
  _display.setCursor(0, y0 + step * 3);
  _display.print("Ver:");
  if (!d.fileExists) {
    _display.print("---");
  } else {
    _display.print(d.version);
    _display.print(d.version == d.expectedVersion ? " OK" : " BAD");
    _display.print(" exp:");
    _display.print(d.expectedVersion);
  }

  // Row 4: servo + MIDI counts
  _display.setCursor(0, y0 + step * 4);
  if (d.fileExists && d.magic == d.expectedMagic && d.version == d.expectedVersion) {
    _display.print("Srv:");
    _display.print(d.savedServoCount);
    _display.print(" MIDI:");
    _display.print(d.savedMidiCount);
  } else {
    _display.print("No valid data");
  }

  footer("Long=back");
  _display.display();
}

// ---------------------------------------------------------------------------
// Protocol selection screen
// ---------------------------------------------------------------------------
void OledUi::drawSelectProtocol(int activeProtocol, int cursor) {
  header("Select Protocol");

  const int y0   = 12;
  const int step = 12;

  for (int i = 0; i < BUS_PROTOCOL_COUNT; ++i) {
    int y = y0 + i * step;
    bool isCursor = (i == cursor);
    bool isActive = (i == activeProtocol);

    if (isCursor) {
      _display.fillRect(0, y - 1, 128, 11, _OLED_COLOR_ON);
      _display.setTextColor(_OLED_COLOR_OFF);
    }

    _display.setCursor(4, y);
    _display.print(isActive ? "\x10 " : "  "); // filled arrow = current
    _display.print(BUS_PROTOCOL_NAMES[i]);

    if (isCursor) _display.setTextColor(_OLED_COLOR_ON);
  }

  footer("Shrt=select  Lng=back");
  _display.display();
}