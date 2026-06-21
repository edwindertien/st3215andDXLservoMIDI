#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TinyUSB.h>

#include "config.h"
#include "app_state.h"
#include "model/servo_model.h"

#include "drivers/iservo_bus.h"
#include "drivers/servo_bus.h"
#include "drivers/dxl1_bus.h"
#include "drivers/dxl2_bus.h"
#include "drivers/rcpwm_bus.h"
#include "drivers/oled_ui.h"
#include "drivers/midi_engine.h"
#include "drivers/persist.h"
#include "drivers/usb_host_engine.h"
#include "app/app.h"

// ---------------------------------------------------------------------------
// Encoder: select type based on config.h
// ---------------------------------------------------------------------------
#if defined(ENCODER_M5_8UNIT)
  #include "drivers/encoder_8unit.h"
  using ActiveEncoder = Encoder8Unit;
  static Encoder8Unit encoder(Wire, HW::ENC_8_ADDR);
#else
  #include "drivers/encoder_unit.h"
  using ActiveEncoder = UnitEncoder;
  // On BOARD_PICO_GROVE the OLED lives on Wire (I2C0, GP4/GP5) and the
  // encoder lives on Wire1 (I2C1, GP6/GP7).  Passing Wire here would cause
  // encoder.begin() to call setSDA/setSCL/begin on Wire, overwriting the OLED
  // bus configuration and breaking display communication.
  #if defined(BOARD_PICO_GROVE)
    static UnitEncoder encoder(Wire1, HW::ENC_ADDR);
  #else
    static UnitEncoder encoder(Wire, HW::ENC_ADDR);
  #endif
#endif

// ---------------------------------------------------------------------------
// App state and peripherals
// ---------------------------------------------------------------------------
AppState appState;
OledUi   ui(HW::OLED_W, HW::OLED_H, &Wire, HW::OLED_ADDR);
App      app(appState, (IEncoder&)encoder, ui, &st3215Bus, midiEngine);

IServoBus* busForProtocol(BusProtocol p) {
  switch (p) {
    case BusProtocol::DXL1:  return &dxl1Bus;
    case BusProtocol::DXL2:  return &dxl2Bus;
    case BusProtocol::RCPWM: return &rcpwmBus;
    case BusProtocol::SC09:
      st3215Bus.setScsMode(true);   // lock reg 48, posMax 1023
      return &st3215Bus;
    default:
      st3215Bus.setScsMode(false);  // lock reg 55, posMax 4095
      return &st3215Bus;
  }
}

// ---------------------------------------------------------------------------
// rescueDxl1Baud() — one-shot baud recovery for a bricked DXL1 servo.
//
// Call from setup() when a servo is stuck at an unexpected baud (e.g. after
// using Dynamixel Wizard set it to 9600).  Tries every baud in DXL1_BAUD_TABLE,
// sends a WRITE to reg 4 (BAUD_RATE) with the target register value, then
// switches to the target baud and verifies with a ping.
//
// Parameters:
//   servoId     — the servo's current ID (2 in your case)
//   targetBaud  — the baud you want to restore (1000000)
//
// Set RESCUE_ENABLED to 1 to activate, 0 to skip (normal operation).
// After a successful recovery: set back to 0, rebuild, and re-flash.
// ---------------------------------------------------------------------------
#define RESCUE_ENABLED      0   // ← set to 1 to run recovery, then back to 0
#define PASSTHROUGH_ENABLED 0   // ← set to 1 to use Pico as transparent RS485↔USB bridge
                                 //   Dynamixel Wizard connects to the USB CDC port and
                                 //   talks directly to the servo at whatever baud it sets.

static void rescueDxl1Baud(uint8_t servoId, uint32_t targetBaud) {
#if RESCUE_ENABLED
  // Register value for target baud: reg = (2000000 / baud) - 1
  uint8_t targetReg = (uint8_t)((2000000UL / targetBaud) - 1);

  // All DXL1 baud rates to try — includes 9615 (~9600) which Wizard calls "9600"
  static const uint32_t tryBauds[] = {
    9615, 19230, 57142, 100000, 117647, 200000, 250000, 1000000
  };

  Serial.println(F("[RESCUE] DXL1 baud recovery starting..."));
  Serial.print(F("[RESCUE] Target: ")); Serial.print(targetBaud);
  Serial.print(F(" bd (reg=")); Serial.print(targetReg); Serial.println(')');

  for (uint32_t baud : tryBauds) {
    Serial.print(F("[RESCUE] Trying ")); Serial.print(baud); Serial.print(F(" bd... "));
    dxl1Bus.setBaud(baud);
    delay(10);

    if (!dxl1Bus.ping(servoId)) {
      Serial.println(F("no response"));
      continue;
    }

    Serial.print(F("FOUND! Writing reg 4 = "));
    Serial.print(targetReg);
    Serial.print(F(" ... "));

    // Torque off before EEPROM write
    dxl1Bus.torqueEnable(servoId, false);
    delay(10);

    // Raw WRITE: reg 4, value = targetReg
    // Uses the existing writeByte path which handles STATUS_RETURN_LEVEL=1
    uint8_t params[2] = { 4, targetReg };
    // sendInstruction is private — use saveBaud which does the same thing
    // saveBaud index 7 = 1000000 in DXL1_BAUD_TABLE
    int idx = -1;
    for (int i = 0; i < DXL1_BAUD_COUNT; ++i)
      if (DXL1_BAUD_TABLE[i] == targetBaud) { idx = i; break; }

    if (idx < 0) {
      Serial.println(F("ERROR: targetBaud not in table!"));
      return;
    }

    bool wrote = dxl1Bus.saveBaud(servoId, idx);
    delay(100); // EEPROM settle — generous

    // Switch to target baud and verify
    dxl1Bus.setBaud(targetBaud);
    delay(20);
    bool verified = dxl1Bus.ping(servoId);

    if (verified) {
      Serial.println(F("OK"));
      Serial.println(F("[RESCUE] SUCCESS — servo responding at target baud."));
      Serial.println(F("[RESCUE] Set RESCUE_ENABLED 0, rebuild, and re-flash."));
      StatusLed::green();
      // Blink green rapidly to signal success
      for (int i = 0; i < 10; ++i) {
        StatusLed::green(); delay(150);
        StatusLed::off();   delay(150);
      }
    } else {
      Serial.println(F("wrote but ping at target baud failed — check wiring"));
      StatusLed::red();
    }
    return; // found the servo — don't try other bauds
  }

  Serial.println(F("[RESCUE] FAILED — servo not found at any baud."));
  Serial.println(F("  Check: ID correct? RS-485 wiring? Power?"));
  // Blink red slowly to signal failure
  StatusLed::red();
  for (int i = 0; i < 6; ++i) {
    StatusLed::red(); delay(500);
    StatusLed::off(); delay(500);
  }
#endif // RESCUE_ENABLED
}

static void onCCReceived(uint8_t channel, uint8_t cc, uint8_t value) {
  app.onMidiCC(channel, cc, value);
}
static void onAnyReceived(MidiMsgType type, uint8_t channel,
                           uint8_t byte1, uint8_t byte2, int16_t int14) {
  app.onMidiAny(type, channel, byte1, byte2, int14);
}

// ---------------------------------------------------------------------------
// Core 0 setup / loop
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Passthrough mode — Pico acts as transparent USB-CDC ↔ RS485 bridge.
//
// Dynamixel Wizard (or any tool) connects to the USB CDC COM port.
// The Pico mirrors whatever baud the host sets on the CDC port to Serial1,
// and forwards bytes in both directions with no parsing or timing overhead.
// The auto-direction RS485 adapter handles TX/RX switching automatically.
//
// Usage:
//   1. Set PASSTHROUGH_ENABLED 1 above, rebuild and flash.
//   2. Open Dynamixel Wizard, select the Pico's CDC COM port.
//   3. Wizard will find and communicate with servos directly.
//   4. When done, set PASSTHROUGH_ENABLED 0 and reflash normally.
// ---------------------------------------------------------------------------

#if PASSTHROUGH_ENABLED

static uint32_t _ptBaud = 0;

static void passthroughSetup() {
  // Minimal USB init — just bring up CDC, skip MIDI and the whole app stack
  Serial.begin(57600);          // initial CDC baud (Wizard will change it)
  unsigned long t = millis();
  while (!Serial && millis() - t < 2000) delay(10); // wait for CDC enumeration

  Serial1.setTX(HW::SERVO_TX_PIN);
  Serial1.setRX(HW::SERVO_RX_PIN);
  _ptBaud = 57600;
  Serial1.begin(_ptBaud);

  StatusLed::begin();
  StatusLed::blue(); // blue = passthrough mode active
}

static void passthroughLoop() {
  // Track CDC baud changes — Wizard sets a new baud when scanning
  uint32_t hostBaud = Serial.baud();
  if (hostBaud > 0 && hostBaud != _ptBaud) {
    _ptBaud = hostBaud;
    Serial1.begin(_ptBaud);
  }

  // USB CDC → RS485 (host/Wizard → servo)
  while (Serial.available()) {
    Serial1.write((uint8_t)Serial.read());
  }

  // RS485 → USB CDC (servo → host/Wizard)
  while (Serial1.available()) {
    Serial.write((uint8_t)Serial1.read());
  }
}

#endif // PASSTHROUGH_ENABLED

// ---------------------------------------------------------------------------

void setup() {
#if PASSTHROUGH_ENABLED
  passthroughSetup();
  return;
#endif
  // Status LED + Buzzer
  StatusLed::begin();
  Buzzer::begin();
  StatusLed::amber();

  // USB MIDI device must be first — also brings up CDC if CFG_TUD_CDC=1
  midiEngine.setOnCC(onCCReceived);
  midiEngine.setOnAny(onAnyReceived);
  midiEngine.begin();
  { unsigned long t = millis();
    while (!midiEngine.isMounted() && millis() - t < 1500) delay(10); }

  // CDC serial — open early so boot messages appear even if I2C hangs later.
  Serial.begin(115200);

  // I2C bus recovery — clock SCL 9 times before Wire.begin() to free any
  // slave device that is holding SDA low from a previous failed transaction.
  // Without this, Wire.begin() on RP2040 can hang indefinitely if the OLED
  // or encoder was mid-transaction when the MCU was reset.
  pinMode(HW::OLED_SDA_PIN, INPUT_PULLUP);
  pinMode(HW::OLED_SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; ++i) {
    digitalWrite(HW::OLED_SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(HW::OLED_SCL_PIN, LOW);  delayMicroseconds(5);
  }
  // STOP condition: SDA low → SDA high while SCL high
  pinMode(HW::OLED_SDA_PIN, OUTPUT);
  digitalWrite(HW::OLED_SDA_PIN, LOW);
  digitalWrite(HW::OLED_SCL_PIN, HIGH); delayMicroseconds(5);
  digitalWrite(HW::OLED_SDA_PIN, HIGH); delayMicroseconds(5);

  Wire.setSDA(HW::OLED_SDA_PIN);
  Wire.setSCL(HW::OLED_SCL_PIN);
  Wire.begin();
  Wire.setClock(HW::ENC_I2C_HZ);
  Serial.println(F("[BOOT] Wire init OK"));

#if defined(BOARD_PICO_GROVE)
  Wire1.setSDA(HW::ENC_SDA_PIN);
  Wire1.setSCL(HW::ENC_SCL_PIN);
  Wire1.begin();
  Wire1.setClock(HW::ENC_I2C_HZ);
#endif

  // OLED
  appState.oledOk = ui.begin();
  if (appState.oledOk) ui.splash("Servo Tester",
#if defined(BOARD_PICO_GROVE)
    "Pico+Grove"
#elif defined(BOARD_CUSTOM_RS485)
    "Pico+RS485"
#else
    "XIAO+RS485"
#endif
  );

  // Encoder
#if defined(ENCODER_M5_8UNIT)
  appState.encoderOk = encoder.begin();
  if (appState.encoderOk) encoder.setLedColor(0, 0x002000);
#else
  appState.encoderOk = encoder.begin(HW::ENC_SDA_PIN, HW::ENC_SCL_PIN, HW::ENC_I2C_HZ);
  if (appState.encoderOk) encoder.setLedColor(0, 0x002000);
#endif

  // Servo UART
  Serial1.setTX(HW::SERVO_TX_PIN);
  Serial1.setRX(HW::SERVO_RX_PIN);
  st3215Bus.begin(Serial1, HW::SERVO_BAUD);
  dxl1Bus.begin(Serial1, 57142);   // DXL1 factory default: 2000000/(34+1) = 57142, NOT 57600
  dxl2Bus.begin(Serial1, 57600);   // DXL2 factory default: true 57600
  rcpwmBus.begin(Serial1, 0);      // ignores both args — configures 6 PWM GPIO instead

  usbHost.begin(HW::USB_HOST_DP_PIN, HW::USB_HOST_DM_PIN);
  persist.begin();

  // Baud recovery — only runs when RESCUE_ENABLED 1 in rescueDxl1Baud() above
  rescueDxl1Baud(/*servoId=*/2, /*targetBaud=*/1000000UL);

  if (appState.oledOk) ui.splash("Starting...");
  app.begin();

  Serial.println(F("[BOOT] Servo Tester ready. Commands:"));
  Serial.println(F("  p = DXL2 raw ping ID1 @ 57600"));
  Serial.println(F("  b = DXL2 broadcast ping @ 57600"));
  Serial.println(F("  d = DXL1 ping ID1 via dxl1Bus (57142 baud)"));
  Serial.println(F("  e = DXL1 raw bytes ping ID1 @ 57142 (loopback test)"));
  Serial.println(F("  a = DXL2 ping ID1 on ALL baud rates"));
  Serial.println(F("  w = DXL2 Wizard-exact ping @ 1Mbaud (hardcoded CRC 19 4E)"));
  Serial.println(F("  x = DE-pin loopback: toggle GP2 DE, send 0xAA, check RX echo"));
  Serial.println(F("  i = I2C scan on Wire (GP4/GP5) — find actual OLED address"));
  Serial.println(F("  m = SC09 max-limit write diagnostic (ID=1)"));
  Serial.print(F("[BOOT] oledOk="));    Serial.print(appState.oledOk    ? "YES" : "NO");
  Serial.print(F("  encoderOk="));      Serial.println(appState.encoderOk ? "YES" : "NO");
  if (HW::SERVO_DE_PIN >= 0) {
    Serial.print(F("[BOOT] SERVO_DE_PIN=")); Serial.println((int)HW::SERVO_DE_PIN);
  }

  StatusLed::green();
  Buzzer::beep(1800, 50);
  delay(200);
}

void loop() {
#if PASSTHROUGH_ENABLED
  passthroughLoop();
  return;
#endif
  app.tick();

  // Serial debug commands (non-blocking check)
  if (Serial.available()) {
    char cmd = (char)Serial.read();
    if (cmd == 'p' || cmd == 'P') {
      dxl2Bus.setBaud(57600);
      Serial.println(F("\n--- DXL2 raw ping ID=1 @ 57600 ---"));
      dxl2Bus.rawPingDump(1, Serial);
    }
    else if (cmd == 'b' || cmd == 'B') {
      dxl2Bus.setBaud(57600);
      Serial.println(F("\n--- DXL2 broadcast ping @ 57600 ---"));
      dxl2Bus.rawBroadcastDump(Serial);
    }
    else if (cmd == 'd' || cmd == 'D') {
      // DXL1 ping via the actual dxl1Bus driver — tests the full code path
      dxl1Bus.setBaud(57142);
      bool ok = dxl1Bus.ping(1);
      int mv = -1, tc = -1;
      if (ok) { dxl1Bus.readVoltage(1, mv); dxl1Bus.readTemperature(1, tc); }
      // All Serial prints AFTER servo comms
      Serial.println(F("\n--- DXL1Bus ping ID=1 @ 57142 ---"));
      Serial.print(F("ping(1) = "));
      Serial.println(ok ? F("OK") : F("FAIL (timeout or bad checksum)"));
      if (!ok) {
        Serial.println(F("  Tips:"));
        Serial.println(F("  - Check RS-485 wiring (A/B not swapped)"));
        Serial.println(F("  - Servo may be at a different baud -> use 'a' to scan all"));
        Serial.println(F("  - Use 'e' to test raw loopback (adapter RX path)"));
      } else {
        Serial.print(F("  Voltage: ")); Serial.print(mv); Serial.println(F(" mV"));
        Serial.print(F("  Temp:    ")); Serial.print(tc); Serial.println(F(" degC"));
      }
    }
    else if (cmd == 'e' || cmd == 'E') {
      // Raw byte loopback — bypasses dxl1Bus, confirms RS485 TX and RX path.
      // Manually asserts DE pin if wired (SERVO_DE_PIN >= 0).
      Serial1.end();
      Serial1.begin(57142);
      while (Serial1.available()) Serial1.read();

      constexpr int8_t dePin = HW::SERVO_DE_PIN;
      if (dePin >= 0) { pinMode(dePin, OUTPUT); digitalWrite(dePin, HIGH); }

      uint8_t dxl1ping[] = {0xFF,0xFF,0x01,0x02,0x01,0xFB};
      for (uint8_t b : dxl1ping) Serial1.write(b);
      Serial1.flush();

      if (dePin >= 0) {
        // baud-derived guard before releasing DE
        uint32_t guardUs = (3UL * 10UL * 1000000UL) / 57142UL + 200UL;
        delayMicroseconds(guardUs);
        digitalWrite(dePin, LOW);
      }

      // Buffer capture with microsecond timestamps
      static const int EBUF = 32;
      uint8_t  eb[EBUF]; uint32_t et[EBUF]; int en = 0;
      unsigned long et0 = micros();
      while (micros() - et0 < 100000UL && en < EBUF) { // 100ms window
        if (Serial1.available()) {
          eb[en] = Serial1.read(); et[en] = micros() - et0; ++en;
        }
      }
      dxl1Bus.setBaud(57142); // restore driver ownership

      // All prints after capture
      Serial.println(F("\n--- Raw DXL1 ping ID=1 @ 57142 (adapter RX test) ---"));
      Serial.print(F("TX: "));
      for (uint8_t b : dxl1ping) { if (b < 0x10) Serial.print('0'); Serial.print(b, HEX); Serial.print(' '); }
      Serial.println();
      Serial.print(F("RX: "));
      for (int i = 0; i < en; ++i) {
        Serial.print('['); Serial.print(et[i]); Serial.print(F("us] "));
        if (eb[i] < 0x10) Serial.print('0'); Serial.print(eb[i], HEX); Serial.print(' ');
      }
      if (!en) Serial.println(F("(nothing)"));
      else { Serial.println(); Serial.print(F("Total: ")); Serial.println(en); }
      Serial.println(F("Expected (no echo adapter): FF FF 01 02 00 FC"));
      Serial.println(F("Expected (echo adapter):    FF FF 01 02 01 FB  FF FF 01 02 00 FC"));
    }
    else if (cmd == 'a' || cmd == 'A') {
      // Scan all DXL2 baud rates — setBaud() and capture BEFORE any header print
      Serial.println(F("\n--- DXL2 raw ping ID=1 on ALL baud rates ---"));
      uint32_t bauds[] = {9600, 57142, 57600, 100000, 115200, 117647, 1000000};
      for (uint32_t baud : bauds) {
        dxl2Bus.setBaud(baud);
        // Print baud label then immediately capture — rawPingDump() buffers
        // internally so no USB print happens inside the receive window.
        Serial.print(F("  @ ")); Serial.print(baud); Serial.print(F(": "));
        Serial.flush(); // drain CDC TX so print arrives before capture starts
        dxl2Bus.rawPingDump(1, Serial);
      }
      dxl2Bus.setBaud(57600);
    }
    else if (cmd == 'w' || cmd == 'W') {
      // Send the EXACT Wizard ping packet at 1Mbaud — hardcoded, no CRC computation.
      // FF FF FD 00 01 03 00 01 19 4E  (Wizard-verified CRC)
      dxl2Bus.setBaud(1000000);
      delay(10);
      while (Serial1.available()) Serial1.read(); // drain

      const uint8_t wizPing[] = {0xFF,0xFF,0xFD,0x00,0x01,0x03,0x00,0x01,0x19,0x4E};
      {
        constexpr int8_t dePin = HW::SERVO_DE_PIN;
        if (dePin >= 0) { pinMode(dePin, OUTPUT); digitalWrite(dePin, HIGH); }
        for (uint8_t b : wizPing) Serial1.write(b);
        Serial1.flush();
        if (dePin >= 0) {
          uint32_t guardUs = (3UL * 10UL * 1000000UL) / 1000000UL + 200UL;
          delayMicroseconds(guardUs);
          digitalWrite(dePin, LOW);
        }
        delayMicroseconds(500);
      }

      // Buffer capture — all Serial.print AFTER
      static const int WBUF = 64;
      uint8_t  wb[WBUF]; uint32_t wt[WBUF]; int wn = 0;
      unsigned long wt0 = micros();
      while (micros() - wt0 < 5000 && wn < WBUF) {
        if (Serial1.available()) { wb[wn] = Serial1.read(); wt[wn] = micros() - wt0; ++wn; }
      }

      Serial.println(F("\n--- Wizard-exact ping @ 1Mbaud: FF FF FD 00 01 03 00 01 19 4E ---"));
      Serial.print(F("TX: "));
      for (uint8_t b : wizPing) { if (b < 0x10) Serial.print('0'); Serial.print(b, HEX); Serial.print(' '); }
      Serial.println();
      Serial.print(F("RX: "));
      for (int i = 0; i < wn; ++i) {
        Serial.print('['); Serial.print(wt[i]); Serial.print(F("us] "));
        if (wb[i] < 0x10) Serial.print('0'); Serial.print(wb[i], HEX); Serial.print(' ');
      }
      if (wn == 0) Serial.print(F("(nothing)"));
      Serial.println();
      Serial.print(F("Total: ")); Serial.print(wn); Serial.println(F(" bytes"));
      Serial.println(F("Expected: 14 bytes starting FF FF FD 00 01 07 00 55 ..."));
    }
    else if (cmd == 'x' || cmd == 'X') {
      // DE-pin loopback diagnostic for wired RS485 boards.
      // Step 1: confirm DE pin value can be read back (GPIO sanity)
      // Step 2: send one byte WITH DE asserted, check RX path
      // Step 3: send one byte WITHOUT DE, check nothing arrives (loopback absent)
      Serial.println(F("\n--- DE-pin / RS485 loopback diagnostic ---"));
      constexpr int8_t dePin = HW::SERVO_DE_PIN;
      if (dePin < 0) {
        Serial.println(F("SERVO_DE_PIN = -1 (auto-direction build) — DE test not applicable"));
      } else {
      Serial.print(F("SERVO_DE_PIN = ")); Serial.println((int)dePin);

      // Configure
      pinMode(dePin, OUTPUT);
      Serial1.end();
      Serial1.begin(57600);
      while (Serial1.available()) Serial1.read();

      // --- Test A: DE HIGH (TX mode) — send 0xAA, expect echo on RX ---
      digitalWrite(dePin, HIGH);
      delayMicroseconds(50);
      Serial1.write((uint8_t)0xAA);
      Serial1.flush();
      delayMicroseconds(500);
      digitalWrite(dePin, LOW);

      // Buffer capture
      uint8_t rxA[8]; uint32_t tA[8]; int nA = 0;
      unsigned long t0A = micros();
      while (micros() - t0A < 3000 && nA < 8) {
        if (Serial1.available()) { rxA[nA] = Serial1.read(); tA[nA] = micros()-t0A; ++nA; }
      }
      Serial.print(F("A) DE=HIGH, TX 0xAA: RX bytes: "));
      if (!nA) { Serial.println(F("(none) <- DE not enabling TX, or RX wiring broken")); }
      else { for(int i=0;i<nA;i++){Serial.print(rxA[i],HEX);Serial.print(' ');} Serial.println(); }

      // --- Test B: DE LOW (RX mode) — send 0x55, expect nothing ---
      while (Serial1.available()) Serial1.read();
      digitalWrite(dePin, LOW);
      Serial1.write((uint8_t)0x55);
      Serial1.flush();
      delayMicroseconds(500);

      uint8_t rxB[8]; int nB = 0;
      unsigned long t0B = micros();
      while (micros() - t0B < 3000 && nB < 8) {
        if (Serial1.available()) { rxB[nB++] = Serial1.read(); }
      }
      Serial.print(F("B) DE=LOW,  TX 0x55: RX bytes: "));
      if (!nB) { Serial.println(F("(none) <- correct: DE low blocks TX")); }
      else { for(int i=0;i<nB;i++){Serial.print(rxB[i],HEX);Serial.print(' ');} Serial.println(F("<- unexpected echo?")); }

      Serial.println(F("Expected: A=AA (echo), B=none. If A=none: check GP2->DE wiring or DE polarity."));
        dxl1Bus.setBaud(57142); // restore
      } // end dePin >= 0
    }
    else if (cmd == 'i' || cmd == 'I') {
      // I2C bus scan — shows every address that acknowledges on Wire (GP4/GP5)
      // Use this to find the actual OLED address when oledOk=NO
      Serial.println(F("\n--- I2C scan on Wire ---"));
      int found = 0;
      for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          Serial.print(F("  0x"));
          if (addr < 0x10) Serial.print('0');
          Serial.println(addr, HEX);
          ++found;
        }
      }
      if (!found) Serial.println(F("  (nothing found)"));
      Serial.print(F("Total: ")); Serial.print(found); Serial.println(F(" device(s)"));
      Serial.println(F("Expected: 0x3C=OLED, 0x40=encoder"));
    }
    else if (cmd == 'm' || cmd == 'M') {
      // SC09 max-angle-limit write diagnostic.
      // Uses the app's currently active servo ID and baud — NOT a hardcoded
      // ID=1, since the bus may not be at the right baud or the active servo
      // may have a different ID. This avoids talking into the void.
      Serial.println(F("\n--- SC09 max-limit write diagnostic ---"));

      if (appState.servoCount == 0 || appState.activeIndex < 0 ||
          appState.activeIndex >= appState.servoCount) {
        Serial.println(F("No active servo selected in the app — select one first."));
      } else {
        uint8_t id = appState.ids[appState.activeIndex];
        Serial.print(F("Using active servo ID=")); Serial.println(id);
        Serial.print(F("Current scanBaudIndex=")); Serial.println(appState.scanBaudIndex);

        bool wasScs = st3215Bus.getScsMode();
        st3215Bus.setScsMode(true);
        // Ensure the bus is actually at the servo's current baud —
        // setScsMode() only changes byte order, NOT the UART baud rate.
        st3215Bus.setBaud(BAUD_TABLE[appState.scanBaudIndex]);
        delay(20);

        uint8_t dId; int dMin, dMax, dTorq, dOfs, dMode, dBaud;

        Serial.println(F("Step 0: ping to confirm servo responds at this baud"));
        bool pingOk = st3215Bus.ping(id);
        Serial.print(F("  ping=")); Serial.println(pingOk ? "OK" : "FAILED");

        if (!pingOk) {
          Serial.println(F("Ping failed — servo not responding at current baud."));
          Serial.println(F("Check Flash Diag / Select Servo screen to confirm baud."));
        } else {
          Serial.println(F("Step 0b: check lock register BEFORE unlock"));
          int lockBefore = st3215Bus.rawReadByte(id, 48);
          Serial.print(F("  reg48 (lock) before unlock = ")); Serial.println(lockBefore);

          Serial.println(F("Step 0c: manually unlock and verify"));
          int unlockResult = st3215Bus.rawWriteByte(id, 48, 0);
          Serial.print(F("  writeByte(48,0) returned: ")); Serial.println(unlockResult);
          delay(20);
          int lockAfterUnlock = st3215Bus.rawReadByte(id, 48);
          Serial.print(F("  reg48 (lock) after unlock = ")); Serial.println(lockAfterUnlock);

          Serial.println(F("Step 1: read current min/max"));
          bool ok1 = st3215Bus.loadConfig(id, dId, dMin, dMax, dTorq, dOfs, dMode, dBaud);
          Serial.print(F("  ok=")); Serial.print(ok1);
          Serial.print(F("  min=")); Serial.print(dMin);
          Serial.print(F("  max=")); Serial.println(dMax);

          Serial.println(F("Step 2: read current actual position"));
          int curPos = st3215Bus.readPosition(id);
          Serial.print(F("  position=")); Serial.println(curPos);

          Serial.println(F("Step 3: write min=100, max=900 (via saveMinMax/writeWord)"));
          bool okWrite = st3215Bus.saveMinMax(id, 100, 900);
          Serial.print(F("  saveMinMax() returned: ")); Serial.println(okWrite ? "true" : "false");

          delay(50);

          Serial.println(F("Step 3b: re-unlock and write max via RAW byte writes (bypass writeWord)"));
          st3215Bus.rawWriteByte(id, 48, 0); // unlock
          delay(20);
          // End=1 (SCS big-endian): DataL=900>>8=3 goes to reg11, DataH=900&0xFF=132 goes to reg12
          int w11 = st3215Bus.rawWriteByte(id, 11, 3);
          delay(10);
          int verify11early = st3215Bus.rawReadByte(id, 11);
          Serial.print(F("  reg11 immediately after writing it = ")); Serial.println(verify11early);
          int w12 = st3215Bus.rawWriteByte(id, 12, 132);
          delay(10);
          st3215Bus.rawWriteByte(id, 48, 1); // relock
          delay(20);
          Serial.print(F("  write reg11=3 returned: ")); Serial.println(w11);
          Serial.print(F("  write reg12=132 returned: ")); Serial.println(w12);
          int verify11 = st3215Bus.rawReadByte(id, 11);
          int verify12 = st3215Bus.rawReadByte(id, 12);
          Serial.print(F("  readback reg11=")); Serial.print(verify11);
          Serial.print(F("  reg12=")); Serial.println(verify12);

          Serial.println(F("Step 4: read back min/max immediately after write"));
          bool ok2 = st3215Bus.loadConfig(id, dId, dMin, dMax, dTorq, dOfs, dMode, dBaud);
          Serial.print(F("  ok=")); Serial.print(ok2);
          Serial.print(F("  min=")); Serial.print(dMin);
          Serial.print(F("  max=")); Serial.println(dMax);

          Serial.println(F("Step 5: raw register read (bypass readWord helper)"));
          int rawMinL = st3215Bus.rawReadByte(id, 9);
          int rawMinH = st3215Bus.rawReadByte(id, 10);
          int rawMaxL = st3215Bus.rawReadByte(id, 11);
          int rawMaxH = st3215Bus.rawReadByte(id, 12);
          Serial.print(F("  reg9 (minL)=")); Serial.print(rawMinL);
          Serial.print(F("  reg10(minH)=")); Serial.print(rawMinH);
          Serial.print(F("  reg11(maxL)=")); Serial.print(rawMaxL);
          Serial.print(F("  reg12(maxH)=")); Serial.println(rawMaxH);

          Serial.println(F("Step 6: read lock register (should be 1=locked after save)"));
          int lockVal = st3215Bus.rawReadByte(id, 48);
          Serial.print(F("  reg48 (lock)=")); Serial.println(lockVal);
        }

        st3215Bus.setScsMode(wasScs);
      }
      Serial.println(F("--- diagnostic complete ---"));
    }
  }

  if (appState.midi.active) StatusLed::amber();
  else                       StatusLed::green();
}

// ---------------------------------------------------------------------------
// Core 1 — PIO-USB host stack (only when USE_TINYUSB_HOST is defined)
// ---------------------------------------------------------------------------

#if defined(USE_TINYUSB_HOST)
#include <pio_usb.h>
void setup1() {
  delay(500);
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = HW::USB_HOST_DP_PIN;
  tuh_configure(1, TUH_CFGID_RPI_PIO_USB, &pio_cfg);
  tuh_init(1);
}
void loop1() { tuh_task(); }
#else
void setup1() {}
void loop1()  {}
#endif