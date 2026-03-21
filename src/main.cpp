#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TinyUSB.h>

#include "config.h"
#include "app_state.h"
#include "model/servo_model.h"

#include "drivers/encoder_unit.h"
#include "drivers/oled_ui.h"
#include "drivers/iservo_bus.h"
#include "drivers/servo_bus.h"
#include "drivers/dxl1_bus.h"
#include "drivers/midi_engine.h"
#include "drivers/persist.h"
#include "app/app.h"

AppState    appState;
UnitEncoder encoder(Wire1, HW::ENC_ADDR);
OledUi      ui(HW::OLED_W, HW::OLED_H, &Wire, HW::OLED_ADDR);

// App is constructed with ST3215 by default;
// loadPersistedState() calls app.setBus() if a different protocol was saved.
App app(appState, encoder, ui, &st3215Bus, midiEngine);

// ---------------------------------------------------------------------------
// Select the concrete bus object for the given protocol and call setBus().
// Called on boot (after loading persist) and from the SelectProtocol screen.
// ---------------------------------------------------------------------------
IServoBus* busForProtocol(BusProtocol p) {
  switch (p) {
    case BusProtocol::DXL1:   return &dxl1Bus;
    case BusProtocol::ST3215: // fall-through
    default:                  return &st3215Bus;
  }
}

static void onCCReceived(uint8_t channel, uint8_t cc, uint8_t value) {
  app.onMidiCC(channel, cc, value);
}

static void onAnyReceived(MidiMsgType type, uint8_t channel,
                           uint8_t byte1, uint8_t byte2, int16_t int14) {
  app.onMidiAny(type, channel, byte1, byte2, int14);
}

void setup() {
  // USB MIDI must be first — before any other peripheral
  midiEngine.setOnCC(onCCReceived);
  midiEngine.setOnAny(onAnyReceived);
  midiEngine.begin();

  delay(200);
  {
    unsigned long t = millis();
    while (!midiEngine.isMounted() && millis() - t < 1500) delay(10);
  }

  Wire.setSDA(HW::OLED_SDA_PIN);
  Wire.setSCL(HW::OLED_SCL_PIN);
  Wire.begin();

  Serial1.setTX(HW::SERVO_TX_PIN);
  Serial1.setRX(HW::SERVO_RX_PIN);

  appState.oledOk = ui.begin();
  if (appState.oledOk) ui.splash("Servo Tester", "Init...");

  appState.encoderOk = encoder.begin(HW::ENC_SDA_PIN, HW::ENC_SCL_PIN, HW::ENC_I2C_HZ);
  if (appState.encoderOk) encoder.setLedColor(0, 0x002000);

  // Initialise both bus implementations — app.begin() will call setBaud on the active one
  st3215Bus.begin(Serial1, HW::SERVO_BAUD);
  dxl1Bus.begin(Serial1, 57600); // Serial1 shared; begin() sets baud

  persist.begin();

  if (appState.oledOk) ui.splash("Starting...");
  app.begin();

  delay(300);
}

void loop() {
  app.tick();
}
