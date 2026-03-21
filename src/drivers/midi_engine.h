#pragma once
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include "../model/servo_model.h"

using MidiCCCallback  = void (*)(uint8_t channel, uint8_t cc, uint8_t value);
using MidiAnyCallback = void (*)(MidiMsgType type,
                                  uint8_t channel,
                                  uint8_t byte1,
                                  uint8_t byte2,
                                  int16_t int14);

class MidiEngine {
public:
  void begin();
  void tick();
  void sendCC(uint8_t channel, uint8_t cc, uint8_t value);
  void panic();
  void setOnCC(MidiCCCallback cb)   { _onCC  = cb; }
  void setOnAny(MidiAnyCallback cb) { _onAny = cb; }
  bool isMounted() const;

  // Public: accessed by MIDI_CREATE_INSTANCE macro and trampolines
  Adafruit_USBD_MIDI _usbMidi;
  MidiCCCallback     _onCC  = nullptr;
  MidiAnyCallback    _onAny = nullptr;
};

extern MidiEngine midiEngine;
