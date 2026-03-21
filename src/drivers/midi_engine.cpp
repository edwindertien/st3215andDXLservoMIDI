#include "midi_engine.h"

MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, midiEngine._usbMidi, _midiLib);

MidiEngine midiEngine;

// ---------------------------------------------------------------------------
// Trampolines — MIDI library uses static C callbacks
// ---------------------------------------------------------------------------

static void onCCTrampoline(uint8_t channel, uint8_t cc, uint8_t value) {
  if (midiEngine._onCC)
    midiEngine._onCC(channel, cc, value);
  if (midiEngine._onAny)
    midiEngine._onAny(MidiMsgType::CC, channel, cc, value, 0);
}

static void onNoteOnTrampoline(uint8_t channel, uint8_t note, uint8_t velocity) {
  if (midiEngine._onAny)
    midiEngine._onAny(MidiMsgType::NoteOn, channel, note, velocity, 0);
}

static void onNoteOffTrampoline(uint8_t channel, uint8_t note, uint8_t velocity) {
  if (midiEngine._onAny)
    midiEngine._onAny(MidiMsgType::NoteOff, channel, note, velocity, 0);
}

static void onPitchBendTrampoline(uint8_t channel, int bend) {
  if (midiEngine._onAny)
    midiEngine._onAny(MidiMsgType::PitchBend, channel, 0, 0, (int16_t)bend);
}

static void onProgramChangeTrampoline(uint8_t channel, uint8_t number) {
  if (midiEngine._onAny)
    midiEngine._onAny(MidiMsgType::ProgramChange, channel, number, 0, 0);
}

static void onAfterTouchChannelTrampoline(uint8_t channel, uint8_t pressure) {
  if (midiEngine._onAny)
    midiEngine._onAny(MidiMsgType::AfterTouch, channel, pressure, 0, 0);
}

// ---------------------------------------------------------------------------

void MidiEngine::begin() {
  _midiLib.begin(MIDI_CHANNEL_OMNI);
  _midiLib.setHandleControlChange(onCCTrampoline);
  _midiLib.setHandleNoteOn(onNoteOnTrampoline);
  _midiLib.setHandleNoteOff(onNoteOffTrampoline);
  _midiLib.setHandlePitchBend(onPitchBendTrampoline);
  _midiLib.setHandleProgramChange(onProgramChangeTrampoline);
  _midiLib.setHandleAfterTouchChannel(onAfterTouchChannelTrampoline);
  _midiLib.turnThruOff();
}

void MidiEngine::tick() {
  _midiLib.read();
}

void MidiEngine::sendCC(uint8_t channel, uint8_t cc, uint8_t value) {
  if (!isMounted()) return;
  _midiLib.sendControlChange(cc, value, channel);
}

void MidiEngine::panic() {
  if (!isMounted()) return;
  for (uint8_t ch = 1; ch <= 16; ++ch) {
    _midiLib.sendControlChange(121, 0, ch);
    _midiLib.sendControlChange(123, 0, ch);
  }
}

bool MidiEngine::isMounted() const {
  return TinyUSBDevice.mounted();
}
