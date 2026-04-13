#pragma once
#include <Arduino.h>
#include "../model/servo_model.h"

// ---------------------------------------------------------------------------
// UsbHostEngine — PIO-based USB host running on Core 1.
//
// Supports three device classes simultaneously:
//   • USB MIDI (class 0x01/0x03, subclass 0x03) — e.g. AKAI MIDImix
//   • HID Gamepad / Joystick                    — e.g. BetaFPV gamepad
//   • HID Multi-axis controller (SpaceMouse)    — VID 0x046d / 0x256f
//
// Core 1 receives USB events via TinyUSB host callbacks and pushes
// HostInputEvent structs into a lock-free single-producer/single-consumer
// ring buffer.  Core 0 drains it each tick() via drainEvents().
//
// GPIO wiring (add USB-A jack with 22Ω resistors on D+ and D−):
//   GP16 → D+    GP17 → D−    VBUS → USB-A VBus    GND → GND
// ---------------------------------------------------------------------------

// Known supported device VID/PID entries (used for identification only —
// the TinyUSB class drivers handle the actual USB protocol).
struct KnownUsbDevice {
  uint16_t    vid;
  uint16_t    pid;
  const char* name;      // short name for OLED display
};

// Event type delivered to Core 0
enum class HostEvtType : uint8_t {
  Connected,             // device plugged in
  Disconnected,          // device removed
  MidiCC,                // USB MIDI CC message from host device
  MidiNoteOn,            // USB MIDI Note On
  MidiNoteOff,           // USB MIDI Note Off
  HidAxis,               // HID axis value (signed 16-bit)
  HidButton,             // HID button state (0=up, 1=down)
};

struct HostInputEvent {
  HostEvtType type      = HostEvtType::Connected;
  uint8_t     devIndex  = 0;   // which connected device (0-based)
  uint8_t     channel   = 0;   // MIDI channel (1-16) or 0 for HID
  uint8_t     index     = 0;   // CC#, note#, axis index, button index
  int16_t     value     = 0;   // CC/velocity value (0-127), axis (-32768..32767), button (0/1)
};

// Ring buffer capacity — must be power of 2
static constexpr int HOST_EVT_RING_SIZE = 32;

class UsbHostEngine {
public:
  // Called from Core 0 setup() — registers the Core 1 entry and starts PIO-USB
  void begin(uint8_t dpPin = 16, uint8_t dmPin = 17);

  // Called from Core 0 tick() — drains pending events into caller's buffer.
  // Returns number of events written (up to maxEvents).
  int  drainEvents(HostInputEvent* out, int maxEvents);

  // Called from Core 0 — true if at least one device is connected
  bool hasDevice() const { return _connectedCount > 0; }
  int  connectedCount() const { return _connectedCount; }

  // Device name for OLED (short, ≤8 chars). Returns "---" if slot empty.
  const char* deviceName(int slot) const;

  // Called from Core 1 TinyUSB callbacks — push events into ring buffer
  void pushEvent(const HostInputEvent& evt);

  // Called from Core 1 — update connected device count + name
  void notifyConnected(uint8_t devAddr, uint16_t vid, uint16_t pid);
  void notifyDisconnected(uint8_t devAddr);

private:
  // Lock-free SPSC ring buffer (Core 1 writes, Core 0 reads)
  volatile HostInputEvent _ring[HOST_EVT_RING_SIZE];
  volatile uint32_t       _head = 0;  // write index (Core 1)
  volatile uint32_t       _tail = 0;  // read  index (Core 0)

  volatile int  _connectedCount = 0;
  char          _devName[4][12]; // up to 4 simultaneous devices, 11 chars + NUL

  static constexpr KnownUsbDevice KNOWN_DEVICES[] = {
    { 0x09e8, 0x0028, "MIDImix"    },  // AKAI MIDImix
    { 0x09e8, 0x003e, "APC mini"   },  // AKAI APC mini mk2
    { 0x046d, 0xc626, "SpaceNav"   },  // 3Dconnexion SpaceNavigator
    { 0x046d, 0xc627, "SpaceMse"   },  // 3Dconnexion SpaceMouse
    { 0x046d, 0xc628, "SpaceMse"   },  // 3Dconnexion SpaceMouse (alt PID)
    { 0x256f, 0xc635, "SpaceMse"   },  // 3Dconnexion SpaceMouse (new VID)
    { 0x256f, 0xc652, "SpCmpact"   },  // 3Dconnexion SpaceMouse Compact
    { 0xc2df, 0x0001, "BetaFPV"    },  // BetaFPV gamepad
    { 0x046d, 0xc215, "Gamepad"    },  // Logitech gamepad (wired)
    { 0x046d, 0xc21d, "F310"       },  // Logitech F310
    { 0x046d, 0xc21e, "F510"       },  // Logitech F510
    { 0x046d, 0xc21f, "F710"       },  // Logitech F710
  };
};

extern UsbHostEngine usbHost;
