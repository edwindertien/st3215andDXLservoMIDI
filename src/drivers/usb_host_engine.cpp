#include "usb_host_engine.h"

// ---------------------------------------------------------------------------
// TinyUSB host headers — only compiled when USE_TINYUSB_HOST is defined
// (set in platformio.ini when Pico-PIO-USB is present).
// ---------------------------------------------------------------------------
#if defined(USE_TINYUSB_HOST)
#include <Adafruit_TinyUSB.h>
#include <pio_usb.h>
#endif

UsbHostEngine usbHost;

// ---------------------------------------------------------------------------
// Core 0 API
// ---------------------------------------------------------------------------

void UsbHostEngine::begin(uint8_t dpPin, uint8_t dmPin) {
  for (int i = 0; i < 4; ++i) _devName[i][0] = '\0';
  _head = _tail = 0;
  _connectedCount = 0;

#if defined(USE_TINYUSB_HOST)
  // PIO-USB uses PIO0 on Core 1.  The TinyUSB host stack is initialised
  // in setup1() / loop1() in main.cpp.  Here we just record the pin config.
  // (Pin configuration is set globally via the PIO-USB defines in platformio.ini.)
  (void)dpPin; (void)dmPin;
#else
  (void)dpPin; (void)dmPin;
#endif
}

int UsbHostEngine::drainEvents(HostInputEvent* out, int maxEvents) {
  int n = 0;
  while (n < maxEvents) {
    uint32_t h = _head;
    uint32_t t = _tail;
    if (h == t) break;
    __dmb();
    // Copy volatile struct field-by-field — compiler can't auto-copy volatile
    uint32_t idx = t % HOST_EVT_RING_SIZE;
    out[n].type     = (HostEvtType)_ring[idx].type;
    out[n].devIndex = (uint8_t)   _ring[idx].devIndex;
    out[n].channel  = (uint8_t)   _ring[idx].channel;
    out[n].index    = (uint8_t)   _ring[idx].index;
    out[n].value    = (int16_t)   _ring[idx].value;
    __dmb();
    _tail = t + 1;
    ++n;
  }
  return n;
}

const char* UsbHostEngine::deviceName(int slot) const {
  if (slot < 0 || slot >= 4 || _devName[slot][0] == '\0') return "---";
  return _devName[slot];
}

// ---------------------------------------------------------------------------
// Core 1 API (called from TinyUSB callbacks)
// ---------------------------------------------------------------------------

void UsbHostEngine::pushEvent(const HostInputEvent& evt) {
  uint32_t h = _head;
  uint32_t next = h + 1;
  if ((next % HOST_EVT_RING_SIZE) == (_tail % HOST_EVT_RING_SIZE)) {
    return; // buffer full — drop
  }
  // Write volatile struct field-by-field
  uint32_t idx = h % HOST_EVT_RING_SIZE;
  _ring[idx].type     = evt.type;
  _ring[idx].devIndex = evt.devIndex;
  _ring[idx].channel  = evt.channel;
  _ring[idx].index    = evt.index;
  _ring[idx].value    = evt.value;
  __dmb();
  _head = next;
}

void UsbHostEngine::notifyConnected(uint8_t devAddr, uint16_t vid, uint16_t pid) {
  // Find a name for this device
  const char* name = "USB Dev";
  for (const auto& d : KNOWN_DEVICES) {
    if (d.vid == vid && d.pid == pid) { name = d.name; break; }
  }
  int slot = (devAddr > 0 && devAddr <= 4) ? devAddr - 1 : 0;
  strncpy(_devName[slot], name, 11);
  _devName[slot][11] = '\0';
  _connectedCount++;

  HostInputEvent e;
  e.type     = HostEvtType::Connected;
  e.devIndex = (uint8_t)slot;
  e.value    = 0;
  pushEvent(e);
}

void UsbHostEngine::notifyDisconnected(uint8_t devAddr) {
  int slot = (devAddr > 0 && devAddr <= 4) ? devAddr - 1 : 0;
  _devName[slot][0] = '\0';
  if (_connectedCount > 0) _connectedCount--;

  HostInputEvent e;
  e.type     = HostEvtType::Disconnected;
  e.devIndex = (uint8_t)slot;
  pushEvent(e);
}

// ---------------------------------------------------------------------------
// TinyUSB host callbacks — only compiled when PIO-USB host is active
// ---------------------------------------------------------------------------

#if defined(USE_TINYUSB_HOST)

// ---- Mount / Unmount ----

void tuh_mount_cb(uint8_t dev_addr) {
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  usbHost.notifyConnected(dev_addr, vid, pid);
}

void tuh_umount_cb(uint8_t dev_addr) {
  usbHost.notifyDisconnected(dev_addr);
}

// ---- HID callbacks ----

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                       uint8_t const* desc_report, uint16_t desc_len) {
  (void)desc_report; (void)desc_len;
  // Request first report
  tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  (void)dev_addr; (void)instance;
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance,
                                  uint8_t const* report, uint16_t len) {
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
  uint8_t slot = (dev_addr > 0 && dev_addr <= 4) ? dev_addr - 1 : 0;

  if (itf_protocol == HID_ITF_PROTOCOL_GAMEPAD ||
      itf_protocol == HID_ITF_PROTOCOL_NONE) {
    // Generic HID — parse axes and buttons from raw report.
    // We use a simple heuristic that works for most gamepads and the SpaceMouse:
    //   - First N pairs of bytes = signed 16-bit axes (little-endian)
    //   - Last bytes = button bitmask
    // This covers: standard gamepad (4+ axes, 10+ buttons),
    //              SpaceMouse (6 axes in two 6-byte reports, IDs 1 and 2).

    if (len == 0) { tuh_hid_receive_report(dev_addr, instance); return; }

    // SpaceMouse: report ID in first byte, 3 axes of 2 bytes each
    uint8_t reportId = report[0];
    if (len == 7 && (reportId == 1 || reportId == 2)) {
      // SpaceMouse format: [reportId, x_lo, x_hi, y_lo, y_hi, z_lo, z_hi]
      uint8_t axisBase = (reportId == 1) ? 0 : 3; // axes 0-2 or 3-5
      for (int a = 0; a < 3; ++a) {
        int16_t val = (int16_t)((uint16_t)report[1 + a*2] |
                                ((uint16_t)report[2 + a*2] << 8));
        HostInputEvent e;
        e.type     = HostEvtType::HidAxis;
        e.devIndex = slot;
        e.channel  = 0;
        e.index    = (uint8_t)(axisBase + a);
        e.value    = val;
        usbHost.pushEvent(e);
      }
    } else {
      // Generic gamepad: signed 16-bit axes starting at byte 0 (or 1 if report ID present)
      uint8_t offset = (len > 4 && report[0] < 8) ? 1 : 0; // skip report ID if present
      int numAxes = (len - offset) / 2;
      if (numAxes > 8) numAxes = 8;
      for (int a = 0; a < numAxes; ++a) {
        int16_t val = (int16_t)((uint16_t)report[offset + a*2] |
                                ((uint16_t)report[offset + a*2 + 1] << 8));
        HostInputEvent e;
        e.type     = HostEvtType::HidAxis;
        e.devIndex = slot;
        e.channel  = 0;
        e.index    = (uint8_t)a;
        e.value    = val;
        usbHost.pushEvent(e);
      }
      // Button bytes after axes
      int axisBytes = numAxes * 2 + offset;
      if ((int)len > axisBytes) {
        for (int btn = 0; btn < ((int)len - axisBytes) * 8 && btn < 32; ++btn) {
          uint8_t byteIdx = axisBytes + btn / 8;
          uint8_t bitMask = 1 << (btn % 8);
          HostInputEvent e;
          e.type     = HostEvtType::HidButton;
          e.devIndex = slot;
          e.channel  = 0;
          e.index    = (uint8_t)btn;
          e.value    = (report[byteIdx] & bitMask) ? 1 : 0;
          usbHost.pushEvent(e);
        }
      }
    }
  }

  tuh_hid_receive_report(dev_addr, instance);
}

// ---- USB MIDI callbacks ----

void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep,
                        uint8_t num_cables_rx, uint16_t num_cables_tx) {
  (void)in_ep; (void)out_ep; (void)num_cables_rx; (void)num_cables_tx;
  // Device already notified via tuh_mount_cb
  (void)dev_addr;
}

void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance) {
  (void)dev_addr; (void)instance;
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets) {
  uint8_t slot = (dev_addr > 0 && dev_addr <= 4) ? dev_addr - 1 : 0;
  uint8_t packet[4];
  while (tuh_midi_packet_read(dev_addr, packet)) {
    uint8_t status  = packet[1];
    uint8_t msgType = status & 0xF0;
    uint8_t channel = (status & 0x0F) + 1; // 1-based

    HostInputEvent e;
    e.devIndex = slot;
    e.channel  = channel;

    if (msgType == 0xB0) { // CC
      e.type  = HostEvtType::MidiCC;
      e.index = packet[2]; // CC number
      e.value = packet[3]; // CC value
      usbHost.pushEvent(e);
    } else if (msgType == 0x90 && packet[3] > 0) { // Note On
      e.type  = HostEvtType::MidiNoteOn;
      e.index = packet[2]; // note number
      e.value = packet[3]; // velocity
      usbHost.pushEvent(e);
    } else if (msgType == 0x80 || (msgType == 0x90 && packet[3] == 0)) { // Note Off
      e.type  = HostEvtType::MidiNoteOff;
      e.index = packet[2]; // note number
      e.value = 0;
      usbHost.pushEvent(e);
    }
    // Other MIDI messages (PB, PC, AT) logged but not routed in Step 1
  }
  (void)num_packets;
}

#endif // USE_TINYUSB_HOST
