# Developer Context — Servo Tester

This document captures architecture decisions, non-obvious implementation details, bug history, and hardware findings accumulated during development. It complements README.md and is intended for anyone reading or modifying the source code.

---

## Architecture Overview

### Three buses, one UART

All three servo protocols (ST3215, DXL1, DXL2) share a single hardware UART (`Serial1`, GP0/GP1). Each bus driver stores a pointer to `Serial1` and calls `_serial->end()` followed by `_serial->begin(baud)` whenever the baud rate changes. The `end()` before `begin()` is **mandatory** on the RP2040 Earle Philhower core — calling `begin()` on an already-open port at a different baud is silently ignored without the prior `end()`, leaving the UART at the old rate. This was the root cause of many intermittent scan failures during development.

The correct initialisation sequence in `setup()`:
```
Serial1.setTX(GP0); Serial1.setRX(GP1);
st3215Bus.begin(Serial1, 1000000);  // end()+begin(1M)
dxl1Bus.begin(Serial1, 57142);      // end()+begin(57142)
dxl2Bus.begin(Serial1, 57600);      // end()+begin(57600)
// Serial1 ends up at 57600 after setup()
// loadPersistedState() then calls _bus->setBaud(persisted_baud)
// which does end()+begin() to claim the correct rate
```

### IServoBus interface

All three protocols implement `IServoBus` (pure virtual). `App` holds a `IServoBus* _bus` pointer that is swapped at runtime via `busForProtocol()` in `main.cpp`. No protocol-specific code exists in `app.cpp`.

### App tick structure

`App::tick()` runs every `loop()` iteration (no fixed rate — just as fast as possible):
1. `handleInput()` — encoder, button debounce
2. `updateFeedback()` — servo status polling (when on relevant screens)
3. `tickMidi()` — if MIDI active (rate-gated internally at 8ms TX and 8ms RX)
4. `tickHostInput()` — USB host event drain (always, but currently a stub)
5. Lazy persist write — if `_persistDirty`, write to flash and clear flag
6. `render()` — OLED update (rate-gated internally)
7. `_midi.tick()` — calls `_midiLib.read()` to drain USB MIDI RX

---

## ST3215 Echo Drain

The ST3215 uses the SCServo library (`SMS_STS`). The library's `wFlushSCS()` is called after every TX packet and before reading the servo response — but the base implementation is empty. With an auto-direction adapter (no DE/RE pin), TX bytes are echoed back to RX. The echo passes the SCServo checksum validation because the PING packet checksum is algebraically identical to a STATUS response checksum for the same ID.

**Fix:** `EchoSMS_STS` subclass in `servo_bus.h` overrides `wFlushSCS()`:
```cpp
void wFlushSCS() override {
    pSerial->flush();           // wait for TX shift register to empty
    delayMicroseconds(waitUs);  // 1 byte-time margin at current baud
    while (pSerial->read() != -1) {} // drain echo (NOT drain-until-empty loop)
}
```

Critical subtleties:
- `flush()` must be called first — `write()` is non-blocking; without `flush()` the drain runs before the echo has arrived
- The drain must be **bounded by time, not "until empty"** — an open-ended drain loop consumes the servo response which arrives ~50µs after the echo ends
- `_baud` must be kept in sync with `Serial1` baud whenever `setBaud()` is called

### Why it appeared to work before

Before the `end()+begin()` fix, `Serial1` was left in a partially-reinitialised state after `setBaud()`. The RP2040 UART RX was in a subtly broken condition that caused echo bytes to arrive with framing errors and be silently discarded — so the library never saw the echo. Once `end()+begin()` made the UART work correctly, the echo arrived cleanly and the false-positive problem appeared.

---

## DXL2 CRC Table

The Dynamixel Protocol 2.0 CRC-16 uses a lookup table with 256 uint16 entries. Our original table had 48 wrong entries at indices 208–255 (0xD0–0xFF). This caused the wrong CRC to be computed for any packet that fed bytes ≥ 0xD0 into the CRC accumulator during the `((crc >> 8) ^ b) & 0xFF` step — which includes the 0xFF header bytes of every ping packet.

The correct table is from the Robotis official `protocol.c`. The corrected entries start at index 208:
```cpp
0x82E3,0x02E6,0x02EC,0x82E9,... // correct
// vs wrong:
0x02E0,0x82E5,0x82EF,0x02EA,... // previous (transposed pairs)
```

Verification: `crc16([0xFF,0xFF,0xFD,0x00,0x01,0x03,0x00,0x01])` must equal `0x4E19` (bytes `19 4E` in the packet). The READ packet CRC `0xD535` (bytes `35 D5`) can also be used as a cross-check.

---

## DXL1 Baud Rates

Dynamixel Protocol 1.0 MX-series baud formula: `actual = 2,000,000 / (register + 1)`. The factory default register is 34, giving `2,000,000 / 35 = 57,142` baud — not 57,600. Dynamixel Wizard labels this as "57600" but the actual UART must be set to 57,142 or every frame will have framing errors. This is a known Robotis documentation ambiguity.

---

## MIDI Timing Architecture

### TX path (outgoing CC — arm position → DAW)

One servo is polled per TX tick (round-robin). Tick interval: `MIDI_TX_INTERVAL_MS = 8ms`. With N servos, each servo's position is read and sent as CC at `1000 / (8 × N)` Hz. With 6 servos: ~21 Hz per servo.

The TX tick fires only when `runMode != RecvOnly`.

### RX path (incoming CC — DAW → arm)

Every RX tick (`MIDI_RX_INTERVAL_MS = 8ms`), all bindings with `needsSend = true` call `setPosition()`. `needsSend` is true when:
- New `pendingCC` arrived (set in `onMidiCC()` interrupt callback), OR
- `smoothing > 0` and `smoothPos` hasn't converged to `lastRawTarget` within 1 count

The RX tick fires only when `runMode != SendOnly`.

### pendingCC — last-write-wins

`pendingCC` is an `int8_t` storing the most recent CC value (-1 = none pending). If multiple CC messages arrive within one 8ms RX window, only the last is applied. This is intentional: applying every intermediate value would cause the bus to be hammered with setPosition commands faster than the servo can execute them.

`lastRawTarget` stores the last `ccToPos()` result separately from `pendingCC`. After `pendingCC` is consumed, the smoothing loop continues advancing `smoothPos` toward `lastRawTarget` on subsequent ticks, ensuring the servo always reaches its final position even after CC messages stop.

### Jump filter

Applied in `onMidiCC()` (interrupt context) before `pendingCC` is written:
```cpp
if (m.jumpFilter > 0 && b.lastRecv >= 0) {
    if (abs((int)value - (int)(uint8_t)b.lastRecv) > (int)m.jumpFilter)
        continue; // reject
}
```
`lastRecv` is only updated when a CC is accepted, so the filter holds its baseline until a legitimate incremental value arrives. The rxFlash indicator fires even on filtered CCs so the user can see filtering is active.

### Global speed/acc CC

When a speed or acceleration CC arrives, `_app.speed` / `_app.acc` is updated immediately. No `setPosition()` calls are made — the new value takes effect naturally on the next per-servo `setPosition()` in the RX loop. An earlier implementation called `setPosition()` with `_app.targetPos` (the live-control single position) for all servos, which snapped all arms to the center position.

---

## Persistence

`PersistentConfig` is a flat struct written as raw bytes to `/config.bin` on LittleFS. On load, magic number and version are checked; any mismatch triggers a fresh scan and rewrite.

**Current version: 10.** Version history:
- 6 — original
- 7 — added DXL1 protocol
- 8 — added DXL2 protocol
- 9 — added `midiRunMode`
- 10 — added `midiJumpFilter`

**Common mistake:** Adding fields to `PersistentConfig` without bumping `PERSIST_VERSION` causes the struct to be read with stale/wrong data silently. Always bump the version when the struct layout changes.

**Gotcha that caused settings loss:** `savePersistedState()` builds the `cfg` struct correctly but must call `persist.save(cfg)` at the end. This call was accidentally dropped during a refactor, causing every power cycle to lose all settings. The `_persistDirty` flag, `markDirty()` calls, and the lazy-write in `tick()` were all correct — the save function itself just wasn't writing.

### What is and isn't persisted

**Persisted in flash (LittleFS):** servo IDs, active index, protocol, scan baud index, torque state, speed, acc, all MIDI bindings (CC, channel, invert, smoothing), MIDI run mode, jump filter.

**Persisted in servo EPROM (written by Save Changes):** ID, min/max limits, torque limit, center offset, mode (servo/wheel), baud rate.

**Not persisted anywhere:** `lastRawTarget`, `smoothPos`, `lastRecv`, `pendingCC` — these are reset to -1/-1.0f on every boot and rebuild from live data.

---

## MIDI Run Modes

Three modes stored as `MidiRunMode` enum in `MidiState::runMode`:

| Value | TX gated | RX gated | Torque on entry |
|-------|----------|----------|-----------------|
| `SendRecv` (0) | No | No | Unchanged |
| `SendOnly` (1) | No | Yes | **Disabled on all servos** |
| `RecvOnly` (2) | Yes | No | Unchanged |

The torque disable on entering `SendOnly` is intentional safety behaviour for hand-driven recording. Torque is **not** automatically re-enabled when leaving `SendOnly` — the arm may have moved during recording and re-enabling torque could cause a sudden jolt. The user must explicitly re-enable torque in Live Control.

---

## Hardware Notes

### Auto-direction RS-485 and TTL adapters

All three protocols work with auto-direction adapters (no DE/RE pin). The adapter monitors the TX line and switches direction automatically. This means TX bytes are always echoed back to RX. Each driver handles this differently:

- **DXL1/DXL2:** `drainEcho(byteCount)` — waits a baud-derived window after `flush()` then reads and discards exactly the TX byte count
- **ST3215:** `EchoSMS_STS::wFlushSCS()` — called by the SCServo library after every TX; does `flush()` + 1-byte-time wait + bounded drain

### TTL Dynamixel without direction pin

A direction-pin-free TTL circuit can be built with a single 74AHCT1G07 (open-drain buffer, SOT-23-5):
- Input from MCU TX (3.3V logic, AHCT threshold ≈ 1.6V)
- Output: open-drain → pull-up resistor to bus voltage (3.3V or 5V)
- MCU RX taps the shared data line
- For 5V servos: add a BZX84-C3V3 Zener on the RX line to protect the Pico GPIO

This is electrically identical to the RS-485 auto-direction principle: pull-up holds the line high when both MCU and servo are idle; either side drives it low when transmitting.

### RP2040 UART `begin()` without `end()`

On the Earle Philhower RP2040 Arduino core, calling `HardwareSerial::begin(newBaud)` on an already-open port does nothing if the port is currently open — the baud rate is not changed. This is documented behaviour. Always call `end()` first to force full UART hardware reinitialisation. This applies to every baud change in all three bus drivers.

### I2C bus sharing (XIAO)

The XIAO expansion board shares one I2C bus between OLED and encoder. At 400 kHz, an OLED frame transfer takes ~23ms. The encoder can be polled within that window because the I2C transactions are short (~68µs), but if a frame is in progress the encoder read is deferred until the frame completes. In practice this adds at most one frame (~23ms) of encoder input lag. Both environments run at 400 kHz.

---

## USB Host (not yet active)

`usb_host_engine.cpp` contains a full PIO-USB host implementation behind `#if defined(USE_TINYUSB_HOST)`. This flag is intentionally absent from `platformio.ini` because enabling it conflicts with the USB MIDI device stack (both cannot share the same USB PHY configuration in the current TinyUSB/Arduino-Pico build).

`setup1()` and `loop1()` compile to empty stubs. `usbHost.begin()` initialises the event ring buffer but does nothing with hardware. `tickHostInput()` runs every tick and drains the ring buffer (always empty). Zero bytes of the USB host implementation have run on hardware.

To enable: add a separate `env:pico_host` environment with `USE_TINYUSB_HOST` defined and `USE_TINYUSB` removed. This will break USB MIDI device mode in that build.

---

## Known Issues / Future Work

- **USB host untested** — see above
- **DXL2 full MIDI validation** — basic ping/read/position confirmed; full MIDI binding flow with XH430 not explicitly tested end-to-end
- **XIAO hardware suspect** — one XIAO unit showed no serial communication despite correct firmware; replaced with Pico for production use
- **No per-servo torque enable in MIDI Run** — torque is a global flag (`_app.torqueEnabled`); in a multi-servo arm with mixed torque requirements, all servos share the same torque state
- **Smoothing convergence threshold** — currently 1 count; with smoothing=127 (very slow) and min/max spanning a small range, the servo may never quite reach the target