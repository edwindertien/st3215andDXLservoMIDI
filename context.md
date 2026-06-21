# Developer Context — Servo Tester

This document captures architecture decisions, non-obvious implementation details, bug history, and hardware findings accumulated during development. It complements README.md and is intended for anyone reading or modifying the source code.

---

## Architecture Overview

### Three buses, five protocols, one UART

ST3215, SC09, DXL1, and DXL2 share a single hardware UART (`Serial1`, GP0/GP1). RC PWM uses dedicated GPIO pins and never touches `Serial1` at all. Each serial bus driver stores a pointer to `Serial1` and calls `_serial->end()` followed by `_serial->begin(baud)` whenever the baud rate changes. The `end()` before `begin()` is **mandatory** on the RP2040 Earle Philhower core — calling `begin()` on an already-open port at a different baud is silently ignored without the prior `end()`, leaving the UART at the old rate.

SC09 is **not** a separate `IServoBus` implementation — it's a runtime mode flag (`_scsMode`) on `ST3215Bus`, since the SC09 (SCS protocol) and ST3215 (STS protocol) share the same SCServo library family and packet framing, differing only in byte order, EEPROM lock register, and position range.

The correct initialisation sequence in `setup()`:
```
Serial1.setTX(GP0); Serial1.setRX(GP1);
st3215Bus.begin(Serial1, 1000000);  // end()+begin(1M)
dxl1Bus.begin(Serial1, 57142);      // end()+begin(57142)
dxl2Bus.begin(Serial1, 57600);      // end()+begin(57600)
rcpwmBus.begin(Serial1, 0);         // ignores both args — configures 6 PWM GPIO instead
// Serial1 ends up at 57600 after setup()
// loadPersistedState() then calls _bus->setBaud(persisted_baud)
// which does end()+begin() to claim the correct rate (no-op for RC PWM)
```

### IServoBus interface

All five protocols implement `IServoBus` (pure virtual). `App` holds a `IServoBus* _bus` pointer that is swapped at runtime via `busForProtocol()` in `main.cpp`. No protocol-specific code exists in the bulk of `app.cpp` — the one exception is the RC PWM fast-path in `handleSelectProtocolInput()`, which bypasses the scan screen entirely since RC PWM has no bus to scan.

```cpp
IServoBus* busForProtocol(BusProtocol p) {
  switch (p) {
    case BusProtocol::DXL1:  return &dxl1Bus;
    case BusProtocol::DXL2:  return &dxl2Bus;
    case BusProtocol::RCPWM: return &rcpwmBus;
    case BusProtocol::SC09:
      st3215Bus.setScsMode(true);   // lock reg 48, posMax 1023, End=1
      return &st3215Bus;
    default:
      st3215Bus.setScsMode(false);  // lock reg 55, posMax 4095, End=0
      return &st3215Bus;
  }
}
```

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

The ST3215/SC09 use the SCServo library (`SMS_STS` base class, subclassed as `EchoSMS_STS`). The library's `wFlushSCS()` is called after every TX packet and before reading the servo response — the base implementation is empty. With an auto-direction adapter (no DE/RE pin), TX bytes are echoed back to RX. The echo passes the SCServo checksum validation because the PING packet checksum is algebraically identical to a STATUS response checksum for the same ID.

**Fix:** `EchoSMS_STS` subclass in `servo_bus.h` overrides both `writeSCS()` (to assert a wired DE pin before TX, if present) and `wFlushSCS()`:
```cpp
void wFlushSCS() override {
    pSerial->flush();           // wait for TX shift register to empty
    if (_dePin >= 0) {
      // Wired direction control: guard delay then release DE. No echo.
      delayMicroseconds(guardUs);
      digitalWrite(_dePin, LOW);
    } else {
      // Auto-direction adapter: echo arrives on RX; drain it (bounded, not until-empty).
      delayMicroseconds(waitUs);
      while (pSerial->read() != -1) {}
    }
}
```

Critical subtleties:
- `flush()` must be called first — `write()` is non-blocking; without `flush()` the drain runs before the echo has arrived
- The drain must be **bounded by time, not "until empty"** — an open-ended drain loop consumes the servo response which arrives ~50µs after the echo ends
- `_baud` must be kept in sync with `Serial1` baud whenever `setBaud()` is called
- `_dePin` is set from `HW::SERVO_DE_PIN` in `begin()` — `-1` for auto-direction builds (Pico Grove, XIAO), a real GPIO number for the wired-RS485 custom board

### Why it appeared to work before the end()+begin() fix

Before the `end()+begin()` fix, `Serial1` was left in a partially-reinitialised state after `setBaud()`. The RP2040 UART RX was in a subtly broken condition that caused echo bytes to arrive with framing errors and be silently discarded — so the library never saw the echo. Once `end()+begin()` made the UART work correctly, the echo arrived cleanly and the false-positive scan problem appeared, leading to the fix above.

---

## SC09 — Detailed Findings

The SC09 reuses `ST3215Bus`/`EchoSMS_STS` rather than being a separate driver. Three distinct bugs were found and fixed, each requiring real hardware diagnostics to pin down precisely — documentation and library source alone were insufficient or actively misleading at points.

### 1. Byte order (`End` flag)

The SCServo library's `SMS_STS` class sets `End = 0` (little-endian word encoding) in its constructor; the separate `SCSCL` class (intended for SC09/SC15-family servos) sets `End = 1` (big-endian) instead. Since `EchoSMS_STS` inherits from `SMS_STS`, it always had `End = 0` — **every word read/write to an SC09 had its bytes swapped**, producing garbage min/max values (observed: min reading back as 5120, max as 60163) and position reads failing entirely (`SMS_STS::ReadPos()` additionally applies a sign-bit mask that corrupted SC09-range values once the byte swap was also in play).

**Fix:** `setScsMode(bool)` sets `_servo.End = scs ? 1 : 0` immediately, and `begin()` also sets it from the current `_scsMode` so a baud change doesn't reset it. `readPosition()` was changed to call `_servo.readWord()` directly rather than `_servo.ReadPos()`, bypassing the sign-bit mask that doesn't apply to SC09's 10-bit range.

### 2. EEPROM lock register address

`SMS_STS_LOCK = 55` (0x37) on ST3215; `SCSCL_LOCK = 48` (0x30) on SC09 — **different addresses**. The original code called `_servo.unLockEprom()`/`LockEprom()`, which hardcodes `SMS_STS_LOCK = 55`. On an SC09, address 55 is unrelated to the lock state, so EEPROM stayed locked throughout every "unlock → write → relock" sequence, and writes went to SRAM-only shadow registers that reverted on power cycle.

**Fix:** replaced library helper calls with a macro pair selecting the register by `_scsMode`:
```cpp
#define LOCK_REG  (_scsMode ? SCS_LOCK_REG : STS_LOCK_REG)   // 48 : 55
#define UNLOCK(id) _servo.writeByte((id), LOCK_REG, 0)
#define RELOCK(id) _servo.writeByte((id), LOCK_REG, 1)
```
Applied consistently across `saveId`, `saveMinMax`, `saveBaud` (the only EEPROM writes still relevant to SC09 — `saveTorqueLimit`, `saveCenterOffset`, and `saveMode` are no-ops in SC09 mode since those registers don't exist on the SC09 control table).

### 3. Max-angle-limit `writeWord()` EEPROM commit failure

After fixing (1) and (2), min-limit writes via `saveMinMax()` worked reliably for arbitrary values, persisted across power cycles, and were confirmed correct by direct register reads. **Max-limit writes via the same code path consistently failed to commit**, despite the servo returning a successful Ack for the write command. The servo silently kept its previous max value.

This was diagnosed with a dedicated serial debug command (`m`) added specifically for this bug, rather than guessed from documentation. The diagnostic process:

1. Confirmed the servo was reachable (`ping`) and at the expected ID/baud — an earlier diagnostic attempt failed silently because it hardcoded ID=1 and never set the baud, talking into the void. Fixed by reading the actual active servo ID and baud index from `appState` instead of assuming.
2. Read the lock register before/after a manual unlock — confirmed the unlock genuinely took effect (`reg48: 1 → 0`).
3. Called `saveMinMax(id, 100, 900)` via the normal app code path — it returned `true`, but a subsequent `loadConfig()` still showed `max=1003` (the old value).
4. As an isolated test, wrote the same target value (900) to registers 11 and 12 as **two separate single-byte `writeByte()` calls** instead of one `writeWord()` call — this **succeeded** and persisted, confirmed by immediate readback and by a later `loadConfig()` call.

Conclusion: the SC09's firmware (or this particular unit's firmware revision) does not reliably commit a 2-byte EEPROM write delivered in a single packet to this specific register pair, while two independent 1-byte write commands to the same addresses succeed every time. The min-limit register pair (9/10) does not exhibit this problem with `writeWord()` — only the max-limit pair (11/12) does.

**Fix:** `saveMinMax()` branches on `_scsMode`. In SC09 mode, both min and max are written as four separate `writeByte()` calls (low byte, then high byte, per limit) with 10ms settling delays between each, computing the `End=1` byte split manually:
```cpp
uint8_t minL = (uint8_t)(minWire >> 8);   // End=1: low-address byte = high byte of value
uint8_t minH = (uint8_t)(minWire & 0xFF); // End=1: high-address byte = low byte of value
```
The byte values were verified against the diagnostic's confirmed-working raw register values before committing the fix (e.g. `max=900` → `reg11=3, reg12=132`, matching exactly what the raw single-byte diagnostic had already shown to persist correctly). ST3215/STS mode is completely unaffected — it still uses the original `writeWord()` calls.

This is the kind of bug that cannot be found by inspecting the library or datasheet alone; it required building a targeted on-device diagnostic, capturing real register state, and iterating based on what the actual hardware did.

---

## DXL2 CRC Table

The Dynamixel Protocol 2.0 CRC-16 uses a lookup table with 256 uint16 entries. The original table had 48 wrong entries at indices 208–255 (0xD0–0xFF). This caused the wrong CRC to be computed for any packet that fed bytes ≥ 0xD0 into the CRC accumulator during the `((crc >> 8) ^ b) & 0xFF` step — which includes the 0xFF header bytes of every ping packet.

The correct table is from the Robotis official `protocol.c`. Verification: `crc16([0xFF,0xFF,0xFD,0x00,0x01,0x03,0x00,0x01])` must equal `0x4E19` (bytes `19 4E` in the packet). The READ packet CRC `0xD535` (bytes `35 D5`) can also be used as a cross-check.

---

## DXL1 Baud Rates

Dynamixel Protocol 1.0 MX-series baud formula: `actual = 2,000,000 / (register + 1)`. The factory default register is 34, giving `2,000,000 / 35 = 57,142` baud — not 57,600. Dynamixel Wizard labels this as "57600" but the actual UART must be set to 57,142 or every frame will have framing errors. This is a known Robotis documentation ambiguity.

---

## RC PWM Driver

`RCPWMBus` (`rcpwm_bus.h/.cpp`) implements `IServoBus` using the RP2040's native hardware PWM peripheral (`hardware/pwm.h` from the Pico SDK), not software bit-banging.

### Hardware mapping
6 channels map to GPIO pins defined per board in `HW::RCPWM_PINS[6]`:
- `BOARD_CUSTOM_RS485`: GP6, 7, 8, 26, 27, 28
- `BOARD_PICO_GROVE`: GP10, 11, 12, 13, 18, 19 (GP6/7 unavailable — taken by the encoder's I2C1 bus on this board)
- `BOARD_XIAO_EXPANSION`: GP2, 8, 9, 10, 28, 29 (the only 6 free pins physically exposed on the XIAO RP2040 module's expansion board — confirmed against the actual pinout, not assumed)

Each pin maps to an RP2040 PWM slice/channel pair via `pwm_gpio_to_slice_num()`/`pwm_gpio_to_channel()`. Multiple channels can share a slice (e.g. GP6=slice3A, GP7=slice3B) — this is fine since all channels run at the same 50Hz frequency; `pwm_init()` is idempotent per-slice for a shared frequency configuration.

### PWM timing
```
clock = 125 MHz, divider = 64, wrap = 39062
→ frequency = 125e6 / (64 × 39063) ≈ 50.00 Hz
→ 1 µs = 1.953125 counts
800 µs  → 1562 counts (min)
1500 µs → 2930 counts (centre)
2200 µs → 4297 counts (max)
```
`usToCounts()` computes `(us * 125) / 64` directly (integer math, off by at most 1 count / 0.5µs — negligible).

### Protocol semantics
- `posMin()`/`posMax()` return **800/2200 directly** (microseconds), not an abstract 0–4095 range — this makes the Live Control "T:" readout directly meaningful and was a deliberate design choice after the initial implementation used 0–4095 and required an extra mental mapping step.
- `scan()` does no bus communication at all — it instantly fills `ids[0..5] = {1,2,3,4,5,6}`.
- `torqueEnable(true)` starts outputting pulses at the channel's last commanded position (`setPulse(ch, usToCounts(_pos[ch]))`); `torqueEnable(false)` sets the PWM level to 0, meaning the pin stays low for the entire 20ms period — most RC servos interpret sustained signal absence as "release," going limp. This exactly matches `Servo.attach()`/`Servo.detach()` semantics from the Arduino Servo library.
- `readPosition()` returns the last commanded value, since there's no physical feedback wire on a standard RC servo.
- All `save*()` methods are no-ops returning `false` (or `true` where a "nothing to do" success makes more sense) — RC servos have no EEPROM.

### Protocol-switch fast path

Selecting RC PWM in `handleSelectProtocolInput()` bypasses the normal scan-prompt flow entirely:
```cpp
if (chosen == BusProtocol::RCPWM) {
  _app.servoCount = _bus->scan(_app.ids, RCPWM_CH_COUNT, lastId); // instant, no bus traffic
  _app.activeIndex = 0;
  loadActiveServoRuntime();
  loadStagedConfigFromActive();
  _app.torqueEnabled = true;
  for (uint8_t s = 0; s < _app.servoCount; ++s)
    _bus->torqueEnable(_app.ids[s], true);   // attach all 6 channels immediately
  ...
  return; // skip the "scan needed" prompt entirely
}
```
The `torqueEnable(true)` loop was added after an initial implementation shipped without it — the symptom was "menu works, channels populate, but zero PWM output." `_app.torqueEnabled` defaults to whatever was last persisted from a previous protocol session (often `false` from DXL2, which intentionally starts torque-off), and nothing in the original RC PWM code path ever flipped it on. Since `RCPWMBus::setPosition()` itself also internally gates on its own per-channel `_enabled[]` array, there were effectively two independent "off" switches that both needed to be flipped — this fix flips both by routing through the normal `torqueEnable()` call.

### Home menu gating

The `Scan Bus`, `Configure`, and `Save Changes` Home menu items each check `_app.protocol == BusProtocol::RCPWM` and show a brief splash ("Not needed" / "Not available") instead of entering those screens, since none of them have any meaning for RC PWM (no bus to scan, no EEPROM to configure or save).

### No timing/clock-domain interference with MIDI or system timing

This was explicitly verified rather than assumed, since it's a reasonable thing to worry about when adding a new hardware peripheral:
- RP2040's PWM peripheral block is entirely separate hardware from the SysTick timer driving `millis()`/`micros()` and from the general Timer peripheral — setting a PWM slice's clock divider cannot affect system timing.
- No pin overlap between RC PWM channels and UART/I2C/USB-host pins on any board (checked explicitly per-board).
- `pwm_set_chan_level()` (used by both `setPosition()` and `torqueEnable()`) is a single hardware register write — sub-microsecond, non-blocking. This is *cheaper* per call than any serial bus protocol's `setPosition()` (which involves a full UART TX with echo-drain timing), so RC PWM channels never become the bottleneck in the 8ms MIDI TX/RX tick budget.

---

## MIDI Timing Architecture

### TX path (outgoing CC — arm position → DAW)

One servo is polled per TX tick (round-robin). Tick interval: `MIDI_TX_INTERVAL_MS = 8ms`. With N servos, each servo's position is read and sent as CC at `1000 / (8 × N)` Hz. With 6 servos: ~21 Hz per servo. (Originally 25ms interval gave only ~6.7Hz per servo with 6 servos — reduced after the user observed choppy recorded automation in Ableton.)

The TX tick fires only when `runMode != RecvOnly`.

### RX path (incoming CC — DAW → arm)

Every RX tick (`MIDI_RX_INTERVAL_MS = 8ms`), all bindings with `needsSend = true` call `setPosition()`. `needsSend` is true when:
- New `pendingCC` arrived (set in `onMidiCC()` interrupt callback), OR
- `smoothing > 0` and `smoothPos` hasn't converged to `lastRawTarget` within 1 count

The RX tick fires only when `runMode != SendOnly`.

An earlier implementation called `setPosition()` unconditionally every tick for every binding regardless of whether anything changed — with 6 servos and a 1ms inter-servo delay this consumed the entire tick budget even at rest, causing severe processing delays when changing speed/acc/smoothing mid-stream. The `needsSend` gate and removal of the unconditional per-tick delay fixed this; idle ticks now do zero bus work.

### pendingCC — last-write-wins

`pendingCC` is an `int8_t` storing the most recent CC value (-1 = none pending). If multiple CC messages arrive within one 8ms RX window, only the last is applied — applying every intermediate value would hammer the bus faster than servos can execute commands.

`lastRawTarget` stores the last computed `ccToPos()` result separately from `pendingCC`. After `pendingCC` is consumed, the smoothing loop continues advancing `smoothPos` toward `lastRawTarget` on subsequent ticks, ensuring the servo always reaches its final position even after CC messages stop arriving — without this, the smoothing filter would stall at whatever position it reached on the single tick the last CC was processed.

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

When a speed or acceleration CC arrives, `_app.speed`/`_app.acc` is updated immediately. No `setPosition()` calls are made — the new value takes effect naturally on the next per-servo `setPosition()` in the RX loop. An earlier implementation called `setPosition()` with `_app.targetPos` (the live-control single position, not the per-servo MIDI target) for all servos when speed/acc changed, which snapped every arm to the live-control default position (centre) whenever the user touched the speed or acceleration fader in Ableton.

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

## Persistence

`PersistentConfig` is a flat struct written as raw bytes to `/config.bin` on LittleFS. On load, magic number and version are checked; any mismatch triggers a fresh scan and rewrite.

**Current version: 12.** Version history:
- 6 — original
- 7 — added DXL1 protocol
- 8 — added DXL2 protocol
- 9 — added `midiRunMode`
- 10 — added `midiJumpFilter`
- 11 — added `scsMode`
- 12 — added RCPWM protocol

**Common mistake:** Adding fields to `PersistentConfig` without bumping `PERSIST_VERSION` causes the struct to be read with stale/wrong data silently. Always bump the version when the struct layout changes.

**Gotcha that caused settings loss:** `savePersistedState()` builds the `cfg` struct correctly but must call `persist.save(cfg)` at the end. This call was accidentally dropped during a refactor (a `str_replace` edit that merged the closing brace of a preceding block with the call site), causing every power cycle to lose all settings. The `_persistDirty` flag, `markDirty()` calls, and the lazy-write in `tick()` were all correct — the save function itself just wasn't writing. Worth double-checking after any large edit to `savePersistedState()`.

### What is and isn't persisted

**Persisted in flash (LittleFS):** servo IDs, active index, protocol, scan baud index, torque state, speed, acc, all MIDI bindings (CC, channel, invert, smoothing), MIDI run mode, jump filter, SC09/STS sub-mode.

**Persisted in servo EPROM (written by Save Changes, not applicable to RC PWM):** ID, min/max limits, torque limit, center offset, mode (servo/wheel), baud rate.

**Not persisted anywhere:** `lastRawTarget`, `smoothPos`, `lastRecv`, `pendingCC` — these are reset to -1/-1.0f on every boot and rebuild from live data. RC PWM's `_pos[]`/`_enabled[]` arrays are also runtime-only, reset to centre/disabled on `begin()` then immediately re-enabled by the protocol-switch fast path if RC PWM was the persisted active protocol.

---

## Hardware Notes

### Auto-direction vs wired RS-485

Two distinct direction-control schemes are supported, selected per-board via `HW::SERVO_DE_PIN` (`-1` = auto-direction, a real GPIO = wired):

- **Auto-direction adapters** (Pico Grove, XIAO): no DE/RE pin. The adapter monitors the TX line and switches direction automatically. TX bytes are always echoed back to RX. Each driver handles this: DXL1/DXL2 use `drainEcho(byteCount)` (waits a baud-derived window after `flush()` then reads and discards exactly the TX byte count); ST3215/SC09 use `EchoSMS_STS::wFlushSCS()` (bounded time-based drain, not drain-until-empty).
- **Wired RS485** (custom board, GP2): the transceiver's DE/RE pin is driven directly by firmware. `txBegin()` asserts HIGH before TX, `txEnd()` releases LOW after a guard delay. No echo is produced since the transceiver physically blocks RX during TX. `EchoSMS_STS::writeSCS()` (both overloads) and `wFlushSCS()` both branch on `_dePin >= 0` to select the correct behaviour.

### TTL Dynamixel without direction pin (alternative hardware)

A direction-pin-free TTL circuit can be built with a single 74AHCT1G07 (open-drain buffer, SOT-23-5):
- Input from MCU TX (3.3V logic, AHCT threshold ≈ 1.6V)
- Output: open-drain → pull-up resistor to bus voltage (3.3V or 5V)
- MCU RX taps the shared data line
- For 5V servos: add a BZX84-C3V3 Zener on the RX line to protect the Pico GPIO

This is electrically identical to the RS-485 auto-direction principle: pull-up holds the line high when both MCU and servo are idle; either side drives it low when transmitting.

### ST3215/SC09 on a 5V (Dynamixel-spec) TTL bus

The ST3215/SC09 are 3.3V logic devices. Some Robotis-style TTL converters pull the bus up to 5V. This works in practice for testing — the ST3215's open-drain output can sink current against a 5V pull-up fine, and its output-low voltage (~0.4V) is well within the Pico's VIL threshold (0.8V max) — but it's electrically out of spec for the Pico's 3.3V-rated GPIO seeing 5V on RX. A 3.3V Zener clamp (BZX84-C3V3, cathode to data line, anode to GND) is recommended for any permanent installation.

### RP2040 UART `begin()` without `end()`

On the Earle Philhower RP2040 Arduino core, calling `HardwareSerial::begin(newBaud)` on an already-open port does nothing if the port is currently open — the baud rate is not changed. This is documented behaviour. Always call `end()` first to force full UART hardware reinitialisation. This applies to every baud change in all serial bus drivers (not RC PWM, which never touches `Serial1`).

### I2C bus sharing and recovery

All non-`BOARD_PICO_GROVE` boards share one I2C bus between OLED and encoder (Pico Grove keeps them on separate buses, Wire and Wire1). At 400 kHz, an OLED frame transfer takes ~23ms; the encoder's short transactions (~68µs) can be deferred by at most one frame, adding negligible input lag.

`Wire.begin()` on RP2040 can hang indefinitely if a device is holding SDA low from a previous failed transaction (e.g. MCU reset mid-I2C-transfer). Fixed by manually clocking SCL 9 times (with SDA as input-pullup) before `Wire.begin()`, followed by a manufactured STOP condition — this matches the standard I2C bus-recovery procedure and runs unconditionally in `setup()` before any I2C peripheral is touched. `Serial.begin()` was also moved to before this recovery sequence so boot diagnostics are visible even if I2C subsequently hangs.

### Custom board bring-up sequence (debugging notes from real bring-up)

When validating the `BOARD_CUSTOM_RS485` environment for the first time, several issues were found and fixed in sequence — documented here as a reference for bringing up future board variants:

1. **`#if/#elif/#else/#endif` chain broken** — inserting the new board's `#elif` block accidentally left a stray `#else` with no matching `#endif`, making the entire rest of `config.h` (including the `ST3215`, `UI` namespaces) unreachable. Caused a cascade of "not declared in this scope" errors across every file that includes `config.h`.
2. **Constants silently dropped during a `str_replace`** — adding `SERVO_DE_PIN` to the Pico Grove block via string replacement matched on the wrong span and ate several adjacent constant declarations (`OLED_ROTATION`, `OLED_W/H/ADDR`, `ENC_SDA_PIN`) along with it. Always re-`view` a file immediately after editing it, especially with str_replace operations that touch dense constant blocks.
3. **Display didn't initialise; no serial output** — root cause was a stuck I2C bus (no pull-ups on long wire runs), not a software issue; the firmware itself wasn't hanging at all, but `Wire.begin()` was. The fix (bus recovery sequence, described above) ended up being broadly useful, but the actual permanent fix for this specific board was physically adding I2C pull-up resistors.
4. **`#if (HW::SERVO_DE_PIN < 0)`** — the preprocessor cannot evaluate C++ namespace expressions or `constexpr` values; this must be a runtime `if` statement, not a `#if`. A recurring mistake worth flagging: any conditional involving `HW::` constants needs to be `if`, never `#if`/`#ifdef`.
5. **DXL2 worked over the wired-RS485 transceiver but ST3215 initially did not** — `Dxl1Bus`/`Dxl2Bus` already had `_dePin` constructor parameters (unused, always `-1`, on the auto-direction builds) which just needed to be wired to `HW::SERVO_DE_PIN`. `EchoSMS_STS` had no DE-pin concept at all and needed it added from scratch (see "Auto-direction vs wired RS-485" above).
6. **Missing `#include "../config.h"`** — `servo_bus.cpp` didn't include `config.h`, so `HW::SERVO_DE_PIN` wasn't visible there even after being correctly defined. `dxl1_bus.cpp`/`dxl2_bus.cpp` already had the include (which is why they worked first) — `servo_bus.cpp` was the one file missing it. A reminder that each `.cpp` needs its own explicit includes; transitively-available symbols through a chain of headers are not guaranteed.

---

## USB Host (not yet active)

`usb_host_engine.cpp` contains a full PIO-USB host implementation behind `#if defined(USE_TINYUSB_HOST)`. This flag is intentionally absent from `platformio.ini` because enabling it conflicts with the USB MIDI device stack (both cannot share the same USB PHY configuration in the current TinyUSB/Arduino-Pico build).

`setup1()` and `loop1()` compile to empty stubs. `usbHost.begin()` initialises the event ring buffer but does nothing with hardware. `tickHostInput()` runs every tick and drains the ring buffer (always empty). Zero bytes of the USB host implementation have run on hardware.

To enable: add a separate `env:pico_host` environment with `USE_TINYUSB_HOST` defined and `USE_TINYUSB` removed. This will break USB MIDI device mode in that build.

---

## Known Issues / Future Work

- **USB host untested** — see above
- **DXL2 full MIDI validation** — basic ping/read/position confirmed; full MIDI binding flow with XH430 not explicitly tested end-to-end
- **XIAO hardware suspect** — one XIAO unit showed no serial communication despite correct firmware; replaced with Pico for production use. Root cause not conclusively identified — possibly a Grove-port wiring mixup (UART vs I2C ports look identical) rather than a hardware fault.
- **No per-servo torque enable in MIDI Run** — torque is a global flag (`_app.torqueEnabled`); in a multi-servo arm with mixed torque requirements, all servos share the same torque state. Not an issue for RC PWM since all 6 channels are torque-enabled together by design.
- **Smoothing convergence threshold** — currently 1 count; with smoothing=127 (very slow) and min/max spanning a small range, the servo may never quite reach the target before the next CC arrives.
- **SC09 EEPROM commit bug root cause unconfirmed at the firmware level** — the workaround (4 single-byte writes instead of 2 word writes) is proven reliable by repeated testing, but *why* the SC09 rejects the 2-byte packet write specifically for the max-limit register pair (and not the min-limit pair) was not traced into the servo's own firmware — only empirically characterised from the outside. Worth retesting if a different SC09 batch/firmware revision is sourced in future, in case the bug is unit-specific rather than universal to the SC09 line.