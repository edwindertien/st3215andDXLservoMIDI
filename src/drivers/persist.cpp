#include "persist.h"
#include <LittleFS.h>

Persist persist;

bool Persist::begin() {
  // Earle Philhower core: LittleFS.begin() mounts the filesystem.
  // board_build.filesystem_size in platformio.ini reserves the flash region.
  // Returns false if mount fails (e.g. first boot — filesystem not yet formatted).
  if (!LittleFS.begin()) {
    // Try to format and mount again
    LittleFS.format();
    if (!LittleFS.begin()) return false;
  }
  _mounted = true;
  return true;
}

bool Persist::load(PersistentConfig& dst) {
  if (!_mounted) return false;
  if (!LittleFS.exists(PERSIST_PATH)) return false;

  File f = LittleFS.open(PERSIST_PATH, "r");
  if (!f) return false;

  PersistentConfig tmp;
  size_t n = f.read(reinterpret_cast<uint8_t*>(&tmp), sizeof(tmp));
  f.close();

  if (n != sizeof(tmp))               return false;
  if (tmp.magic != PERSIST_MAGIC)     return false;
  if (tmp.version != PERSIST_VERSION) return false;

  dst = tmp;
  return true;
}

bool Persist::save(const PersistentConfig& cfg) {
  if (!_mounted) return false;

  const char* tmpPath = "/config.tmp";

  File f = LittleFS.open(tmpPath, "w");
  if (!f) return false;

  size_t n = f.write(reinterpret_cast<const uint8_t*>(&cfg), sizeof(cfg));
  f.flush();
  f.close();

  if (n != sizeof(cfg)) {
    LittleFS.remove(tmpPath);
    return false;
  }

  LittleFS.remove(PERSIST_PATH);
  LittleFS.rename(tmpPath, PERSIST_PATH);
  return true;
}

// ---------------------------------------------------------------------------
// Diagnostics — returns a snapshot of filesystem and config file status
// ---------------------------------------------------------------------------
PersistDiag Persist::diagnose() {
  PersistDiag d;
  d.mounted = _mounted;

  if (!_mounted) return d;

  FSInfo info;
  if (LittleFS.info(info)) {
    d.totalBytes = (uint32_t)info.totalBytes;
    d.usedBytes  = (uint32_t)info.usedBytes;
  }

  d.fileExists = LittleFS.exists(PERSIST_PATH);

  if (d.fileExists) {
    File f = LittleFS.open(PERSIST_PATH, "r");
    if (f) {
      d.fileSize = f.size();
      f.close();
    }
    // Peek at magic+version without full validation
    PersistentConfig tmp;
    File f2 = LittleFS.open(PERSIST_PATH, "r");
    if (f2) {
      size_t n = f2.read(reinterpret_cast<uint8_t*>(&tmp), sizeof(tmp));
      f2.close();
      if (n == sizeof(tmp)) {
        d.magic   = tmp.magic;
        d.version = tmp.version;
        d.savedServoCount = tmp.servoCount;
        d.savedMidiCount  = tmp.midiCount;
      }
    }
  }

  d.expectedSize    = sizeof(PersistentConfig);
  d.expectedMagic   = PERSIST_MAGIC;
  d.expectedVersion = PERSIST_VERSION;

  return d;
}
