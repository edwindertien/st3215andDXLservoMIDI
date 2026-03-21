#pragma once
#include "model/servo_model.h"
#include "config.h"

struct AppState {
  bool oledOk    = false;
  bool encoderOk = false;

  BusProtocol protocol = BusProtocol::ST3215;  // active servo protocol

  uint8_t ids[ST3215::MAX_SCAN_IDS] = {0};
  uint8_t servoCount = 0;
  int activeIndex    = 0;

  int targetPos       = ST3215::POS_MID;
  int actualPos       = -1;
  int speed           = ST3215::DEFAULT_SPEED;
  int acc             = ST3215::DEFAULT_ACC;
  bool torqueEnabled  = true;

  int lastPingId = -1;

  ScreenId screen  = ScreenId::Home;
  int menuIndex    = 0;
  bool editing     = false;

  ServoParam editParam = ServoParam::None;
  int editValue        = 0;

  ServoConfigBuffer cfg;

  bool saveOk = false;
  char saveMessage[32] = "Not saved";

  int scanBaudIndex      = 0;
  int scanAllBaudStep    = 0;
  uint32_t scanAllCurrentBaud = 0;

  int pendingMode = 0;

  MidiState midi;

  unsigned long lastFeedbackMs = 0;
  unsigned long lastUiMs       = 0;
};
