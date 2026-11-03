// =============================================================================
//  FfbEngine.h  -  turns the game's force-feedback commands into one torque.
// =============================================================================
//
//  How force feedback reaches the wheel (HID PID):
//   A game doesn't send "torque = 40 %" every millisecond. Instead it creates
//   *effects* on the device and then mostly leaves them running:
//     - "Constant force, 40 % to the right"   (updated often by most games)
//     - "Spring, centered, this stiff"        (self-centering)
//     - "Damper, this strong"                 (heavy steering at low speed)
//     - "Sine wave, 20 Hz, small"             (engine rumble, kerbs)
//   Windows' PID driver translates the game's DirectInput calls into USB
//   reports, which arrive here:
//     1. Create New Effect  -> we pick a free slot (1..40) and tell the PC
//                              its number via the Block Load report
//     2. Set Effect         -> duration, gain, direction for that slot
//     3. Set Constant/Periodic/Condition/Ramp/Envelope -> the effect's values
//     4. Effect Operation   -> start / stop it
//     5. Block Free         -> delete it
//   Then, 1000 times a second, computeForce() adds up every playing effect
//   using the current wheel position and speed, and returns one torque.
//
//  The spring/damper/friction/inertia ("condition") effects are calculated
//  here on the wheel, which is why they feel smooth even though the game only
//  sends them once: the Teensy reacts to the wheel position every millisecond.
//
//  This file has no Arduino code, so the whole engine is unit-tested on a PC.
// =============================================================================
#pragma once
#include <stdint.h>

#include "FfbReportSizes.h"

namespace ffb {

// Effect types, numbered as in the HID descriptor (tools/gen_ffb_descriptor.py).
enum EffectType : uint8_t {
  ET_NONE = 0,
  ET_CONSTANT = 1,
  ET_RAMP = 2,
  ET_SQUARE = 3,
  ET_SINE = 4,
  ET_TRIANGLE = 5,
  ET_SAW_UP = 6,
  ET_SAW_DOWN = 7,
  ET_SPRING = 8,
  ET_DAMPER = 9,
  ET_INERTIA = 10,
  ET_FRICTION = 11,
  ET_COUNT
};

// Report IDs, as in the HID descriptor.
namespace report {
constexpr uint8_t SET_EFFECT = 1, SET_ENVELOPE = 2, SET_CONDITION = 3, SET_PERIODIC = 4,
                  SET_CONSTANT = 5, SET_RAMP = 6, EFFECT_OPERATION = 10, BLOCK_FREE = 11,
                  DEVICE_CONTROL = 12, DEVICE_GAIN = 13;
constexpr uint8_t CREATE_NEW_EFFECT = 5, BLOCK_LOAD = 6, POOL = 7;  // feature reports
constexpr uint8_t TYPE_INPUT = 1, TYPE_OUTPUT = 2, TYPE_FEATURE = 3;
}  // namespace report

constexpr uint16_t DURATION_INFINITE = 0xFFFF;

struct Condition {
  int16_t cpOffset = 0;      // center point, -10000..10000
  int16_t posCoef = 0;       // strength right of center, -10000..10000
  int16_t negCoef = 0;       // strength left of center
  uint16_t posSat = 10000;   // max force right, 0..10000
  uint16_t negSat = 10000;   // max force left
  uint16_t deadBand = 0;     // no force within +-deadBand of center
};

struct Effect {
  uint8_t type = ET_NONE;    // ET_NONE = slot is free
  bool playing = false;
  // Set Effect
  uint16_t durationMs = DURATION_INFINITE;
  uint16_t startDelayMs = 0;
  uint8_t gain = 255;
  uint8_t axesEnable = 1;    // bit0 = X, bit1 = Y
  bool directionEnable = false;
  uint16_t direction = 9000; // polar angle in 1/100 degree (9000 = 90 deg)
  // Envelope (fades in/out of constant, ramp and periodic effects)
  uint16_t attackLevel = 0, fadeLevel = 0, attackTimeMs = 0, fadeTimeMs = 0;
  // Constant
  int16_t magnitude = 0;
  // Ramp
  int16_t rampStart = 0, rampEnd = 0;
  // Periodic
  uint16_t periodicMagnitude = 0;
  int16_t periodicOffset = 0;
  uint16_t phase = 0;        // 1/100 degree
  uint16_t periodMs = 100;
  // Condition: one block per axis; we only use X (block 0)
  Condition condition;
  // Playback
  uint32_t startMs = 0;      // when it (re)started, after the start delay
  uint8_t loopCount = 1;
};

// The wheel's state, in the units the engine wants.
struct WheelState {
  float position = 0;      // -1 = full left lock, +1 = full right lock
  float velocity = 0;      // -1..1, see FfbSettings::damperFullSpeedDegS
  float acceleration = 0;  // -1..1, see FfbSettings::inertiaFullAccelDegS2
};

struct FfbSettings {
  // Forces whose direction comes from the game (constant, ramp, periodic) get
  // multiplied by -1 when this is set. Conditions (spring etc.) are worked out
  // from the wheel position here, so they are always the right way round.
  bool invertDirectional = false;
};

class Engine {
 public:
  FfbSettings settings;

  Engine() { reset(); }

  // Forget all effects and return to power-up state.
  void reset();

  // ---- Called from the USB interrupt -------------------------------------------
  // `data` starts with the report ID byte, like on the wire.
  void onSetReport(uint8_t type, const uint8_t* data, uint32_t len, uint32_t nowMs);
  // Fills `buf` (starting with the report ID) and returns its length, or 0.
  uint32_t onGetReport(uint8_t type, uint8_t id, uint8_t* buf, uint32_t maxLen);

  // ---- Called from the main loop --------------------------------------------------
  // Returns the total torque the game asks for, -1..+1 (+ = right).
  float computeForce(const WheelState& w, uint32_t nowMs);

  bool actuatorsEnabled() const { return actuatorsEnabled_; }
  bool paused() const { return paused_; }
  uint8_t deviceGain() const { return deviceGain_; }
  int playingCount() const;
  int allocatedCount() const;
  uint32_t reportCount() const { return reportCount_; }
  const Effect& effect(uint8_t blockIndex) const { return effects_[blockIndex - 1]; }

  // Torque of one effect at this moment, before device gain (for tests and
  // the serial monitor). Also handles finishing effects whose time is up.
  float effectForce(Effect& e, const WheelState& w, uint32_t nowMs);

 private:
  Effect* get(uint8_t blockIndex);
  void createEffect(uint8_t type);
  void startEffect(Effect& e, uint8_t loops, uint32_t nowMs);

  Effect effects_[ffb_size::MAX_EFFECTS];
  bool actuatorsEnabled_ = true;
  bool paused_ = false;
  uint8_t deviceGain_ = 255;
  // Result of the last Create New Effect, read back by the PC as Block Load.
  uint8_t lastBlockIndex_ = 0;
  uint8_t lastBlockStatus_ = 3;  // 1 = success, 2 = full, 3 = error
  uint32_t reportCount_ = 0;
};

}  // namespace ffb
