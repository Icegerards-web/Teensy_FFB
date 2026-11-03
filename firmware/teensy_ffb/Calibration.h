// =============================================================================
//  Calibration.h  -  calibration values that survive a power cycle.
// =============================================================================
//
//  The Teensy 4.0 has a small "EEPROM": memory that keeps its contents when
//  the power is off (on the Teensy 4 it is really a reserved bit of flash,
//  but the EEPROM library makes it behave like classic Arduino EEPROM).
//
//  We store one Calibration struct at address 0. The `magic` and `version`
//  fields let us recognise our own data: a brand-new Teensy holds random
//  bytes there, and if we ever add fields the version number changes. When
//  either doesn't match we fall back to the defaults instead of using garbage.
// =============================================================================
#pragma once
#include <EEPROM.h>

#include "config.h"

struct PedalCal {
  int32_t rawMin;  // reading with the pedal released
  int32_t rawMax;  // reading with the pedal fully pressed
};

// Version 1 layout (step 1). Kept so an older save can be upgraded.
struct CalibrationV1 {
  uint32_t magic;
  uint16_t version;
  int32_t steeringCenter;
  float steeringRangeDeg;
  PedalCal throttle;
  PedalCal brake;
  PedalCal clutch;
};

struct Calibration {
  uint32_t magic;
  uint16_t version;
  int32_t steeringCenter;   // sensor's center value (see PositionSensor)
  float steeringRangeDeg;   // lock-to-lock rotation the game sees
  PedalCal throttle;
  PedalCal brake;
  PedalCal clutch;
  // --- added in version 2 (motor + force feedback) ---
  float maxTorque;          // 0..1, hard ceiling on motor power
  float endstopStiffness;   // torque per degree past full lock
  uint8_t armAtBoot;        // 1 = motor allowed on as soon as it powers up
  uint8_t ffbInvert;        // 1 = flip the direction of game forces
  uint8_t reserved[2];
  float ffbGain;            // 0..1, overall strength of game forces
  float damping;            // 0..1, always-on "thick oil" feel, tames wobble
  float friction;           // 0..1, always-on dry friction feel
  float minTorque;          // 0..1, smallest torque that still moves the motor
};

namespace calibration {

constexpr uint32_t MAGIC = 0x46464257;  // the letters "FFBW" as a number
constexpr uint16_t VERSION = 2;
constexpr int EEPROM_ADDRESS = 0;

inline Calibration defaults() {
  Calibration c;
  c.magic = MAGIC;
  c.version = VERSION;
  c.steeringCenter = ADC_MAX / 2;
  c.steeringRangeDeg = DEFAULT_STEERING_RANGE_DEG;
  c.throttle = {0, ADC_MAX};
  c.brake = {0, ADC_MAX};
  c.clutch = {0, ADC_MAX};
  c.maxTorque = DEFAULT_MAX_TORQUE;
  c.endstopStiffness = DEFAULT_ENDSTOP_STIFFNESS;
  c.armAtBoot = 0;
  c.ffbInvert = 0;
  c.reserved[0] = c.reserved[1] = 0;
  c.ffbGain = DEFAULT_FFB_GAIN;
  c.damping = DEFAULT_DAMPING;
  c.friction = DEFAULT_FRICTION;
  c.minTorque = DEFAULT_MIN_TORQUE;
  return c;
}

// Returns true if valid saved data was found; otherwise `out` gets defaults.
inline bool load(Calibration& out) {
  EEPROM.get(EEPROM_ADDRESS, out);
  if (out.magic == MAGIC && out.version == VERSION) return true;
  if (out.magic == MAGIC && out.version == 1) {
    // Saved by the step 1 firmware: keep the steering and pedal calibration,
    // and use defaults for the new motor settings.
    CalibrationV1 old;
    EEPROM.get(EEPROM_ADDRESS, old);
    out = defaults();
    out.steeringCenter = old.steeringCenter;
    out.steeringRangeDeg = old.steeringRangeDeg;
    out.throttle = old.throttle;
    out.brake = old.brake;
    out.clutch = old.clutch;
    return true;
  }
  out = defaults();
  return false;
}

inline void save(const Calibration& c) {
  // EEPROM.put only rewrites bytes that changed, which spares the flash.
  EEPROM.put(EEPROM_ADDRESS, c);
}

}  // namespace calibration
