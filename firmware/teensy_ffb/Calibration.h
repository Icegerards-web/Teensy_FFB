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

struct Calibration {
  uint32_t magic;
  uint16_t version;
  int32_t steeringCenter;   // sensor's center value (see PositionSensor)
  float steeringRangeDeg;   // lock-to-lock rotation the game sees
  PedalCal throttle;
  PedalCal brake;
  PedalCal clutch;
};

namespace calibration {

constexpr uint32_t MAGIC = 0x46464257;  // the letters "FFBW" as a number
constexpr uint16_t VERSION = 1;
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
  return c;
}

// Returns true if valid saved data was found; otherwise `out` gets defaults.
inline bool load(Calibration& out) {
  EEPROM.get(EEPROM_ADDRESS, out);
  if (out.magic != MAGIC || out.version != VERSION) {
    out = defaults();
    return false;
  }
  return true;
}

inline void save(const Calibration& c) {
  // EEPROM.put only rewrites bytes that changed, which spares the flash.
  EEPROM.put(EEPROM_ADDRESS, c);
}

}  // namespace calibration
