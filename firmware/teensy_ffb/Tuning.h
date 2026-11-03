// =============================================================================
//  Tuning.h  -  the "feel" settings added on top of the game's forces.
// =============================================================================
//
//  These run all the time (when the motor is on), whatever the game does:
//
//   damping   A force against the wheel's speed, like stirring thick oil.
//             A little of it stops the wheel oscillating (wobbling left-right
//             on its own) when you let go, which DIY wheels with springy
//             gears or pot noise tend to do.
//   friction  A constant drag against the direction of motion, like dry
//             friction in a real steering rack. Makes the wheel feel heavier.
//   minTorque A brushed motor plus gears needs a certain torque before it
//             moves at all, so small forces (gentle road texture) get lost.
//             This lifts every non-zero force by that amount:
//               output = sign * (minTorque + (1 - minTorque) * |input|)
//             Set it just below the level where the wheel starts to move by
//             itself (try "test constant" with increasing values).
//
//  Order in the sketch: game forces * gain -> + damping/friction -> minTorque
//  -> ForceSafety (end stops, max torque, slew) -> motor.
//
//  No Arduino code here, so it is tested on a PC in test/test_logic.cpp.
// =============================================================================
#pragma once

struct TuningSettings {
  float damping = 0;    // 0..1, torque at full speed (see FFB_FULL_SPEED_DEG_S)
  float friction = 0;   // 0..1
  float minTorque = 0;  // 0..1
};

namespace tuning {

inline float clamp1(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }

// velocity: wheel speed as a fraction of FFB_FULL_SPEED_DEG_S (-1..1 or more).
inline float apply(float requested, float velocity, const TuningSettings& s) {
  float t = requested;
  t -= s.damping * clamp1(velocity);
  // Friction fades in over the first 2 % of full speed, so it doesn't buzz
  // back and forth when the wheel is standing still.
  t -= s.friction * clamp1(velocity * 50.0f);
  t = clamp1(t);
  // Below 0.5 % we treat the request as "no force", so minTorque doesn't
  // turn sensor noise into a constant buzz.
  const float zero = 0.005f;
  if (s.minTorque > 0 && (t > zero || t < -zero)) {
    float mag = t > 0 ? t : -t;
    mag = s.minTorque + (1.0f - s.minTorque) * mag;
    t = t > 0 ? mag : -mag;
  }
  return t;
}

}  // namespace tuning
