// =============================================================================
//  AxisMath.h  -  the pure math that turns raw sensor numbers into the numbers
//  the game sees.
// =============================================================================
//
//  None of this touches Arduino pins, so it can be unit-tested on a normal PC
//  (see test/test_logic.cpp). Keeping the math separate from the hardware is
//  also what lets us swap the pot for an encoder later without touching it.
//
//  How a game controller "axis" works:
//   A joystick axis is just a number inside a fixed range. With the Teensy's
//   built-in Joystick that range is 0..1023, where 512 is the middle; with
//   our force-feedback USB type it is -32767..32767 for steering and
//   0..32767 for pedals.
//   Windows doesn't know or care that it is a steering wheel; the game maps
//   0 = full left, 1023 = full right. For pedals 0 = released, 1023 = floored.
// =============================================================================
#pragma once
#include <stdint.h>

namespace axis {

constexpr int32_t OUT_MIN = 0;
constexpr int32_t OUT_MAX = 1023;
constexpr int32_t OUT_CENTER = 512;

inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Turn a fraction 0.0 .. 1.0 into an axis value 0 .. 1023 (rounded).
inline int32_t fractionToAxis(float f) {
  f = clampf(f, 0.0f, 1.0f);
  return (int32_t)(f * (OUT_MAX - OUT_MIN) + 0.5f) + OUT_MIN;
}

// Convert a raw pot reading into an angle in degrees, 0 = wheel centered,
// negative = left, positive = right.
//   raw        : ADC reading, e.g. 0..4095
//   centerRaw  : the ADC reading when the wheel is straight (from calibration)
//   adcMax     : 4095 for a 12-bit ADC
//   travelDeg  : how many degrees the full pot sweep 0..adcMax represents
inline float potRawToDegrees(int32_t raw, int32_t centerRaw, int32_t adcMax,
                             float travelDeg, bool invert) {
  float deg = (float)(raw - centerRaw) * travelDeg / (float)adcMax;
  return invert ? -deg : deg;
}

// Steering angle as a fraction of full lock: -1 = full left, 0 = center,
// +1 = full right. rangeDeg is the total lock-to-lock rotation, e.g. 270,
// so full lock is +-135 deg. Turning further just stays at -1 / +1.
inline float steeringFraction(float angleDeg, float rangeDeg) {
  if (rangeDeg <= 0.0f) return 0.0f;
  return clampf(angleDeg / (rangeDeg * 0.5f), -1.0f, 1.0f);
}

// Convert a steering angle into the 10-bit steering axis value:
//   -135 deg -> 0 (full left), 0 deg -> ~512 (center), +135 deg -> 1023.
inline int32_t steeringToAxis(float angleDeg, float rangeDeg) {
  if (rangeDeg <= 0.0f) return OUT_CENTER;
  return fractionToAxis(steeringFraction(angleDeg, rangeDeg) * 0.5f + 0.5f);
}

// Same, as the 16-bit signed value the force-feedback USB type sends
// (-32767..32767). More steps = smoother steering with a fine sensor.
inline int16_t steeringToAxis16(float angleDeg, float rangeDeg) {
  float f = steeringFraction(angleDeg, rangeDeg) * 32767.0f;
  return (int16_t)(f >= 0 ? f + 0.5f : f - 0.5f);
}

// Pedal reading as a fraction 0.0 (released) .. 1.0 (fully pressed).
//   rawMin / rawMax : readings at "released" and "fully pressed" (calibration).
//                     They may be either way round; the pedal works both ways.
//   deadLow/deadHigh: fractions of travel ignored at each end (e.g. 0.03).
inline float pedalFraction(int32_t raw, int32_t rawMin, int32_t rawMax,
                           float deadLow, float deadHigh, bool invert) {
  if (rawMax == rawMin) return 0.0f;  // not calibrated: avoid divide by 0
  float f = (float)(raw - rawMin) / (float)(rawMax - rawMin);  // 0..1
  if (invert) f = 1.0f - f;
  // Stretch the part between the dead zones back out to the full 0..1.
  float usable = 1.0f - deadLow - deadHigh;
  if (usable <= 0.0f) return f >= 0.5f ? 1.0f : 0.0f;
  return clampf((f - deadLow) / usable, 0.0f, 1.0f);
}

// Pedal as a 10-bit axis value 0..1023.
inline int32_t pedalToAxis(int32_t raw, int32_t rawMin, int32_t rawMax,
                           float deadLow, float deadHigh, bool invert) {
  return fractionToAxis(pedalFraction(raw, rawMin, rawMax, deadLow, deadHigh, invert));
}

// Pedal as the 0..32767 value the force-feedback USB type sends.
inline uint16_t pedalToAxis16(int32_t raw, int32_t rawMin, int32_t rawMax,
                              float deadLow, float deadHigh, bool invert) {
  return (uint16_t)(pedalFraction(raw, rawMin, rawMax, deadLow, deadHigh, invert) * 32767.0f + 0.5f);
}

// Estimates how fast the wheel is turning (degrees per second) from two
// angle readings in a row. The raw difference is noisy, so it is smoothed
// with an "exponential moving average": each new value only moves the
// estimate part of the way (`smoothing` = 0.1 means 10 % of the way).
// Used by the damper and friction effects, which depend on wheel speed.
class VelocityEstimator {
 public:
  explicit VelocityEstimator(float smoothing = 0.1f) : k_(smoothing) {}
  float update(float angleDeg, float dtSeconds) {
    if (!init_ || dtSeconds <= 0.0f) {
      init_ = true;
      last_ = angleDeg;
      return vel_;
    }
    float raw = (angleDeg - last_) / dtSeconds;
    last_ = angleDeg;
    vel_ += k_ * (raw - vel_);
    return vel_;
  }
  float value() const { return vel_; }

 private:
  float k_;
  bool init_ = false;
  float last_ = 0;
  float vel_ = 0;
};

}  // namespace axis
