// =============================================================================
//  ForceSafety.h  -  every torque request passes through here before it can
//  reach the motor.
// =============================================================================
//
//  A force-feedback motor is strong enough to hurt a thumb or wrist, so the
//  rule is: nothing talks to the motor directly. Games (step 3), test commands
//  and tuning effects only *request* a torque, and this class decides what the
//  motor actually gets. In order:
//
//   1. Faults.  If the motor is not armed, the sensor looks broken, the PC has
//      stopped talking to us, the e-stop is pressed or the loop stalled:
//      torque is 0, full stop. Sensor and stall faults "latch": they stay
//      active until you type "motor on", so a flaky wire can't make the motor
//      jump back to life on its own.
//   2. End stops. Past the steering range, any force pushing the wheel
//      further out is removed, and a stiff spring pushes it back in. This is
//      what stops the wheel at "full lock" like a real car.
//   3. Torque limit. The result is clamped to +-maxTorque.
//   4. Slew limit. The torque may only change by a small step per
//      millisecond, so a sudden full-power request becomes a fast but smooth
//      ramp instead of a violent jerk.
//
//  This file has no Arduino code, so test/test_logic.cpp can test it on a PC.
// =============================================================================
#pragma once
#include <stdint.h>

struct SafetySettings {
  float maxTorque = 0.3f;           // 0..1
  float endstopStiffness = 0.1f;    // torque per degree past the end
  float halfRangeDeg = 135.0f;      // end stop sits at +-this angle
  float slewPerMs = 0.05f;          // max torque change per millisecond
  uint32_t sensorFaultMs = 10;      // how long the sensor must look bad
};

struct SafetyInputs {
  float requestedTorque = 0;  // what effects/games want, -1..1
  float angleDeg = 0;         // wheel angle, 0 = center
  bool armed = false;         // user switched the motor on
  bool sensorHealthy = true;
  bool hostAlive = true;      // PC is still reading our USB reports
  bool estopPressed = false;
  bool loopStalled = false;   // set by the stall-guard interrupt
  uint32_t nowMs = 0;
};

enum class SafetyState : uint8_t {
  Ok,
  NotArmed,
  SensorFault,  // latched
  StallFault,   // latched
  HostLost,
  EStop,
};

inline const char* safetyStateName(SafetyState s) {
  switch (s) {
    case SafetyState::Ok: return "ok";
    case SafetyState::NotArmed: return "motor off";
    case SafetyState::SensorFault: return "SENSOR FAULT (type 'motor on' to retry)";
    case SafetyState::StallFault: return "LOOP STALL FAULT (type 'motor on' to retry)";
    case SafetyState::HostLost: return "PC not connected";
    case SafetyState::EStop: return "E-STOP pressed";
  }
  return "?";
}

class ForceSafety {
 public:
  SafetySettings settings;

  // Clear latched faults. Called when the user types "motor on".
  void clearFaults() {
    sensorFault_ = false;
    stallFault_ = false;
    sensorBadSinceValid_ = false;
    output_ = 0;
  }

  // Returns the torque the motor should get right now.
  float process(const SafetyInputs& in) {
    // --- 1. Faults ------------------------------------------------------------
    if (!in.sensorHealthy) {
      if (!sensorBadSinceValid_) {
        sensorBadSinceValid_ = true;
        sensorBadSinceMs_ = in.nowMs;
      } else if (in.nowMs - sensorBadSinceMs_ >= settings.sensorFaultMs) {
        sensorFault_ = true;
      }
    } else {
      sensorBadSinceValid_ = false;
    }
    if (in.loopStalled) stallFault_ = true;

    if (stallFault_) return stop(SafetyState::StallFault);
    if (sensorFault_) return stop(SafetyState::SensorFault);
    if (in.estopPressed) return stop(SafetyState::EStop);
    if (!in.armed) return stop(SafetyState::NotArmed);
    if (!in.hostAlive) return stop(SafetyState::HostLost);
    state_ = SafetyState::Ok;

    // --- 2. End stops -----------------------------------------------------------
    float t = in.requestedTorque;
    float past = 0;  // degrees past the end stop (always >= 0)
    if (in.angleDeg > settings.halfRangeDeg) {
      past = in.angleDeg - settings.halfRangeDeg;
      if (t > 0) t = 0;                         // no pushing further right
      t -= past * settings.endstopStiffness;    // spring back to the left
    } else if (in.angleDeg < -settings.halfRangeDeg) {
      past = -settings.halfRangeDeg - in.angleDeg;
      if (t < 0) t = 0;                         // no pushing further left
      t += past * settings.endstopStiffness;    // spring back to the right
    }

    // --- 3. Torque limit --------------------------------------------------------
    t = clamp(t, -settings.maxTorque, settings.maxTorque);

    // --- 4. Slew limit ----------------------------------------------------------
    uint32_t dt = haveLast_ ? in.nowMs - lastMs_ : 1;
    if (dt == 0) dt = 1;
    if (dt > 50) dt = 50;
    lastMs_ = in.nowMs;
    haveLast_ = true;
    float maxStep = settings.slewPerMs * (float)dt;
    output_ = clamp(t, output_ - maxStep, output_ + maxStep);
    return output_;
  }

  SafetyState state() const { return state_; }
  float output() const { return output_; }

 private:
  static float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  float stop(SafetyState s) {
    state_ = s;
    output_ = 0;  // stop is immediate: no slew-limiting on the way down
    lastMs_ = 0;
    haveLast_ = false;
    return 0;
  }

  SafetyState state_ = SafetyState::NotArmed;
  float output_ = 0;
  bool sensorFault_ = false;
  bool stallFault_ = false;
  bool sensorBadSinceValid_ = false;
  uint32_t sensorBadSinceMs_ = 0;
  uint32_t lastMs_ = 0;
  bool haveLast_ = false;
};
