// =============================================================================
//  Bts7960Driver.h  -  drives a brushed DC motor through a BTS7960 board.
// =============================================================================
//
//  How the BTS7960 works:
//   It is an "H-bridge": two power switches per motor wire, so it can connect
//   each motor wire to either + or - of the supply. That lets current flow
//   through the motor in either direction (= either rotation direction).
//    - RPWM pulsing, LPWM low  -> motor turns one way
//    - LPWM pulsing, RPWM low  -> motor turns the other way
//    - R_EN / L_EN low         -> everything off, motor spins freely
//
//  PWM (pulse width modulation):
//   The pin switches on and off 20,000 times per second. The fraction of time
//   it is on (the "duty cycle") sets the average voltage, and for a DC motor
//   torque is roughly proportional to that (at low speed). 50 % duty is about
//   half the torque.
// =============================================================================
#pragma once
#include <Arduino.h>

#include "MotorDriver.h"
#include "config.h"

class Bts7960Driver : public MotorDriver {
 public:
  Bts7960Driver(int rpwmPin, int lpwmPin, int enablePin, bool invert)
      : rpwm_(rpwmPin), lpwm_(lpwmPin), en_(enablePin), invert_(invert) {}

  void begin() override {
    pinMode(en_, OUTPUT);
    digitalWriteFast(en_, LOW);  // off first, before anything else
    pinMode(rpwm_, OUTPUT);
    pinMode(lpwm_, OUTPUT);
    analogWriteResolution(MOTOR_PWM_BITS);
    analogWriteFrequency(rpwm_, MOTOR_PWM_FREQUENCY_HZ);
    analogWriteFrequency(lpwm_, MOTOR_PWM_FREQUENCY_HZ);
    analogWrite(rpwm_, 0);
    analogWrite(lpwm_, 0);
    torque_ = 0;
  }

  void setTorque(float t) override {
    if (t > 1.0f) t = 1.0f;
    if (t < -1.0f) t = -1.0f;
    torque_ = t;
    if (invert_) t = -t;
    const int maxDuty = (1 << MOTOR_PWM_BITS) - 1;
    int duty = (int)((t >= 0 ? t : -t) * maxDuty + 0.5f);
    if (duty == 0) {
      analogWrite(rpwm_, 0);
      analogWrite(lpwm_, 0);
      digitalWriteFast(en_, LOW);
      return;
    }
    // Always set the "off" side first so both sides are never on together.
    if (t > 0) {
      analogWrite(lpwm_, 0);
      analogWrite(rpwm_, duty);
    } else {
      analogWrite(rpwm_, 0);
      analogWrite(lpwm_, duty);
    }
    digitalWriteFast(en_, HIGH);
  }

  void emergencyStop() override {
    // Pulling enable low is enough to cut the power stage; it is a single
    // register write, so it's safe inside an interrupt.
    digitalWriteFast(en_, LOW);
    torque_ = 0;
  }

  float torque() const override { return torque_; }

 private:
  int rpwm_, lpwm_, en_;
  bool invert_;
  volatile float torque_ = 0;
};
