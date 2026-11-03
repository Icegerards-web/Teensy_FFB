// =============================================================================
//  PotPositionSensor.h  -  steering position from a potentiometer.
// =============================================================================
//
//  How it works:
//   A pot is a resistor with a sliding contact (the wiper). With 3.3 V across
//   the outer legs, the wiper voltage moves from 0 V to 3.3 V as you turn it.
//   The Teensy's ADC (analog-to-digital converter) turns that voltage into a
//   number: 0 at 0 V, 4095 at 3.3 V (12-bit). We remember the number you get
//   with the wheel straight ("center"), and every count away from it is a
//   fixed number of degrees: POT_TRAVEL_DEG / 4095.
//
//  This class is one implementation of the PositionSensor interface.
// =============================================================================
#pragma once
#include <Arduino.h>

#include "AxisMath.h"
#include "PositionSensor.h"
#include "config.h"

class PotPositionSensor : public PositionSensor {
 public:
  PotPositionSensor(int pin, float travelDeg, bool invert)
      : pin_(pin), travelDeg_(travelDeg), invert_(invert) {}

  void begin() override {
    pinMode(pin_, INPUT);  // plain analog input, no pull-up (it would skew the reading)
    raw_ = analogRead(pin_);
  }

  void update() override { raw_ = analogRead(pin_); }

  float angleDeg() const override {
    return axis::potRawToDegrees(raw_, center_, ADC_MAX, travelDeg_, invert_);
  }

  void setCenterHere() override { center_ = raw_; }

  long centerCalibration() const override { return center_; }
  void setCenterCalibration(long value) override { center_ = value; }

  bool healthy() const override {
    // Sitting hard against 0 or 4095 usually means a loose wire, not a real
    // position (see POT_FAULT_MARGIN in config.h).
    return raw_ > POT_FAULT_MARGIN && raw_ < ADC_MAX - POT_FAULT_MARGIN;
  }

  long rawValue() const override { return raw_; }

 private:
  int pin_;
  float travelDeg_;
  bool invert_;
  int32_t raw_ = 0;
  int32_t center_ = ADC_MAX / 2;  // until calibrated, assume the pot's middle
};
