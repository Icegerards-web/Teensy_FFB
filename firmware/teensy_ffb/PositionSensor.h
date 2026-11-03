// =============================================================================
//  PositionSensor.h  -  the "interface" for anything that measures where the
//  steering wheel is pointing.
// =============================================================================
//
//  What an interface is (in C++ terms):
//   This class only lists the functions a steering sensor must have. It
//   contains no code for any specific sensor. A potentiometer, a quadrature
//   encoder or a magnetic AS5600 sensor can each be written as a separate class
//   that "inherits" from PositionSensor and fills in these functions.
//
//   The rest of the firmware only ever talks to a PositionSensor, so swapping
//   the pot for an encoder later means writing one new class and changing one
//   line in teensy_ffb.ino. The steering math, USB code and (in step 2) the
//   motor safety code stay the same.
//
//   "virtual ... = 0" means: "every sensor class MUST provide this function".
// =============================================================================
#pragma once

class PositionSensor {
 public:
  virtual ~PositionSensor() {}

  // Set up pins / hardware. Called once from setup().
  virtual void begin() = 0;

  // Take a fresh measurement. Called once per loop, before angleDeg().
  virtual void update() = 0;

  // Wheel angle in degrees: 0 = straight, negative = left, positive = right.
  virtual float angleDeg() const = 0;

  // Tell the sensor "where the wheel is right now is straight ahead".
  virtual void setCenterHere() = 0;

  // The number the sensor uses to remember where "center" is (for a pot: the
  // ADC reading at center). The main sketch saves it to EEPROM and hands it
  // back after power-up, without needing to know what kind of sensor this is.
  virtual long centerCalibration() const = 0;
  virtual void setCenterCalibration(long value) = 0;

  // false if the sensor looks broken (e.g. disconnected wire). Step 2 will use
  // this to switch the motor off, because forces based on a wrong position
  // could be dangerous.
  virtual bool healthy() const = 0;

  // A raw number for the serial monitor, to help with wiring and calibration.
  virtual long rawValue() const = 0;
};
