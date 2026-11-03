// =============================================================================
//  MotorDriver.h  -  the "interface" for anything that can twist the wheel.
// =============================================================================
//
//  Same idea as PositionSensor.h: this lists what a motor driver must be able
//  to do, without saying how. Bts7960Driver.h is the version for the BTS7960
//  board and a brushed DC motor. If you move to a hoverboard (brushless) motor
//  with an ODrive or VESC later, you write e.g. an ODriveDriver class with
//  these same functions and change one line in teensy_ffb.ino.
//
//  Torque is a number from -1.0 to +1.0:
//    +1.0 = full power turning the wheel to the RIGHT
//    -1.0 = full power to the LEFT
//     0.0 = no force
// =============================================================================
#pragma once

class MotorDriver {
 public:
  virtual ~MotorDriver() {}

  // Set up pins. The motor must start switched OFF.
  virtual void begin() = 0;

  // Ask for a torque between -1 and +1. Values outside are clamped.
  virtual void setTorque(float torque) = 0;

  // Switch the driver off completely (no power to the motor at all).
  // Must be safe to call from an interrupt, because the safety timer uses it.
  virtual void emergencyStop() = 0;

  // The last torque that was actually applied (for the serial monitor).
  virtual float torque() const = 0;
};
