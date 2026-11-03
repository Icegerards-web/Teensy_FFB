// =============================================================================
//  Debouncer.h  -  cleans up the flicker of a mechanical button.
// =============================================================================
//
//  When a switch closes, its metal contacts bounce for a few milliseconds, so
//  the pin rapidly flips between pressed and released. Without filtering, one
//  paddle pull could register as several gear shifts.
//
//  The rule here: the "official" state only changes once the raw pin has kept
//  the new value for at least `debounceMs` milliseconds without flipping back.
//
//  The class takes the time as an argument instead of calling millis() itself,
//  which makes it easy to test on a PC with made-up times.
// =============================================================================
#pragma once
#include <stdint.h>

class Debouncer {
 public:
  explicit Debouncer(uint32_t debounceMs = 5) : debounceMs_(debounceMs) {}

  // Feed in the latest raw reading (true = pressed) and the current time.
  // Returns the debounced state.
  bool update(bool rawPressed, uint32_t nowMs) {
    if (rawPressed != lastRaw_) {
      // The pin changed: restart the "has it been steady?" timer.
      lastRaw_ = rawPressed;
      lastChangeMs_ = nowMs;
    } else if (rawPressed != stable_ && (nowMs - lastChangeMs_) >= debounceMs_) {
      // The pin has held a new value long enough: accept it.
      stable_ = rawPressed;
    }
    return stable_;
  }

  bool pressed() const { return stable_; }

 private:
  uint32_t debounceMs_;
  bool lastRaw_ = false;
  bool stable_ = false;
  uint32_t lastChangeMs_ = 0;
};
