// Unit tests for the pure logic (axis math + button debouncing).
// These run on your PC, not on the Teensy:
//
//     g++ -std=c++17 -I firmware/teensy_ffb test/test_logic.cpp -o test/build/test_logic
//     ./test/build/test_logic
//
// (or just run ./test/run_tests.sh)
#include <cstdio>
#include <cstdlib>

#include "AxisMath.h"
#include "Debouncer.h"

static int failures = 0;
#define CHECK_EQ(actual, expected)                                              \
  do {                                                                          \
    long a_ = (long)(actual), e_ = (long)(expected);                            \
    if (a_ != e_) {                                                             \
      std::printf("FAIL %s:%d  %s = %ld, expected %ld\n", __FILE__, __LINE__,   \
                  #actual, a_, e_);                                             \
      failures++;                                                               \
    }                                                                           \
  } while (0)
#define CHECK_NEAR(actual, expected, tol)                                       \
  do {                                                                          \
    double a_ = (actual), e_ = (expected);                                      \
    if (a_ < e_ - (tol) || a_ > e_ + (tol)) {                                   \
      std::printf("FAIL %s:%d  %s = %f, expected %f\n", __FILE__, __LINE__,      \
                  #actual, a_, e_);                                             \
      failures++;                                                               \
    }                                                                           \
  } while (0)

static void testSteering() {
  using namespace axis;
  // 270 deg range: center, both full locks, and beyond lock (clamped).
  CHECK_EQ(steeringToAxis(0, 270), 512);
  CHECK_EQ(steeringToAxis(-135, 270), 0);
  CHECK_EQ(steeringToAxis(135, 270), 1023);
  CHECK_EQ(steeringToAxis(-500, 270), 0);
  CHECK_EQ(steeringToAxis(500, 270), 1023);
  // Half way to right lock is three quarters of the axis.
  CHECK_EQ(steeringToAxis(67.5f, 270), 767);
  // Invalid range falls back to center.
  CHECK_EQ(steeringToAxis(10, 0), 512);
}

static void testPot() {
  using namespace axis;
  CHECK_NEAR(potRawToDegrees(2048, 2048, 4095, 270, false), 0.0, 1e-4);
  CHECK_NEAR(potRawToDegrees(4095, 0, 4095, 270, false), 270.0, 1e-3);
  CHECK_NEAR(potRawToDegrees(1048, 2048, 4095, 270, false), -65.934, 1e-2);
  CHECK_NEAR(potRawToDegrees(1048, 2048, 4095, 270, true), 65.934, 1e-2);
}

static void testPedal() {
  using namespace axis;
  // Calibrated 1000 (released) .. 3000 (pressed), 3 % dead zone each end.
  CHECK_EQ(pedalToAxis(1000, 1000, 3000, 0.03f, 0.03f, false), 0);
  CHECK_EQ(pedalToAxis(1050, 1000, 3000, 0.03f, 0.03f, false), 0);     // inside low dead zone
  CHECK_EQ(pedalToAxis(3000, 1000, 3000, 0.03f, 0.03f, false), 1023);
  CHECK_EQ(pedalToAxis(2950, 1000, 3000, 0.03f, 0.03f, false), 1023);  // inside high dead zone
  CHECK_NEAR(pedalToAxis(2000, 1000, 3000, 0.03f, 0.03f, false), 511.5, 0.5);  // half way
  CHECK_EQ(pedalToAxis(500, 1000, 3000, 0.03f, 0.03f, false), 0);      // below calibration
  CHECK_EQ(pedalToAxis(4000, 1000, 3000, 0.03f, 0.03f, false), 1023);  // above calibration
  // A pedal wired "backwards" (reading falls as you press) via swapped min/max...
  CHECK_EQ(pedalToAxis(3000, 3000, 1000, 0.0f, 0.0f, false), 0);
  CHECK_EQ(pedalToAxis(1000, 3000, 1000, 0.0f, 0.0f, false), 1023);
  // ...or via the invert flag.
  CHECK_EQ(pedalToAxis(1000, 1000, 3000, 0.0f, 0.0f, true), 1023);
  // Uncalibrated (min == max) never divides by zero.
  CHECK_EQ(pedalToAxis(1234, 2000, 2000, 0.03f, 0.03f, false), 0);
}

static void testDebouncer() {
  Debouncer d(5);
  CHECK_EQ(d.update(false, 0), false);
  // Bouncy press: flickers for a few ms, then settles pressed at t=12.
  CHECK_EQ(d.update(true, 10), false);
  CHECK_EQ(d.update(false, 11), false);
  CHECK_EQ(d.update(true, 12), false);
  CHECK_EQ(d.update(true, 16), false);  // only 4 ms steady
  CHECK_EQ(d.update(true, 17), true);   // 5 ms steady -> pressed
  CHECK_EQ(d.update(true, 100), true);
  // A single 1 ms glitch while held does not release it.
  CHECK_EQ(d.update(false, 101), true);
  CHECK_EQ(d.update(true, 102), true);
  CHECK_EQ(d.update(true, 200), true);
  // Real release.
  CHECK_EQ(d.update(false, 300), true);
  CHECK_EQ(d.update(false, 305), false);
  // Works across the millis() wrap-around (~49 days of uptime).
  Debouncer w(5);
  w.update(false, 0xFFFFFFF0u);
  w.update(true, 0xFFFFFFFEu);
  CHECK_EQ(w.update(true, 3), true);
}

int main() {
  testSteering();
  testPot();
  testPedal();
  testDebouncer();
  if (failures) {
    std::printf("%d check(s) failed\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("All tests passed\n");
  return EXIT_SUCCESS;
}
