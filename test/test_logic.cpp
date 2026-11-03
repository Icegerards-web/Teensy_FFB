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
#include "ForceSafety.h"

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

static SafetyInputs okInputs(uint32_t now) {
  SafetyInputs in;
  in.armed = true;
  in.nowMs = now;
  return in;
}

static void testSafety() {
  ForceSafety s;
  s.settings.maxTorque = 0.3f;
  s.settings.slewPerMs = 1.0f;  // no slew limiting for most checks
  s.settings.halfRangeDeg = 135;
  s.settings.endstopStiffness = 0.1f;

  // Not armed -> always 0.
  SafetyInputs in = okInputs(1);
  in.armed = false;
  in.requestedTorque = 0.5f;
  CHECK_NEAR(s.process(in), 0, 1e-6);
  CHECK_EQ((int)s.state(), (int)SafetyState::NotArmed);

  // Armed: clamped to maxTorque.
  in = okInputs(2);
  in.requestedTorque = 0.9f;
  CHECK_NEAR(s.process(in), 0.3, 1e-6);
  in = okInputs(3);
  in.requestedTorque = -0.9f;
  CHECK_NEAR(s.process(in), -0.3, 1e-6);

  // Host lost -> 0 immediately, comes back by itself when host returns.
  in = okInputs(4);
  in.requestedTorque = 0.2f;
  in.hostAlive = false;
  CHECK_NEAR(s.process(in), 0, 1e-6);
  in.hostAlive = true;
  in.nowMs = 5;
  CHECK_NEAR(s.process(in), 0.2, 1e-6);

  // End stop: 2 degrees past right lock, game pushing right -> pushed back left.
  in = okInputs(6);
  in.angleDeg = 137;
  in.requestedTorque = 0.25f;
  CHECK_NEAR(s.process(in), -0.2, 1e-5);
  // Far past the end: spring is limited by maxTorque.
  in = okInputs(7);
  in.angleDeg = -200;
  CHECK_NEAR(s.process(in), 0.3, 1e-6);
  // Inside the range the end stop does nothing.
  in = okInputs(8);
  in.angleDeg = 100;
  in.requestedTorque = 0.1f;
  CHECK_NEAR(s.process(in), 0.1, 1e-6);

  // Sensor fault must persist for sensorFaultMs, then latches.
  ForceSafety f;
  f.settings.slewPerMs = 1.0f;
  f.settings.sensorFaultMs = 10;
  in = okInputs(100);
  in.requestedTorque = 0.1f;
  in.sensorHealthy = false;
  CHECK_NEAR(f.process(in), 0.1, 1e-6);  // one bad reading: still running
  in.nowMs = 110;
  CHECK_NEAR(f.process(in), 0, 1e-6);    // bad for 10 ms: fault
  CHECK_EQ((int)f.state(), (int)SafetyState::SensorFault);
  in.sensorHealthy = true;
  in.nowMs = 111;
  CHECK_NEAR(f.process(in), 0, 1e-6);    // still latched although sensor is fine
  f.clearFaults();
  in.nowMs = 112;
  CHECK_NEAR(f.process(in), 0.1, 1e-6);  // cleared by "motor on"

  // Loop stall latches too.
  in = okInputs(200);
  in.loopStalled = true;
  CHECK_NEAR(f.process(in), 0, 1e-6);
  in.loopStalled = false;
  in.nowMs = 201;
  CHECK_NEAR(f.process(in), 0, 1e-6);
  CHECK_EQ((int)f.state(), (int)SafetyState::StallFault);

  // Slew limit: 0.05 per ms -> 0.3 takes 6 ms to reach.
  ForceSafety r;
  r.settings.maxTorque = 1.0f;
  r.settings.slewPerMs = 0.05f;
  in = okInputs(1000);
  in.requestedTorque = 0.3f;
  CHECK_NEAR(r.process(in), 0.05, 1e-6);  // first sample counts as 1 ms
  in.nowMs = 1002;
  CHECK_NEAR(r.process(in), 0.15, 1e-6);
  in.nowMs = 1010;
  CHECK_NEAR(r.process(in), 0.3, 1e-6);
}

static void testVelocity() {
  axis::VelocityEstimator v(1.0f);  // no smoothing, to check the math
  v.update(0, 0.001f);
  CHECK_NEAR(v.update(0.1f, 0.001f), 100.0, 1e-2);  // 0.1 deg in 1 ms = 100 deg/s
  axis::VelocityEstimator smooth(0.1f);
  smooth.update(0, 0.001f);
  CHECK_NEAR(smooth.update(0.1f, 0.001f), 10.0, 1e-2);  // moves 10 % of the way
}

int main() {
  testSafety();
  testVelocity();
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
