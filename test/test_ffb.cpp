// Unit tests for FfbEngine: plays the same sequence of USB reports that
// Windows' force-feedback driver sends, and checks the resulting torque.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "FfbEngine.h"

using namespace ffb;

static int failures = 0;
#define CHECK(cond)                                                     \
  do {                                                                  \
    if (!(cond)) {                                                      \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
      failures++;                                                       \
    }                                                                   \
  } while (0)
#define CHECK_NEAR(actual, expected, tol)                                         \
  do {                                                                            \
    double a_ = (actual), e_ = (expected);                                        \
    if (std::fabs(a_ - e_) > (tol)) {                                             \
      std::printf("FAIL %s:%d  %s = %f, expected %f\n", __FILE__, __LINE__,      \
                  #actual, a_, e_);                                               \
      failures++;                                                                 \
    }                                                                             \
  } while (0)

using Bytes = std::vector<uint8_t>;

static void put16(Bytes& b, int v) {
  b.push_back(v & 0xFF);
  b.push_back((v >> 8) & 0xFF);
}

static void out(Engine& e, const Bytes& b, uint32_t now = 0) {
  e.onSetReport(report::TYPE_OUTPUT, b.data(), b.size(), now);
}

// Create New Effect + read Block Load, like Windows does. Returns the index.
static uint8_t create(Engine& e, uint8_t type) {
  Bytes b = {report::CREATE_NEW_EFFECT, type};
  put16(b, 0);
  CHECK(b.size() == ffb_size::CREATE_NEW_EFFECT);
  e.onSetReport(report::TYPE_FEATURE, b.data(), b.size(), 0);
  uint8_t buf[64];
  uint32_t n = e.onGetReport(report::TYPE_FEATURE, report::BLOCK_LOAD, buf, sizeof(buf));
  CHECK(n == ffb_size::BLOCK_LOAD);
  CHECK(buf[0] == report::BLOCK_LOAD);
  return buf[2] == 1 ? buf[1] : 0;
}

static void setEffect(Engine& e, uint8_t idx, uint8_t type, uint16_t duration, uint8_t gain,
                      uint8_t axes, uint16_t direction) {
  Bytes b = {report::SET_EFFECT, idx, type};
  put16(b, duration);
  put16(b, 0);  // trigger repeat
  put16(b, 0);  // sample period
  put16(b, 0);  // start delay
  b.push_back(gain);
  b.push_back(1);  // trigger button
  b.push_back(axes);
  put16(b, direction);
  put16(b, 0);
  CHECK(b.size() == ffb_size::SET_EFFECT);
  out(e, b);
}

static void start(Engine& e, uint8_t idx, uint32_t now = 0, uint8_t op = 1, uint8_t loops = 1) {
  out(e, {report::EFFECT_OPERATION, idx, op, loops}, now);
}

static void constant(Engine& e, uint8_t idx, int mag) {
  Bytes b = {report::SET_CONSTANT, idx};
  put16(b, mag);
  CHECK(b.size() == ffb_size::SET_CONSTANT);
  out(e, b);
}

static void condition(Engine& e, uint8_t idx, int cp, int pos, int neg, int sat, int db) {
  Bytes b = {report::SET_CONDITION, idx, 0};
  put16(b, cp);
  put16(b, pos);
  put16(b, neg);
  put16(b, sat);
  put16(b, sat);
  put16(b, db);
  CHECK(b.size() == ffb_size::SET_CONDITION);
  out(e, b);
}

static void testPool() {
  Engine e;
  uint8_t buf[64];
  uint32_t n = e.onGetReport(report::TYPE_FEATURE, report::POOL, buf, sizeof(buf));
  CHECK(n == ffb_size::POOL);
  CHECK(buf[3] == ffb_size::MAX_EFFECTS);
  CHECK(buf[4] & 1);  // device managed pool
  // Fill every slot, then one more must report "full".
  for (unsigned i = 1; i <= ffb_size::MAX_EFFECTS; i++) CHECK(create(e, ET_CONSTANT) == i);
  CHECK(create(e, ET_CONSTANT) == 0);
  e.onGetReport(report::TYPE_FEATURE, report::BLOCK_LOAD, buf, sizeof(buf));
  CHECK(buf[2] == 2);  // full
  // Free slot 7 and it gets reused.
  out(e, {report::BLOCK_FREE, 7});
  CHECK(create(e, ET_SINE) == 7);
  // Invalid type -> error.
  CHECK(create(e, 99) == 0);
}

static void testConstant() {
  Engine e;
  WheelState w;
  uint8_t c = create(e, ET_CONSTANT);
  CHECK(c == 1);
  setEffect(e, c, ET_CONSTANT, 0xFFFF, 255, 0x01, 0);  // X axis, no direction
  constant(e, c, 5000);
  CHECK_NEAR(e.computeForce(w, 10), 0, 1e-6);  // not started yet
  start(e, c);
  CHECK_NEAR(e.computeForce(w, 10), 0.5, 1e-4);
  constant(e, c, -10000);
  CHECK_NEAR(e.computeForce(w, 11), -1.0, 1e-4);
  // Device gain halves everything.
  out(e, {report::DEVICE_GAIN, 128});
  CHECK_NEAR(e.computeForce(w, 12), -128.0 / 255, 1e-3);
  out(e, {report::DEVICE_GAIN, 255});
  // Effect gain.
  setEffect(e, c, ET_CONSTANT, 0xFFFF, 51, 0x01, 0);
  CHECK_NEAR(e.computeForce(w, 13), -0.2, 1e-3);
  setEffect(e, c, ET_CONSTANT, 0xFFFF, 255, 0x01, 0);
  // Polar direction: 90 deg = full +X, 270 deg = full -X, 0 deg = nothing on X.
  constant(e, c, 4000);
  setEffect(e, c, ET_CONSTANT, 0xFFFF, 255, 0x04, 9000);
  CHECK_NEAR(e.computeForce(w, 14), 0.4, 1e-3);
  setEffect(e, c, ET_CONSTANT, 0xFFFF, 255, 0x04, 27000);
  CHECK_NEAR(e.computeForce(w, 15), -0.4, 1e-3);
  setEffect(e, c, ET_CONSTANT, 0xFFFF, 255, 0x04, 0);
  CHECK_NEAR(e.computeForce(w, 16), 0, 1e-3);
  // Invert setting flips game-direction forces.
  setEffect(e, c, ET_CONSTANT, 0xFFFF, 255, 0x01, 0);
  e.settings.invertDirectional = true;
  CHECK_NEAR(e.computeForce(w, 17), -0.4, 1e-3);
  e.settings.invertDirectional = false;
  // Stop, pause, disable actuators.
  out(e, {report::DEVICE_CONTROL, 5});  // pause
  CHECK_NEAR(e.computeForce(w, 18), 0, 1e-6);
  out(e, {report::DEVICE_CONTROL, 6});  // continue
  CHECK_NEAR(e.computeForce(w, 19), 0.4, 1e-3);
  out(e, {report::DEVICE_CONTROL, 2});  // disable actuators
  CHECK_NEAR(e.computeForce(w, 20), 0, 1e-6);
  out(e, {report::DEVICE_CONTROL, 1});  // enable actuators
  start(e, c, 0, 3);                    // stop
  CHECK_NEAR(e.computeForce(w, 21), 0, 1e-6);
  CHECK(e.playingCount() == 0);
  // Reset frees everything.
  out(e, {report::DEVICE_CONTROL, 4});
  CHECK(e.allocatedCount() == 0);
}

static void testDurationAndLoops() {
  Engine e;
  WheelState w;
  uint8_t c = create(e, ET_CONSTANT);
  setEffect(e, c, ET_CONSTANT, 100, 255, 0x01, 0);
  constant(e, c, 10000);
  start(e, c, 1000);
  CHECK_NEAR(e.computeForce(w, 1050), 1.0, 1e-4);
  CHECK_NEAR(e.computeForce(w, 1100), 0, 1e-6);  // finished after 100 ms
  CHECK(e.playingCount() == 0);
  // Two loops: still playing at 150 ms, done at 200 ms.
  start(e, c, 2000, 1, 2);
  CHECK_NEAR(e.computeForce(w, 2150), 1.0, 1e-4);
  CHECK_NEAR(e.computeForce(w, 2200), 0, 1e-6);
}

static void testEnvelope() {
  Engine e;
  WheelState w;
  uint8_t c = create(e, ET_CONSTANT);
  setEffect(e, c, ET_CONSTANT, 1000, 255, 0x01, 0);
  constant(e, c, 10000);
  Bytes b = {report::SET_ENVELOPE, c};
  put16(b, 0);     // attack level
  put16(b, 2000);  // fade level
  put16(b, 100);   // attack time
  put16(b, 200);   // fade time
  out(e, b);
  start(e, c, 0);
  CHECK_NEAR(e.computeForce(w, 50), 0.5, 1e-3);   // half way through the attack
  CHECK_NEAR(e.computeForce(w, 500), 1.0, 1e-3);  // full strength
  CHECK_NEAR(e.computeForce(w, 900), 0.6, 1e-3);  // half way through the fade (1.0 -> 0.2)
}

static void testSpring() {
  Engine e;
  WheelState w;
  uint8_t s = create(e, ET_SPRING);
  setEffect(e, s, ET_SPRING, 0xFFFF, 255, 0x01, 0);
  condition(e, s, 0, 10000, 10000, 10000, 0);
  start(e, s);
  w.position = 0;
  CHECK_NEAR(e.computeForce(w, 1), 0, 1e-6);
  w.position = 0.5f;  // half way to right lock -> pulled left
  CHECK_NEAR(e.computeForce(w, 2), -0.5, 1e-3);
  w.position = -0.25f;
  CHECK_NEAR(e.computeForce(w, 3), 0.25, 1e-3);
  // Saturation caps it, dead band ignores small movements.
  condition(e, s, 0, 10000, 10000, 3000, 1000);
  w.position = 0.05f;  // inside the 10 % dead band
  CHECK_NEAR(e.computeForce(w, 4), 0, 1e-6);
  w.position = 0.9f;
  CHECK_NEAR(e.computeForce(w, 5), -0.3, 1e-3);
  // Center point offset: spring centered at +20 %.
  condition(e, s, 2000, 10000, 10000, 10000, 0);
  w.position = 0.2f;
  CHECK_NEAR(e.computeForce(w, 6), 0, 1e-4);
  // The invert setting must NOT flip a spring (it would push the wheel away).
  e.settings.invertDirectional = true;
  w.position = 0.7f;
  CHECK_NEAR(e.computeForce(w, 7), -0.5, 1e-3);
  // Y-axis condition blocks are ignored.
  Bytes y = {report::SET_CONDITION, s, 1};
  for (int i = 0; i < 6; i++) put16(y, 0);
  out(e, y);
  CHECK_NEAR(e.computeForce(w, 8), -0.5, 1e-3);
}

static void testDamperFriction() {
  Engine e;
  WheelState w;
  uint8_t d = create(e, ET_DAMPER);
  setEffect(e, d, ET_DAMPER, 0xFFFF, 255, 0x01, 0);
  condition(e, d, 0, 5000, 5000, 10000, 0);
  start(e, d);
  w.velocity = 0.4f;  // turning right -> damper pushes left, proportional to speed
  CHECK_NEAR(e.computeForce(w, 1), -0.2, 1e-3);
  w.velocity = -0.4f;
  CHECK_NEAR(e.computeForce(w, 2), 0.2, 1e-3);
  start(e, d, 0, 3);

  uint8_t f = create(e, ET_FRICTION);
  setEffect(e, f, ET_FRICTION, 0xFFFF, 255, 0x01, 0);
  condition(e, f, 3000, 3000, 3000, 10000, 500);  // offset/dead band are ignored
  start(e, f);
  w.velocity = 0.5f;   // any real speed -> full friction against it
  CHECK_NEAR(e.computeForce(w, 3), -0.3, 1e-3);
  w.velocity = -0.1f;
  CHECK_NEAR(e.computeForce(w, 4), 0.3, 1e-3);
  w.velocity = 0;
  CHECK_NEAR(e.computeForce(w, 5), 0, 1e-6);
}

static void testPeriodic() {
  Engine e;
  WheelState w;
  uint8_t p = create(e, ET_SINE);
  setEffect(e, p, ET_SINE, 0xFFFF, 255, 0x01, 0);
  Bytes b = {report::SET_PERIODIC, p};
  put16(b, 8000);  // magnitude
  put16(b, 0);     // offset
  put16(b, 0);     // phase
  put16(b, 100);   // period 100 ms
  CHECK(b.size() == ffb_size::SET_PERIODIC);
  out(e, b);
  start(e, p, 0);
  CHECK_NEAR(e.computeForce(w, 0), 0, 1e-3);
  CHECK_NEAR(e.computeForce(w, 25), 0.8, 1e-3);   // quarter period: peak
  CHECK_NEAR(e.computeForce(w, 75), -0.8, 1e-3);  // three quarters: trough
  // Square wave with an offset.
  uint8_t q = create(e, ET_SQUARE);
  setEffect(e, q, ET_SQUARE, 0xFFFF, 255, 0x01, 0);
  Bytes sq = {report::SET_PERIODIC, q};
  put16(sq, 2000);
  put16(sq, 1000);
  put16(sq, 0);
  put16(sq, 50);
  out(e, sq);
  start(e, p, 0, 3);  // stop the sine
  start(e, q, 0);
  CHECK_NEAR(e.computeForce(w, 10), 0.3, 1e-3);   // offset + magnitude
  CHECK_NEAR(e.computeForce(w, 30), -0.1, 1e-3);  // offset - magnitude
  // Start solo stops the others.
  start(e, p, 0, 1);
  start(e, q, 0, 2);
  CHECK(e.playingCount() == 1);
}

static void testRampAndSum() {
  Engine e;
  WheelState w;
  uint8_t r = create(e, ET_RAMP);
  setEffect(e, r, ET_RAMP, 1000, 255, 0x01, 0);
  Bytes b = {report::SET_RAMP, r};
  put16(b, -10000);
  put16(b, 10000);
  out(e, b);
  start(e, r, 0);
  CHECK_NEAR(e.computeForce(w, 250), -0.5, 1e-3);
  CHECK_NEAR(e.computeForce(w, 500), 0.0, 1e-3);
  // Two effects add up, and the total is clamped to +-1.
  uint8_t c = create(e, ET_CONSTANT);
  setEffect(e, c, ET_CONSTANT, 0xFFFF, 255, 0x01, 0);
  constant(e, c, 10000);
  start(e, c, 0);
  CHECK_NEAR(e.computeForce(w, 750), 1.0, 1e-3);  // 0.5 + 1.0 clamped
}

static void testBadReports() {
  Engine e;
  WheelState w;
  // Reports for unknown blocks, short reports and unknown IDs are ignored.
  out(e, {report::SET_CONSTANT, 5, 0x10, 0x27});
  out(e, {report::SET_EFFECT, 1});
  out(e, {99, 1, 2, 3});
  uint8_t c = create(e, ET_CONSTANT);
  out(e, {report::SET_CONSTANT, c});  // too short
  start(e, c);
  CHECK_NEAR(e.computeForce(w, 1), 0, 1e-6);
  uint8_t buf[2];
  CHECK(e.onGetReport(report::TYPE_FEATURE, report::BLOCK_LOAD, buf, sizeof(buf)) == 0);
}

int main() {
  testPool();
  testConstant();
  testDurationAndLoops();
  testEnvelope();
  testSpring();
  testDamperFriction();
  testPeriodic();
  testRampAndSum();
  testBadReports();
  if (failures) {
    std::printf("%d FFB check(s) failed\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("All FFB tests passed\n");
  return EXIT_SUCCESS;
}
