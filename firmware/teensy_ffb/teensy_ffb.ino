// =============================================================================
//  teensy_ffb.ino  -  DIY force-feedback steering wheel on a Teensy 4.0
// =============================================================================
//
//  What this sketch does, 1000 times per second:
//    1. read the steering sensor, the pedal pots and the buttons
//    2. turn those readings into joystick numbers (see AxisMath.h)
//    3. send one "report" with all of them to the PC over USB
//    4. work out the motor torque and pass it through the safety layer
//       (ForceSafety.h) to the motor driver
//
//  How the PC knows we are a game controller (HID):
//   USB devices describe themselves when plugged in. A "HID" (Human Interface
//   Device) like a keyboard, mouse or joystick also sends a "report
//   descriptor": a small table that says "my reports contain these axes, this
//   many buttons, each this many bits". Windows reads it and shows the device
//   in "Set up USB game controllers" (run joy.cpl). Games read the same
//   information through DirectInput, which is why BeamNG, LFS and F1 can bind
//   our axes without any driver.
//
//   Two USB types work with this sketch (Arduino IDE: Tools > USB Type):
//    - "Serial + FFB Wheel" (needs tools/patch_teensy_core.py once): our own
//      descriptor with 16-bit axes PLUS the "PID" force-feedback section, so
//      games can send forces. This is the normal choice.
//    - "Serial + Keyboard + Mouse + Joystick": Teensyduino's stock joystick
//      (10-bit axes), no game forces. Handy for testing without the patch.
//
//  Force feedback (step 3): see FfbEngine.h for how game effects arrive and
//  are turned into one torque, and ForceSafety.h for what stands between that
//  torque and the motor.
//
//  Axis mapping (games let you re-bind these, so the names don't matter much):
//    X        = steering       Y       = throttle
//    Z        = brake          Z-rot   = clutch
//    Buttons  = in the order of BUTTON_PINS in config.h (paddles first)
//
//  Serial menu: open the Serial Monitor (115200 baud, "Newline" line ending)
//  and type `help`.
// =============================================================================
#include "AxisMath.h"
#include "Bts7960Driver.h"
#include "Calibration.h"
#include "Debouncer.h"
#include "FfbUsb.h"
#include "ForceSafety.h"
#include "PotPositionSensor.h"
#include "Tuning.h"
#include "config.h"

#if !defined(USB_SERIAL_HID) && !defined(JOYSTICK_INTERFACE)
#error "Set Tools > USB Type to 'Serial + FFB Wheel' (or 'Serial + Keyboard + Mouse + Joystick' without force feedback)."
#endif

// ----- The steering sensor ---------------------------------------------------
// This is the ONE place that decides which kind of sensor we use. To switch to
// an encoder later: write an EncoderPositionSensor class and change this line.
PotPositionSensor potSensor(STEERING_POT_PIN, POT_TRAVEL_DEG, STEERING_INVERT);
PositionSensor& steering = potSensor;

// ----- The motor driver ------------------------------------------------------
// Same idea: the ONE place that decides which motor driver we use.
Bts7960Driver bts7960(MOTOR_RPWM_PIN, MOTOR_LPWM_PIN, MOTOR_ENABLE_PIN, MOTOR_INVERT);
MotorDriver& motor = bts7960;

ForceSafety safety;
axis::VelocityEstimator velocity;      // wheel speed, deg/s
axis::VelocityEstimator acceleration;  // change of speed, deg/s^2 (for inertia)

// ----- Pedals ----------------------------------------------------------------
struct Pedal {
  const char* name;
  int pin;      // -1 = not fitted
  bool invert;
  PedalCal* cal;  // points into `cal` below, so calibration edits apply live
  int32_t raw;

  void begin() {
    if (pin >= 0) pinMode(pin, INPUT);
  }
  void update() { raw = (pin >= 0) ? analogRead(pin) : 0; }
  int32_t axisValue() const {
    if (pin < 0) return axis::OUT_MIN;
    return axis::pedalToAxis(raw, cal->rawMin, cal->rawMax,
                             PEDAL_DEADZONE_LOW, PEDAL_DEADZONE_HIGH, invert);
  }
  uint16_t axisValue16() const {
    if (pin < 0) return 0;
    return axis::pedalToAxis16(raw, cal->rawMin, cal->rawMax,
                               PEDAL_DEADZONE_LOW, PEDAL_DEADZONE_HIGH, invert);
  }
};

Calibration cal;

Pedal pedals[] = {
    {"throttle", THROTTLE_PIN, THROTTLE_INVERT, &cal.throttle, 0},
    {"brake", BRAKE_PIN, BRAKE_INVERT, &cal.brake, 0},
    {"clutch", CLUTCH_PIN, CLUTCH_INVERT, &cal.clutch, 0},
};
constexpr int NUM_PEDALS = sizeof(pedals) / sizeof(pedals[0]);
Pedal& throttle = pedals[0];
Pedal& brake = pedals[1];
Pedal& clutch = pedals[2];

// ----- Buttons ---------------------------------------------------------------
constexpr int NUM_BUTTONS = sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]);
static_assert(NUM_BUTTONS <= 32, "The Teensy joystick supports at most 32 buttons");
Debouncer buttons[NUM_BUTTONS];

// ----- Pedal calibration state -----------------------------------------------
// While this is running we just record the lowest and highest reading of each
// pedal. Press every pedal all the way down and let it go during the window.
bool pedalCalActive = false;
uint32_t pedalCalEndMs = 0;
constexpr uint32_t PEDAL_CAL_DURATION_MS = 8000;

// ----- Motor state -------------------------------------------------------------
bool motorArmed = false;      // the user's on/off switch ("motor on"/"motor off")
uint32_t lastHostOkMs = 0;    // last time the PC accepted a USB report
uint32_t motorTestStartMs = 0;
bool motorTestActive = false;
constexpr float MOTOR_TEST_TORQUE = 0.15f;

// Test effects you can switch on from the Serial Monitor to feel and tune the
// wheel without a game ("test spring", "test off", ...).
enum class TestEffect : uint8_t { Off, Spring, Damper, Constant, Sine };
TestEffect testEffect = TestEffect::Off;
float testStrength = 0.3f;  // 0..1

// Loop stall guard: the main loop writes the time into `loopHeartbeatMs` on
// every pass. A timer interrupt (which keeps firing even if loop() is stuck)
// checks it every millisecond, and if it hasn't changed for
// LOOP_STALL_TIMEOUT_MS it cuts the motor itself.
IntervalTimer stallGuardTimer;
volatile uint32_t loopHeartbeatMs = 0;
volatile bool loopStalled = false;

void stallGuardIsr() {
  if (millis() - loopHeartbeatMs > LOOP_STALL_TIMEOUT_MS) {
    motor.emergencyStop();
    loopStalled = true;
  }
}

bool monitorOn = false;
elapsedMicros sinceReport;   // Teensy helper: counts up microseconds by itself
elapsedMillis sinceMonitor;

void applyCalibrationToSafety() {
  safety.settings.maxTorque = cal.maxTorque;
  safety.settings.endstopStiffness = cal.endstopStiffness;
  safety.settings.halfRangeDeg = cal.steeringRangeDeg * 0.5f;
  safety.settings.slewPerMs = TORQUE_SLEW_PER_MS;
  safety.settings.sensorFaultMs = SENSOR_FAULT_MS;
  ffbEngine.settings.invertDirectional = cal.ffbInvert;
}

// =============================================================================
void setup() {
  // The motor driver goes first, so the motor is guaranteed off from the
  // very start.
  motor.begin();

  Serial.begin(115200);  // on a Teensy the baud rate is ignored; USB is always fast

  analogReadResolution(ADC_BITS);
  analogReadAveraging(ADC_AVERAGING);

  // Load saved calibration (or defaults on a fresh Teensy).
  bool loaded = calibration::load(cal);
  applyCalibrationToSafety();
  motorArmed = cal.armAtBoot;

  steering.begin();
  steering.setCenterCalibration(cal.steeringCenter);
  for (Pedal& p : pedals) p.begin();
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    buttons[i] = Debouncer(BUTTON_DEBOUNCE_MS);
  }
  if (ESTOP_PIN >= 0) pinMode(ESTOP_PIN, INPUT_PULLUP);

#if !HAVE_FFB
  // "Manual send" = only send a USB report when we call send_now(), so all
  // axes and buttons arrive at the PC together in one consistent snapshot.
  Joystick.useManualSend(true);
#endif

  // Give the Serial Monitor a moment to connect, but never wait for it:
  // the wheel must work in games without anything listening on Serial.
  uint32_t start = millis();
  while (!Serial && millis() - start < 1500) {
  }
  Serial.println(HAVE_FFB ? F("\nTeensy FFB wheel - force feedback enabled (USB type: Serial + FFB Wheel)")
                          : F("\nTeensy FFB wheel - NO force feedback (stock joystick USB type)"));
  Serial.println(loaded ? F("Loaded saved calibration.")
                        : F("No saved calibration found, using defaults. Type 'help'."));
  Serial.println(motorArmed ? F("Motor is ARMED (autoarm is on).")
                            : F("Motor is off. Type 'motor on' to enable it."));

  loopHeartbeatMs = millis();
  stallGuardTimer.begin(stallGuardIsr, 1000);  // every 1000 us
}

// =============================================================================
void readInputs() {
  steering.update();
  for (Pedal& p : pedals) p.update();

  uint32_t now = millis();
  for (int i = 0; i < NUM_BUTTONS; i++) {
    bool rawPressed = digitalRead(BUTTON_PINS[i]) == LOW;  // LOW = pressed (pull-up wiring)
    buttons[i].update(rawPressed, now);
  }

  if (pedalCalActive) {
    for (Pedal& p : pedals) {
      if (p.pin < 0) continue;
      if (p.raw < p.cal->rawMin) p.cal->rawMin = p.raw;
      if (p.raw > p.cal->rawMax) p.cal->rawMax = p.raw;
    }
    if ((int32_t)(now - pedalCalEndMs) >= 0) {
      pedalCalActive = false;
      Serial.println(F("Pedal calibration done:"));
      for (Pedal& p : pedals) {
        if (p.pin < 0) continue;
        Serial.printf("  %-8s released=%ld pressed=%ld\n", p.name,
                      (long)p.cal->rawMin, (long)p.cal->rawMax);
      }
      Serial.println(F("Type 'save' to keep it after power-off."));
    }
  }
}

// Returns true if the PC accepted the report (used by the host watchdog).
bool sendReport() {
#if HAVE_FFB
  uint32_t bits = 0;
  for (int i = 0; i < NUM_BUTTONS; i++) {
    if (buttons[i].pressed()) bits |= (1UL << i);
  }
  return ffbSendWheelReport(axis::steeringToAxis16(steering.angleDeg(), cal.steeringRangeDeg),
                            throttle.axisValue16(), brake.axisValue16(), clutch.axisValue16(),
                            bits);
#else
  Joystick.X(axis::steeringToAxis(steering.angleDeg(), cal.steeringRangeDeg));
  Joystick.Y(throttle.axisValue());
  Joystick.Z(brake.axisValue());
  Joystick.Zrotate(clutch.axisValue());
  Joystick.sliderLeft(0);
  Joystick.sliderRight(0);
  Joystick.hat(-1);  // -1 = hat not pressed
  for (int i = 0; i < NUM_BUTTONS; i++) {
    Joystick.button(i + 1, buttons[i].pressed());  // joystick buttons count from 1
  }
  // usb_joystick_send() is what Joystick.send_now() calls inside; using it
  // directly tells us whether the PC took the report (0) or not (-1).
  return usb_joystick_send() == 0;
#endif
}

// Work out the torque we'd like, then let the safety layer decide.
void updateMotor() {
  uint32_t now = millis();
  float requested = 0;

  // --- Game forces -----------------------------------------------------------
  // The engine wants position/speed as fractions (see FfbEngine.h WheelState).
  ffb::WheelState w;
  w.position = axis::steeringFraction(steering.angleDeg(), cal.steeringRangeDeg);
  w.velocity = velocity.value() / FFB_FULL_SPEED_DEG_S;
  w.acceleration = acceleration.value() / FFB_FULL_ACCEL_DEG_S2;
  requested += ffbComputeForce(w, now) * cal.ffbGain;

  // --- Test effects (no game needed) -------------------------------------------
  switch (testEffect) {
    case TestEffect::Off: break;
    case TestEffect::Spring: requested -= testStrength * w.position; break;
    case TestEffect::Damper: requested -= testStrength * w.velocity * 4.0f; break;
    case TestEffect::Constant: requested += testStrength; break;
    case TestEffect::Sine:  // 2 shakes per second, like a rumble strip
      requested += testStrength * sinf(2.0f * 3.14159265f * 2.0f * (now % 1000) / 1000.0f);
      break;
  }

  // --- Always-on feel: damping, friction, minimum torque (Tuning.h) ------------
  TuningSettings ts;
  ts.damping = cal.damping;
  ts.friction = cal.friction;
  ts.minTorque = cal.minTorque;
  requested = tuning::apply(requested, w.velocity, ts);

  // --- Test command (end stops are added inside ForceSafety) ------------------
  if (motorTestActive) {
    uint32_t t = now - motorTestStartMs;
    if (t < 500) requested = MOTOR_TEST_TORQUE;          // half a second right
    else if (t < 1000) requested = -MOTOR_TEST_TORQUE;   // half a second left
    else {
      motorTestActive = false;
      Serial.println(F("Motor test done. Did it push right first, then left? "
                       "If it went left first, set MOTOR_INVERT in config.h."));
    }
  }

  SafetyInputs in;
  in.requestedTorque = requested;
  in.angleDeg = steering.angleDeg();
  in.armed = motorArmed;
  in.sensorHealthy = steering.healthy();
  in.hostAlive = usb_configuration != 0 && (now - lastHostOkMs) < HOST_TIMEOUT_MS;
  in.estopPressed = ESTOP_PIN >= 0 && digitalRead(ESTOP_PIN) == LOW;
  in.loopStalled = loopStalled;
  in.nowMs = now;

  SafetyState before = safety.state();
  float torque = safety.process(in);
  motor.setTorque(torque);

  SafetyState after = safety.state();
  if (after != before && after != SafetyState::Ok && after != SafetyState::NotArmed) {
    Serial.printf("Motor stopped: %s\n", safetyStateName(after));
  }
  if (in.estopPressed && motorArmed) {
    motorArmed = false;  // e-stop also disarms, so letting go doesn't restart it
  }
}

// ----- Serial menu -----------------------------------------------------------
int parseOnOff(const char* s);
bool parsePercent(const char* s, float lo, float hi, float& out);

void printHelp() {
  Serial.println(F(
      "Calibration:\n"
      "  show          print all readings once\n"
      "  monitor       toggle printing readings 10x per second\n"
      "  center        hold the wheel straight, then type this\n"
      "  range N       set lock-to-lock rotation in degrees (e.g. range 270)\n"
      "  pedals        8 s window: press each pedal fully and release it\n"
      "Motor:\n"
      "  motor on      enable the motor (also clears latched faults)\n"
      "  motor off     disable the motor\n"
      "  motor test    gentle push right, then left (checks direction)\n"
      "  maxtorque N   motor power ceiling in percent (e.g. maxtorque 30)\n"
      "  endstop N     end-stop stiffness, percent torque per degree (e.g. 10)\n"
      "  autoarm on|off  enable the motor automatically at power-up\n"
      "Force feedback:\n"
      "  ffb           show what the game is doing (effects, gain)\n"
      "  gain N        overall strength of game forces in percent (e.g. gain 70)\n"
      "  invert on|off flip the direction of game forces (if the wheel pulls\n"
      "                the wrong way in corners)\n"
      "Tuning (always on, on top of the game):\n"
      "  damping N     resistance to fast turning, percent (stops wobble)\n"
      "  friction N    constant drag when turning, percent\n"
      "  mintorque N   lift small forces above the motor's sticking point\n"
      "  test spring|damper|constant|sine [N]   feel an effect without a game\n"
      "  test off      stop the test effect\n"
      "  settings      print every setting\n"
      "Storage:\n"
      "  save          store settings so they survive power-off\n"
      "  reset         go back to default settings (then 'save' to keep)\n"
      "Tip: match 'range' to the steering rotation set in the game."));
}

void printReadings() {
  Serial.printf("steer raw=%4ld angle=%7.1f axis=%4ld %s | ", steering.rawValue(),
                steering.angleDeg(),
                (long)axis::steeringToAxis(steering.angleDeg(), cal.steeringRangeDeg),
                steering.healthy() ? "ok" : "SENSOR FAULT?");
  for (Pedal& p : pedals) {
    if (p.pin < 0) continue;
    Serial.printf("%s raw=%4ld axis=%4ld | ", p.name, (long)p.raw, (long)p.axisValue());
  }
  Serial.print(F("buttons "));
  for (int i = 0; i < NUM_BUTTONS; i++) Serial.print(buttons[i].pressed() ? '1' : '0');
  Serial.printf(" | motor %+5.1f%% (%s)\n", motor.torque() * 100.0f,
                safetyStateName(safety.state()));
}

void printFfbStatus() {
  if (!HAVE_FFB) {
    Serial.println(F("No force feedback: select USB Type 'Serial + FFB Wheel'."));
    return;
  }
  static const char* const names[] = {"-", "constant", "ramp", "square", "sine", "triangle",
                                      "saw up", "saw down", "spring", "damper", "inertia",
                                      "friction"};
  Serial.printf("FFB: %d effects created, %d playing, device gain %d%%, actuators %s%s, "
                "%lu reports received\n",
                ffbEngine.allocatedCount(), ffbEngine.playingCount(),
                ffbEngine.deviceGain() * 100 / 255, ffbEngine.actuatorsEnabled() ? "on" : "off",
                ffbEngine.paused() ? " (paused)" : "", (unsigned long)ffbEngine.reportCount());
  for (unsigned i = 1; i <= ffb_size::MAX_EFFECTS; i++) {
    const ffb::Effect& e = ffbEngine.effect(i);
    if (e.type == ffb::ET_NONE) continue;
    Serial.printf("  #%-2u %-9s %s gain %3d%%\n", i, names[e.type],
                  e.playing ? "PLAYING" : "stopped", e.gain * 100 / 255);
  }
  Serial.printf("Our settings: gain %.0f%%, invert %s, max torque %.0f%%\n", cal.ffbGain * 100,
                cal.ffbInvert ? "on" : "off", cal.maxTorque * 100);
}

void printSettings() {
  Serial.printf("Steering: range %.0f deg, center %ld\n", cal.steeringRangeDeg,
                (long)steering.centerCalibration());
  Serial.printf("Motor: %s, autoarm %s, max torque %.0f%%, end stop %.0f%%/deg\n",
                motorArmed ? "on" : "off", cal.armAtBoot ? "on" : "off", cal.maxTorque * 100,
                cal.endstopStiffness * 100);
  Serial.printf("FFB: gain %.0f%%, invert %s\n", cal.ffbGain * 100, cal.ffbInvert ? "on" : "off");
  Serial.printf("Tuning: damping %.0f%%, friction %.0f%%, min torque %.0f%%\n",
                cal.damping * 100, cal.friction * 100, cal.minTorque * 100);
}

// "test spring 30", "test sine", "test off", ...
void handleTestCommand(char* args) {
  char* num = strchr(args, ' ');
  if (num) {
    *num++ = '\0';
    float pct;
    if (!parsePercent(num, 0, 100, pct)) return;
    testStrength = pct;
  }
  if (strcmp(args, "off") == 0) testEffect = TestEffect::Off;
  else if (strcmp(args, "spring") == 0) testEffect = TestEffect::Spring;
  else if (strcmp(args, "damper") == 0) testEffect = TestEffect::Damper;
  else if (strcmp(args, "constant") == 0) testEffect = TestEffect::Constant;
  else if (strcmp(args, "sine") == 0) testEffect = TestEffect::Sine;
  else {
    Serial.println(F("Use: test spring|damper|constant|sine [percent], or test off"));
    return;
  }
  if (testEffect == TestEffect::Off) {
    Serial.println(F("Test effect off."));
  } else {
    Serial.printf("Test effect on at %.0f%%%s\n", testStrength * 100,
                  motorArmed ? "" : " (type 'motor on' to feel it)");
  }
}

// Parses "on"/"off" after a command; returns -1 if neither.
int parseOnOff(const char* s) {
  if (strcmp(s, "on") == 0) return 1;
  if (strcmp(s, "off") == 0) return 0;
  return -1;
}

// Parses a percentage and checks it is within lo..hi. Returns false if not.
bool parsePercent(const char* s, float lo, float hi, float& out) {
  float v = atof(s);
  if (v < lo || v > hi) {
    Serial.printf("Value must be between %.0f and %.0f.\n", lo, hi);
    return false;
  }
  out = v / 100.0f;
  return true;
}

void handleCommand(char* line) {
  float pct;
  if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
    printHelp();
  } else if (strcmp(line, "show") == 0) {
    printReadings();
  } else if (strcmp(line, "monitor") == 0) {
    monitorOn = !monitorOn;
    Serial.println(monitorOn ? F("Monitor on (type 'monitor' again to stop)") : F("Monitor off"));
  } else if (strcmp(line, "center") == 0) {
    steering.setCenterHere();
    cal.steeringCenter = steering.centerCalibration();
    Serial.printf("Center set to %ld. Type 'save' to keep it.\n", (long)cal.steeringCenter);
  } else if (strncmp(line, "range ", 6) == 0) {
    float deg = atof(line + 6);
    if (deg < 30 || deg > 1080) {
      Serial.println(F("Range must be between 30 and 1080 degrees."));
    } else {
      cal.steeringRangeDeg = deg;
      applyCalibrationToSafety();
      Serial.printf("Steering range set to %.0f degrees. Type 'save' to keep it.\n", deg);
      if (deg > POT_TRAVEL_DEG) {
        Serial.println(F("Note: that is more than the pot can measure, so the axis "
                         "will never reach full lock."));
      }
    }
  } else if (strcmp(line, "pedals") == 0) {
    // Start from the current readings and let min/max grow outward.
    for (Pedal& p : pedals) {
      p.cal->rawMin = p.raw;
      p.cal->rawMax = p.raw;
    }
    pedalCalActive = true;
    pedalCalEndMs = millis() + PEDAL_CAL_DURATION_MS;
    Serial.println(F("Pedal calibration: press each pedal all the way down and release it..."));
  } else if (strcmp(line, "motor on") == 0) {
    safety.clearFaults();
    loopStalled = false;
    motorArmed = true;
    Serial.println(F("Motor ON. Type 'motor off' to disable."));
  } else if (strcmp(line, "motor off") == 0) {
    motorArmed = false;
    motorTestActive = false;
    testEffect = TestEffect::Off;
    Serial.println(F("Motor OFF."));
  } else if (strcmp(line, "motor test") == 0) {
    if (!motorArmed) {
      Serial.println(F("Type 'motor on' first."));
    } else {
      motorTestActive = true;
      motorTestStartMs = millis();
      Serial.println(F("Motor test: pushing right, then left. Keep your hands clear."));
    }
  } else if (strncmp(line, "maxtorque ", 10) == 0) {
    if (parsePercent(line + 10, 0, 100, pct)) {
      cal.maxTorque = pct;
      applyCalibrationToSafety();
      Serial.printf("Max torque %.0f%%. Type 'save' to keep it.\n", pct * 100);
    }
  } else if (strncmp(line, "endstop ", 8) == 0) {
    if (parsePercent(line + 8, 0, 100, pct)) {
      cal.endstopStiffness = pct;
      applyCalibrationToSafety();
      Serial.printf("End-stop stiffness %.0f%% per degree. Type 'save' to keep it.\n",
                    pct * 100);
    }
  } else if (strncmp(line, "autoarm ", 8) == 0) {
    int v = parseOnOff(line + 8);
    if (v < 0) {
      Serial.println(F("Use 'autoarm on' or 'autoarm off'."));
    } else {
      cal.armAtBoot = v;
      Serial.println(v ? F("Motor will be enabled at power-up. Type 'save' to keep it.")
                       : F("Motor stays off at power-up. Type 'save' to keep it."));
    }
  } else if (strcmp(line, "ffb") == 0) {
    printFfbStatus();
  } else if (strncmp(line, "gain ", 5) == 0) {
    if (parsePercent(line + 5, 0, 200, pct)) {
      cal.ffbGain = pct;
      Serial.printf("FFB gain %.0f%%. Type 'save' to keep it.\n", pct * 100);
    }
  } else if (strncmp(line, "invert ", 7) == 0) {
    int v = parseOnOff(line + 7);
    if (v < 0) {
      Serial.println(F("Use 'invert on' or 'invert off'."));
    } else {
      cal.ffbInvert = v;
      applyCalibrationToSafety();
      Serial.println(v ? F("Game forces inverted. Type 'save' to keep it.")
                       : F("Game forces normal. Type 'save' to keep it."));
    }
  } else if (strncmp(line, "damping ", 8) == 0) {
    if (parsePercent(line + 8, 0, 100, pct)) {
      cal.damping = pct;
      Serial.printf("Damping %.0f%%. Type 'save' to keep it.\n", pct * 100);
    }
  } else if (strncmp(line, "friction ", 9) == 0) {
    if (parsePercent(line + 9, 0, 100, pct)) {
      cal.friction = pct;
      Serial.printf("Friction %.0f%%. Type 'save' to keep it.\n", pct * 100);
    }
  } else if (strncmp(line, "mintorque ", 10) == 0) {
    if (parsePercent(line + 10, 0, 50, pct)) {
      cal.minTorque = pct;
      Serial.printf("Min torque %.0f%%. Type 'save' to keep it.\n", pct * 100);
    }
  } else if (strncmp(line, "test ", 5) == 0) {
    handleTestCommand(line + 5);
  } else if (strcmp(line, "settings") == 0) {
    printSettings();
  } else if (strcmp(line, "save") == 0) {
    cal.steeringCenter = steering.centerCalibration();
    calibration::save(cal);
    Serial.println(F("Saved."));
  } else if (strcmp(line, "reset") == 0) {
    cal = calibration::defaults();
    steering.setCenterCalibration(cal.steeringCenter);
    applyCalibrationToSafety();
    pedalCalActive = false;
    Serial.println(F("Defaults restored (not saved yet)."));
  } else if (line[0] != '\0') {
    Serial.printf("Unknown command '%s'. Type 'help'.\n", line);
  }
}

// Collect typed characters until Enter, without ever pausing the wheel.
void pollSerial() {
  static char buf[32];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      buf[len] = '\0';
      handleCommand(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = tolower(c);
    }
  }
}

// =============================================================================
void loop() {
  loopHeartbeatMs = millis();
  pollSerial();

  if (sinceReport >= REPORT_INTERVAL_US) {
    sinceReport = 0;
    readInputs();
    velocity.update(steering.angleDeg(), REPORT_INTERVAL_US * 1e-6f);
    acceleration.update(velocity.value(), REPORT_INTERVAL_US * 1e-6f);
    if (sendReport()) lastHostOkMs = millis();
    updateMotor();
  }

  if (monitorOn && sinceMonitor >= 100) {
    sinceMonitor = 0;
    printReadings();
  }
}
