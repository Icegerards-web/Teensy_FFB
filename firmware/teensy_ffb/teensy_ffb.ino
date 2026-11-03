// =============================================================================
//  teensy_ffb.ino  -  Step 1: the wheel as a plain USB game controller.
// =============================================================================
//
//  What this sketch does, 1000 times per second:
//    1. read the steering sensor, the pedal pots and the buttons
//    2. turn those readings into joystick numbers (see AxisMath.h)
//    3. send one "report" with all of them to the PC over USB
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
//   For this step we use the descriptor that ships with Teensyduino (USB Type
//   "Serial + Keyboard + Mouse + Joystick"): 6 axes of 0..1023, 32 buttons and
//   a hat switch. Step 3 (force feedback) needs our own descriptor with the
//   extra "PID" force-feedback section; that is when we'll replace it.
//
//  Axis mapping (games let you re-bind these, so the names don't matter much):
//    X        = steering       Y       = throttle
//    Z        = brake          Z-rot   = clutch
//    Buttons  = in the order of BUTTON_PINS in config.h (paddles first)
//
//  Serial calibration menu: open the Serial Monitor (115200 baud, "Newline"
//  line ending) and type `help`.
// =============================================================================
#include "AxisMath.h"
#include "Calibration.h"
#include "Debouncer.h"
#include "PotPositionSensor.h"
#include "config.h"

#if !defined(USB_SERIAL_HID) && !defined(JOYSTICK_INTERFACE)
#error "Set Tools > USB Type to 'Serial + Keyboard + Mouse + Joystick' in the Arduino IDE."
#endif

// ----- The steering sensor ---------------------------------------------------
// This is the ONE place that decides which kind of sensor we use. To switch to
// an encoder later: write an EncoderPositionSensor class and change this line.
PotPositionSensor potSensor(STEERING_POT_PIN, POT_TRAVEL_DEG, STEERING_INVERT);
PositionSensor& steering = potSensor;

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

bool monitorOn = false;
elapsedMicros sinceReport;   // Teensy helper: counts up microseconds by itself
elapsedMillis sinceMonitor;

// =============================================================================
void setup() {
  Serial.begin(115200);  // on a Teensy the baud rate is ignored; USB is always fast

  analogReadResolution(ADC_BITS);
  analogReadAveraging(ADC_AVERAGING);

  // Load saved calibration (or defaults on a fresh Teensy).
  bool loaded = calibration::load(cal);

  steering.begin();
  steering.setCenterCalibration(cal.steeringCenter);
  for (Pedal& p : pedals) p.begin();
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    buttons[i] = Debouncer(BUTTON_DEBOUNCE_MS);
  }

  // "Manual send" = only send a USB report when we call send_now(), so all
  // axes and buttons arrive at the PC together in one consistent snapshot.
  Joystick.useManualSend(true);

  // Give the Serial Monitor a moment to connect, but never wait for it:
  // the wheel must work in games without anything listening on Serial.
  uint32_t start = millis();
  while (!Serial && millis() - start < 1500) {
  }
  Serial.println(F("\nTeensy FFB wheel - step 1 (USB controller, no force feedback yet)"));
  Serial.println(loaded ? F("Loaded saved calibration.")
                        : F("No saved calibration found, using defaults. Type 'help'."));
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

void sendReport() {
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
  Joystick.send_now();
}

// ----- Serial menu -----------------------------------------------------------
void printHelp() {
  Serial.println(F(
      "Commands:\n"
      "  show        print all readings once\n"
      "  monitor     toggle printing readings 10x per second\n"
      "  center      hold the wheel straight, then type this\n"
      "  range N     set lock-to-lock rotation in degrees (e.g. range 270)\n"
      "  pedals      8 s window: press each pedal fully and release it\n"
      "  save        store calibration so it survives power-off\n"
      "  reset       go back to default calibration (then 'save' to keep)\n"
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
  Serial.println();
}

void handleCommand(char* line) {
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
  } else if (strcmp(line, "save") == 0) {
    cal.steeringCenter = steering.centerCalibration();
    calibration::save(cal);
    Serial.println(F("Saved."));
  } else if (strcmp(line, "reset") == 0) {
    cal = calibration::defaults();
    steering.setCenterCalibration(cal.steeringCenter);
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
  pollSerial();

  if (sinceReport >= REPORT_INTERVAL_US) {
    sinceReport = 0;
    readInputs();
    sendReport();
  }

  if (monitorOn && sinceMonitor >= 100) {
    sinceMonitor = 0;
    printReadings();
  }
}
