# Teensy FFB wheel

Firmware for a DIY force-feedback steering wheel on a **Teensy 4.0**.

| Step | What | Status |
|------|------|--------|
| 1 | Wheel, pedals, buttons and paddles show up as a USB game controller | done |
| 2 | Safe motor drive (BTS7960) with limits, end stops and a watchdog | done |
| 3 | Force feedback from games (HID PID / DirectInput) | done (untested in games) |
| 4 | Tuning and per-game testing (BeamNG, Live for Speed, F1 2020) | tuning tools done, see [docs/testing-and-tuning.md](docs/testing-and-tuning.md) |

## Files

```
firmware/teensy_ffb/
  teensy_ffb.ino        main sketch: reads inputs, sends USB reports, serial menu
  config.h              ALL pins and settings - the file you edit
  PositionSensor.h      interface any steering sensor must follow (pot, encoder, ...)
  PotPositionSensor.h   the potentiometer version of that interface
  AxisMath.h            the math from raw readings to joystick values
  Debouncer.h           filters out switch bounce on buttons/paddles
  MotorDriver.h         interface any motor driver must follow
  Bts7960Driver.h       the BTS7960 (IBT-2) version of that interface
  ForceSafety.h         faults, end stops, torque and slew limits for the motor
  FfbEngine.h/.cpp      force-feedback effects from games -> one torque
  FfbUsb.h/.cpp         connects the engine to the patched Teensy USB code
  FfbReportSizes.h      generated: byte size of each USB report
  Tuning.h              always-on feel: damping, friction, minimum torque
docs/testing-and-tuning.md  step-by-step bring-up, per-game notes, tuning table
core_patch/usb_ffb_desc.h   generated: the USB HID descriptor (with force feedback)
tools/
  patch_teensy_core.py  adds the "Serial + FFB Wheel" USB type to Teensyduino
  gen_ffb_descriptor.py generates the descriptor (only needed to change it)
  pio_patch_core.py     PlatformIO runs the patch automatically with this
  Calibration.h         stores calibration in EEPROM so it survives power-off
test/                   unit tests for AxisMath + Debouncer that run on a PC
platformio.ini          optional PlatformIO build file
```

## Wiring

> **The Teensy 4.0 is a 3.3 V board.** Power every pot from the **3.3V** pin,
> never 5V. 5 V on an analog pin can damage it.

- **Steering pot and pedal pots:** outer legs to 3.3V and GND, middle leg
  (wiper) to the analog pin set in `config.h`.
- **Buttons and paddles:** one side to the pin in `config.h`, other side to GND.
  No resistors needed: the code turns on the Teensy's internal pull-ups.
- **BTS7960 motor driver (IBT-2 board):**
  - RPWM to pin 8, LPWM to pin 9 (placeholders, see `config.h`)
  - R_EN and L_EN joined together, to pin 10
  - **a 10k resistor from that R_EN/L_EN wire to GND**, so the driver stays off
    while the Teensy boots or is being programmed
  - VCC to the Teensy 3.3V, GND to Teensy GND
  - B+/B- to the motor power supply, M+/M- to the motor
- **Optional e-stop button:** set `ESTOP_PIN` in `config.h`, wire to GND.

The pin numbers in `config.h` are placeholders until the real wiring is known.

## Building and uploading

**Arduino IDE** (with Teensyduino installed):

1. **Once** (and again after every Teensyduino update): close the Arduino IDE
   and run `python tools/patch_teensy_core.py` from this folder. It finds your
   Teensyduino install and adds a new USB type, "Serial + FFB Wheel". (Needs
   Python 3. If it can't find Teensyduino, it tells you how to pass the path.)
2. Open `firmware/teensy_ffb/teensy_ffb.ino`.
3. Tools > Board > **Teensy 4.0**.
4. Tools > USB Type > **Serial + FFB Wheel**.
5. Upload.

Without the patch you can still pick **Serial + Keyboard + Mouse + Joystick**:
the wheel works as a plain controller and the motor still does end stops, but
games can't send forces.

**PlatformIO:** `pio run -t upload`, then `pio device monitor` for the menu.
The patch is applied automatically.

## First-time setup

1. On Windows, press Win+R, type `joy.cpl`. The wheel shows up as
   **Teensy FFB Wheel** (or "Serial/Keyboard/Mouse/Joystick" with the stock
   USB type). Open **Properties**: the X axis should move as you turn the
   wheel, the pedals on Y / Z / Z-rotation, and the buttons should light up.
2. Open the Serial Monitor (115200, line ending "Newline") and type `help`.
3. Hold the wheel straight and type `center`.
4. Type `range 270` (or whatever rotation you set in the game).
5. Type `pedals`, then within 8 seconds press each pedal all the way down and
   let it go.
6. Type `save`. Calibration now survives unplugging.

### Turning the motor on (step 2)

The motor is **off** at every power-up until you allow it. Keep your hands off
the wheel for the first test.

1. `motor on` enables it. The end stops are now active: turn past full lock
   and you'll feel the wheel push back.
2. `motor test` pushes gently right for half a second, then left. If it goes
   left first, set `MOTOR_INVERT = true` in `config.h` (or swap the motor
   wires).
3. `maxtorque 30` caps motor power at 30 % (the default). Raise it slowly.
4. `endstop 10` sets how hard the end stop pushes back (percent torque per
   degree past full lock).
5. `autoarm on` + `save` if you want the motor enabled at power-up.

`motor off` disables it at any time.

### Force feedback (step 3)

With the "Serial + FFB Wheel" USB type, Windows sees a force-feedback device
and games can use it. Our own `motor on` switch and all the safety limits
still apply on top.

- `ffb` shows what the game is doing right now: which effects it created,
  which are playing, and the game's own strength setting.
- `gain 70` sets the overall strength of game forces (percent; up to 200).
- `invert on` flips the direction of the game's forces. If the wheel pulls
  you *into* corners instead of back towards straight, turn this on. Springs
  and dampers are calculated on the wheel and are never flipped.

### Tuning (step 4)

`damping N`, `friction N` and `mintorque N` shape the feel on top of the
game, and `test spring|damper|constant|sine [N]` lets you feel effects without
a game. `settings` prints everything. The full bring-up order, per-game notes
for BeamNG, Live for Speed and F1 2020, and a tuning table are in
[docs/testing-and-tuning.md](docs/testing-and-tuning.md).

`monitor` prints all live readings 10 times per second, which is handy for
checking wiring. A "SENSOR FAULT?" next to the steering reading means the pot is
reading at the very end of its range, usually a loose wiper wire.

## How it works

**USB HID.** A keyboard, mouse or joystick is a USB "Human Interface Device".
When it is plugged in, it sends the PC a *report descriptor*, a small table
that says what is in its messages ("6 axes of 10 bits, 32 buttons, a hat
switch"). Windows uses that table to understand the *reports* that follow.
Games read the same information through DirectInput, so no driver is needed.
The Teensy's "Joystick" USB type comes with a ready-made descriptor, so in this
step we just fill in the values and call `Joystick.send_now()` 1000 times per
second. Force feedback (step 3) needs extra sections in the descriptor (the
"PID" part, for Physical Interface Device), which is when we will replace the
built-in one with our own.

**Axes.** An axis is just a number between 0 and 1023. For steering, 512 is
center, 0 is full left lock and 1023 full right lock, where "full lock" is half
the `range` either side of center. Pedals go from 0 (released) to 1023 (floored)
with a small dead zone at each end so a resting foot reads 0 and you can always
reach 100 %.

**From pot to angle.** The ADC turns the pot voltage (0 to 3.3 V) into a number
from 0 to 4095. The reading at center is remembered, and each count away from it
is `POT_TRAVEL_DEG / 4095` degrees. The ADC averages 8 samples per read to
remove jitter.

**Swappable sensor.** The rest of the code only talks to a `PositionSensor`
(see `PositionSensor.h`), which promises an angle in degrees. Switching to an
encoder later means writing an `EncoderPositionSensor` class and changing one
line in `teensy_ffb.ino`.

**Motor safety.** Nothing talks to the motor directly. Every torque request
goes through `ForceSafety`, which (1) gives zero torque if the motor isn't
armed, the steering sensor looks broken, the PC has stopped reading our USB
reports for 100 ms, or the e-stop is pressed, (2) adds the end-stop spring past
full lock and removes any force pushing further out, (3) clamps to `maxtorque`,
and (4) limits how fast torque can change so nothing jerks. On top of that a
timer interrupt checks every millisecond that the main loop is still running;
if it hasn't run for 50 ms, the interrupt switches the driver off by itself.
Sensor and stall faults stay latched until you type `motor on`.

**PWM and the H-bridge.** The BTS7960 can connect each motor wire to + or -,
so current can flow either way (= either direction). The Teensy switches it on
and off 20,000 times a second; the fraction of "on" time (duty cycle) sets the
average voltage, and with it roughly the torque.

**Force feedback (HID PID).** A game doesn't send "torque = 40 %" every
millisecond. It *creates effects* on the wheel and then mostly lets them run:
a constant force (updated often), a centering spring, a damper, a sine-wave
rumble, and so on. Windows' built-in PID driver turns the game's DirectInput
calls into USB reports: "create a new effect" (we pick a free slot and report
its number back), "set its values", "start it", "stop it". The descriptor that
lists all these reports is generated by `tools/gen_ffb_descriptor.py`. Every
millisecond `FfbEngine` adds up all playing effects, using the current wheel
position and speed for springs, dampers and friction, and the result goes
through the safety layer to the motor.

**Why patch the Teensy core?** The USB descriptors and the code that answers
the PC's USB requests are part of the Teensy core, which a sketch can't
replace. The patch adds a new USB type with our descriptor and forwards the
PC's force-feedback requests to the sketch. Everything is inside
`#if defined(USB_FFB_WHEEL)`, so other USB types and projects are unchanged,
and the original files are kept as `.orig`.

**Debouncing.** A switch contact flickers for a few milliseconds when pressed.
A press only counts once the pin has been steady for 5 ms, so one paddle pull is
one gear change.

## Running the unit tests

The axis math, debouncer, safety layer and the whole force-feedback engine
have no Arduino code in them, so they can be tested on a PC with a C++
compiler. The FFB tests replay the same sequence of reports Windows sends:

```
./test/run_tests.sh
```
