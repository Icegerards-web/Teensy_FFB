# Teensy FFB wheel

Firmware for a DIY force-feedback steering wheel on a **Teensy 4.0**.

| Step | What | Status |
|------|------|--------|
| 1 | Wheel, pedals, buttons and paddles show up as a USB game controller | **this code** |
| 2 | Safe motor drive (BTS7960) with limits, end stops and a watchdog | next |
| 3 | Force feedback from games (HID PID / DirectInput) | later |
| 4 | Tuning and per-game testing (BeamNG, Live for Speed, F1 2020) | later |

## Files

```
firmware/teensy_ffb/
  teensy_ffb.ino        main sketch: reads inputs, sends USB reports, serial menu
  config.h              ALL pins and settings - the file you edit
  PositionSensor.h      interface any steering sensor must follow (pot, encoder, ...)
  PotPositionSensor.h   the potentiometer version of that interface
  AxisMath.h            the math from raw readings to joystick values
  Debouncer.h           filters out switch bounce on buttons/paddles
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
- Pins 8 and 9 are kept free for the motor driver in step 2.

The pin numbers in `config.h` are placeholders until the real wiring is known.

## Building and uploading

**Arduino IDE** (with Teensyduino installed):

1. Open `firmware/teensy_ffb/teensy_ffb.ino`.
2. Tools > Board > **Teensy 4.0**.
3. Tools > USB Type > **Serial + Keyboard + Mouse + Joystick**. (This is what
   makes the Teensy show up as a game controller. If you forget it, the sketch
   stops with an error telling you so.)
4. Upload.

**PlatformIO:** `pio run -t upload`, then `pio device monitor` for the menu.

## First-time setup

1. On Windows, press Win+R, type `joy.cpl`, and open the Teensy's
   **Properties**. You should see the X axis move as you turn the wheel, the
   pedals on the Y / Z / Z-rotation axes and the buttons light up.
2. Open the Serial Monitor (115200, line ending "Newline") and type `help`.
3. Hold the wheel straight and type `center`.
4. Type `range 270` (or whatever rotation you set in the game).
5. Type `pedals`, then within 8 seconds press each pedal all the way down and
   let it go.
6. Type `save`. Calibration now survives unplugging.

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

**Debouncing.** A switch contact flickers for a few milliseconds when pressed.
A press only counts once the pin has been steady for 5 ms, so one paddle pull is
one gear change.

## Running the unit tests

The axis math and the debouncer have no Arduino code in them, so they can be
tested on a PC with a C++ compiler:

```
./test/run_tests.sh
```
