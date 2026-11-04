# Testing and tuning guide (step 4)

Work through this in order. Each stage checks one thing, so when something is
wrong you know where to look. Keep `maxtorque` low (the default is 30 %) until
the end, and keep your hands clear the first time the motor moves.

All commands are typed in the Serial Monitor (115200 baud, line ending
"Newline"). `help` lists them, `settings` shows every current value, and
nothing is kept after a power cycle until you type `save`.

## 1. Inputs (motor disconnected or `motor off`)

1. Upload with USB Type **Serial + FFB Wheel** (see the README for the
   one-time core patch).
2. Run `joy.cpl` on Windows. You should see **Teensy FFB Wheel**. Open
   Properties: turn the wheel, press the pedals and buttons.
3. Calibrate: `center`, `range 270` (or your rotation), `pedals`, `save`.
4. `monitor` shows the raw values. Steering should read about `0.0` degrees
   when straight and change smoothly. If it jumps around by more than a degree
   at rest, the pot wiring is noisy: shorten the wires, twist them, or add a
   100 nF capacitor from the wiper pin to GND.

## 2. Motor direction and end stops (no game)

1. Connect the motor power. Type `motor on`.
2. `motor test`: the wheel pushes right, then left. If it went left first, set
   `MOTOR_INVERT = true` in `config.h` and upload again.
3. Turn the wheel past full lock by hand. You should feel the end stop push
   back. Adjust with `endstop N` (percent torque per degree past the end).
4. Feel the test effects:
   - `test spring 30`: pulls towards center. If it pushes **away** from
     center, the motor direction is wrong (go back to step 2).
   - `test damper 30`: heavy when you turn fast, light when slow.
   - `test sine 15`: rumble.
   - `test constant 5`, then 6, 7, ...: find the smallest value at which the
     wheel starts to move by itself. Set `mintorque` a little **below** that.
   - `test off` when done.
5. If the wheel oscillates (wobbles left-right by itself) with the spring on,
   raise `damping` in steps of 2-5 %.

## 3. Force feedback in a game

Start with games that are known to have good DirectInput force feedback.

**Before the first game:** run the FFB Monitor (`python tools/ffb_monitor.py`,
see the README) next to the game, or type `ffb` in the Serial Monitor while the
game is running. You should see effects being created and "PLAYING". If you see `0 effects created` the
game isn't sending force feedback to this wheel: check that the wheel is
selected as the FFB device in the game's controls menu.

**The direction check (do this in every new game):** drive in a straight line
and steer a little to one side. The wheel should pull you **back towards
straight**. In a corner it should feel heavier the harder you turn. If it pulls
you *into* the turn, type `invert on`, then `save`. (Springs and dampers are
worked out on the wheel and are never flipped, so `invert` only changes the
forces the game sends.)

### BeamNG.drive

- In Options > Controls, bind the steering axis to the wheel's X axis and the
  pedals, then open the steering binding's force-feedback settings and enable
  FFB for this device.
- BeamNG calculates its forces from the car's physics and sends them mostly
  as a constant force, so `ffb` usually shows one constant effect playing.
- Set the in-game wheel rotation the same as our `range`.

### Live for Speed

- In Options > Controls, choose the wheel, set the axes, and set a force
  feedback strength (start around 50 %).
- Set the wheel turn (rotation) to match `range`.
- LFS also mainly uses a constant force, so it's a good second test.

### F1 2020

- F1 2020 has preset profiles for known wheels; for a DIY wheel you'll build a
  custom controller profile and bind steering, throttle, brake and the paddles
  yourself.
- Do this game last. Its force feedback may expect a known wheel. If `ffb`
  shows no effects, or they behave oddly, the fallback from the plan applies:
  read the game's UDP telemetry on the PC and calculate forces there. That is
  a separate PC-side project and not part of this firmware yet.

## 4. Tuning reference

| Setting | Command | Default | What it does | Raise it when |
|---|---|---|---|---|
| Max torque | `maxtorque N` | 30 % | Hard ceiling on motor power | Everything is safe and you want stronger forces |
| FFB gain | `gain N` | 70 % | Scales all game forces | Game forces feel weak (after the game's own strength) |
| Invert | `invert on/off` | off | Flips game force direction | Wheel pulls into corners |
| End stop | `endstop N` | 10 %/deg | Stiffness past full lock | You can push through the end stop too easily |
| Damping | `damping N` | 5 % | Resistance to fast turning | Wheel wobbles or oscillates on its own |
| Friction | `friction N` | 0 % | Constant drag | Wheel feels too light or "loose" |
| Min torque | `mintorque N` | 0 % | Lifts small forces over the motor's sticking point | Small details (road texture) get lost |

Rules of thumb:
- Turn the game's own FFB strength up first, then `gain`, and only then
  `maxtorque`.
- If forces "clip" (they feel flat at a maximum and details disappear in hard
  corners), turn the **game's** strength or `gain` down, not `maxtorque`.
- Change one setting at a time and drive the same corner to compare.
- `save` when you're happy.

## 5. Troubleshooting

| Symptom | Likely cause | What to do |
|---|---|---|
| Wheel not shown as "Teensy FFB Wheel" | Wrong USB Type, or core not patched | Re-run `tools/patch_teensy_core.py`, restart the IDE, pick "Serial + FFB Wheel" |
| Game says the device has no force feedback | Windows cached the old device | Unplug, uninstall the device in Device Manager, plug in again |
| `ffb` shows effects but the motor does nothing | Motor off or a fault | `settings` and `show`: is the motor on? Any fault message? `motor on` clears latched faults |
| "Motor stopped: SENSOR FAULT" | Pot reads at the very end of its range | Check the wiper wire; make sure full lock doesn't reach the pot's own end |
| "Motor stopped: PC not connected" | PC stopped reading USB reports (sleep, unplug) | Comes back by itself when the PC is back |
| Wheel shakes or buzzes at rest | Pot noise amplified by spring/min torque | Lower `mintorque`, raise `damping`, improve the pot wiring |
| Forces come in late or feel "rubbery" | Slew limit or heavy damping | Lower `damping`; `TORQUE_SLEW_PER_MS` in `config.h` is the ramp limit |
