// =============================================================================
//  config.h  -  every setting you are likely to change lives in this file.
// =============================================================================
//
//  PIN NUMBERS BELOW ARE PLACEHOLDERS until the real wiring is confirmed.
//  Change a number here and re-upload; nothing else in the code needs editing.
//
//  Wiring reminders for the Teensy 4.0:
//   * It is a 3.3 V board. Power every potentiometer (steering + pedals) from
//     the 3.3V pin, NOT from 5V. 5 V on an analog pin can damage the Teensy.
//   * A pot has 3 legs: outer legs go to 3.3V and GND, the middle leg (the
//     "wiper") goes to the analog pin listed here.
//   * Buttons and paddles: one side to the pin listed here, other side to GND.
//     No resistor needed; we switch on the Teensy's internal pull-up resistor,
//     so the pin reads HIGH when released and LOW when pressed.
//   * Analog-capable pins on the Teensy 4.0 are A0-A9 (= pins 14-23).
//   * Step 2 (motor) will need two PWM pins for the BTS7960. Good candidates
//     that are kept free below: pins 8 and 9.
// =============================================================================
#pragma once
#include <stdint.h>

// ----- Steering position sensor (potentiometer for now) ----------------------
constexpr int STEERING_POT_PIN = 14;  // A0   <-- PLACEHOLDER

// How many degrees the pot shaft turns between its two ends. Most cheap pots
// are around 270-300 degrees. If the pot is geared to the wheel, put the
// number of *wheel* degrees that sweeps the pot end-to-end here.
constexpr float POT_TRAVEL_DEG = 270.0f;

// If turning the wheel right makes the steering go left in games, set true.
constexpr bool STEERING_INVERT = false;

// Default steering rotation reported to the game (lock to lock). The serial
// command "range 270" changes it at run time and "save" stores it.
constexpr float DEFAULT_STEERING_RANGE_DEG = 270.0f;

// ----- Pedals ----------------------------------------------------------------
// Use -1 for a pedal you don't have; its axis then simply stays at 0.
constexpr int THROTTLE_PIN = 15;  // A1   <-- PLACEHOLDER
constexpr int BRAKE_PIN    = 16;  // A2   <-- PLACEHOLDER
constexpr int CLUTCH_PIN   = 17;  // A3   <-- PLACEHOLDER (-1 if no clutch)

// Set true for a pedal whose reading goes DOWN when you press it.
constexpr bool THROTTLE_INVERT = false;
constexpr bool BRAKE_INVERT    = false;
constexpr bool CLUTCH_INVERT   = false;

// Dead zones, as a fraction of pedal travel (0.03 = 3 %). The low one ignores
// a resting foot or pot jitter; the high one makes sure you can hit 100 %.
constexpr float PEDAL_DEADZONE_LOW  = 0.03f;
constexpr float PEDAL_DEADZONE_HIGH = 0.03f;

// ----- Buttons and paddles ---------------------------------------------------
// The position in this list is the button number the PC sees, starting at 1:
// first entry = "Button 1", second = "Button 2", and so on (max 32).
constexpr uint8_t BUTTON_PINS[] = {
    2,  // Button 1: left shift paddle   <-- PLACEHOLDER
    3,  // Button 2: right shift paddle  <-- PLACEHOLDER
    4,  // Button 3                      <-- PLACEHOLDER
    5,  // Button 4                      <-- PLACEHOLDER
    6,  // Button 5                      <-- PLACEHOLDER
    7,  // Button 6                      <-- PLACEHOLDER
};

// A mechanical switch "bounces" (flickers on/off for a few ms) when pressed.
// A press only counts once the pin has been steady for this long.
constexpr uint32_t BUTTON_DEBOUNCE_MS = 5;

// ----- Analog reading --------------------------------------------------------
// 12-bit ADC = readings 0-4095. Averaging several samples per read removes
// most of the electrical noise (jitter) from the pots.
constexpr int ADC_BITS = 12;
constexpr int ADC_MAX = (1 << ADC_BITS) - 1;  // 4095
constexpr int ADC_AVERAGING = 8;               // 1, 4, 8, 16 or 32

// If the pot reading sits at the very ends of the ADC range, the wire to the
// wiper is probably broken. Used to flag a sensor fault (important for step 2,
// where a faulty sensor must stop the motor). Tip: build the wheel so the pot
// never quite reaches its own ends at full lock, so a real end can't look
// like a fault.
constexpr int POT_FAULT_MARGIN = 8;

// ----- USB reporting ---------------------------------------------------------
// How often we send the controller state to the PC. 1000 us = 1000 times/s.
constexpr uint32_t REPORT_INTERVAL_US = 1000;

// =============================================================================
//  STEP 2: MOTOR (BTS7960 / "IBT-2" driver board)
// =============================================================================
//  Wiring (all PLACEHOLDERS until confirmed):
//    BTS7960 RPWM  -> MOTOR_RPWM_PIN   (PWM pin: drives the motor one way)
//    BTS7960 LPWM  -> MOTOR_LPWM_PIN   (PWM pin: drives the motor the other way)
//    BTS7960 R_EN and L_EN, joined together -> MOTOR_ENABLE_PIN
//    BTS7960 VCC   -> Teensy 3.3V    BTS7960 GND -> Teensy GND
//    BTS7960 B+/B- -> motor power supply,  M+/M- -> motor
//  Put a 10k resistor from the joined R_EN/L_EN wire to GND. Then, while the
//  Teensy is booting or being re-programmed (pins floating), the driver stays
//  switched OFF instead of possibly turning on.
// =============================================================================
constexpr int MOTOR_RPWM_PIN   = 8;   // <-- PLACEHOLDER
constexpr int MOTOR_LPWM_PIN   = 9;   // <-- PLACEHOLDER
constexpr int MOTOR_ENABLE_PIN = 10;  // <-- PLACEHOLDER

// If positive torque turns the wheel LEFT instead of right, set true
// (or swap the two motor wires). Check with the serial command "motor test".
constexpr bool MOTOR_INVERT = false;

// 20 kHz is above human hearing, so the motor doesn't whine.
constexpr float MOTOR_PWM_FREQUENCY_HZ = 20000.0f;
constexpr int MOTOR_PWM_BITS = 12;  // duty cycle 0..4095

// Optional emergency stop button (to GND). Pressing it cuts the motor until
// you type "motor on" again. -1 = no button.
constexpr int ESTOP_PIN = -1;  // <-- PLACEHOLDER

// ----- Safety defaults (each can be changed at run time and saved) ------------
// Hard ceiling on motor power as a fraction of full power (0.30 = 30 %).
// START LOW. Raise it with "maxtorque N" once everything behaves.
constexpr float DEFAULT_MAX_TORQUE = 0.30f;

// End stop: past the steering range (see "range"), a stiff virtual spring
// pushes the wheel back. Torque per degree past the end, as a fraction.
constexpr float DEFAULT_ENDSTOP_STIFFNESS = 0.10f;  // 10 % per degree

// Biggest torque change allowed per millisecond. Prevents violent jolts:
// 0.05 means going from 0 to 100 % takes at least 20 ms.
constexpr float TORQUE_SLEW_PER_MS = 0.05f;

// If the USB host (the PC) stops reading our reports for this long, the
// motor is switched off. Happens on unplug, PC sleep or a hung driver.
constexpr uint32_t HOST_TIMEOUT_MS = 100;

// If the main loop stops running for this long (a firmware bug or crash),
// a timer interrupt switches the motor off by itself.
constexpr uint32_t LOOP_STALL_TIMEOUT_MS = 50;  // > the 30 ms USB send timeout

// The steering sensor must look broken for this long before it counts as a
// fault (avoids reacting to a single noisy reading).
constexpr uint32_t SENSOR_FAULT_MS = 10;

// =============================================================================
//  STEPS 3-4: FORCE FEEDBACK DEFAULTS (all changeable via the serial menu)
// =============================================================================
constexpr float DEFAULT_FFB_GAIN   = 0.70f;  // overall strength of game forces
constexpr float DEFAULT_DAMPING    = 0.05f;  // always-on damping
constexpr float DEFAULT_FRICTION   = 0.00f;  // always-on friction
constexpr float DEFAULT_MIN_TORQUE = 0.00f;  // overcome motor/gear stiction
