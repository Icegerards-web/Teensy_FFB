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
