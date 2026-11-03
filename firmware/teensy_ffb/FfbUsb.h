// =============================================================================
//  FfbUsb.h  -  connects the FFB engine to the (patched) Teensy USB code.
// =============================================================================
//
//  Only used with the "Serial + FFB Wheel" USB type (USB_FFB_WHEEL), which
//  tools/patch_teensy_core.py adds to Teensyduino. With that USB type:
//   - the PC's force-feedback requests arrive in ffb_set_report_callback() /
//     ffb_get_report_callback() (FfbUsb.cpp), inside the USB interrupt, and
//     go straight to the engine;
//   - the wheel/pedal/button state goes out as input report 1 (13 bytes).
// =============================================================================
#pragma once
#include <stdint.h>

#include "FfbEngine.h"

#if defined(USB_FFB_WHEEL)
#define HAVE_FFB 1
#else
#define HAVE_FFB 0
#endif

extern ffb::Engine ffbEngine;

// Sends input report 1. Returns true if the PC took it.
bool ffbSendWheelReport(int16_t steering, uint16_t throttle, uint16_t brake, uint16_t clutch,
                        uint32_t buttons);

// Runs the engine with the USB interrupt paused, so a report arriving halfway
// through can't leave an effect half-updated.
float ffbComputeForce(const ffb::WheelState& w, uint32_t nowMs);
