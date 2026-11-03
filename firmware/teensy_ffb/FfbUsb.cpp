// FfbUsb.cpp - see FfbUsb.h
#include <Arduino.h>

#include "FfbReportSizes.h"
#include "FfbUsb.h"

ffb::Engine ffbEngine;

#if HAVE_FFB

static_assert(JOYSTICK_SIZE == ffb_size::INPUT_WHEEL,
              "Teensy core patch and HID descriptor disagree on the input report size. "
              "Re-run tools/patch_teensy_core.py.");

// These two are called by the patched core (usb.c) from the USB interrupt.
// "extern C" because the core is C code and looks them up by plain name.
extern "C" uint32_t ffb_get_report_callback(uint8_t type, uint8_t id, uint8_t* buf,
                                            uint32_t maxlen) {
  return ffbEngine.onGetReport(type, id, buf, maxlen);
}

extern "C" void ffb_set_report_callback(uint8_t type, uint8_t id, const uint8_t* buf,
                                        uint32_t len) {
  (void)id;  // the report ID is also the first byte of buf
  ffbEngine.onSetReport(type, buf, len, millis());
}

bool ffbSendWheelReport(int16_t steering, uint16_t throttle, uint16_t brake, uint16_t clutch,
                        uint32_t buttons) {
  // Byte layout of input report 1 (see tools/gen_ffb_descriptor.py):
  //   [0] report ID = 1
  //   [1-2] X steering   [3-4] Y throttle   [5-6] Z brake   [7-8] Rz clutch
  //   [9-12] buttons 1..32, one bit each
  uint8_t r[ffb_size::INPUT_WHEEL];
  r[0] = 1;
  r[1] = (uint16_t)steering & 0xFF;
  r[2] = (uint16_t)steering >> 8;
  r[3] = throttle & 0xFF;
  r[4] = throttle >> 8;
  r[5] = brake & 0xFF;
  r[6] = brake >> 8;
  r[7] = clutch & 0xFF;
  r[8] = clutch >> 8;
  r[9] = buttons & 0xFF;
  r[10] = (buttons >> 8) & 0xFF;
  r[11] = (buttons >> 16) & 0xFF;
  r[12] = buttons >> 24;
  memcpy(usb_joystick_data, r, sizeof(r));
  return usb_joystick_send() == 0;
}

float ffbComputeForce(const ffb::WheelState& w, uint32_t nowMs) {
  NVIC_DISABLE_IRQ(IRQ_USB1);
  float f = ffbEngine.computeForce(w, nowMs);
  NVIC_ENABLE_IRQ(IRQ_USB1);
  return f;
}

#else  // no FFB USB type: harmless stand-ins so the sketch still builds

bool ffbSendWheelReport(int16_t, uint16_t, uint16_t, uint16_t, uint32_t) { return false; }
float ffbComputeForce(const ffb::WheelState&, uint32_t) { return 0; }

#endif
