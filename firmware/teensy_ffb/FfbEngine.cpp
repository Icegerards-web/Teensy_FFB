// FfbEngine.cpp - see FfbEngine.h for how this fits together.
#include "FfbEngine.h"

#include <math.h>

namespace ffb {

namespace {

// Reports arrive as raw bytes, least significant byte first ("little endian").
inline uint16_t u16(const uint8_t* d, int i) { return (uint16_t)(d[i] | (d[i + 1] << 8)); }
inline int16_t i16(const uint8_t* d, int i) { return (int16_t)u16(d, i); }

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

constexpr float PI_F = 3.14159265f;

bool isCondition(uint8_t t) {
  return t == ET_SPRING || t == ET_DAMPER || t == ET_INERTIA || t == ET_FRICTION;
}

bool isPeriodic(uint8_t t) { return t >= ET_SQUARE && t <= ET_SAW_DOWN; }

// The standard "condition" formula from the PID spec. `metric` is the
// quantity the effect reacts to (position for a spring, speed for a damper),
// scaled to -10000..10000. Returns the force in -10000..10000 that the
// condition produces; the caller flips the sign so it *resists* the metric.
float conditionForce(const Condition& c, float metric) {
  float lo = (float)c.cpOffset - c.deadBand;
  float hi = (float)c.cpOffset + c.deadBand;
  // A saturation of 0 would mean "no force at all"; games that leave it at 0
  // mean "no limit", so treat it as the maximum.
  float posSat = c.posSat ? c.posSat : 10000.0f;
  float negSat = c.negSat ? c.negSat : 10000.0f;
  if (metric > hi) {
    float f = (float)c.posCoef * (metric - hi) / 10000.0f;
    return clampf(f, -posSat, posSat);
  }
  if (metric < lo) {
    float f = (float)c.negCoef * (metric - lo) / 10000.0f;
    return clampf(f, -negSat, negSat);
  }
  return 0;
}

}  // namespace

void Engine::reset() {
  for (Effect& e : effects_) e = Effect();
  // Actuators start enabled: DirectInput doesn't always send "enable" after
  // a reset. Our own arm switch ("motor on") is the real safety switch.
  actuatorsEnabled_ = true;
  paused_ = false;
  deviceGain_ = 255;
  lastBlockIndex_ = 0;
  lastBlockStatus_ = 3;
}

Effect* Engine::get(uint8_t blockIndex) {
  if (blockIndex < 1 || blockIndex > ffb_size::MAX_EFFECTS) return nullptr;
  Effect* e = &effects_[blockIndex - 1];
  return e->type == ET_NONE ? nullptr : e;
}

void Engine::createEffect(uint8_t type) {
  if (type == ET_NONE || type >= ET_COUNT) {
    lastBlockIndex_ = 0;
    lastBlockStatus_ = 3;  // error
    return;
  }
  for (unsigned i = 0; i < ffb_size::MAX_EFFECTS; i++) {
    if (effects_[i].type == ET_NONE) {
      effects_[i] = Effect();
      effects_[i].type = type;
      lastBlockIndex_ = (uint8_t)(i + 1);
      lastBlockStatus_ = 1;  // success
      return;
    }
  }
  lastBlockIndex_ = 0;
  lastBlockStatus_ = 2;  // full
}

void Engine::startEffect(Effect& e, uint8_t loops, uint32_t nowMs) {
  e.playing = true;
  e.loopCount = loops == 0 ? 1 : loops;  // 255 = forever
  e.startMs = nowMs + e.startDelayMs;
}

void Engine::onSetReport(uint8_t type, const uint8_t* d, uint32_t len, uint32_t nowMs) {
  if (len < 1) return;
  reportCount_++;
  uint8_t id = d[0];

  if (type == report::TYPE_FEATURE) {
    if (id == report::CREATE_NEW_EFFECT && len >= 2) createEffect(d[1]);
    return;
  }
  if (type != report::TYPE_OUTPUT) return;

  switch (id) {
    case report::SET_EFFECT: {
      if (len < ffb_size::SET_EFFECT) return;
      Effect* e = get(d[1]);
      if (!e) return;
      if (d[2] != ET_NONE && d[2] < ET_COUNT) e->type = d[2];
      e->durationMs = u16(d, 3);
      // d[5..6] trigger repeat interval, d[7..8] sample period: not used
      e->startDelayMs = u16(d, 9);
      e->gain = d[11];
      // d[12] trigger button: not used
      e->axesEnable = d[13] & 0x03;
      e->directionEnable = (d[13] & 0x04) != 0;
      e->direction = u16(d, 14);
      break;
    }
    case report::SET_ENVELOPE: {
      if (len < ffb_size::SET_ENVELOPE) return;
      Effect* e = get(d[1]);
      if (!e) return;
      e->attackLevel = u16(d, 2);
      e->fadeLevel = u16(d, 4);
      e->attackTimeMs = u16(d, 6);
      e->fadeTimeMs = u16(d, 8);
      break;
    }
    case report::SET_CONDITION: {
      if (len < ffb_size::SET_CONDITION) return;
      Effect* e = get(d[1]);
      if (!e) return;
      if ((d[2] & 0x0F) != 0) return;  // block 1 = Y axis: a wheel has no Y
      Condition& c = e->condition;
      c.cpOffset = i16(d, 3);
      c.posCoef = i16(d, 5);
      c.negCoef = i16(d, 7);
      c.posSat = u16(d, 9);
      c.negSat = u16(d, 11);
      c.deadBand = u16(d, 13);
      break;
    }
    case report::SET_PERIODIC: {
      if (len < ffb_size::SET_PERIODIC) return;
      Effect* e = get(d[1]);
      if (!e) return;
      e->periodicMagnitude = u16(d, 2);
      e->periodicOffset = i16(d, 4);
      e->phase = u16(d, 6);
      e->periodMs = u16(d, 8);
      break;
    }
    case report::SET_CONSTANT: {
      if (len < ffb_size::SET_CONSTANT) return;
      Effect* e = get(d[1]);
      if (!e) return;
      e->magnitude = i16(d, 2);
      break;
    }
    case report::SET_RAMP: {
      if (len < ffb_size::SET_RAMP) return;
      Effect* e = get(d[1]);
      if (!e) return;
      e->rampStart = i16(d, 2);
      e->rampEnd = i16(d, 4);
      break;
    }
    case report::EFFECT_OPERATION: {
      if (len < ffb_size::EFFECT_OPERATION) return;
      Effect* e = get(d[1]);
      if (!e) return;
      uint8_t op = d[2];
      if (op == 1) {  // start
        startEffect(*e, d[3], nowMs);
      } else if (op == 2) {  // start solo: stop everything else first
        for (Effect& other : effects_) other.playing = false;
        startEffect(*e, d[3], nowMs);
      } else if (op == 3) {  // stop
        e->playing = false;
      }
      break;
    }
    case report::BLOCK_FREE: {
      if (len < ffb_size::BLOCK_FREE) return;
      if (d[1] == 0xFF) {  // some drivers use 0xFF for "all"
        for (Effect& e : effects_) e = Effect();
      } else if (Effect* e = get(d[1])) {
        *e = Effect();
      }
      break;
    }
    case report::DEVICE_CONTROL: {
      if (len < ffb_size::DEVICE_CONTROL) return;
      switch (d[1]) {
        case 1: actuatorsEnabled_ = true; break;   // enable actuators
        case 2: actuatorsEnabled_ = false; break;  // disable actuators
        case 3:                                    // stop all effects
          for (Effect& e : effects_) e.playing = false;
          break;
        case 4: reset(); break;                    // device reset
        case 5: paused_ = true; break;             // pause
        case 6: paused_ = false; break;            // continue
      }
      break;
    }
    case report::DEVICE_GAIN: {
      if (len < ffb_size::DEVICE_GAIN) return;
      deviceGain_ = d[1];
      break;
    }
  }
}

uint32_t Engine::onGetReport(uint8_t type, uint8_t id, uint8_t* buf, uint32_t maxLen) {
  if (type != report::TYPE_FEATURE) return 0;
  if (id == report::BLOCK_LOAD && maxLen >= ffb_size::BLOCK_LOAD) {
    unsigned freeSlots = 0;
    for (const Effect& e : effects_) freeSlots += (e.type == ET_NONE);
    uint16_t available = (uint16_t)(freeSlots * 0xFFFFu / ffb_size::MAX_EFFECTS);
    buf[0] = report::BLOCK_LOAD;
    buf[1] = lastBlockIndex_;
    buf[2] = lastBlockStatus_;
    buf[3] = available & 0xFF;
    buf[4] = available >> 8;
    return ffb_size::BLOCK_LOAD;
  }
  if (id == report::POOL && maxLen >= ffb_size::POOL) {
    buf[0] = report::POOL;
    buf[1] = 0xFF;  // RAM pool size (just "plenty")
    buf[2] = 0xFF;
    buf[3] = ffb_size::MAX_EFFECTS;  // simultaneous effects
    buf[4] = 0x01;  // bit0: device managed pool = yes, bit1: shared blocks = no
    return ffb_size::POOL;
  }
  return 0;
}

int Engine::playingCount() const {
  int n = 0;
  for (const Effect& e : effects_) n += (e.type != ET_NONE && e.playing);
  return n;
}

int Engine::allocatedCount() const {
  int n = 0;
  for (const Effect& e : effects_) n += (e.type != ET_NONE);
  return n;
}

float Engine::effectForce(Effect& e, const WheelState& w, uint32_t nowMs) {
  if (e.type == ET_NONE || !e.playing) return 0;
  int32_t t = (int32_t)(nowMs - e.startMs);
  if (t < 0) return 0;  // still in its start delay

  bool infinite = e.durationMs == DURATION_INFINITE || e.durationMs == 0;
  if (!infinite) {
    // Loop count: play the duration this many times (255 = forever).
    if (t >= (int32_t)e.durationMs) {
      if (e.loopCount == 255 || e.loopCount > 1) {
        if (e.loopCount != 255) e.loopCount--;
        e.startMs += e.durationMs;
        t -= e.durationMs;
      } else {
        e.playing = false;
        return 0;
      }
    }
  }

  float force = 0;  // -10000..10000

  if (isCondition(e.type)) {
    // Only the X axis exists on a wheel.
    if (!(e.axesEnable & 0x01) && !e.directionEnable) return 0;
    float metric = 0;
    switch (e.type) {
      case ET_SPRING: metric = w.position * 10000.0f; break;
      case ET_DAMPER: metric = w.velocity * 10000.0f; break;
      case ET_INERTIA: metric = w.acceleration * 10000.0f; break;
      case ET_FRICTION:
        // Friction: a constant drag against the direction of motion. Scaled
        // so it fades in over the first few % of speed (no buzz at rest).
        metric = clampf(w.velocity * 50.0f, -1.0f, 1.0f) * 10000.0f;
        break;
    }
    if (e.type == ET_FRICTION) {
      // Friction has no center point; only the coefficients matter.
      Condition c = e.condition;
      c.cpOffset = 0;
      c.deadBand = 0;
      force = -conditionForce(c, metric);
    } else {
      force = -conditionForce(e.condition, metric);
    }
  } else {
    // ---- Effects whose force level comes from the game ------------------------
    float level = 0;  // signed force before the envelope
    float envMag = 0; // the magnitude the envelope shapes
    if (e.type == ET_CONSTANT) {
      level = e.magnitude;
      envMag = fabsf(level);
    } else if (e.type == ET_RAMP) {
      float frac = infinite ? 0.0f : (float)t / (float)e.durationMs;
      level = e.rampStart + (e.rampEnd - e.rampStart) * frac;
      envMag = fabsf(level);
    } else if (isPeriodic(e.type)) {
      envMag = e.periodicMagnitude;
    }

    // Envelope: attack ramps the strength in, fade ramps it out at the end.
    float shaped = envMag;
    if (e.attackTimeMs > 0 && t < e.attackTimeMs) {
      shaped = e.attackLevel + (envMag - e.attackLevel) * (float)t / e.attackTimeMs;
    } else if (!infinite && e.fadeTimeMs > 0 && t > (int32_t)e.durationMs - e.fadeTimeMs) {
      float remaining = (float)((int32_t)e.durationMs - t);
      shaped = e.fadeLevel + (envMag - e.fadeLevel) * remaining / e.fadeTimeMs;
    }

    if (isPeriodic(e.type)) {
      float period = e.periodMs ? (float)e.periodMs : 1.0f;
      float cycle = (float)t / period + (float)e.phase / 36000.0f;
      cycle -= floorf(cycle);  // 0..1 through the wave
      float wave = 0;
      switch (e.type) {
        case ET_SQUARE: wave = cycle < 0.5f ? 1.0f : -1.0f; break;
        case ET_SINE: wave = sinf(2.0f * PI_F * cycle); break;
        case ET_TRIANGLE:
          wave = cycle < 0.25f ? 4.0f * cycle
               : cycle < 0.75f ? 2.0f - 4.0f * cycle
                               : 4.0f * cycle - 4.0f;
          break;
        case ET_SAW_UP: wave = 2.0f * cycle - 1.0f; break;
        case ET_SAW_DOWN: wave = 1.0f - 2.0f * cycle; break;
      }
      force = e.periodicOffset + shaped * wave;
    } else {
      force = level < 0 ? -shaped : shaped;
    }

    // Direction: the share of this force that lands on the wheel's X axis.
    // In polar form 90 deg (9000) means "towards +X", 270 deg towards -X.
    float axisShare;
    if (e.directionEnable) {
      axisShare = sinf((float)e.direction * (2.0f * PI_F / 36000.0f));
    } else {
      axisShare = (e.axesEnable & 0x01) ? 1.0f : 0.0f;
    }
    force *= axisShare;
    if (settings.invertDirectional) force = -force;
  }

  return force / 10000.0f * ((float)e.gain / 255.0f);
}

float Engine::computeForce(const WheelState& w, uint32_t nowMs) {
  float total = 0;
  for (Effect& e : effects_) {
    e.lastForce = effectForce(e, w, nowMs);
    total += e.lastForce;
  }
  if (paused_ || !actuatorsEnabled_) return 0;
  total *= (float)deviceGain_ / 255.0f;
  return clampf(total, -1.0f, 1.0f);
}

}  // namespace ffb
