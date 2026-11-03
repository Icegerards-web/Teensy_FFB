#!/usr/bin/env python3
"""Generates the USB HID report descriptor for the force-feedback wheel.

You only need this if you want to CHANGE the descriptor. It writes:
  core_patch/usb_ffb_desc.h              the descriptor bytes (copied into the
                                         Teensy core by patch_teensy_core.py)
  firmware/teensy_ffb/FfbReportSizes.h   the byte size of every report, so the
                                         firmware can check its parsing matches

How to read this file
---------------------
A HID report descriptor is a list of small "items". Each item is a tag byte
followed by 0, 1, 2 or 4 data bytes. For example `Usage(0x30)` (Usage X) is the
two bytes 09 30. Items either *describe* the next field (usage page, usage,
logical min/max, size in bits, count) or *create* fields (Input, Output,
Feature). Everything between Collection() and EndCollection() belongs together.

  Input   = data the device sends to the PC (axes, buttons)
  Output  = data the PC sends to the device (effect settings, start/stop)
  Feature = data either side can read/write on request (e.g. "create a new
            effect" + "which slot did you put it in?")

The force-feedback part follows the USB "Physical Interface Device" (PID)
spec. Windows' built-in PID driver reads it and turns DirectInput force
feedback calls from games into these reports, so no custom PC driver is needed.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

out = []        # list of (bytes, comment, indent)
indent = [0]


def item(tag, size_bits_value, comment, nbytes=None):
    """Append a short item. tag already contains the type/tag bits (size bits 0)."""
    v = size_bits_value
    if nbytes is None:
        if v is None:
            nbytes = 0
        elif -128 <= v <= 127 or (0 <= v <= 0xFF and tag in UNSIGNED_TAGS):
            nbytes = 1
        elif -32768 <= v <= 32767 or (0 <= v <= 0xFFFF and tag in UNSIGNED_TAGS):
            nbytes = 2
        else:
            nbytes = 4
    size_code = {0: 0, 1: 1, 2: 2, 4: 3}[nbytes]
    data = [tag | size_code]
    if nbytes:
        data += list((v & ((1 << (8 * nbytes)) - 1)).to_bytes(nbytes, "little"))
    out.append((data, comment, indent[0]))


# Tags whose values are never negative, so 0x80..0xFF can stay one byte.
UNSIGNED_TAGS = {0x04, 0x08, 0x18, 0x28, 0x74, 0x94, 0x84, 0x80, 0x90, 0xB0, 0xA0}


def UsagePage(v, name): item(0x04, v, f"Usage Page ({name})")
def Usage(v, name): item(0x08, v, f"Usage ({name})")
def Usage32(page, v, name): item(0x08, (page << 16) | v, f"Usage ({name})", nbytes=4)
def UsageMin(v): item(0x18, v, f"Usage Minimum ({v})")
def UsageMax(v): item(0x28, v, f"Usage Maximum ({v})")
def LogMin(v): item(0x14, v, f"Logical Minimum ({v})")
def LogMax(v): item(0x24, v, f"Logical Maximum ({v})")
def PhysMin(v): item(0x34, v, f"Physical Minimum ({v})")
def PhysMax(v): item(0x44, v, f"Physical Maximum ({v})")
def UnitExp(v): item(0x54, v & 0xF, f"Unit Exponent ({v})", nbytes=1)
def Unit(v, name): item(0x64, v, f"Unit ({name})", nbytes=2 if v > 0xFF else 1)
def Size(v): item(0x74, v, f"Report Size ({v})")
def Count(v): item(0x94, v, f"Report Count ({v})")
def ReportID(v): item(0x84, v, f"Report ID ({v})")


def Collection(kind):
    names = {0: "Physical", 1: "Application", 2: "Logical"}
    item(0xA0, kind, f"Collection ({names[kind]})")
    indent[0] += 1


def EndCollection():
    indent[0] -= 1
    item(0xC0, None, "End Collection")


# Main items. Flag values: bit0 Constant, bit1 Variable, bit2 Relative.
DATA_ARR_ABS = 0x00
DATA_VAR_ABS = 0x02
CNST_VAR_ABS = 0x03


def Input(f): item(0x80, f, f"Input ({flags(f)})")
def Output(f): item(0x90, f, f"Output ({flags(f)})")
def Feature(f): item(0xB0, f, f"Feature ({flags(f)})")


def flags(f):
    return ", ".join(["Constant" if f & 1 else "Data",
                      "Variable" if f & 2 else "Array",
                      "Relative" if f & 4 else "Absolute"])


# Sizes of each report, in bits, tracked as we go: (type, id) -> bits
sizes = {}
cur = {"id": None}
pending = {"size": 0, "count": 0}


def track(kind):
    key = (kind, cur["id"])
    sizes[key] = sizes.get(key, 0) + pending["size"] * pending["count"]


# Wrap Size/Count/IO so we can compute report sizes.
_Size, _Count, _ReportID = Size, Count, ReportID


def Size(v):
    pending["size"] = v
    _Size(v)


def Count(v):
    pending["count"] = v
    _Count(v)


def ReportID(v):
    cur["id"] = v
    _ReportID(v)


_Input, _Output, _Feature = Input, Output, Feature


def Input(f):
    track("input")
    _Input(f)


def Output(f):
    track("output")
    _Output(f)


def Feature(f):
    track("feature")
    _Feature(f)


PID = 0x0F
GD = 0x01
ORDINAL = 0x0A
MAX_EFFECTS = 40

# Effect types, in the order the firmware numbers them (1, 2, 3, ...).
EFFECT_TYPES = [
    (0x26, "ET Constant Force"), (0x27, "ET Ramp"), (0x30, "ET Square"),
    (0x31, "ET Sine"), (0x32, "ET Triangle"), (0x33, "ET Sawtooth Up"),
    (0x34, "ET Sawtooth Down"), (0x40, "ET Spring"), (0x41, "ET Damper"),
    (0x42, "ET Inertia"), (0x43, "ET Friction"),
]


def effect_block_index():
    Usage(0x22, "Effect Block Index")
    LogMin(1); LogMax(MAX_EFFECTS); PhysMin(1); PhysMax(MAX_EFFECTS)
    Size(8); Count(1)


def reset_phys():
    PhysMin(0); PhysMax(0)


def ms_unit():
    Unit(0x1003, "SI Linear: seconds"); UnitExp(-3)


def no_unit():
    Unit(0, "None"); UnitExp(0)


# =============================================================================
#  The descriptor
# =============================================================================
UsagePage(GD, "Generic Desktop")
Usage(0x04, "Joystick")
Collection(1)

# ---- Input report 1: the wheel, pedals and buttons ----------------------------
ReportID(1)
Usage(0x01, "Pointer")
Collection(0)
Usage(0x30, "X - steering")
LogMin(-32767); LogMax(32767)
Size(16); Count(1); Input(DATA_VAR_ABS)
Usage(0x31, "Y - throttle"); Usage(0x32, "Z - brake"); Usage(0x35, "Rz - clutch")
LogMin(0); LogMax(32767)
Size(16); Count(3); Input(DATA_VAR_ABS)
EndCollection()
UsagePage(0x09, "Button")
UsageMin(1); UsageMax(32); LogMin(0); LogMax(1)
Size(1); Count(32); Input(DATA_VAR_ABS)

# ---- Input report 2: PID State ------------------------------------------------
UsagePage(PID, "Physical Interface")
Usage(0x92, "PID State Report")
Collection(2)
ReportID(2)
Usage(0x9F, "Device Paused"); Usage(0xA0, "Actuators Enabled")
Usage(0xA4, "Safety Switch"); Usage(0xA5, "Actuator Override Switch")
Usage(0xA6, "Actuator Power")
LogMin(0); LogMax(1); PhysMin(0); PhysMax(1)
Size(1); Count(5); Input(DATA_VAR_ABS)
Count(3); Input(CNST_VAR_ABS)
Usage(0x94, "Effect Playing")
LogMin(0); LogMax(1); PhysMin(0); PhysMax(1)
Size(1); Count(1); Input(DATA_VAR_ABS)
Usage(0x22, "Effect Block Index")
LogMin(1); LogMax(MAX_EFFECTS); PhysMin(1); PhysMax(MAX_EFFECTS)
Size(7); Count(1); Input(DATA_VAR_ABS)
EndCollection()

# ---- Output report 1: Set Effect ------------------------------------------------
Usage(0x21, "Set Effect Report")
Collection(2)
ReportID(1)
effect_block_index(); Output(DATA_VAR_ABS)
Usage(0x25, "Effect Type")
Collection(2)
for u, n in EFFECT_TYPES:
    Usage(u, n)
LogMin(1); LogMax(len(EFFECT_TYPES)); PhysMin(1); PhysMax(len(EFFECT_TYPES))
Size(8); Count(1); Output(DATA_ARR_ABS)
EndCollection()
Usage(0x50, "Duration"); Usage(0x54, "Trigger Repeat Interval")
Usage(0x51, "Sample Period"); Usage(0xA7, "Start Delay")
LogMin(0); LogMax(32767); PhysMin(0); PhysMax(32767)
ms_unit()
Size(16); Count(4); Output(DATA_VAR_ABS)
no_unit()
Usage(0x52, "Gain")
LogMin(0); LogMax(255); PhysMin(0); PhysMax(10000)
Size(8); Count(1); Output(DATA_VAR_ABS)
Usage(0x53, "Trigger Button")
LogMin(1); LogMax(8); PhysMin(1); PhysMax(8)
Size(8); Count(1); Output(DATA_VAR_ABS)
Usage(0x55, "Axes Enable")
Collection(2)
Usage32(GD, 0x30, "X"); Usage32(GD, 0x31, "Y")
LogMin(0); LogMax(1); PhysMin(0); PhysMax(1)
Size(1); Count(2); Output(DATA_VAR_ABS)
EndCollection()
Usage(0x56, "Direction Enable")
Count(1); Output(DATA_VAR_ABS)
Count(5); Output(CNST_VAR_ABS)
Usage(0x57, "Direction")
Collection(2)
Usage32(ORDINAL, 1, "Ordinal 1"); Usage32(ORDINAL, 2, "Ordinal 2")
Unit(0x14, "Eng Rotation: degrees"); UnitExp(-2)
LogMin(0); LogMax(36000); PhysMin(0); PhysMax(36000)
Size(16); Count(2); Output(DATA_VAR_ABS)
no_unit()
EndCollection()
EndCollection()

# ---- Output report 2: Set Envelope ------------------------------------------------
Usage(0x5A, "Set Envelope Report")
Collection(2)
ReportID(2)
effect_block_index(); Output(DATA_VAR_ABS)
Usage(0x5B, "Attack Level"); Usage(0x5D, "Fade Level")
LogMin(0); LogMax(10000); PhysMin(0); PhysMax(10000)
Size(16); Count(2); Output(DATA_VAR_ABS)
Usage(0x5C, "Attack Time"); Usage(0x5E, "Fade Time")
LogMin(0); LogMax(32767); PhysMin(0); PhysMax(32767)
ms_unit()
Size(16); Count(2); Output(DATA_VAR_ABS)
no_unit()
EndCollection()

# ---- Output report 3: Set Condition (spring, damper, inertia, friction) ----------
Usage(0x5F, "Set Condition Report")
Collection(2)
ReportID(3)
effect_block_index(); Output(DATA_VAR_ABS)
Usage(0x23, "Parameter Block Offset")
LogMin(0); LogMax(1); PhysMin(0); PhysMax(1)
Size(4); Count(1); Output(DATA_VAR_ABS)
Count(1); Output(CNST_VAR_ABS)
Usage(0x60, "CP Offset"); Usage(0x61, "Positive Coefficient")
Usage(0x62, "Negative Coefficient")
LogMin(-10000); LogMax(10000); PhysMin(-10000); PhysMax(10000)
Size(16); Count(3); Output(DATA_VAR_ABS)
Usage(0x63, "Positive Saturation"); Usage(0x64, "Negative Saturation")
Usage(0x65, "Dead Band")
LogMin(0); LogMax(10000); PhysMin(0); PhysMax(10000)
Size(16); Count(3); Output(DATA_VAR_ABS)
EndCollection()

# ---- Output report 4: Set Periodic (sine, square, ...) -----------------------------
Usage(0x6E, "Set Periodic Report")
Collection(2)
ReportID(4)
effect_block_index(); Output(DATA_VAR_ABS)
Usage(0x70, "Magnitude")
LogMin(0); LogMax(10000); PhysMin(0); PhysMax(10000)
Size(16); Count(1); Output(DATA_VAR_ABS)
Usage(0x6F, "Offset")
LogMin(-10000); LogMax(10000); PhysMin(-10000); PhysMax(10000)
Size(16); Count(1); Output(DATA_VAR_ABS)
Usage(0x71, "Phase")
Unit(0x14, "Eng Rotation: degrees"); UnitExp(-2)
LogMin(0); LogMax(35999); PhysMin(0); PhysMax(35999)
Size(16); Count(1); Output(DATA_VAR_ABS)
Usage(0x72, "Period")
ms_unit()
LogMin(0); LogMax(32767); PhysMin(0); PhysMax(32767)
Size(16); Count(1); Output(DATA_VAR_ABS)
no_unit()
EndCollection()

# ---- Output report 5: Set Constant Force -------------------------------------------
Usage(0x73, "Set Constant Force Report")
Collection(2)
ReportID(5)
effect_block_index(); Output(DATA_VAR_ABS)
Usage(0x70, "Magnitude")
LogMin(-10000); LogMax(10000); PhysMin(-10000); PhysMax(10000)
Size(16); Count(1); Output(DATA_VAR_ABS)
EndCollection()

# ---- Output report 6: Set Ramp Force -----------------------------------------------
Usage(0x74, "Set Ramp Force Report")
Collection(2)
ReportID(6)
effect_block_index(); Output(DATA_VAR_ABS)
Usage(0x75, "Ramp Start"); Usage(0x76, "Ramp End")
LogMin(-10000); LogMax(10000); PhysMin(-10000); PhysMax(10000)
Size(16); Count(2); Output(DATA_VAR_ABS)
EndCollection()

# ---- Output report 10: Effect Operation (start / stop) ------------------------------
Usage(0x77, "Effect Operation Report")
Collection(2)
ReportID(10)
effect_block_index(); Output(DATA_VAR_ABS)
Usage(0x78, "Effect Operation")
Collection(2)
Usage(0x79, "Op Effect Start"); Usage(0x7A, "Op Effect Start Solo")
Usage(0x7B, "Op Effect Stop")
LogMin(1); LogMax(3); PhysMin(1); PhysMax(3)
Size(8); Count(1); Output(DATA_ARR_ABS)
EndCollection()
Usage(0x7C, "Loop Count")
LogMin(0); LogMax(255); PhysMin(0); PhysMax(255)
Size(8); Count(1); Output(DATA_VAR_ABS)
EndCollection()

# ---- Output report 11: PID Block Free (delete an effect) ----------------------------
Usage(0x90, "PID Block Free Report")
Collection(2)
ReportID(11)
effect_block_index(); Output(DATA_VAR_ABS)
EndCollection()

# ---- Output report 12: PID Device Control --------------------------------------------
Usage(0x96, "PID Device Control")
Collection(2)
ReportID(12)
Usage(0x97, "DC Enable Actuators"); Usage(0x98, "DC Disable Actuators")
Usage(0x99, "DC Stop All Effects"); Usage(0x9A, "DC Device Reset")
Usage(0x9B, "DC Device Pause"); Usage(0x9C, "DC Device Continue")
LogMin(1); LogMax(6); PhysMin(1); PhysMax(6)
Size(8); Count(1); Output(DATA_ARR_ABS)
EndCollection()

# ---- Output report 13: Device Gain (overall strength from the game) -----------------
Usage(0x7D, "Device Gain Report")
Collection(2)
ReportID(13)
Usage(0x7E, "Device Gain")
LogMin(0); LogMax(255); PhysMin(0); PhysMax(10000)
Size(8); Count(1); Output(DATA_VAR_ABS)
EndCollection()

# ---- Feature report 5: Create New Effect (PC -> device) ------------------------------
Usage(0xAB, "Create New Effect Report")
Collection(2)
ReportID(5)
Usage(0x25, "Effect Type")
Collection(2)
for u, n in EFFECT_TYPES:
    Usage(u, n)
LogMin(1); LogMax(len(EFFECT_TYPES)); PhysMin(1); PhysMax(len(EFFECT_TYPES))
Size(8); Count(1); Feature(DATA_ARR_ABS)
EndCollection()
UsagePage(GD, "Generic Desktop")
Usage(0x3B, "Byte Count")
LogMin(0); LogMax(511); PhysMin(0); PhysMax(511)
Size(16); Count(1); Feature(DATA_VAR_ABS)
UsagePage(PID, "Physical Interface")
EndCollection()

# ---- Feature report 6: PID Block Load (device -> PC: "your effect is in slot N") -----
Usage(0x89, "PID Block Load Report")
Collection(2)
ReportID(6)
effect_block_index(); Feature(DATA_VAR_ABS)
Usage(0x8B, "Block Load Status")
Collection(2)
Usage(0x8C, "Block Load Success"); Usage(0x8D, "Block Load Full")
Usage(0x8E, "Block Load Error")
LogMin(1); LogMax(3); PhysMin(1); PhysMax(3)
Size(8); Count(1); Feature(DATA_ARR_ABS)
EndCollection()
Usage(0xAC, "RAM Pool Available")
LogMin(0); LogMax(65535); PhysMin(0); PhysMax(65535)
Size(16); Count(1); Feature(DATA_VAR_ABS)
EndCollection()

# ---- Feature report 7: PID Pool (device -> PC: "I can hold 40 effects") --------------
Usage(0x7F, "PID Pool Report")
Collection(2)
ReportID(7)
Usage(0x80, "RAM Pool Size")
LogMin(0); LogMax(65535); PhysMin(0); PhysMax(65535)
Size(16); Count(1); Feature(DATA_VAR_ABS)
Usage(0x83, "Simultaneous Effects Max")
LogMin(0); LogMax(255); PhysMin(0); PhysMax(255)
Size(8); Count(1); Feature(DATA_VAR_ABS)
Usage(0xA9, "Device Managed Pool"); Usage(0xAA, "Shared Parameter Blocks")
LogMin(0); LogMax(1); PhysMin(0); PhysMax(1)
Size(1); Count(2); Feature(DATA_VAR_ABS)
Count(6); Feature(CNST_VAR_ABS)
EndCollection()

EndCollection()  # Joystick application collection

assert indent[0] == 0, "unbalanced collections"


# =============================================================================
#  Write the output files
# =============================================================================
def report_bytes(kind, rid):
    bits = sizes[(kind, rid)]
    assert bits % 8 == 0, f"{kind} report {rid} is {bits} bits, not whole bytes"
    return bits // 8 + 1  # +1 for the report ID byte


flat = [b for data, _, _ in out for b in data]

lines = []
lines.append("// GENERATED by tools/gen_ffb_descriptor.py - edit that file, not this one.")
lines.append("// USB HID report descriptor for the Teensy FFB wheel (joystick + PID force feedback).")
lines.append("// This file is copied into the Teensy core by tools/patch_teensy_core.py.")
lines.append(f"// {len(flat)} bytes.")
lines.append("static uint8_t joystick_report_desc[] = {")
for data, comment, ind in out:
    hexs = ", ".join(f"0x{b:02X}" for b in data) + ","
    lines.append(f"        {hexs:<34}// {'  ' * ind}{comment}")
lines.append("};")
desc_path = os.path.join(ROOT, "core_patch", "usb_ffb_desc.h")
os.makedirs(os.path.dirname(desc_path), exist_ok=True)
with open(desc_path, "w") as f:
    f.write("\n".join(lines) + "\n")

names = {
    ("input", 1): "INPUT_WHEEL", ("input", 2): "INPUT_PID_STATE",
    ("output", 1): "SET_EFFECT", ("output", 2): "SET_ENVELOPE",
    ("output", 3): "SET_CONDITION", ("output", 4): "SET_PERIODIC",
    ("output", 5): "SET_CONSTANT", ("output", 6): "SET_RAMP",
    ("output", 10): "EFFECT_OPERATION", ("output", 11): "BLOCK_FREE",
    ("output", 12): "DEVICE_CONTROL", ("output", 13): "DEVICE_GAIN",
    ("feature", 5): "CREATE_NEW_EFFECT", ("feature", 6): "BLOCK_LOAD",
    ("feature", 7): "POOL",
}
assert set(names) == set(sizes), sorted(set(sizes) ^ set(names))
h = ["// GENERATED by tools/gen_ffb_descriptor.py - edit that file, not this one.",
     "// Size in bytes (including the 1-byte report ID) of every report in the",
     "// HID descriptor. FfbEngine checks incoming reports against these.",
     "#pragma once", "",
     "namespace ffb_size {"]
for key in sorted(names, key=lambda k: (k[0], k[1])):
    kind, rid = key
    h.append(f"constexpr unsigned {names[key]} = {report_bytes(kind, rid)};  // {kind} report {rid}")
h.append(f"constexpr unsigned MAX_EFFECTS = {MAX_EFFECTS};")
h.append(f"constexpr unsigned DESCRIPTOR_BYTES = {len(flat)};")
h.append("}  // namespace ffb_size")
with open(os.path.join(ROOT, "firmware", "teensy_ffb", "FfbReportSizes.h"), "w") as f:
    f.write("\n".join(h) + "\n")

if __name__ == "__main__":
    print(f"descriptor: {len(flat)} bytes")
    for key in sorted(names, key=lambda k: (k[0], k[1])):
        print(f"  {key[0]:8} {key[1]:3} {names[key]:18} {report_bytes(*key)} bytes")
