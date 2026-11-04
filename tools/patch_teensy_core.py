#!/usr/bin/env python3
"""Adds the "Serial + FFB Wheel" USB type to your Teensyduino installation.

Why this is needed
------------------
The USB descriptors (how the Teensy introduces itself to the PC) live inside
the Teensy "core", the library Teensyduino compiles into every sketch. A
sketch can't replace them, and force feedback needs a different descriptor
plus code that answers the PC's force-feedback requests. So we add a new USB
type to the core. All changes are wrapped in `#if defined(USB_FFB_WHEEL)`, so
every other USB type (and every other project) is completely unaffected.

What it changes
---------------
  cores/teensy4/usb_desc.h     new USB_FFB_WHEEL section (serial + joystick)
  cores/teensy4/usb_desc.c     uses our report descriptor, bigger send buffer
  cores/teensy4/usb.c          forwards HID SET_REPORT / GET_REPORT requests on
                               the joystick interface to the sketch
  cores/teensy4/usb_ffb_desc.h copied from core_patch/ (the descriptor itself)
  boards.txt                   adds "Serial + FFB Wheel" to Tools > USB Type
Each original file is saved once as <name>.orig before the first change.

Usage
-----
  python tools/patch_teensy_core.py            (searches the usual places)
  python tools/patch_teensy_core.py <path to the teensy4 core folder>

It is safe to run again (already-patched files are skipped, except the
descriptor which is refreshed). Re-run it after updating Teensyduino.
PlatformIO users don't need to run it: platformio.ini does it automatically.
"""
import glob
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MARK = "TEENSY_FFB_PATCH"

USB_DESC_H_BLOCK = """#elif defined(USB_FFB_WHEEL)  // TEENSY_FFB_PATCH: serial + force-feedback wheel
  #define VENDOR_ID		0x16C0
  #define PRODUCT_ID		0x04F0
  #define MANUFACTURER_NAME	{'T','e','e','n','s','y','d','u','i','n','o'}
  #define MANUFACTURER_NAME_LEN	11
  #define PRODUCT_NAME		{'T','e','e','n','s','y',' ','F','F','B',' ','W','h','e','e','l'}
  #define PRODUCT_NAME_LEN	16
  #define EP0_SIZE		64
  #define NUM_ENDPOINTS		5
  #define NUM_INTERFACE		3
  #define CDC_IAD_DESCRIPTOR	1
  #define CDC_STATUS_INTERFACE	0
  #define CDC_DATA_INTERFACE	1	// Serial
  #define CDC_ACM_ENDPOINT	2
  #define CDC_RX_ENDPOINT       3
  #define CDC_TX_ENDPOINT       4
  #define CDC_ACM_SIZE          16
  #define CDC_RX_SIZE_480       512
  #define CDC_TX_SIZE_480       512
  #define CDC_RX_SIZE_12        64
  #define CDC_TX_SIZE_12        64
  #define JOYSTICK_INTERFACE    2	// Wheel + force feedback
  #define JOYSTICK_ENDPOINT     5
  #define JOYSTICK_SIZE         13	// input report 1, see core_patch/usb_ffb_desc.h
  #define JOYSTICK_INTERVAL     1
  #define ENDPOINT2_CONFIG	ENDPOINT_RECEIVE_UNUSED + ENDPOINT_TRANSMIT_INTERRUPT
  #define ENDPOINT3_CONFIG	ENDPOINT_RECEIVE_BULK + ENDPOINT_TRANSMIT_UNUSED
  #define ENDPOINT4_CONFIG	ENDPOINT_RECEIVE_UNUSED + ENDPOINT_TRANSMIT_BULK
  #define ENDPOINT5_CONFIG	ENDPOINT_RECEIVE_UNUSED + ENDPOINT_TRANSMIT_INTERRUPT

"""

USB_C_GLOBALS = """
#if defined(USB_FFB_WHEEL) // TEENSY_FFB_PATCH
// Force feedback: the PC sends effect settings with HID SET_REPORT and asks
// for replies with GET_REPORT, both on endpoint 0. The sketch implements
// these two functions (see FfbUsb.cpp); the weak versions here do nothing.
static uint8_t ffb_ep0_buffer[64] __attribute__ ((aligned(32)));
uint32_t ffb_get_report_callback(uint8_t type, uint8_t id, uint8_t *buf, uint32_t maxlen) __attribute__((weak));
uint32_t ffb_get_report_callback(uint8_t type, uint8_t id, uint8_t *buf, uint32_t maxlen) { return 0; }
void ffb_set_report_callback(uint8_t type, uint8_t id, const uint8_t *buf, uint32_t len) __attribute__((weak));
void ffb_set_report_callback(uint8_t type, uint8_t id, const uint8_t *buf, uint32_t len) { }
#endif
"""

USB_C_SETUP_CASES = """
#if defined(USB_FFB_WHEEL) // TEENSY_FFB_PATCH
	  case 0x0921: // HID SET_REPORT: receive the data, then ffb_set_report_callback()
		if (setup.wIndex == JOYSTICK_INTERFACE && setup.wLength > 0
		  && setup.wLength <= sizeof(ffb_ep0_buffer)) {
			endpoint0_setupdata.bothwords = setup.bothwords;
			endpoint0_receive(ffb_ep0_buffer, setup.wLength, 1);
			return;
		}
		break;
	  case 0x01A1: // HID GET_REPORT: ask the sketch for the reply
		if (setup.wIndex == JOYSTICK_INTERFACE) {
			uint32_t n = ffb_get_report_callback(setup.wValue >> 8, setup.wValue & 0xFF,
				ffb_ep0_buffer, sizeof(ffb_ep0_buffer));
			if (n > setup.wLength) n = setup.wLength;
			if (n > 0) {
				endpoint0_transmit(ffb_ep0_buffer, n, 0);
				return;
			}
		}
		break;
	  case 0x0A21: // HID SET_IDLE: accept and ignore
		if (setup.wIndex == JOYSTICK_INTERFACE) {
			endpoint0_receive(NULL, 0, 0);
			return;
		}
		break;
#endif
"""

USB_C_COMPLETE = """
#if defined(USB_FFB_WHEEL) // TEENSY_FFB_PATCH
	if (setup.wRequestAndType == 0x0921 && setup.wIndex == JOYSTICK_INTERFACE) {
		ffb_set_report_callback(setup.wValue >> 8, setup.wValue & 0xFF,
			ffb_ep0_buffer, setup.wLength);
	}
#endif
"""


class PatchError(Exception):
    pass


def insert_once(text, anchor, block, after=True, fname=""):
    if text.count(anchor) < 1:
        raise PatchError(f"{fname}: could not find '{anchor.strip()}'. "
                         "Your Teensyduino version may be too different; please report it.")
    i = text.index(anchor)
    pos = i + len(anchor) if after else i
    return text[:pos] + block + text[pos:]


def patch_usb_desc_h(text):
    return insert_once(text, "#elif defined(USB_SERIAL_HID)", USB_DESC_H_BLOCK,
                       after=False, fname="usb_desc.h")


def patch_usb_desc_c(text):
    # The stock line is "#ifdef JOYSTICK_INTERFACE", but cores that were
    # modified before (e.g. "#if defined(JOYSTICK_INTERFACE) && !defined(X)")
    # are accepted too, as long as the joystick descriptor block follows.
    m = re.search(r"^[ \t]*#[ \t]*(?:ifdef[ \t]+JOYSTICK_INTERFACE|if[ \t]+defined[ \t]*\(?[ \t]*"
                  r"JOYSTICK_INTERFACE\b[^\n]*)[ \t]*\n[ \t]*#[ \t]*if[ \t]+JOYSTICK_SIZE[ \t]*==[ \t]*12",
                  text, re.M)
    if not m:
        raise PatchError("usb_desc.c: joystick report descriptor not found")
    start = m.start()
    end_m = re.compile(r"#[ \t]*endif[ \t]*//[ \t]*JOYSTICK_INTERFACE[^\n]*").search(text, start)
    if not end_m:
        raise PatchError("usb_desc.c: end of the joystick report descriptor not found")
    end = end_m.end()
    text = (text[:start]
            + "#if defined(USB_FFB_WHEEL) // TEENSY_FFB_PATCH\n#include \"usb_ffb_desc.h\"\n#else\n"
            + text[start:end]
            + "\n#endif // USB_FFB_WHEEL (TEENSY_FFB_PATCH)"
            + text[end:])
    buf = "uint8_t usb_descriptor_buffer[CONFIG_DESC_SIZE];"
    if buf not in text:
        raise PatchError("usb_desc.c: usb_descriptor_buffer not found")
    # The report descriptor (~1 KB) is copied into this buffer before sending,
    # so it must be at least that big.
    text = text.replace(buf, (
        "#if defined(USB_FFB_WHEEL) // TEENSY_FFB_PATCH\n"
        "uint8_t usb_descriptor_buffer[sizeof(joystick_report_desc) > CONFIG_DESC_SIZE ? "
        "sizeof(joystick_report_desc) : CONFIG_DESC_SIZE];\n"
        "#else\n" + buf + "\n#endif"), 1)
    return text


def patch_usb_c(text):
    text = insert_once(text, "static uint8_t reply_buffer[8];\n", USB_C_GLOBALS, fname="usb.c")
    text = insert_once(text, "\tswitch (setup.wRequestAndType) {\n", USB_C_SETUP_CASES,
                       fname="usb.c")
    anchor = "static void endpoint0_complete(void)\n{\n\tsetup_t setup;\n\n\tsetup.bothwords = endpoint0_setupdata.bothwords;\n"
    text = insert_once(text, anchor, USB_C_COMPLETE, fname="usb.c")
    return text


def prepare_patch(path, fn):
    """Works out the patched text without writing anything. Returns the new
    text, or None if the file is already patched. Raises PatchError."""
    with open(path, newline="") as f:
        text = f.read()
    if MARK in text:
        print(f"  already patched: {path}")
        return None
    crlf = "\r\n" in text
    text = text.replace("\r\n", "\n")
    new = fn(text)
    if crlf:
        new = new.replace("\n", "\r\n")
    return new


def write_patch(path, new):
    if not os.path.exists(path + ".orig"):
        shutil.copy2(path, path + ".orig")
    with open(path, "w", newline="") as f:
        f.write(new)
    print(f"  patched: {path}")


def patch_boards_txt(path):
    with open(path, newline="") as f:
        text = f.read()
    if MARK in text:
        print(f"  already patched: {path}")
        return
    nl = "\r\n" if "\r\n" in text else "\n"
    lines = text.replace("\r\n", "\n").split("\n")
    add = [f"# {MARK}: Serial + FFB Wheel USB type for the Teensy 4.0"]
    for line in lines:
        if line.startswith("teensy40.menu.usb.serialhid"):
            add.append(line.replace("teensy40.menu.usb.serialhid", "teensy40.menu.usb.ffbwheel", 1)
                       .replace("USB_SERIAL_HID", "USB_FFB_WHEEL"))
    if len(add) == 1:
        print(f"  note: no Teensy 4.0 'serialhid' entry in {path}; skipped the menu entry")
        return
    # the menu label line is "teensy40.menu.usb.ffbwheel=<label>"
    add = [("teensy40.menu.usb.ffbwheel=Serial + FFB Wheel"
            if l.startswith("teensy40.menu.usb.ffbwheel=") else l) for l in add]
    if not os.path.exists(path + ".orig"):
        shutil.copy2(path, path + ".orig")
    with open(path, "a", newline="") as f:
        f.write(nl + nl.join(add) + nl)
    print(f"  patched: {path}  (restart the Arduino IDE to see the new USB Type)")


def find_cores():
    home = os.path.expanduser("~")
    patterns = [
        # Arduino IDE 2 / Boards Manager installs
        os.path.join(os.environ.get("LOCALAPPDATA", ""), "Arduino15", "packages", "teensy",
                     "hardware", "avr", "*", "cores", "teensy4"),
        os.path.join(home, "AppData", "Local", "Arduino15", "packages", "teensy", "hardware",
                     "avr", "*", "cores", "teensy4"),
        os.path.join(home, "Library", "Arduino15", "packages", "teensy", "hardware", "avr", "*",
                     "cores", "teensy4"),
        os.path.join(home, ".arduino15", "packages", "teensy", "hardware", "avr", "*", "cores",
                     "teensy4"),
        # PlatformIO
        os.path.join(home, ".platformio", "packages", "framework-arduinoteensy", "cores",
                     "teensy4"),
    ]
    found = []
    for p in patterns:
        found += [d for d in glob.glob(p) if os.path.isdir(d)]
    return sorted(set(found))


def patch_core(core):
    print(f"Patching {core}")
    # Work out all three changes first, and only write if every one of them
    # succeeds, so a core is never left half-patched.
    jobs = [("usb_desc.h", patch_usb_desc_h), ("usb_desc.c", patch_usb_desc_c),
            ("usb.c", patch_usb_c)]
    results = []
    for name, fn in jobs:
        path = os.path.join(core, name)
        results.append((path, prepare_patch(path, fn)))
    shutil.copy2(os.path.join(ROOT, "core_patch", "usb_ffb_desc.h"),
                 os.path.join(core, "usb_ffb_desc.h"))
    print(f"  copied: usb_ffb_desc.h")
    for path, new in results:
        if new is not None:
            write_patch(path, new)
    boards = os.path.join(os.path.dirname(os.path.dirname(core)), "boards.txt")
    if os.path.exists(boards):
        patch_boards_txt(boards)


def main(argv):
    cores = argv[1:] or find_cores()
    if not cores:
        print("Could not find a Teensyduino core folder. Pass its path, e.g.\n"
              "  python tools/patch_teensy_core.py "
              "\"C:/Users/<you>/AppData/Local/Arduino15/packages/teensy/hardware/avr/1.59.0/cores/teensy4\"")
        return 1
    failed = False
    for c in cores:
        try:
            patch_core(c)
        except PatchError as e:
            print(f"  ERROR: {e}\n  (nothing was changed in this core)")
            failed = True
    if failed:
        return 1
    print("Done. In the Arduino IDE choose Tools > USB Type > 'Serial + FFB Wheel'.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
