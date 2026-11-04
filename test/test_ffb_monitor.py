"""Tests for the parsing part of tools/ffb_monitor.py (no window needed).

    python test/test_ffb_monitor.py
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
try:
    import ffb_monitor as m
except ImportError as e:  # e.g. Python without tkinter
    print(f"skipped: {e}")
    sys.exit(0)

# Lines exactly as printStream() in teensy_ffb.ino prints them.
FIRMWARE_LINES = [
    "Motor ON. Type 'motor off' to disable.",
    "@S t=12345 ffb=1 ang=-12.50 pos=-0.0926 vel=3.2 game=0.1500 req=0.1400 out=0.1400 "
    "armed=1 state=0 act=1 paused=0 dgain=255 reports=1500",
    "@E i=1 type=1 play=1 gain=255 f=0.2000",
    "@E i=2 type=8 play=0 gain=128 f=0.0000",
    "@END",
]


class ParserTest(unittest.TestCase):
    def test_snapshot(self):
        text = []
        p = m.StreamParser(on_text=text.append)
        snaps = [s for s in (p.feed(l) for l in FIRMWARE_LINES) if s]
        self.assertEqual(len(snaps), 1)
        s = snaps[0]
        self.assertEqual(s["state"]["reports"], 1500)
        self.assertAlmostEqual(s["state"]["ang"], -12.5)
        self.assertEqual(s["state"]["ffb"], 1)
        self.assertEqual([e["type"] for e in s["effects"]], [1, 8])
        self.assertEqual(s["effects"][1]["play"], 0)
        self.assertEqual(text, ["Motor ON. Type 'motor off' to disable."])

    def test_partial_snapshot_is_dropped(self):
        p = m.StreamParser()
        # An @E or @END without a preceding @S (e.g. we connected mid-snapshot).
        self.assertIsNone(p.feed("@E i=1 type=1 play=1 gain=255 f=0.1"))
        self.assertIsNone(p.feed("@END"))
        self.assertIsNotNone(p.feed("@S reports=1") or p.feed("@END"))

    def test_activity(self):
        a = m.ActivityTracker(window=1.0)
        a.update(0.0, 100)
        a.update(0.5, 150)
        a.update(1.0, 200)
        self.assertAlmostEqual(a.rate(), 100.0)
        a.update(1.5, 200)  # game went quiet
        a.update(2.5, 200)
        self.assertAlmostEqual(a.rate(), 0.0)
        a.update(3.0, 5)  # Teensy restarted
        self.assertEqual(a.rate(), 0.0)

    def test_demo_speaks_same_format(self):
        d = m.DemoSource()
        p = m.StreamParser()
        snap = None
        for _ in range(20):
            snap = p.feed(d.readline()) or snap
        self.assertIsNotNone(snap)
        for key in ("ffb", "game", "req", "out", "reports", "state", "dgain", "ang"):
            self.assertIn(key, snap["state"])


if __name__ == "__main__":
    unittest.main()
