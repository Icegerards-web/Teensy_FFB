#!/usr/bin/env python3
"""FFB Monitor: a small window that shows what force feedback games send.

Run it on the PC the wheel is plugged into:

    pip install pyserial          (once)
    python tools/ffb_monitor.py

Close the Arduino Serial Monitor first: only one program at a time can use
the Teensy's serial port. Try it without a wheel with:

    python tools/ffb_monitor.py --demo

How it works
------------
The window opens the Teensy's USB serial port and types "stream on", the
same as you could in the Serial Monitor. The firmware then sends a snapshot
20 times per second: lines that start with "@" (see printStream() in
teensy_ffb.ino). This app reads those lines, ignores everything else (or
shows it in the console box at the bottom), and draws:

  - whether the game is talking to the wheel right now: the firmware counts
    every force-feedback report from the PC, so if that count goes up, the
    game is sending FFB
  - every effect the game has created (spring, constant force, ...), whether
    it's playing, and how much force it adds right now
  - the game's total force, what we ask the motor for after tuning, and what
    the motor really gets after the safety layer
  - a graph of the last 10 seconds

The force-feedback data itself still goes over the separate HID (game
controller) part of the USB connection, so running this app doesn't change
what the game sees.
"""
import argparse
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

# Effect type numbers -> names, same order as FfbEngine.h.
EFFECT_NAMES = {
    1: "Constant", 2: "Ramp", 3: "Square", 4: "Sine", 5: "Triangle",
    6: "Sawtooth up", 7: "Sawtooth down", 8: "Spring", 9: "Damper",
    10: "Inertia", 11: "Friction",
}
# SafetyState in ForceSafety.h
SAFETY_NAMES = {
    0: ("OK", "good"), 1: ("Motor off (type 'motor on')", "warn"),
    2: ("SENSOR FAULT", "bad"), 3: ("LOOP STALL FAULT", "bad"),
    4: ("PC not connected", "bad"), 5: ("E-STOP pressed", "bad"),
}
TEENSY_VID = 0x16C0
HISTORY_SECONDS = 10.0


# =============================================================================
#  Parsing (no GUI code, so it can be tested on its own)
# =============================================================================
def parse_fields(text):
    """'a=1 b=2.5' -> {'a': 1, 'b': 2.5}. Numbers become int or float."""
    out = {}
    for part in text.split():
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        try:
            out[k] = int(v)
        except ValueError:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


class StreamParser:
    """Collects '@S', '@E' and '@END' lines into complete snapshots.

    feed(line) returns a finished snapshot dict at each '@END', otherwise
    None. A snapshot looks like {'state': {...}, 'effects': [{...}, ...]}.
    Lines that don't start with '@' are passed to on_text (the console).
    """

    def __init__(self, on_text=None):
        self.on_text = on_text
        self._state = None
        self._effects = []

    def feed(self, line):
        line = line.strip()
        if not line:
            return None
        if not line.startswith("@"):
            if self.on_text:
                self.on_text(line)
            return None
        if line.startswith("@S "):
            self._state = parse_fields(line[3:])
            self._effects = []
        elif line.startswith("@E ") and self._state is not None:
            self._effects.append(parse_fields(line[3:]))
        elif line == "@END" and self._state is not None:
            snap = {"state": self._state, "effects": self._effects}
            self._state, self._effects = None, []
            return snap
        return None


class ActivityTracker:
    """Turns the firmware's ever-growing report counter into 'reports per
    second', and decides whether the game is sending force feedback."""

    def __init__(self, window=1.0):
        self.window = window
        self.samples = []  # (time, count)

    def update(self, now, count):
        if self.samples and count < self.samples[-1][1]:
            self.samples = []  # Teensy restarted: counter went back to 0
        self.samples.append((now, count))
        while len(self.samples) > 2 and now - self.samples[0][0] > self.window:
            self.samples.pop(0)

    def rate(self):
        if len(self.samples) < 2:
            return 0.0
        (t0, c0), (t1, c1) = self.samples[0], self.samples[-1]
        return (c1 - c0) / (t1 - t0) if t1 > t0 else 0.0


# =============================================================================
#  Data sources: the real Teensy, or a simulated one for --demo
# =============================================================================
class SerialSource:
    def __init__(self, port):
        import serial  # pyserial
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.ser.write(b"stream on\n")

    def readline(self):
        raw = self.ser.readline()
        return raw.decode("utf-8", "replace") if raw else ""

    def write(self, text):
        self.ser.write(text.encode() + b"\n")

    def close(self):
        try:
            self.ser.write(b"stream off\n")
            self.ser.close()
        except Exception:
            pass


class DemoSource:
    """Pretends to be a wheel in a game: a centering spring, a damper, a
    constant force that follows a pretend corner, and a rumble that comes and
    goes. Speaks exactly the same line format as the firmware."""

    def __init__(self):
        self.t0 = time.time()
        self.reports = 0
        self.pending = []
        self.last = 0
        self.armed = 1

    def readline(self):
        if not self.pending:
            time.sleep(0.05)
            self._make_snapshot()
        return self.pending.pop(0)

    def _make_snapshot(self):
        t = time.time() - self.t0
        pos = 0.45 * math.sin(t * 0.7)
        vel = 0.45 * 0.7 * math.cos(t * 0.7) * 135
        game_on = (t % 20) < 15  # game "paused" for 5 s out of every 20
        effects = []
        game = 0.0
        if game_on:
            self.reports += 60
            spring = -0.4 * pos
            damper = -0.05 * vel / 360
            const = 0.3 * math.sin(t * 0.3)
            rumble = 0.08 * math.sin(t * 2 * math.pi * 12) if (t % 6) < 2 else 0.0
            effects = [
                (1, 1, 1, const), (2, 8, 1, spring), (3, 9, 1, damper),
                (4, 4, 1 if rumble else 0, rumble),
            ]
            game = max(-1, min(1, (const + spring + damper + rumble) * 0.7))
        out = max(-0.3, min(0.3, game)) if self.armed else 0.0
        lines = [
            f"@S t={int(t*1000)} ffb=1 ang={pos*135:.2f} pos={pos:.4f} vel={vel:.1f} "
            f"game={game:.4f} req={game:.4f} out={out:.4f} armed={self.armed} "
            f"state={0 if self.armed else 1} act=1 paused=0 dgain=255 reports={self.reports}"
        ]
        for i, ty, play, f in effects:
            lines.append(f"@E i={i} type={ty} play={play} gain=255 f={f:.4f}")
        lines.append("@END")
        self.pending = [l + "\n" for l in lines]

    def write(self, text):
        text = text.strip()
        self.pending.append(f"(demo) you typed: {text}\n")
        if text == "motor on":
            self.armed = 1
        elif text == "motor off":
            self.armed = 0

    def close(self):
        pass


def list_ports():
    """Returns [(device, description)], Teensy ports first."""
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return []
    ports = []
    for p in lp.comports():
        is_teensy = getattr(p, "vid", None) == TEENSY_VID
        label = f"{p.device}  {'(Teensy) ' if is_teensy else ''}{p.description or ''}".strip()
        ports.append((not is_teensy, p.device, label))
    return [(d, l) for _, d, l in sorted(ports)]


# =============================================================================
#  The window
# =============================================================================
COLORS = {
    "bg": "#16181d", "panel": "#1f2229", "text": "#e6e8eb", "dim": "#8b919c",
    "grid": "#2c3039", "good": "#3fb950", "warn": "#d29922", "bad": "#f85149",
    "game": "#58a6ff", "req": "#bc8cff", "out": "#ffa657", "pos": "#8b919c",
}


class ForceBar(tk.Canvas):
    """A horizontal bar centered at 0: left = force to the left, right = right."""

    def __init__(self, master, color, width=260, height=18):
        super().__init__(master, width=width, height=height, bg=COLORS["panel"],
                         highlightthickness=0)
        self.color, self.w, self.h = color, width, height

    def set(self, value):
        self.delete("all")
        mid = self.w / 2
        self.create_line(mid, 0, mid, self.h, fill=COLORS["grid"])
        v = max(-1.0, min(1.0, value))
        x = mid + v * (mid - 2)
        if abs(x - mid) >= 1:
            self.create_rectangle(min(mid, x), 3, max(mid, x), self.h - 3, fill=self.color,
                                  outline="")


class Graph(tk.Canvas):
    def __init__(self, master, width=720, height=200):
        super().__init__(master, width=width, height=height, bg=COLORS["panel"],
                         highlightthickness=0)
        self.w, self.h = width, height
        self.series = {"pos": [], "game": [], "out": []}
        self.bind("<Configure>", self._resize)

    def _resize(self, e):
        self.w, self.h = e.width, e.height

    def add(self, now, state):
        for k in self.series:
            self.series[k].append((now, float(state.get(k, 0))))
            while self.series[k] and now - self.series[k][0][0] > HISTORY_SECONDS:
                self.series[k].pop(0)

    def draw(self, now):
        self.delete("all")
        w, h = self.w, self.h
        for frac in (-1, -0.5, 0, 0.5, 1):
            y = h / 2 - frac * (h / 2 - 6)
            self.create_line(0, y, w, y, fill=COLORS["grid"], dash=() if frac == 0 else (2, 4))
            self.create_text(4, y, text=f"{frac:+.1f}" if frac else "0", anchor="w",
                             fill=COLORS["dim"], font=("TkDefaultFont", 8))
        # Motor output is drawn wide underneath, game force thin on top, so you
        # can still see both when they are the same.
        for key, color, width in (("pos", COLORS["pos"], 1), ("out", COLORS["out"], 4),
                                  ("game", COLORS["game"], 2)):
            pts = []
            for t, v in self.series[key]:
                x = w - (now - t) / HISTORY_SECONDS * w
                y = h / 2 - max(-1, min(1, v)) * (h / 2 - 6)
                pts += [x, y]
            if len(pts) >= 4:
                self.create_line(*pts, fill=color, width=width)


class App:
    def __init__(self, root, demo=False):
        self.root = root
        self.demo = demo
        self.source = None
        self.lines = queue.Queue()
        self.reader = None
        self.running = False
        self.parser = StreamParser(on_text=self._console_line)
        self.activity = ActivityTracker()
        self.last_snapshot_time = 0.0

        root.title("FFB Monitor")
        root.configure(bg=COLORS["bg"])
        root.minsize(760, 640)
        style = ttk.Style(root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background=COLORS["bg"], foreground=COLORS["text"])
        style.configure("Treeview", background=COLORS["panel"], fieldbackground=COLORS["panel"],
                        foreground=COLORS["text"], rowheight=22)
        style.configure("Treeview.Heading", background=COLORS["grid"], foreground=COLORS["text"])
        style.configure("TButton", background=COLORS["grid"], foreground=COLORS["text"],
                        bordercolor=COLORS["grid"])
        style.map("TButton", background=[("active", COLORS["panel"])])
        style.configure("TCombobox", fieldbackground=COLORS["panel"], background=COLORS["grid"],
                        foreground=COLORS["text"], arrowcolor=COLORS["text"])
        style.map("TCombobox", fieldbackground=[("readonly", COLORS["panel"])],
                  foreground=[("readonly", COLORS["text"])])

        self._build_top()
        self._build_status()
        self._build_forces()
        self._build_graph()
        self._build_effects()
        self._build_console()

        root.protocol("WM_DELETE_WINDOW", self.quit)
        self.root.after(50, self._tick)
        if demo:
            self.connect()

    # ---- layout -------------------------------------------------------------
    def _label(self, master, text="", size=10, color="text", bold=False, **kw):
        return tk.Label(master, text=text, bg=master["bg"], fg=COLORS[color],
                        font=("TkDefaultFont", size, "bold" if bold else "normal"), **kw)

    def _panel(self, title):
        frame = tk.Frame(self.root, bg=COLORS["panel"], padx=10, pady=8)
        frame.pack(fill="x", padx=10, pady=(0, 8))
        self._label(frame, title, 9, "dim", bold=True).pack(anchor="w")
        return frame

    def _build_top(self):
        bar = tk.Frame(self.root, bg=COLORS["bg"], padx=10, pady=10)
        bar.pack(fill="x")
        self._label(bar, "Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=38, state="readonly")
        self.port_box.pack(side="left", padx=6)
        ttk.Button(bar, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(bar, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=6)
        self.conn_label = self._label(bar, "Not connected", color="dim")
        self.conn_label.pack(side="left", padx=8)
        self.refresh_ports()

    def _build_status(self):
        f = self._panel("IS THE GAME SENDING FORCE FEEDBACK?")
        row = tk.Frame(f, bg=COLORS["panel"])
        row.pack(fill="x", pady=(4, 0))
        self.lamp = tk.Canvas(row, width=26, height=26, bg=COLORS["panel"], highlightthickness=0)
        self.lamp.pack(side="left")
        self.big = self._label(row, "Waiting for the wheel...", 16, bold=True)
        self.big.pack(side="left", padx=8)
        self.detail = self._label(f, "", 9, "dim", justify="left")
        self.detail.pack(anchor="w", pady=(4, 0))
        self._set_lamp("dim")

    def _build_forces(self):
        f = self._panel("FORCES  (left = pushes the wheel left, right = right; full bar = 100 % motor power)")
        self.bars = {}
        for key, name, color in (("game", "Game force (after gain)", "game"),
                                 ("req", "Asked from motor (+ tuning, tests)", "req"),
                                 ("out", "Motor really gets (after safety)", "out")):
            row = tk.Frame(f, bg=COLORS["panel"])
            row.pack(fill="x", pady=2)
            self._label(row, name, width=32, anchor="w").pack(side="left")
            bar = ForceBar(row, COLORS[color])
            bar.pack(side="left")
            val = self._label(row, "", width=8, anchor="e")
            val.pack(side="left")
            self.bars[key] = (bar, val)
        self.wheel_label = self._label(f, "", 9, "dim")
        self.wheel_label.pack(anchor="w", pady=(4, 0))

    def _build_graph(self):
        f = self._panel("LAST 10 SECONDS")
        legend = tk.Frame(f, bg=COLORS["panel"])
        legend.pack(anchor="w")
        for name, color in (("wheel position", "pos"), ("game force", "game"),
                            ("motor output", "out")):
            self._label(legend, "━━ " + name, 9, color).pack(side="left", padx=(0, 12))
        self.graph = Graph(f, height=140)
        self.graph.pack(fill="x", pady=(4, 0))

    def _build_effects(self):
        f = self._panel("EFFECTS THE GAME HAS CREATED")
        cols = ("slot", "type", "status", "gain", "force")
        self.tree = ttk.Treeview(f, columns=cols, show="headings", height=5)
        for c, w, anchor in (("slot", 50, "center"), ("type", 140, "w"), ("status", 100, "w"),
                             ("gain", 70, "e"), ("force", 320, "w")):
            self.tree.heading(c, text=c.capitalize())
            self.tree.column(c, width=w, anchor=anchor, stretch=(c == "force"))
        self.tree.tag_configure("playing", foreground=COLORS["good"])
        self.tree.tag_configure("stopped", foreground=COLORS["dim"])
        self.tree.pack(fill="x", pady=(4, 0))

    def _build_console(self):
        f = self._panel("WHEEL MESSAGES  (you can type commands here, e.g. motor on, ffb, settings)")
        self.console = tk.Text(f, height=4, bg=COLORS["bg"], fg=COLORS["text"],
                               insertbackground=COLORS["text"], relief="flat",
                               font=("TkFixedFont", 9), state="disabled")
        self.console.pack(fill="both", expand=True, pady=(4, 4))
        row = tk.Frame(f, bg=COLORS["panel"])
        row.pack(fill="x")
        self.cmd = tk.Entry(row, bg=COLORS["bg"], fg=COLORS["text"],
                            insertbackground=COLORS["text"], relief="flat")
        self.cmd.pack(side="left", fill="x", expand=True)
        self.cmd.bind("<Return>", lambda e: self.send_command())
        ttk.Button(row, text="Send", command=self.send_command).pack(side="left", padx=(6, 0))

    # ---- connection ---------------------------------------------------------
    def refresh_ports(self):
        ports = list_ports()
        self.port_map = {label: dev for dev, label in ports}
        labels = list(self.port_map)
        if self.demo:
            labels = ["Demo (simulated wheel)"] + labels
        self.port_box["values"] = labels
        if labels and not self.port_var.get():
            self.port_var.set(labels[0])
        if not labels:
            self.port_var.set("")
            self._console_line("No serial ports found. Is the wheel plugged in? "
                               "(pip install pyserial if you haven't)")

    def toggle_connect(self):
        if self.running:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        try:
            if self.demo and (self.port_var.get().startswith("Demo") or not self.port_var.get()):
                self.source = DemoSource()
                name = "demo"
            else:
                dev = self.port_map.get(self.port_var.get())
                if not dev:
                    self._console_line("Pick a port first.")
                    return
                self.source = SerialSource(dev)
                name = dev
        except ImportError:
            self._console_line("pyserial is missing. Run:  pip install pyserial")
            return
        except Exception as e:  # port busy, unplugged, ...
            self._console_line(f"Could not open the port: {e}")
            self._console_line("Is the Arduino Serial Monitor still open? Close it and retry.")
            return
        self.running = True
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        self.connect_btn.configure(text="Disconnect")
        self.conn_label.configure(text=f"Connected to {name}", fg=COLORS["good"])
        self.activity = ActivityTracker()

    def disconnect(self):
        self.running = False
        if self.source:
            self.source.close()
        self.source = None
        self.connect_btn.configure(text="Connect")
        self.conn_label.configure(text="Not connected", fg=COLORS["dim"])

    def _read_loop(self):
        # Runs in a background thread so a slow port never freezes the window.
        while self.running:
            try:
                line = self.source.readline()
            except Exception as e:
                self.lines.put(("error", str(e)))
                break
            if line:
                self.lines.put(("line", line))

    def send_command(self):
        text = self.cmd.get().strip()
        if not text:
            return
        self.cmd.delete(0, "end")
        if not self.source:
            self._console_line("Not connected.")
            return
        self._console_line(f"> {text}")
        self.source.write(text)

    def quit(self):
        self.disconnect()
        self.root.destroy()

    # ---- updating -----------------------------------------------------------
    def _console_line(self, text):
        self.console.configure(state="normal")
        self.console.insert("end", text + "\n")
        lines = int(self.console.index("end-1c").split(".")[0])
        if lines > 300:
            self.console.delete("1.0", f"{lines - 300}.0")
        self.console.see("end")
        self.console.configure(state="disabled")

    def _set_lamp(self, color):
        self.lamp.delete("all")
        self.lamp.create_oval(3, 3, 23, 23, fill=COLORS.get(color, color), outline="")

    def _tick(self):
        latest = None
        try:
            while True:
                kind, payload = self.lines.get_nowait()
                if kind == "error":
                    self._console_line(f"Connection lost: {payload}")
                    self.disconnect()
                    break
                snap = self.parser.feed(payload)
                if snap:
                    latest = snap
        except queue.Empty:
            pass
        now = time.time()
        if latest:
            self.last_snapshot_time = now
            self._show(latest, now)
        elif self.running and now - self.last_snapshot_time > 1.5:
            self._set_lamp("warn")
            self.big.configure(text="Connected, but no data from the wheel")
            self.detail.configure(text="Is the latest firmware uploaded? It needs the "
                                       "'stream' command (type 'help' below to check).")
        self.graph.draw(now)
        self.root.after(50, self._tick)

    def _show(self, snap, now):
        s = snap["state"]
        effects = snap["effects"]
        self.activity.update(now, s.get("reports", 0))
        rate = self.activity.rate()
        playing = [e for e in effects if e.get("play")]

        if not s.get("ffb", 0):
            self._set_lamp("bad")
            self.big.configure(text="Firmware built without force feedback")
            self.detail.configure(text="Pick Tools > USB Type > 'Serial + FFB Wheel' and upload "
                                       "again (run tools/patch_teensy_core.py first).")
        elif rate > 0.5 and playing:
            self._set_lamp("good")
            self.big.configure(text="YES: the game is sending force feedback")
        elif rate > 0.5 or playing:
            self._set_lamp("warn")
            self.big.configure(text="The game is talking to the wheel" if rate > 0.5
                               else "Effects are playing (game is quiet right now)")
        elif effects:
            self._set_lamp("warn")
            self.big.configure(text="Effects exist, but nothing is playing")
        else:
            self._set_lamp("dim")
            self.big.configure(text="No: nothing from the game yet")

        safety_text, safety_color = SAFETY_NAMES.get(s.get("state", -1), ("?", "dim"))
        dgain = round(s.get("dgain", 255) * 100 / 255)
        self.detail.configure(text=(
            f"{rate:.0f} FFB reports per second   |   {len(effects)} effects, "
            f"{len(playing)} playing   |   game strength {dgain} %   |   "
            f"actuators {'on' if s.get('act') else 'OFF'}"
            f"{'   |   PAUSED by game' if s.get('paused') else ''}   |   "
            f"motor: {safety_text}"))

        for key, (bar, label) in self.bars.items():
            v = float(s.get(key, 0))
            bar.set(v)
            label.configure(text=f"{v * 100:+.0f} %")
        self.wheel_label.configure(text=f"Wheel angle {s.get('ang', 0):+.1f} deg   "
                                        f"speed {s.get('vel', 0):+.0f} deg/s")

        self.graph.add(now, s)
        self.tree.delete(*self.tree.get_children())
        for e in effects:
            f = float(e.get("f", 0))
            n = int(min(20, round(abs(f) * 40)))
            bar = ("█" * n) if n else "·"
            arrow = "←" if f < 0 else "→"
            self.tree.insert("", "end", tags=("playing" if e.get("play") else "stopped",), values=(
                e.get("i"), EFFECT_NAMES.get(e.get("type"), f"type {e.get('type')}"),
                "PLAYING" if e.get("play") else "stopped",
                f"{round(e.get('gain', 255) * 100 / 255)} %",
                f"{arrow} {bar}  {f * 100:+.1f} %"))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--demo", action="store_true",
                    help="show a simulated wheel, no hardware needed")
    args = ap.parse_args()
    root = tk.Tk()
    App(root, demo=args.demo)
    root.mainloop()


if __name__ == "__main__":
    main()
