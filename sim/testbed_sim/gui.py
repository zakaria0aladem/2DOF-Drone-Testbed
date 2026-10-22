"""gui.py - a window that shows the simulated rig while your firmware runs.

    python -m testbed_sim --gui                     (the demo firmware)
    python -m testbed_sim Debug/drone_fw.elf --gui  (yours)

The simulator runs on its own thread, in the same loop as without --gui, so
USB port 2 is still at socket://localhost:5333 for tbbridge.py. About 25
times a second the window draws:

  * a top view: the frame turning on the yaw joint, props coloured by throttle
  * a side view: the frame tilting on the pitch joint, thrust arrows, end stops
  * a plot of yaw and pitch over the last 10 s
  * the four ESCs: pulse width, state, motor speed, thrust
  * what the firmware prints on USB port 1, and the [sim] messages

The buttons send the same commands you can type in the console (k1, reset,
push yaw 0.05, pause...). Only tkinter is needed, which comes with Python.
"""
from __future__ import annotations

import collections
import math
import queue
import sys
import threading
import time
import traceback
import tkinter as tk
from tkinter import ttk

from .bridge import serve

HISTORY_S = 10.0          # s of yaw/pitch shown in the plot
FRAME_MS = 40             # window refresh period
PUSH_YAW = 0.05           # N m, a push from the buttons (0.1 s long)
PUSH_PITCH = 0.02
PROP_RADIUS = 0.0635      # m, 5-inch prop (only for drawing)
MAX_LINES = 3000          # console lines kept

YAW_COLOR = "#2563eb"
PITCH_COLOR = "#ea580c"
STATE_COLOR = {"armed": "#15803d", "arming": "#b45309", "no-signal": "#b91c1c", "off": "#6b7280"}
SPEEDS = {"0.25x": 0.25, "0.5x": 0.5, "1x (real time)": 1.0, "2x": 2.0, "as fast as possible": 0.0}


class _Stream:
    """File-like object: what serve() prints goes into a queue for the window."""

    def __init__(self, q, tag):
        self.q, self.tag = q, tag

    def write(self, s):
        if s:
            self.q.put((self.tag, s))

    def flush(self):
        pass


def _mix(c0, c1, f):
    """Colour between c0 and c1 ('#rrggbb'), f in 0..1."""
    f = max(0.0, min(1.0, f))
    a = [int(c0[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(c1[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{round(x + (y - x) * f):02x}" for x, y in zip(a, b))


def _throttle_color(thr):
    if thr <= 0:
        return "#e5e7eb"
    if thr < 0.5:
        return _mix("#fde68a", "#f97316", thr / 0.5)
    return _mix("#f97316", "#dc2626", (thr - 0.5) / 0.5)


def _nice_step(span):
    """A round grid step (1, 2 or 5 x 10^n) giving about 4 lines over span."""
    raw = span / 4
    p = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 5, 10):
        if m * p >= raw:
            return m * p
    return 10 * p


class SimWindow:
    def __init__(self, tb, ports, title="", speed=1.0, duration=None):
        self.tb = tb
        self.ports = ports
        self.cfg = tb.cfg
        self.cmds = queue.Queue()            # window -> simulator thread
        self.text_q = queue.Queue()          # simulator thread -> console box
        self.hist = collections.deque()      # (t, yaw, pitch), filled by the simulator thread
        self.snap = None                     # latest tb.state(), from the simulator thread
        self._next_hist_t = 0.0
        self._last_snap = 0.0
        self._rate = []                      # (wall, sim t) to measure the real speed
        self.done = False
        self.error = None
        tb._log = lambda s: self.text_q.put(("sim", s + "\n"))

        self.root = tk.Tk()
        self.root.title(f"Testbed simulator - {title}" if title else "Testbed simulator")
        self.root.minsize(980, 600)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.report_callback_exception = self._callback_error
        self._build(speed)

        self.thread = threading.Thread(target=self._sim, args=(speed, duration), daemon=True)
        self.thread.start()
        self.root.after(FRAME_MS, self._refresh)

    # ------------------------------------------------------------ layout
    def _build(self, speed):
        root = self.root
        root.columnconfigure(1, weight=1)
        root.rowconfigure(2, weight=1)

        bar = ttk.Frame(root, padding=(8, 6))
        bar.grid(row=0, column=0, columnspan=2, sticky="ew")
        for key in (1, 2):
            b = ttk.Button(bar, text=f"KEY{key}", width=7)
            b.bind("<ButtonPress-1>", lambda e, k=key: self.cmds.put(f"hold k{k}"))
            b.bind("<ButtonRelease-1>", lambda e, k=key: self.cmds.put(f"release k{k}"))
            b.pack(side="left", padx=2)
        ttk.Button(bar, text="RESET", width=7,
                   command=lambda: self.cmds.put("reset")).pack(side="left", padx=(2, 14))
        ttk.Label(bar, text="Push:").pack(side="left")
        for text, cmd in (("yaw ↺", f"push yaw {PUSH_YAW}"),
                          ("yaw ↻", f"push yaw {-PUSH_YAW}"),
                          ("nose ↓", f"push pitch {PUSH_PITCH}"),
                          ("nose ↑", f"push pitch {-PUSH_PITCH}")):
            ttk.Button(bar, text=text, width=8,
                       command=lambda c=cmd: self.cmds.put(c)).pack(side="left", padx=2)
        self.pause_btn = ttk.Button(bar, text="Pause", width=8, command=self._toggle_pause)
        self.pause_btn.pack(side="left", padx=(14, 2))
        ttk.Label(bar, text="Speed:").pack(side="left", padx=(8, 2))
        self.speed_box = ttk.Combobox(bar, values=list(SPEEDS), state="readonly", width=18)
        self.speed_box.set(next((k for k, v in SPEEDS.items() if v == speed), "1x (real time)"))
        self.speed_box.bind("<<ComboboxSelected>>",
                            lambda e: self.cmds.put(f"speed {SPEEDS[self.speed_box.get()]}"))
        self.speed_box.pack(side="left")
        where = "  ".join(f"USB port {'1' if u == 'USART1' else '2'}: socket://localhost:{o['tcp']}"
                          for u, o in self.ports.items() if o.get("tcp") is not None)
        ttk.Label(bar, text=where, foreground="#4b5563").pack(side="right")

        hint = ("Hold KEY1/KEY2 down with the mouse like the real keys. Type console commands "
                "in the box at the bottom (help lists them).")
        ttk.Label(root, text=hint, foreground="#4b5563", padding=(10, 0)).grid(
            row=1, column=0, columnspan=2, sticky="w")

        left = ttk.Frame(root, padding=(8, 4))
        left.grid(row=2, column=0, sticky="ns")
        self.top = tk.Canvas(left, width=330, height=300, bg="white", highlightthickness=1,
                             highlightbackground="#d1d5db")
        self.top.pack(pady=(0, 6))
        self.side = tk.Canvas(left, width=330, height=220, bg="white", highlightthickness=1,
                              highlightbackground="#d1d5db")
        self.side.pack()

        right = ttk.Frame(root, padding=(4, 4, 8, 4))
        right.grid(row=2, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        self.plot = tk.Canvas(right, height=190, bg="white", highlightthickness=1,
                              highlightbackground="#d1d5db")
        self.plot.grid(row=0, column=0, sticky="ew")

        table = ttk.Frame(right, padding=(0, 6))
        table.grid(row=1, column=0, sticky="ew")
        heads = ("ESC", "pin", "pulse", "rate", "state", "motor", "thrust")
        for c, h in enumerate(heads):
            ttk.Label(table, text=h, foreground="#4b5563").grid(row=0, column=c, sticky="w",
                                                                padx=(0, 14))
        self.esc_rows = []
        pins = self.cfg["esc"]["signal_pins"]
        for i in range(4):
            row = {}
            ttk.Label(table, text=f"ESC{i + 1}").grid(row=i + 1, column=0, sticky="w")
            ttk.Label(table, text=pins[i]).grid(row=i + 1, column=1, sticky="w", padx=(0, 14))
            for c, name, w in ((2, "pulse", 9), (3, "rate", 8), (4, "state", 10), (5, "rpm", 11)):
                lab = ttk.Label(table, width=w)
                lab.grid(row=i + 1, column=c, sticky="w", padx=(0, 14))
                row[name] = lab
            bar_c = tk.Canvas(table, width=150, height=14, bg="#f3f4f6", highlightthickness=0)
            bar_c.grid(row=i + 1, column=6, sticky="w")
            row["bar"] = bar_c
            self.esc_rows.append(row)

        status = ttk.Frame(right)
        status.grid(row=2, column=0, sticky="ew", pady=(0, 6))
        self.status = ttk.Label(status, font=("TkDefaultFont", 10))
        self.status.pack(side="left")
        self.led = tk.Label(status, text="● LED", foreground="#9ca3af")
        self.led.pack(side="left", padx=(16, 0))
        self.cpu = ttk.Label(status)
        self.cpu.pack(side="left", padx=(16, 0))

        cons = ttk.Frame(right)
        cons.grid(row=3, column=0, sticky="nsew")
        right.rowconfigure(2, weight=0)
        right.rowconfigure(3, weight=1)
        cons.columnconfigure(0, weight=1)
        cons.rowconfigure(1, weight=1)
        ttk.Label(cons, text="USB port 1 (what the firmware prints) and simulator messages",
                  foreground="#4b5563").grid(row=0, column=0, sticky="w")
        self.text = tk.Text(cons, height=8, wrap="none", font="TkFixedFont", bg="#111827",
                            fg="#e5e7eb", insertbackground="#e5e7eb")
        self.text.tag_configure("sim", foreground="#93c5fd")
        self.text.tag_configure("you", foreground="#fcd34d")
        self.text.grid(row=1, column=0, sticky="nsew")
        sb = ttk.Scrollbar(cons, command=self.text.yview)
        sb.grid(row=1, column=1, sticky="ns")
        self.text.configure(yscrollcommand=sb.set, state="disabled")
        self.entry = ttk.Entry(cons)
        self.entry.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        self.entry.insert(0, "type a command (k1, state, push yaw 0.05, help) and press Enter")
        self.entry.bind("<FocusIn>", self._clear_hint)
        self.entry.bind("<Return>", self._enter)

    def _clear_hint(self, _e):
        if self.entry.get().startswith("type a command"):
            self.entry.delete(0, "end")

    def _enter(self, _e):
        line = self.entry.get().strip()
        self.entry.delete(0, "end")
        if line:
            self._append("you", f"> {line}\n")
            self.cmds.put(line)

    def _toggle_pause(self):
        paused = self.snap and self.snap.get("paused")
        self.cmds.put("resume" if paused else "pause")

    # ------------------------------------------------------------ simulator thread
    def _sim(self, speed, duration):
        try:
            serve(self.tb, self.ports, console="USART1", speed=speed, duration=duration,
                  interactive=False, out=_Stream(self.text_q, "fw"), commands=self.cmds,
                  on_tick=self._tick)
        except OSError as e:
            if getattr(e, "errno", None) in (48, 98, 10048):
                self.error = (f"a TCP port is already in use ({e}). Another simulator is "
                              f"probably still running: close it, or start this one with "
                              f"--port2 5335")
            else:
                self.error = traceback.format_exc()
        except Exception:
            self.error = traceback.format_exc()
        finally:
            if self.error:
                print(f"[sim] {self.error}", file=sys.stderr)
                self.text_q.put(("sim", f"[sim] simulator stopped: {self.error}\n"))
            self.done = True

    def _tick(self, tb, run):
        """Called by serve() after every simulation step (simulator thread)."""
        t = tb.t
        if t >= self._next_hist_t:
            self.hist.append((t, tb.rig.yaw.q, tb.rig.pitch.q))
            self._next_hist_t = t + 0.01
            while self.hist and self.hist[0][0] < t - HISTORY_S:
                self.hist.popleft()
        now = time.perf_counter()
        if now - self._last_snap >= 0.03:
            s = tb.state()
            s["paused"], s["speed"] = run.paused, run.speed
            self.snap = s
            self._last_snap = now

    # ------------------------------------------------------------ window side
    def _append(self, tag, s):
        self.text.configure(state="normal")
        at_end = self.text.yview()[1] > 0.999
        self.text.insert("end", s.replace("\r", ""), (tag,))
        lines = int(self.text.index("end-1c").split(".")[0])
        if lines > MAX_LINES:
            self.text.delete("1.0", f"{lines - MAX_LINES}.0")
        self.text.configure(state="disabled")
        if at_end:
            self.text.see("end")

    def _refresh(self):
        chunks = []
        try:
            while True:
                chunks.append(self.text_q.get_nowait())
        except queue.Empty:
            pass
        for tag, s in chunks:
            self._append(tag, s)
        s = self.snap
        if s:
            self._draw_top(s)
            self._draw_side(s)
            self._draw_plot(list(self.hist))
            self._update_table(s)
            self._update_status(s)
        if self.done:
            self.pause_btn.state(["disabled"])
            self.cpu.configure(text="simulator stopped", foreground="#b91c1c")
            return
        self.root.after(FRAME_MS, self._refresh)

    def _update_status(self, s):
        now = time.perf_counter()
        self._rate.append((now, s["t"]))
        while len(self._rate) > 2 and now - self._rate[0][0] > 2.0:
            self._rate.pop(0)
        (w0, t0), (w1, t1) = self._rate[0], self._rate[-1]
        rate = (t1 - t0) / (w1 - w0) if w1 - w0 > 0.5 else None
        if s["paused"]:
            run = "PAUSED"
        else:
            run = f"{rate:.2f}x real time" if rate is not None else ""
        self.status.configure(text=f"t = {s['t']:8.3f} s   {run:<16}   bus {s['v_bus']:5.2f} V  "
                                   f"{s['current_a']:5.2f} A   buzzer "
                                   + (f"{s['buzzer_hz']:.0f} Hz" if s["buzzer_hz"] else "off"))
        self.pause_btn.configure(text="Resume" if s["paused"] else "Pause")
        if SPEEDS.get(self.speed_box.get()) != s["speed"]:     # changed by a typed command
            self.speed_box.set(next((k for k, v in SPEEDS.items() if v == s["speed"]),
                                    f"{s['speed']:g}x"))
        self.led.configure(foreground="#dc2626" if s["led"] else "#9ca3af")
        if s["cpu_halted"]:
            self.cpu.configure(text=f"CPU stopped: {s['cpu_halted']}", foreground="#b91c1c")
        else:
            self.cpu.configure(text="CPU running", foreground="#15803d")

    def _update_table(self, s):
        for i, row in enumerate(self.esc_rows):
            us, hz, st = s["esc_us"][i], s["esc_hz"][i], s["esc_state"][i]
            row["pulse"].configure(text=f"{us:.0f} us" if us else "no PWM")
            row["rate"].configure(text=f"{hz:.0f} Hz" if hz else "")
            row["state"].configure(text=st, foreground=STATE_COLOR.get(st, "#111827"))
            row["rpm"].configure(text=f"{s['motor_rpm'][i]:6.0f} rpm")
            c = row["bar"]
            c.delete("all")
            thrust = s["thrust_n"][i]
            w = min(150, thrust / 6.0 * 150)          # full bar = 6 N
            c.create_rectangle(0, 0, w, 14, fill=_throttle_color(s["throttle"][i]), width=0)
            c.create_text(4, 7, text=f"{thrust:.2f} N", anchor="w", font=("TkDefaultFont", 8))

    def _draw_top(self, s):
        """Seen from above: x forward is up on the screen, y left is left."""
        c = self.top
        c.delete("all")
        w, h = int(c["width"]), int(c["height"])
        cx, cy = w / 2, h / 2 + 8
        pos = self.cfg["motor"]["positions"]
        spin = self.cfg["motor"]["spin"]
        reach = max(math.hypot(x, y) for x, y in pos) + PROP_RADIUS
        k = (min(w, h) / 2 - 22) / reach
        yaw, pitch = s["yaw"], s["pitch"]
        cyaw, syaw, cp = math.cos(yaw), math.sin(yaw), math.cos(pitch)

        def scr(x, y):
            x *= cp                                       # the tilt shortens the frame
            wx, wy = x * cyaw - y * syaw, x * syaw + y * cyaw
            return cx - wy * k, cy - wx * k

        ring = reach * k + 8
        c.create_oval(cx - ring, cy - ring, cx + ring, cy + ring, outline="#d1d5db", dash=(3, 3))
        for d in range(0, 360, 30):
            a = math.radians(d)
            r0 = ring - (8 if d % 90 == 0 else 4)
            c.create_line(cx - r0 * math.sin(a), cy - r0 * math.cos(a),
                          cx - ring * math.sin(a), cy - ring * math.cos(a), fill="#9ca3af")
        c.create_text(cx, cy - ring - 7, text="0°", fill="#6b7280", font=("TkDefaultFont", 8))
        lim = self.tb.rig.yaw.lim
        if lim:
            for a in lim:
                c.create_line(cx, cy, cx - ring * math.sin(a), cy - ring * math.cos(a),
                              fill="#dc2626", dash=(4, 3))
        for (x, y), sp, thr in zip(pos, spin, s["throttle"]):
            px, py = scr(x, y)
            c.create_line(cx, cy, px, py, width=6, fill="#374151", capstyle="round")
        for i, ((x, y), sp, thr) in enumerate(zip(pos, spin, s["throttle"])):
            px, py = scr(x, y)
            r = PROP_RADIUS * k
            c.create_oval(px - r, py - r, px + r, py + r, fill=_throttle_color(thr),
                          outline="#6b7280", stipple="gray50" if thr <= 0 else "")
            c.create_oval(px - 7, py - 7, px + 7, py + 7, fill="#374151", outline="")
            c.create_text(px, py, text=str(i + 1), fill="white", font=("TkDefaultFont", 8, "bold"))
            c.create_text(px + r * 0.62, py - r * 0.62, text="↺" if sp > 0 else "↻",
                          fill="#374151", font=("TkDefaultFont", 11))
        nose = max(x for x, _ in pos) + PROP_RADIUS * 0.4
        tip, l, r = scr(nose + 0.03, 0), scr(nose, 0.015), scr(nose, -0.015)
        c.create_polygon(*tip, *l, *r, fill="#dc2626", outline="")
        bl = 0.025
        c.create_polygon(*scr(bl, bl), *scr(bl, -bl), *scr(-bl, -bl), *scr(-bl, bl),
                         fill="#1f2937", outline="")
        c.create_text(8, 8, anchor="nw", text="Top view (yaw)", fill="#374151",
                      font=("TkDefaultFont", 9, "bold"))
        c.create_text(8, 24, anchor="nw", text=f"yaw {math.degrees(yaw):+8.1f}°",
                      fill=YAW_COLOR, font=("TkFixedFont", 10))
        c.create_text(8, 40, anchor="nw", text=f"    {math.degrees(s['yaw_rate']):+7.1f}°/s",
                      fill=YAW_COLOR, font=("TkFixedFont", 9))
        c.create_text(w - 6, 8, anchor="ne", text="+ = counter-\nclockwise\nfrom above",
                      fill="#9ca3af", font=("TkDefaultFont", 8), justify="right")

    def _draw_side(self, s):
        """Seen from the right side: nose to the right, nose down = positive pitch."""
        c = self.side
        c.delete("all")
        w, h = int(c["width"]), int(c["height"])
        cx, cy = w / 2, h / 2 + 10
        pos = self.cfg["motor"]["positions"]
        arm = max(abs(x) for x, _ in pos)
        k = (w / 2 - 40) / (arm + PROP_RADIUS)
        th = s["pitch"]
        ct, st = math.cos(th), math.sin(th)

        def scr(x, z):
            return cx + (x * ct + z * st) * k, cy - (-x * st + z * ct) * k

        c.create_polygon(cx, cy, cx - 22, h - 8, cx + 22, h - 8, fill="#e5e7eb", outline="#9ca3af")
        c.create_line(10, h - 8, w - 10, h - 8, fill="#9ca3af")
        lim = self.tb.rig.pitch.lim
        if lim:
            for a in lim:
                ex = cx + (arm + PROP_RADIUS) * math.cos(a) * k
                ey = cy + (arm + PROP_RADIUS) * math.sin(a) * k
                c.create_line(cx, cy, ex, ey, fill="#dc2626", dash=(4, 3))
        c.create_line(*scr(-arm, 0), *scr(arm, 0), width=6, fill="#374151", capstyle="round")
        groups = {}
        for (x, _), t in zip(pos, s["thrust_n"]):
            groups.setdefault(round(x, 4), []).append(t)
        names = {}
        for i, (x, _) in enumerate(pos):
            names.setdefault(round(x, 4), []).append(str(i + 1))
        for x, ts in groups.items():
            c.create_rectangle(*scr(x - 0.012, 0.0), *scr(x + 0.012, 0.025), fill="#4b5563",
                               outline="")
            c.create_line(*scr(x - PROP_RADIUS, 0.03), *scr(x + PROP_RADIUS, 0.03), width=3,
                          fill="#9ca3af")
            f = sum(ts)
            if f > 0.01:
                L = min(0.12, 0.01 + f * 0.04)            # drawn length (m), longer = more thrust
                c.create_line(*scr(x, 0.035), *scr(x, 0.035 + L), arrow="last", width=3,
                              fill="#f97316")
            lx, ly = scr(x, -0.03)
            c.create_text(lx, ly, text="ESC" + "+".join(names[x]) + f"\n{f:.2f} N",
                          fill="#6b7280", font=("TkDefaultFont", 8), justify="center")
        c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="white", outline="#111827", width=2)
        nx, ny = scr(arm + PROP_RADIUS * 0.6, 0)
        c.create_text(nx + 4, ny, text="nose", anchor="w", fill="#dc2626",
                      font=("TkDefaultFont", 8))
        c.create_text(8, 8, anchor="nw", text="Side view (pitch)", fill="#374151",
                      font=("TkDefaultFont", 9, "bold"))
        c.create_text(8, 24, anchor="nw",
                      text=f"pitch {math.degrees(th):+6.1f}°  {math.degrees(s['pitch_rate']):+6.1f}°/s",
                      fill=PITCH_COLOR, font=("TkFixedFont", 10))
        c.create_text(w - 6, 8, anchor="ne", text="+ = nose down", fill="#9ca3af",
                      font=("TkDefaultFont", 8))

    def _draw_plot(self, hist):
        c = self.plot
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 50 or len(hist) < 2:
            return
        x0, x1, y0, y1 = 48, w - 10, 12, h - 24
        t_end = hist[-1][0]
        t_start = max(hist[0][0], t_end - HISTORY_S)
        span_t = max(HISTORY_S, t_end - t_start)
        t_start = t_end - span_t
        yaw = [math.degrees(p[1]) for p in hist]
        pit = [math.degrees(p[2]) for p in hist]
        lo, hi = min(min(yaw), min(pit)), max(max(yaw), max(pit))
        mid, half = (lo + hi) / 2, max(5.0, (hi - lo) / 2 * 1.15)
        lo, hi = mid - half, mid + half
        step = _nice_step(hi - lo)
        g = math.ceil(lo / step) * step
        while g <= hi:
            y = y1 - (g - lo) / (hi - lo) * (y1 - y0)
            c.create_line(x0, y, x1, y, fill="#e5e7eb" if g else "#9ca3af")
            c.create_text(x0 - 4, y, text=f"{g:g}°", anchor="e", fill="#6b7280",
                          font=("TkDefaultFont", 8))
            g += step
        for k in range(0, int(HISTORY_S) + 1, 2):
            x = x1 - k / span_t * (x1 - x0)
            c.create_line(x, y0, x, y1, fill="#f3f4f6")
            c.create_text(x, y1 + 10, text=f"-{k} s" if k else "now", fill="#6b7280",
                          font=("TkDefaultFont", 8))
        every = max(1, len(hist) // max(1, (x1 - x0)))
        for data, color in ((yaw, YAW_COLOR), (pit, PITCH_COLOR)):
            pts = []
            for (t, *_), v in list(zip(hist, data))[::every]:
                pts += [x1 - (t_end - t) / span_t * (x1 - x0),
                        y1 - (v - lo) / (hi - lo) * (y1 - y0)]
            if len(pts) >= 4:
                c.create_line(*pts, fill=color, width=2)
        c.create_text(x0 + 6, y0 + 2, anchor="nw", text="— yaw", fill=YAW_COLOR,
                      font=("TkDefaultFont", 9, "bold"))
        c.create_text(x0 + 60, y0 + 2, anchor="nw", text="— pitch", fill=PITCH_COLOR,
                      font=("TkDefaultFont", 9, "bold"))

    # ------------------------------------------------------------ closing
    def _callback_error(self, exc, val, tb):
        if issubclass(exc, KeyboardInterrupt):
            self.close()
        else:
            traceback.print_exception(exc, val, tb)

    def close(self):
        self.cmds.put("quit")
        self.thread.join(timeout=3)
        self.root.destroy()

    def mainloop(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.close()


def run_gui(tb, ports, title="", speed=1.0, duration=None):
    """Open the window and run the simulator until the window is closed."""
    try:
        win = SimWindow(tb, ports, title=title, speed=speed, duration=duration)
    except tk.TclError as e:
        raise SystemExit(f"[sim] can't open a window ({e}). Run without --gui to use the "
                         f"console only.") from None
    win.mainloop()
