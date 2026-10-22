"""bridge.py - connect the simulated board's USB ports to programs on your PC.

Each USB serial port of the board becomes a TCP port on your computer.
Programs that use pyserial open it with the URL `socket://localhost:PORT`
instead of `COM9`, for example

    python tbbridge.py log --port socket://localhost:5333 --duration 10

On Linux and macOS the bridge can also make a pseudo terminal (`--pty`), a
file like /dev/pts/5 that any serial program opens as if it were a real port.

The simulation is paced to the wall clock (speed 1.0 = real time), so a PC
program sees the board answer with the same timing as the real one.
"""
from __future__ import annotations

import os
import queue
import socket
import sys
import threading
import time

from .core import PS_PER_S


class _Port:
    """One board USB port shared to TCP clients (and optionally a pty)."""

    def __init__(self, tb, uart, tcp_port=None, pty=False, log=print):
        self.tb, self.uart, self.log = tb, uart, log
        self.clients = []
        self.server = None
        self.pty_master = None
        self.pty_name = None
        if tcp_port is not None:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("127.0.0.1", tcp_port))
            s.listen(4)
            s.setblocking(False)
            self.server = s
        if pty:
            if os.name == "nt":
                raise RuntimeError("--pty needs Linux or macOS; on Windows use the TCP port")
            import tty
            master, slave = os.openpty()
            tty.setraw(slave)
            os.set_blocking(master, False)
            self.pty_master, self.pty_slave = master, slave
            self.pty_name = os.ttyname(slave)

    def describe(self):
        where = []
        if self.server:
            where.append(f"socket://localhost:{self.server.getsockname()[1]}")
        if self.pty_name:
            where.append(self.pty_name)
        return " and ".join(where)

    def poll(self):
        """Move bytes both ways. Called between simulation steps."""
        if self.server:
            try:
                while True:
                    c, addr = self.server.accept()
                    c.setblocking(False)
                    c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.clients.append(c)
                    self.log(f"[sim] {self.uart}: PC program connected")
            except (BlockingIOError, InterruptedError):
                pass
        incoming = bytearray()
        for c in list(self.clients):
            try:
                while True:
                    d = c.recv(65536)
                    if not d:
                        raise ConnectionResetError
                    incoming += d
            except (BlockingIOError, InterruptedError):
                pass
            except OSError:
                self.clients.remove(c)
                c.close()
                self.log(f"[sim] {self.uart}: PC program disconnected")
        if self.pty_master is not None:
            try:
                while True:
                    d = os.read(self.pty_master, 65536)
                    if not d:
                        break
                    incoming += d
            except (BlockingIOError, OSError):
                pass
        if incoming:
            self.tb.write(self.uart, incoming)
        out = self.tb.read(self.uart)
        if out:
            for c in list(self.clients):
                try:
                    c.sendall(out)
                except OSError:
                    self.clients.remove(c)
                    c.close()
            if self.pty_master is not None:
                try:
                    os.write(self.pty_master, out)
                except OSError:
                    pass                            # nobody has the pty open
        return out

    def close(self):
        for c in self.clients:
            c.close()
        if self.server:
            self.server.close()
        if self.pty_master is not None:
            os.close(self.pty_master)
            os.close(self.pty_slave)


def _stdin_lines():
    """Lines typed in the console, read on a thread so the sim never waits."""
    q = queue.Queue()

    def reader():
        for line in sys.stdin:
            q.put(line.strip())
    threading.Thread(target=reader, daemon=True).start()
    return q


HELP = """commands (type and press Enter):
  k1 / k2          press KEY1 / KEY2 for 0.2 s
  hold k1          keep KEY1 pressed (release with: release k1)
  reset            press the board's RESET button
  push yaw 0.05    push the frame: torque in N m for 0.1 s (yaw or pitch)
  state            print the true rig state
  quit             stop the simulator"""


def serve(tb, ports, console="USART1", speed=1.0, duration=None, step=0.0005,
          interactive=True, out=sys.stdout):
    """Run the testbed and bridge its USB ports until Ctrl+C / quit / duration.

    ports: {"USART3": {"tcp": 5333, "pty": False}, ...}
    console: UART whose text is printed here (None for none)."""
    log = tb._log
    bridges = []
    for uart, o in ports.items():
        b = _Port(tb, uart, o.get("tcp"), o.get("pty", False), log)
        bridges.append(b)
        log(f"[sim] board USB port {'1' if uart == 'USART1' else '2'} ({uart}): "
            f"{b.describe()}")
    cmds = _stdin_lines() if interactive else None
    if interactive:
        log(HELP)
    held = set()
    wall0 = time.perf_counter()
    sim0 = tb.t
    win_wall, win_sim = wall0, tb.t          # speed measured over the last ~10 s
    last_report = 0.0
    try:
        while duration is None or tb.t - sim0 < duration:
            tb.run(step)
            for b in bridges:
                b.poll()
            if console and console not in ports:
                text = tb.read_text(console)
                if text:
                    out.write(text)
                    out.flush()
            while cmds is not None and not cmds.empty():
                if _command(tb, cmds.get(), held, log) == "quit":
                    return
            # pace to the wall clock
            if speed:
                ahead = (tb.t - sim0) / speed - (time.perf_counter() - wall0)
                if ahead > 0:
                    time.sleep(ahead)
                elif ahead < -0.5:
                    wall0 += -ahead - 0.5          # don't try to catch up forever
                now = time.perf_counter()
                if now - win_wall >= 10:
                    rate = (tb.t - win_sim) / (now - win_wall)
                    if rate < 0.95 * speed and now - last_report > 30:
                        log(f"[sim] note: running at {rate:.2f}x real time "
                            f"(the PC can't keep up; timing seen by PC programs is stretched)")
                        last_report = now
                    win_wall, win_sim = now, tb.t
    except KeyboardInterrupt:
        pass
    finally:
        for b in bridges:
            b.close()


def _command(tb, line, held, log):
    w = line.lower().split()
    if not w:
        return None
    try:
        if w[0] in ("k1", "k2"):
            k = int(w[0][1])
            tb.board.press_key(k, True)
            tb.m.after(int(0.2 * PS_PER_S), lambda: tb.board.press_key(k, False), world=True)
        elif w[0] == "hold" and len(w) > 1:
            tb.board.press_key(int(w[1][-1]), True)
            held.add(w[1])
        elif w[0] == "release" and len(w) > 1:
            tb.board.press_key(int(w[1][-1]), False)
            held.discard(w[1])
        elif w[0] == "reset":
            tb.reset()
        elif w[0] == "push":
            if len(w) < 2 or w[1] not in ("yaw", "pitch"):
                log("[sim] say which joint: push yaw 0.05  or  push pitch -0.02")
                return None
            tq = float(w[2]) if len(w) > 2 else 0.05
            log(f"[sim] pushing {w[1]} with {tq:g} N m for 0.1 s")
            tb.rig.disturbance = [tq if w[1] == "yaw" else 0.0, tq if w[1] == "pitch" else 0.0]
            tb.m.after(int(0.1 * PS_PER_S), lambda: setattr(tb.rig, "disturbance", [0.0, 0.0]),
                       world=True)
        elif w[0] == "state":
            s = tb.state()
            log("[sim] " + ", ".join(f"{k}={_fmt(v)}" for k, v in s.items()))
        elif w[0] in ("quit", "exit", "q"):
            return "quit"
        else:
            log(HELP)
    except (ValueError, IndexError):
        log(HELP)
    return None


def _fmt(v):
    if isinstance(v, float):
        return f"{v:.4g}"
    if isinstance(v, list):
        return "[" + ", ".join(_fmt(x) for x in v) + "]"
    return str(v)
