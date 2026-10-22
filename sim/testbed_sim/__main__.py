"""Command line: run firmware on the simulated testbed.

    python -m testbed_sim Debug/drone_fw.bin
    python -m testbed_sim Debug/drone_fw.elf --config my_rig.toml --set power.volts=11.1
    python -m testbed_sim fw.bin --duration 10 --csv run.csv --fast

USB port 1 (USART1) is printed in this window, USB port 2 (USART3) is
offered at socket://localhost:5333 for tbbridge.py or your own PC program.
"""
from __future__ import annotations

import argparse
import sys

from .api import Testbed


def _parse_set(items):
    """--set a.b=value  ->  {"a": {"b": value}} (value in TOML syntax: 3, 1.5, "x", [1, 2])."""
    from .config import tomllib
    out = {}
    for item in items:
        if "=" not in item:
            raise SystemExit(f"--set needs key=value, got '{item}'")
        key, val = item.split("=", 1)
        try:
            v = tomllib.loads(f"v = {val}")["v"]
        except Exception:
            v = val                                    # bare words are strings
        d = out
        parts = key.strip().split(".")
        for p in parts[:-1]:
            d = d.setdefault(p, {})
        d[parts[-1]] = v
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m testbed_sim",
                                 description="Run STM32F407 firmware on the simulated 2-DOF testbed.")
    ap.add_argument("firmware", help=".bin, .hex or .elf (CubeIDE puts them in Debug/)")
    ap.add_argument("--config", help="your .toml with measured values (see default_config.toml)")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="change one config value, e.g. --set power.current_limit_a=3")
    ap.add_argument("--port2", type=int, default=5333, metavar="TCP",
                    help="TCP port for board USB port 2 / USART3 (default 5333)")
    ap.add_argument("--port1", type=int, default=None, metavar="TCP",
                    help="also offer USB port 1 / USART1 on this TCP port (it is then not "
                         "printed here)")
    ap.add_argument("--pty", action="store_true", help="also make pseudo terminals (Linux/macOS)")
    ap.add_argument("--duration", type=float, default=None, help="stop after this many seconds")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="1 = real time (default), 0.5 = half speed, 0 = as fast as possible")
    ap.add_argument("--fast", action="store_true", help="same as --speed 0")
    ap.add_argument("--csv", help="log the true rig state to this CSV (1 kHz)")
    ap.add_argument("--quiet", action="store_true", help="hide [sim] messages")
    ap.add_argument("--no-input", action="store_true", help="don't read commands from the console")
    a = ap.parse_args(argv)

    tb = Testbed(a.firmware, config=a.config, overrides=_parse_set(a.set),
                 log=lambda s: print(s, file=sys.stderr, flush=True), quiet=a.quiet)
    if a.csv:
        tb.log_csv(a.csv)
    ports = {"USART3": {"tcp": a.port2, "pty": a.pty}}
    if a.port1 is not None:
        ports["USART1"] = {"tcp": a.port1, "pty": a.pty}
    from .bridge import serve
    try:
        serve(tb, ports, console="USART1", speed=0 if a.fast else a.speed,
              duration=a.duration, interactive=not a.no_input)
    except OSError as e:
        if getattr(e, "errno", None) in (48, 98, 10048):      # address in use
            raise SystemExit(f"[sim] a TCP port is already in use ({e}). Another simulator is "
                             f"probably still running: close it, or pick other ports with "
                             f"--port2 5335") from None
        raise
    finally:
        tb.close()
        s = tb.state()
        print(f"[sim] stopped at t={s['t']:.3f} s", file=sys.stderr)


if __name__ == "__main__":
    main()
