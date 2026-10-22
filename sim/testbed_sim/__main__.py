"""Command line: run firmware on the simulated testbed.

    python -m testbed_sim                       (the demo firmware, in this window)
    python -m testbed_sim --gui                 (the same, with the live window)
    python -m testbed_sim Debug/drone_fw.elf --gui
    python -m testbed_sim Debug/drone_fw.elf --config my_rig.toml --set power.volts=11.1
    python -m testbed_sim fw.bin --duration 10 --csv run.csv --fast
    python -m testbed_sim --check               (is everything installed and working?)

USB port 1 (USART1) is printed in this window, USB port 2 (USART3) is
offered at socket://localhost:5333 for tbbridge.py or your own PC program.
"""
from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

DEMO_FW = Path(__file__).resolve().parent.parent / "firmware_demo" / "prebuilt" / "testbed_demo.bin"
UNICORN_VERSION = "2.1.4"


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


def _import_testbed():
    """Import the simulator, with a clear message when a package is missing."""
    try:
        from .api import Testbed
    except ImportError as e:
        raise SystemExit(f"[sim] a Python package is missing ({e}).\n"
                         f"      In the sim folder run:  pip install -r requirements.txt") from None
    return Testbed


def _firmware_path(arg):
    if arg is None:
        print(f"[sim] no firmware file given: running the demo firmware ({DEMO_FW.name})",
              file=sys.stderr)
        return DEMO_FW
    p = Path(arg)
    if not p.is_file():
        raise SystemExit(f"[sim] can't find the firmware file '{arg}'.\n"
                         f"      Build your project in STM32CubeIDE first; the file is in its "
                         f"Debug folder (for example Debug\\drone_fw.elf).")
    if p.suffix.lower() not in (".bin", ".elf", ".hex", ".ihex", ".axf", ".out"):
        print(f"[sim] warning: '{p.name}' is not a .bin, .elf or .hex file; trying it as a raw "
              f".bin", file=sys.stderr)
    return p


def _port_free(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def check(port2=5333):
    """Check the installation step by step and say how to fix what is wrong."""
    from importlib import metadata
    good = True

    def line(ok, what, fix="", warn=False):
        nonlocal good
        tag = "OK  " if ok else ("WARN" if warn else "FAIL")
        print(f"  {tag}  {what}")
        if not ok and fix:
            print(f"        -> {fix}")
        if not ok and not warn:
            good = False

    print("Checking the testbed simulator:")
    v = sys.version_info
    line(v >= (3, 10), f"Python {v.major}.{v.minor}.{v.micro}",
         "install Python 3.10 or newer from python.org")
    install = "in the sim folder run:  pip install -r requirements.txt"
    try:
        uv = metadata.version("unicorn")
        line(uv == UNICORN_VERSION, f"unicorn {uv} (the CPU emulator)",
             f"the simulator needs exactly {UNICORN_VERSION}: pip install unicorn=={UNICORN_VERSION}")
    except metadata.PackageNotFoundError:
        line(False, "unicorn (the CPU emulator) is not installed", install)
    try:
        line(True, f"pyserial {metadata.version('pyserial')} (for tbbridge.py)")
    except metadata.PackageNotFoundError:
        line(False, "pyserial is not installed (tbbridge.py needs it)", install)
    try:
        from .config import tomllib   # noqa: F401
        line(True, "TOML reader for the config files")
    except ImportError:
        line(False, "tomli is not installed (needed on Python 3.10)", install)
    try:
        import tkinter
        tkinter.Tcl()
        line(True, "tkinter (for the --gui window)")
    except Exception:
        line(False, "tkinter is missing: --gui won't work (the rest does)",
             "on Linux: sudo apt install python3-tk", warn=True)
    line(_port_free(port2), f"TCP port {port2} is free (USB port 2 of the simulated board)",
         "another simulator is probably still running: close it, or use --port2 5335",
         warn=True)
    if not good:
        print("Fix the FAIL lines above, then run this check again.")
        return 1

    from .api import Testbed
    print("  ....  booting the demo firmware for 1.5 simulated seconds")
    wall = time.perf_counter()
    tb = Testbed(DEMO_FW, quiet=True)
    tb.run(1.5)
    wall = time.perf_counter() - wall
    text = tb.read_text("USART1")
    line("QMI8658 found" in text, "firmware runs and finds the simulated IMU",
         "the simulator itself is broken: run  python -m unittest discover tests")
    line(all(e == "armed" for e in tb.state()["esc_state"]), "all four simulated ESCs armed",
         "the simulator itself is broken: run  python -m unittest discover tests")
    print(f"        (1.5 s of board time took {wall:.1f} s on this PC)")
    if good:
        print("All good. Try:  python -m testbed_sim --gui")
    return 0 if good else 1


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="python -m testbed_sim",
        description="Run STM32F407 firmware on the simulated 2-DOF testbed. "
                    "Without a firmware file it runs the demo firmware.")
    ap.add_argument("firmware", nargs="?", default=None,
                    help=".bin, .hex or .elf (CubeIDE puts them in Debug/); "
                         "leave out to run the demo firmware")
    ap.add_argument("--gui", action="store_true",
                    help="open a window that shows the rig moving, with buttons for the keys")
    ap.add_argument("--check", action="store_true",
                    help="check the installation and stop")
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

    if a.check:
        return check(a.port2)

    Testbed = _import_testbed()
    fw = _firmware_path(a.firmware)
    try:
        tb = Testbed(fw, config=a.config, overrides=_parse_set(a.set),
                     log=lambda s: print(s, file=sys.stderr, flush=True), quiet=a.quiet)
    except FileNotFoundError as e:
        raise SystemExit(f"[sim] can't open {e.filename}") from None
    except KeyError as e:
        raise SystemExit(f"[sim] config error: {e.args[0]}") from None
    except ValueError as e:
        raise SystemExit(f"[sim] can't read '{fw}': {e}") from None
    if a.csv:
        tb.log_csv(a.csv)
    ports = {"USART3": {"tcp": a.port2, "pty": a.pty}}
    if a.port1 is not None:
        ports["USART1"] = {"tcp": a.port1, "pty": a.pty}
    speed = 0 if a.fast else a.speed
    try:
        if a.gui:
            from .gui import run_gui
            run_gui(tb, ports, title=Path(fw).name, speed=speed, duration=a.duration)
        else:
            from .bridge import serve
            serve(tb, ports, console="USART1", speed=speed,
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
