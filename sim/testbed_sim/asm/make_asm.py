"""Rebuild exc_return.bin from exc_return.s (needs `pip install ziglang`)."""
import subprocess, sys
from pathlib import Path
here = Path(__file__).resolve().parent
zig = [sys.executable, "-m", "ziglang"]
subprocess.check_call(zig + ["cc", "-target", "thumb-freestanding-eabihf", "-mcpu=cortex_m4+vfp4d16sp",
                             "-c", str(here / "exc_return.s"), "-o", str(here / "exc_return.o")])
subprocess.check_call(zig + ["objcopy", "-O", "binary", "--only-section=.text",
                             str(here / "exc_return.o"), str(here / "exc_return.bin")])
(here / "exc_return.o").unlink()
print((here / "exc_return.bin").stat().st_size, "bytes")
