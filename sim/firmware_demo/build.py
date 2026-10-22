"""Build the demo firmware without CubeIDE (Windows, Linux or macOS).

    pip install ziglang          # a C cross-compiler packaged for pip (clang inside)
    python build.py              # -> build/testbed_demo.elf / .bin  (-Os)
    python build.py --debug      # -O0, like CubeIDE's Debug configuration

The first run downloads ST's F4 HAL and CMSIS with git into ./_deps.
You only need this to change the demo: prebuilt/testbed_demo.bin is what the
simulator tests use, and your own CubeIDE builds run in the simulator the same way.
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEPS = HERE / "_deps"
REPOS = {
    "hal": "https://github.com/STMicroelectronics/stm32f4xx-hal-driver",
    "dev": "https://github.com/STMicroelectronics/cmsis-device-f4",
    "cmsis": "https://github.com/STMicroelectronics/cmsis-core",
}
HAL_MODULES = ["", "_cortex", "_rcc", "_rcc_ex", "_gpio", "_dma", "_dma_ex", "_i2c",
               "_tim", "_tim_ex", "_uart", "_adc", "_adc_ex", "_iwdg", "_pwr",
               "_pwr_ex", "_flash", "_flash_ex", "_exti"]


def fetch_deps():
    DEPS.mkdir(exist_ok=True)
    for name, url in REPOS.items():
        if not (DEPS / name).exists():
            print("fetching", url)
            subprocess.check_call(["git", "clone", "-q", "--depth", "1", url, str(DEPS / name)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--debug", action="store_true", help="build with -O0 -g")
    ap.add_argument("--out", default="build")
    args = ap.parse_args()

    fetch_deps()
    out = HERE / args.out
    out.mkdir(exist_ok=True)
    cmsis_inc = DEPS / "cmsis" / "Include"
    if not cmsis_inc.exists():                      # layout differs between mirrors
        cmsis_inc = next(DEPS.glob("cmsis/**/core_cm4.h")).parent

    srcs = sorted((HERE / "Core" / "Src").glob("*.c")) + sorted((HERE / "telemetry").glob("*.c"))
    srcs.append(HERE / "compat" / "minilibc.c")
    srcs += [DEPS / "hal" / "Src" / f"stm32f4xx_hal{m}.c" for m in HAL_MODULES]
    srcs.append(HERE / "Core" / "Src" / "startup_stm32f407xx.s")

    zig = [sys.executable, "-m", "ziglang", "cc"]
    flags = ["-target", "thumb-freestanding-eabihf", "-mcpu=cortex_m4+vfp4d16sp",
             "-DSTM32F407xx", "-DUSE_HAL_DRIVER", "-ffunction-sections", "-fdata-sections",
             "-Wall", "-Wno-unused-parameter",
             "-I", str(HERE / "Core" / "Inc"), "-I", str(HERE / "telemetry"),
             "-I", str(DEPS / "hal" / "Inc"), "-I", str(DEPS / "dev" / "Include"),
             "-I", str(cmsis_inc), "-I", str(HERE / "compat")]
    flags += ["-O0", "-g"] if args.debug else ["-Os", "-g"]
    elf = out / "testbed_demo.elf"
    cmd = zig + flags + [str(s) for s in srcs] + [
        "-Wl,--gc-sections", "-Wl,-e,Reset_Handler", "-T", str(HERE / "STM32F407VETX_FLASH.ld"),
        "-o", str(elf)]
    subprocess.check_call(cmd)
    subprocess.check_call([sys.executable, "-m", "ziglang", "objcopy", "-O", "binary",
                           str(elf), str(out / "testbed_demo.bin")])
    print("built", elf, os.path.getsize(out / "testbed_demo.bin"), "bytes")


if __name__ == "__main__":
    main()
