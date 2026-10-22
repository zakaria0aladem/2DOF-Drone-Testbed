"""Run firmware on the simulated testbed from Python: boot, arm, turn the yaw
joint with a motor pattern, and print what happened.

    python examples/run_firmware.py                      (the demo firmware)
    python examples/run_firmware.py path/to/drone_fw.elf  (yours)

Your firmware has to speak the telemetry protocol on USART3 for the ARM and
MOTOR_CMD part; the rest works with any firmware.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from testbed_sim import Testbed               # noqa: E402
from testbed_sim import tbproto as P          # noqa: E402

fw = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "firmware_demo",
                                                         "prebuilt", "testbed_demo.bin")
tb = Testbed(fw)
tb.log_csv("run_firmware.csv")                # true rig state, 1 kHz

tb.run(3.3)                                   # boot; the ESCs need ~1 s of 1000 us to arm
print(tb.read_text("USART1"))

parser = P.Parser()
tb.read("USART3")                             # drop what was sent before we listen
tb.write("USART3", P.encode(P.T_ARM))

for k in range(1500):                         # 3 s, one command every 2 ms
    u = [1250] * 4 if k < 500 else [1300, 1200, 1200, 1300]   # ESC1+4 faster: yaw turns
    tb.write("USART3", P.motor_cmd(k, u))
    tb.run(0.002)
    for ftype, payload in parser.feed(tb.read("USART3")):
        if ftype == P.T_TELEMETRY:
            last = P.Telemetry.unpack(payload)
    if k % 250 == 0:
        s = tb.state()
        print(f"t={s['t']:.2f} s  yaw={s['yaw']:+.3f} rad  pitch={s['pitch']:+.3f} rad  "
              f"rpm={[round(r) for r in s['motor_rpm']]}  board says q={last.q[0]:+.3f}, "
              f"gyro z={last.gyro[2]:+.3f} rad/s")

print(tb.read_text("USART1"))
tb.close()
print("true rig state saved in run_firmware.csv")
