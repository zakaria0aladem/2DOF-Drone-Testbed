"""Checks that the simulator still behaves like the board.

    cd sim
    python -m unittest discover tests        (about 20 seconds)

They run the demo firmware (firmware_demo/prebuilt/testbed_demo.bin), the
firmware-free plant, the console commands and the example controller.
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from testbed_sim import Testbed, tbproto as P            # noqa: E402
from testbed_sim.plant import RigEnv                     # noqa: E402

FW = os.path.join(os.path.dirname(HERE), "firmware_demo", "prebuilt", "testbed_demo.bin")


def telemetry(tb, parser):
    return [P.Telemetry.unpack(p) for t, p in parser.feed(tb.read("USART3"))
            if t == P.T_TELEMETRY]


def boot_and_arm(tb, parser):
    tb.run(3.3)
    telemetry(tb, parser)
    tb.write("USART3", P.encode(P.T_ARM))
    tb.run(0.02)
    return telemetry(tb, parser)


def drive(tb, parser, u, seconds):
    last = None
    for k in range(int(round(seconds / 0.002))):
        tb.write("USART3", P.motor_cmd(k, u))
        tb.run(0.002)
        f = telemetry(tb, parser)
        last = f[-1] if f else last
    return last


class Boot(unittest.TestCase):
    def test_boot_text_and_telemetry(self):
        tb = Testbed(FW, quiet=True)
        tb.run(2.2)
        text = tb.read_text("USART1")
        self.assertIn("SYSCLK 168 MHz", text)
        self.assertIn("QMI8658 found", text)
        self.assertIn("bat=12002 mV", text)
        par = P.Parser()
        frames = par.feed(tb.read("USART3"))
        hello = [P.Hello.unpack(p) for t, p in frames if t == P.T_HELLO]
        tel = [P.Telemetry.unpack(p) for t, p in frames if t == P.T_TELEMETRY]
        self.assertEqual(hello[0].rate_hz, 500)
        self.assertGreater(len(tel), 300)
        self.assertEqual(par.crc_errors, 0)
        # board time in the frames advances 2 ms per frame
        dts = {b.t_us - a.t_us for a, b in zip(tel, tel[1:])}
        self.assertEqual(dts, {2000})
        # still and level: about 1 g on z, no rotation
        self.assertAlmostEqual(tel[-1].acc[2], 9.81, delta=0.8)
        self.assertLess(max(abs(g) for g in tel[-1].gyro), 0.05)

    def test_same_run_same_result(self):
        outs = []
        for _ in range(2):
            tb = Testbed(FW, quiet=True)
            tb.run(1.8)
            outs.append(tb.read("USART3") + tb.read("USART1"))
        self.assertEqual(outs[0], outs[1])


class ClosedLoop(unittest.TestCase):
    def test_yaw_pitch_and_safety(self):
        tb = Testbed(FW, quiet=True)
        par = P.Parser()
        f = boot_and_arm(tb, par)
        self.assertTrue(f[-1].armed)
        # diagonal pair faster -> yaw turns; encoder and gyro agree with the truth
        drive(tb, par, [1250] * 4, 0.6)
        last = drive(tb, par, [1300, 1200, 1200, 1300], 0.6)
        s = tb.state()
        self.assertGreater(s["yaw"], 0.05)
        self.assertAlmostEqual(last.q[0], s["yaw"], delta=0.01)
        self.assertAlmostEqual(last.gyro[2], s["yaw_rate"], delta=0.05)
        # front pair faster -> nose up (negative pitch) to the end stop
        last = drive(tb, par, [1200, 1350, 1200, 1350], 1.0)
        self.assertLess(tb.state()["pitch"], -0.5)
        self.assertAlmostEqual(last.q[1], tb.state()["pitch"], delta=0.01)
        # commands stop -> timeout flag, motors to idle
        tb.run(0.3)
        f = telemetry(tb, par)
        self.assertTrue(f[-1].flags & P.F_CMD_TIMEOUT)
        self.assertEqual(tb.state()["esc_us"], [1000.0] * 4)
        # KEY1 kills
        drive(tb, par, [1250] * 4, 0.1)
        tb.press_key(1, 0.1)
        f = telemetry(tb, par)
        self.assertTrue(f[-1].flags & P.F_KILLED)
        self.assertFalse(f[-1].armed)


def patch(tb, addr, code):
    tb.m.uc.mem_write(addr, code)
    tb.m.uc.ctl_remove_cache(addr, addr + len(code))      # drop the translated old code


class Faults(unittest.TestCase):
    def test_watchdog_resets(self):
        tb = Testbed(FW, quiet=True)
        logs = []
        tb.m.log = logs.append
        # make the firmware hang where it is, so nobody refreshes the watchdog
        tb.run(1.6)
        patch(tb, tb.m._pc & ~1, b"\xfe\xe7")               # b .  (loop forever)
        tb.run(0.5)
        self.assertTrue(any("IWDG watchdog" in s for s in logs), logs)
        self.assertGreaterEqual(tb.m.resets, 1)

    def test_crash_is_reported(self):
        tb = Testbed(FW, quiet=True)
        logs = []
        tb.m.log = logs.append
        tb.run(1.6)
        patch(tb, tb.m._pc & ~1, b"\xff\xde")               # undefined instruction
        tb.run(0.01)
        self.assertTrue(any("HardFault: undefined instruction" in s for s in logs), logs)


class Plant(unittest.TestCase):
    def test_plant_matches_full_sim_physics(self):
        env = RigEnv(noise=False)
        for _ in range(500):
            env.step([1250] * 4)
        for _ in range(300):
            o = env.step([1300, 1200, 1200, 1300])
        self.assertGreater(o.true_q[0], 0.05)
        self.assertAlmostEqual(o.q[0], o.true_q[0], delta=2 * 3.1416 / 2880)
        self.assertAlmostEqual(o.gyro[2], o.true_dq[0], delta=0.03)

    def test_plant_repeatable(self):
        a = [RigEnv(seed=3).rollout([[1250, 1300, 1300, 1250]] * 200)[-1] for _ in range(2)]
        self.assertEqual(a[0], a[1])

    def test_controller_template_reaches_yaw_target(self):
        sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools", "examples"))
        from controller_template import HoldAngle
        ctrl = HoldAngle(q_ref=0.5)
        ctrl.us_max = 1400                                  # the demo firmware's limit
        env = RigEnv(noise=False)
        o = env.reset()
        for _ in range(int(3 / env.dt)):
            o = env.step(ctrl.step(o, o.t))
        self.assertAlmostEqual(o.true_q[0], 0.5, delta=0.03)


class Console(unittest.TestCase):
    def test_commands_and_clean_lines(self):
        import io
        import queue
        from testbed_sim.bridge import serve
        tb = Testbed(FW, log=lambda s: None)
        cmds = queue.Queue()
        for c in ("speed 0", "push yaw 0.05", "hold k2", "pause", "resume"):
            cmds.put(c)
        out = io.StringIO()
        serve(tb, {"USART3": {"tcp": 0}}, speed=1.0, duration=1.2, interactive=False,
              out=out, commands=cmds)
        self.assertGreater(tb.state()["yaw"], 0.02)          # the push moved the frame
        self.assertIn("\nt=1 s bat=12002 mV", out.getvalue())   # firmware lines stay whole

    def test_command_line_runs_demo_without_arguments(self):
        import contextlib
        import io
        from testbed_sim.__main__ import main
        err = io.StringIO()
        with contextlib.redirect_stdout(io.StringIO()) as out, contextlib.redirect_stderr(err):
            code = main(["--duration", "1.2", "--fast", "--no-input", "--port2", "0"])
        self.assertEqual(code, 0)
        self.assertIn("running the demo firmware", err.getvalue())
        self.assertIn("QMI8658 found", out.getvalue())


if __name__ == "__main__":
    unittest.main()
