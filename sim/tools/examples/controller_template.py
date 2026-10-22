"""Controller interface for tbbridge.py run --controller file.py:Class.

A controller is any class with
    reset(hello)          optional, called once after the handshake
    step(obs, t) -> [u1, u2, u3, u4]   pulse widths in us, called once per telemetry frame

obs is a tbproto.Telemetry: obs.q, obs.dq (rad, rad/s), obs.acc, obs.gyro,
obs.motor_us (what the board applied), obs.flags. t is board time in seconds.

step() runs inside the receive loop, so keep it under the sample period
(2 ms at 500 Hz). A PIML model or a DeepC QP goes here; if it is slower,
lower --rate or run the solver in a worker and return the latest result.
The board clamps to its us_min..us_max, and cuts to idle if commands stop
for 100 ms.
"""


class HoldAngle:
    """Toy PID on joint 1 (yaw) using a differential between motor pairs.

    mix (1, -1, -1, 1) speeds up ESC1+ESC4 and slows ESC2+ESC3: their
    propellers spin the same way, so the drag torque turns the frame in yaw.
    That fits the simulator's default motor order; on the real rig check how
    the ESCs are wired and flip signs until a positive u moves q1 positive.
    The integral term (ki) is needed because joint friction stops a pure PD
    short of q_ref (rad).

        python tbbridge.py run --port socket://localhost:5333 --arm
            --controller examples/controller_template.py:HoldAngle --duration 10
    """

    def __init__(self, q_ref=0.5, base=1250.0, kp=300.0, ki=100.0, kd=40.0,
                 mix=(1, -1, -1, 1), i_max=150.0):
        self.q_ref, self.base, self.mix = q_ref, base, mix
        self.kp, self.ki, self.kd, self.i_max = kp, ki, kd, i_max
        self.us_min, self.us_max = 1000, 2000
        self.integral = 0.0
        self.t_prev = None

    def reset(self, hello):
        self.us_min, self.us_max = hello.us_min, hello.us_max
        self.integral = 0.0
        self.t_prev = None

    def step(self, obs, t):
        e = self.q_ref - obs.q[0]
        dt = 0.0 if self.t_prev is None else t - self.t_prev
        self.t_prev = t
        self.integral = min(self.i_max, max(-self.i_max, self.integral + self.ki * e * dt))
        u = self.kp * e + self.integral - self.kd * obs.dq[0]
        return [min(self.us_max, max(self.us_min, self.base + m * u)) for m in self.mix]


class Replay:
    """Feed a recorded or precomputed input sequence (e.g. for DeepC data collection)."""

    def __init__(self, csv_path, columns=("cmd1", "cmd2", "cmd3", "cmd4")):
        import csv
        with open(csv_path) as f:
            self.rows = [[float(r[c]) for c in columns] for r in csv.DictReader(f)]
        self.k = 0

    def step(self, obs, t):
        u = self.rows[min(self.k, len(self.rows) - 1)]
        self.k += 1
        return u
