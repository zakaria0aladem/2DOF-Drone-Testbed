"""System-identification data from the firmware-free rig model.

Random +-60 us steps (PRBS) on a yaw pattern and a pitch pattern around
1250 us, recorded at 500 Hz to a CSV with the same column names as
tbbridge.py logs from the real rig.

    python examples/plant_prbs.py            -> prbs_sim.csv (60 s of data in a few seconds)
"""
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from testbed_sim.plant import RigEnv          # noqa: E402

env = RigEnv(dt=0.002, seed=1)                # add config="my_rig.toml" once you measured it
env.log_csv("prbs_sim.csv")
rng = random.Random(0)
base, amp, hold = 1250.0, 60.0, 0.05          # us, us, s between possible switches
yaw_mix = (1, -1, -1, 1)                      # ESC1+ESC4 vs ESC2+ESC3
pitch_mix = (-1, 1, -1, 1)                    # front pair (ESC2, ESC4) vs rear pair

a = b = 1
obs = env.reset()
steps_per_hold = round(hold / env.dt)
for k in range(int(60 / env.dt)):
    if k % steps_per_hold == 0:
        a = rng.choice((-1, 1))
        b = rng.choice((-1, 1))
    u = [base + amp * (a * y + b * p) for y, p in zip(yaw_mix, pitch_mix)]
    obs = env.step(u)
env.close()
print(f"60 s recorded to prbs_sim.csv, final yaw {obs.true_q[0]:.2f} rad, "
      f"pitch {obs.true_q[1]:.2f} rad")
