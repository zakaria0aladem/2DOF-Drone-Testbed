"""Larger step test: bigger target changes, 3 s each.
0 (from wherever the rig starts) -> -30 -> +20 -> 0 -> +30.
Everything is saved to logs/step2_<date>_<time>.csv.

Note: the board limits targets to +/- TARGET_LIMIT_DEG in bsp_comm.c
(20 deg unless it was raised), so larger targets are cut to that limit.

Run:  python3 step_test2.py     (press K1 first so the motors are armed)
Ctrl+C stops early (the file is still saved, and the board returns to level).
"""

import time

from datalog import DataLog
from link import BoardLink

# (target in degrees, how long in seconds)
PLAN = [
    (0.0, 3),       # from the starting angle to level
    (-30.0, 3),
    (20.0, 3),
    (0.0, 3),
    (30.0, 3),
]


def wait_for_board(link):
    print("Waiting for telemetry from the board...")
    while link.latest is None:
        time.sleep(0.1)
    if not link.latest["armed"]:
        print("Motors are NOT armed. Press K1, wait for the countdown beeps...")
        while not link.latest["armed"]:
            time.sleep(0.1)
    print("Armed. Starting in 3 s - keep the power switch within reach.")
    time.sleep(3)


def main():
    link = BoardLink()
    link.set_target(0.0)
    try:
        wait_for_board(link)
        log = DataLog(link, "step2")
        total = sum(p[1] for p in PLAN)
        print("Test takes %d s. Saving to %s" % (total, log.path))
        for target, seconds in PLAN:
            link.set_target(target)
            print(">> target %+.0f deg for %d s" % (target, seconds))
            end = time.time() + seconds
            while time.time() < end:
                tel = link.latest
                print("   pitch %6.1f  board target %5.1f" % (tel["pitch"], tel["target"]),
                      end="\r")
                time.sleep(0.2)
            print()
        link.set_target(0.0)
        time.sleep(1)
        print("Done: %d rows saved to %s" % (log.rows, log.close()))
        print("Make the plot:  python3 plot_log.py %s" % log.path)
    except KeyboardInterrupt:
        print("\nStopped early.")
        try:
            print("Saved %d rows to %s" % (log.rows, log.close()))
        except NameError:
            pass
    link.close()


if __name__ == "__main__":
    main()
