"""Automatic test for the report: the Pi changes the target in steps while the
board keeps the rig balanced, then stops sending to show the link-loss safety.
Everything is saved to logs/step_<date>_<time>.csv.

Run:  python3 step_test.py      (press K1 first so the motors are armed)
Ctrl+C stops early (the file is still saved, and the board returns to level).
"""

import time

from datalog import DataLog
from link import BoardLink

# (what to do, target in degrees, how long in seconds)
PLAN = [
    ("hold", 0.0, 4),
    ("hold", 10.0, 6),
    ("hold", 0.0, 6),
    ("hold", -10.0, 6),
    ("hold", 0.0, 5),
    ("hold", 10.0, 5),
    ("stop", 10.0, 4),      # Pi goes silent -> board must return to 0 by itself
    ("hold", 0.0, 4),       # sending again
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
        log = DataLog(link, "step")
        total = sum(p[2] for p in PLAN)
        print("Test takes %d s. Saving to %s" % (total, log.path))
        for action, target, seconds in PLAN:
            link.set_target(target)
            link.sending = (action != "stop")
            if action == "stop":
                print(">> Pi STOPS sending for %d s (rig should go back to level)" % seconds)
            else:
                print(">> target %+.0f deg for %d s" % (target, seconds))
            end = time.time() + seconds
            while time.time() < end:
                tel = link.latest
                print("   pitch %6.1f  board target %5.1f  link %s"
                      % (tel["pitch"], tel["target"], "LOST" if tel["link_lost"] else "ok"),
                      end="\r")
                time.sleep(0.2)
            print()
        link.set_target(0.0)
        link.sending = True
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
