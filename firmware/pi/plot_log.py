"""Makes the report plot from a log file.

Run:  python3 plot_log.py logs/step_20261113_160000.csv
      python3 plot_log.py            (no name = newest file in logs/)
Saves a .png next to the .csv and opens it on screen.
"""

import csv
import glob
import os
import sys

try:
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib missing. Install it with:  sudo apt install -y python3-matplotlib")

from datalog import LOG_DIR


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("The file is empty: " + path)
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def shade(ax, t, flags, color, label):
    """Shades the time ranges where flags == 1."""
    start = None
    for i, f in enumerate(flags + [0]):
        if f and start is None:
            start = t[i]
        elif not f and start is not None:
            ax.axvspan(start, t[i - 1], color=color, alpha=0.15, label=label)
            label = None
            start = None


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        files = sorted(glob.glob(os.path.join(LOG_DIR, "*.csv")), key=os.path.getmtime)
        if not files:
            sys.exit("No log files in " + LOG_DIR)
        path = files[-1]
    d = load(path)
    t = d["t"]
    camera = "cam_x" in d
    n = 3 if camera else 2
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(10, 3 * n))

    ax = axes[0]
    sent = [v if s else float("nan") for v, s in zip(d["pi_target"], d["pi_sending"])]
    ax.plot(t, sent, "--", color="grey", label="Pi sends")
    ax.plot(t, d["target"], color="black", label="board target")
    ax.plot(t, d["pitch"], label="pitch (IMU)")
    ax.plot(t, d["enc_pitch"], alpha=0.7, label="pitch (encoder)")
    shade(ax, t, [int(v) for v in d["link_lost"]], "red", "link lost")
    ax.set_ylabel("angle (deg)")
    ax.set_title("Raspberry Pi target vs rig pitch")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True)

    ax = axes[1]
    ax.plot(t, d["left_us"], label="left motors")
    ax.plot(t, d["right_us"], label="right motors")
    ax.set_ylabel("ESC pulse (us)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True)

    if camera:
        ax = axes[2]
        ax.plot(t, d["cam_x"], label="object position in image (-1 left, +1 right)")
        shade(ax, t, [1 - int(v) for v in d["found"]], "orange", "object not seen")
        ax.set_ylabel("camera")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True)

    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    png = os.path.splitext(path)[0] + ".png"
    fig.savefig(png, dpi=150)
    print("Saved", png)
    plt.show()


if __name__ == "__main__":
    main()
