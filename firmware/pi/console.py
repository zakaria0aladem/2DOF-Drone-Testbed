"""Milestone 3 console: live board data + typing a target angle.

Run:  python3 console.py
Type a number and press Enter to set the target angle (the board limits it to +/-20).
  stop  = stop sending (tests link loss: the board returns to level)
  go    = start sending again
  q     = quit
"""

import threading
import time

from link import BoardLink

PRINT_PERIOD_S = 1.0


def show(link):
    tel = link.latest
    if tel is None:
        print("waiting for $TEL from the board...")
        return
    print("pitch %6.1f  enc %6.1f  yaw %6.1f | target %5.1f  output %6.1f | "
          "L %4d  R %4d | armed %d  link %s | Pi sends %s | bad: board %d, Pi %d"
          % (tel["pitch"], tel["enc_pitch"], tel["enc_yaw"], tel["target"],
             tel["output"], tel["left_us"], tel["right_us"], tel["armed"],
             "LOST" if tel["link_lost"] else "ok",
             "%.1f" % link.target if link.sending else "nothing",
             tel["bad"], link.rx_bad))


def print_loop(link):
    while True:
        show(link)
        time.sleep(PRINT_PERIOD_S)


def main():
    link = BoardLink()
    print("Connected to the board on %s. Type a target angle and press Enter." % link.port)
    threading.Thread(target=print_loop, args=(link,), daemon=True).start()

    try:
        while True:
            cmd = input().strip().lower()
            if cmd in ("q", "quit"):
                break
            elif cmd == "stop":
                link.sending = False
                print(">> stopped sending (the board should go LOST in 0.5 s)")
            elif cmd == "go":
                link.sending = True
                print(">> sending again")
            elif cmd:
                try:
                    link.set_target(float(cmd))
                    print(">> target %.1f sent" % link.target)
                except ValueError:
                    print(">> type a number, stop, go or q")
    except KeyboardInterrupt:
        pass
    link.close()


if __name__ == "__main__":
    main()
