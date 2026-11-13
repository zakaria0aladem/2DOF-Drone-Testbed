"""Milestone 3 camera part: the Pi finds a coloured object and turns its
left/right position in the image into the rig's tilt target. The board still
does all the balancing; the Pi only sends the target ($SET), like console.py.
Object in the middle = level; further left/right = more tilt (max 15 deg).

Run:  python3 camera_track.py
      python3 camera_track.py --no-board    (camera only, to tune the colour)

Keys (click the camera window first):
  s = start / pause following (starts PAUSED: target stays 0)
  f = flip direction (if the rig tilts the opposite way to the object)
  q = quit
The sliders in the "colour" window pick the colour (HSV range).
Every run is saved to logs/camera_<date>_<time>.csv.
"""

import argparse
import time

import cv2
import numpy as np

WIDTH, HEIGHT = 640, 480
MAX_TARGET = 15.0       # deg, the Pi never asks for more (board limit is 20)
DEADBAND = 0.10         # ignore small movements near the middle of the image
SMOOTH = 0.15           # 0..1, smaller = smoother but slower target changes
LOST_TIME = 0.5         # s without the object -> target goes back to 0
MIN_AREA = 400          # pixels, smaller blobs are ignored as noise
ROTATE_180 = False      # set True if the picture is upside down

# starting colour range (HSV). Tune with the sliders, then copy the numbers here.
# OpenCV hue is 0-179: red ~0-10/170-179, orange ~10-25, yellow ~25-35,
# green ~40-80, blue ~100-130.
HSV_LOW = (100, 120, 70)
HSV_HIGH = (130, 255, 255)


class Camera:
    """Picamera2 on the Pi; falls back to a USB webcam (cv2) elsewhere."""

    def __init__(self):
        try:
            from picamera2 import Picamera2
            self.cam = Picamera2()
            # "RGB888" gives the bytes in BGR order, which is what OpenCV wants
            cfg = self.cam.create_preview_configuration(
                main={"size": (WIDTH, HEIGHT), "format": "RGB888"})
            self.cam.configure(cfg)
            self.cam.start()
            self.pi = True
        except ImportError:
            self.cam = cv2.VideoCapture(0)
            self.pi = False

    def read(self):
        if self.pi:
            frame = self.cam.capture_array()
        else:
            ok, frame = self.cam.read()
            if not ok:
                return None
            frame = cv2.resize(frame, (WIDTH, HEIGHT))
        if ROTATE_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def close(self):
        if self.pi:
            self.cam.stop()
        else:
            self.cam.release()


def make_sliders():
    cv2.namedWindow("colour")
    names = ["H low", "S low", "V low", "H high", "S high", "V high"]
    start = list(HSV_LOW) + list(HSV_HIGH)
    for name, value in zip(names, start):
        cv2.createTrackbar(name, "colour", value, 179 if name[0] == "H" else 255,
                           lambda v: None)


def read_sliders():
    g = lambda n: cv2.getTrackbarPos(n, "colour")
    return (np.array([g("H low"), g("S low"), g("V low")]),
            np.array([g("H high"), g("S high"), g("V high")]))


def find_object(frame, low, high):
    """Returns (mask, (x, y, radius) of the biggest blob or None)."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
    if low[0] <= high[0]:
        mask = cv2.inRange(hsv, low, high)
    else:   # hue range that wraps around (red): e.g. low 170, high 10
        mask = cv2.inRange(hsv, low, np.array([179, high[1], high[2]])) | \
               cv2.inRange(hsv, np.array([0, low[1], low[2]]), high)
    mask = cv2.dilate(cv2.erode(mask, None, iterations=2), None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return mask, None
    biggest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(biggest) < MIN_AREA:
        return mask, None
    (x, y), r = cv2.minEnclosingCircle(biggest)
    return mask, (int(x), int(y), int(r))


class Tracker:
    """Turns the object's left/right position into a tilt target."""

    def __init__(self):
        self.active = False         # 's' key
        self.sign = 1.0             # 'f' key
        self.target = 0.0
        self.cam_x = 0.0            # -1 left .. +1 right edge of the image
        self.found = False
        self.last_seen = 0.0

    def update(self, obj):
        now = time.time()
        self.found = obj is not None
        if self.found:
            self.last_seen = now
            self.cam_x = (obj[0] - WIDTH / 2) / (WIDTH / 2)

        lost = now - self.last_seen > LOST_TIME
        if not self.active or lost:
            wanted = 0.0                    # safe: back to level
        else:
            x = self.cam_x if abs(self.cam_x) > DEADBAND else 0.0
            wanted = self.sign * x * MAX_TARGET
        self.target += (wanted - self.target) * SMOOTH     # smooth changes
        return self.target


def draw(frame, mask, obj, tr, tel, fps):
    h, w = frame.shape[:2]
    cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
    band = int(DEADBAND * w / 2)
    cv2.line(frame, (w // 2 - band, 0), (w // 2 - band, h), (128, 128, 128), 1)
    cv2.line(frame, (w // 2 + band, 0), (w // 2 + band, h), (128, 128, 128), 1)
    if obj:
        x, y, r = obj
        cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
        cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)
        cv2.line(frame, (w // 2, y), (x, y), (0, 255, 255), 2)
    # small black-and-white mask in the top-right corner
    small = cv2.resize(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), (w // 4, h // 4))
    frame[0:h // 4, w - w // 4:w] = small

    state = "FOLLOWING" if tr.active else "PAUSED (press s)"
    lines = ["%s   dir %+d" % (state, tr.sign),
             "object: %s   x %+.2f" % ("found" if tr.found else "NOT SEEN", tr.cam_x),
             "Pi target: %+.1f deg" % tr.target]
    if tel:
        lines.append("board: target %+.1f  pitch %+.1f  armed %d  link %s"
                     % (tel["target"], tel["pitch"], tel["armed"],
                        "LOST" if tel["link_lost"] else "ok"))
    else:
        lines.append("board: not connected")
    lines.append("%.0f fps" % fps)
    panel = frame[0:22 * len(lines) + 8, 0:w - w // 4]
    panel[:] = (panel * 0.4).astype(np.uint8)       # darken behind the text
    for i, text in enumerate(lines):
        cv2.putText(frame, text, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if tr.active else (0, 200, 255), 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-board", action="store_true", help="camera only")
    args = ap.parse_args()

    link = log = None
    tr = Tracker()
    if not args.no_board:
        from datalog import DataLog
        from link import BoardLink
        link = BoardLink()
        link.set_target(0.0)
        log = DataLog(link, "camera", extra=lambda: {
            "cam_x": round(tr.cam_x, 3), "found": int(tr.found),
            "following": int(tr.active)})
        print("Board on %s, saving to %s" % (link.port, log.path))

    cam = Camera()
    make_sliders()
    fps, last = 0.0, time.time()
    try:
        while True:
            frame = cam.read()
            if frame is None:
                break
            low, high = read_sliders()
            mask, obj = find_object(frame, low, high)
            target = tr.update(obj)
            if link:
                link.set_target(round(target, 1))
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - last, 1e-3)
            last = now
            draw(frame, mask, obj, tr, link.latest if link else None, fps)
            cv2.imshow("camera", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                tr.active = not tr.active
                print(">> following" if tr.active else ">> paused (target back to 0)")
            elif key == ord("f"):
                tr.sign = -tr.sign
                print(">> direction flipped")
    except KeyboardInterrupt:
        pass
    low, high = read_sliders()
    print("Colour used: HSV_LOW = (%d, %d, %d), HSV_HIGH = (%d, %d, %d)"
          % (tuple(low) + tuple(high)))
    cam.close()
    cv2.destroyAllWindows()
    if link:
        link.set_target(0.0)
        time.sleep(0.3)
        print("Saved %d rows to %s" % (log.rows, log.close()))
        link.close()


if __name__ == "__main__":
    main()
