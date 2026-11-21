"""IoT bridge: board <-> MQTT, plus the web dashboard and camera tracking.

This is the ONE program that owns the board's serial port while the dashboard
is used (do not run console.py, step_test.py, camera_track.py or the ROS 2
node at the same time: only one program can open the port).

What it does
  board  --$TEL-->  this script  --MQTT-->  any dashboard (web page, Node-RED, phone...)
  board  <--$SET/$YAW/$DIS--  this script  <--MQTT--  dashboard buttons

MQTT topics (broker = mosquitto on this Pi, port 1883, websockets on 9001)
  testbed/tel          board telemetry, JSON, every $TEL line (50 Hz).
                       keys = protocol.TEL_FIELDS (+ "pi_time")
  testbed/status       Pi status, JSON, 2 Hz, retained: mode, logging, link...
  testbed/event        short text messages ("mode -> camera", "link lost"...)
  testbed/cmd/pitch    "10.5"   new pitch target in deg (manual mode only)
  testbed/cmd/yaw      "-45"    new yaw target in deg
  testbed/cmd/mode     "manual" or "camera"
  testbed/cmd/log      "start" or "stop"
  testbed/cmd/disarm   anything: disarms the motors (arming stays on K1)
  testbed/gains        gains the board is using, JSON, retained (1 Hz from the board)
  testbed/cmd/gains    {"axis": "pitch", "kp": 6, "ki": 2.5, "kd": 0.85}
                       or {"axis": "yaw", "kp": 3, "ki": 0.5, "kd": 0.5}

Web dashboard: http://<pi address>:8080   (camera picture: /camera.mjpg,
saved logs: /logs/)

Run:  python3 mqtt_bridge.py
      python3 mqtt_bridge.py --port /dev/pts/3 --no-camera   (with fake_board.py)
"""

import argparse
import json
import os
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

import paho.mqtt.client as mqtt

import protocol
from datalog import DataLog, LOG_DIR
from link import BoardLink

HERE = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(HERE, "dashboard")

PITCH_LIMIT = 20.0      # deg, largest pitch target the dashboard may ask for
YAW_LIMIT = 150.0       # deg, keeps the cables from wrapping
BOARD_TIMEOUT = 0.5     # s without $TEL = board not connected
STATUS_PERIOD = 0.5     # s


class Bridge:
    def __init__(self, args):
        self.link = BoardLink(args.port)
        self.link.set_target(0.0)
        self.mode = "manual"            # "manual" or "camera"
        self.manual_pitch = 0.0
        self.log = None
        self.lock = threading.Lock()
        self.was_connected = False
        self.was_lost = False
        self.camera = None if args.no_camera else CameraWorker(self)

        self.mqtt = make_client()
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_message = self._on_message
        self.mqtt.will_set("testbed/status", json.dumps({"bridge": "offline"}), retain=True)
        self.mqtt.connect(args.broker, 1883, keepalive=10)
        self.mqtt.loop_start()

        self.link.listeners.append(self._on_tel)
        self.link.gain_listeners.append(self._on_gains)
        threading.Thread(target=self._status_loop, daemon=True).start()

    # ---------- board -> MQTT ----------
    def _on_tel(self, tel):
        data = {k: tel[k] for k in protocol.TEL_FIELDS if k in tel}
        data["pi_time"] = round(tel["pi_time"], 3)
        self.mqtt.publish("testbed/tel", json.dumps(data))
        lost = bool(tel["link_lost"])
        if lost and not self.was_lost:
            self.event("board reports link lost: pitch target back to 0")
        self.was_lost = lost

    def _on_gains(self, gains):
        self.mqtt.publish("testbed/gains", json.dumps(gains), retain=True)

    def _status_loop(self):
        while True:
            tel = self.link.latest
            connected = tel is not None and time.time() - tel["pi_time"] < BOARD_TIMEOUT
            if connected != self.was_connected:
                self.event("board connected" if connected else "board NOT responding")
                self.was_connected = connected
            cam = self.camera.info() if self.camera else {"available": False}
            status = {
                "bridge": "online",
                "port": self.link.port,
                "board_connected": connected,
                "state": board_state(tel),
                "mode": self.mode,
                "pitch_cmd": round(self.link.target, 1),
                "yaw_cmd": self.link.yaw_target,
                "logging": self.log is not None,
                "log_file": os.path.basename(self.log.path) if self.log else None,
                "log_rows": self.log.rows if self.log else 0,
                "rx_good": self.link.rx_good,
                "rx_bad": self.link.rx_bad,
                "tx": self.link.tx_count,
                "camera": cam,
            }
            self.mqtt.publish("testbed/status", json.dumps(status), retain=True)
            time.sleep(STATUS_PERIOD)

    def event(self, text):
        print(time.strftime("%H:%M:%S"), text)
        self.mqtt.publish("testbed/event", json.dumps({"t": time.time(), "text": text}))

    # ---------- MQTT -> board ----------
    def _on_connect(self, client, *args):
        client.subscribe("testbed/cmd/#")
        self.event("bridge started")

    def _on_message(self, client, userdata, msg):
        cmd = msg.topic.rsplit("/", 1)[-1]
        text = msg.payload.decode("utf-8", "replace").strip()
        with self.lock:
            try:
                self._handle(cmd, text)
            except (ValueError, KeyError, TypeError):
                self.event("rejected %s command %r: not a valid value" % (cmd, text))

    def _handle(self, cmd, text):
        if cmd == "pitch":
            value = clamp_number(text, PITCH_LIMIT)
            self.manual_pitch = value
            if self.mode == "manual":
                self.link.set_target(value)
                self.event("pitch target %.1f" % value)
            else:
                self.event("pitch command ignored: camera mode sets pitch")
        elif cmd == "yaw":
            value = clamp_number(text, YAW_LIMIT)
            self.link.set_yaw(value)
            self.event("yaw target %.1f" % value)
        elif cmd == "mode":
            if text not in ("manual", "camera"):
                raise ValueError
            if text == "camera" and not (self.camera and self.camera.available):
                self.event("camera mode not available (no camera)")
                return
            self.mode = text
            if text == "manual":
                self.link.set_target(0.0)       # safe: back to level
                self.manual_pitch = 0.0
            self.event("mode -> %s" % text)
        elif cmd == "log":
            if text == "start" and self.log is None:
                self.log = DataLog(self.link, "dash", extra=self._log_extra)
                self.event("logging to %s" % os.path.basename(self.log.path))
            elif text == "stop" and self.log is not None:
                path = self.log.close()
                self.event("saved %d rows to %s" % (self.log.rows, os.path.basename(path)))
                self.log = None
        elif cmd == "gains":
            data = json.loads(text)
            axis = data["axis"]
            if axis not in ("pitch", "yaw"):
                raise ValueError
            names = ("kp", "ki", "kd")
            values = [float(data[n]) for n in names]
            for n, v in zip(names, values):
                limit = protocol.GAIN_LIMITS["%s_%s" % (axis, n)]
                if not 0.0 <= v <= limit:
                    self.event("rejected %s %s = %g (allowed 0 to %g)" % (axis, n, v, limit))
                    return
            if axis == "pitch":
                self.link.set_pitch_gains(*values)
            else:
                self.link.set_yaw_gains(*values)
            self.event("%s gains sent: %s" % (axis, ", ".join(
                "%s %g" % (n, v) for n, v in zip(names, values))))
        elif cmd == "disarm":
            self.link.disarm()
            self.mode = "manual"
            self.link.set_target(0.0)
            self.event("DISARM sent")
        else:
            self.event("unknown command %r" % cmd)

    def _log_extra(self):
        extra = {"mode": self.mode, "yaw_cmd": self.link.yaw_target}
        if self.camera:
            extra.update(self.camera.log_columns())
        return extra

    def close(self):
        self.link.set_target(0.0)
        if self.log:
            print("Saved", self.log.close())
        time.sleep(0.3)
        self.mqtt.publish("testbed/status", json.dumps({"bridge": "offline"}), retain=True)
        self.mqtt.loop_stop()
        self.link.close()


class CameraWorker:
    """Runs the colour tracking from camera_track.py without any windows.
    In camera mode the tracker's target becomes the pitch target."""

    def __init__(self, bridge):
        self.bridge = bridge
        self.available = False
        self.jpeg = None                # newest picture for the web page
        self.fps = 0.0
        try:
            import cv2
            import camera_track as ct
            self.cv2, self.ct = cv2, ct
            self.cam = ct.Camera()
            if self.cam.read() is None:
                raise RuntimeError("camera gives no picture")
        except Exception as err:        # no OpenCV, no camera...
            print("Camera not available:", err)
            return
        self.tracker = ct.Tracker()
        self.available = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        ct, cv2 = self.ct, self.cv2
        low, high = ct.np.array(ct.HSV_LOW), ct.np.array(ct.HSV_HIGH)
        last = time.time()
        while True:
            frame = self.cam.read()
            if frame is None:
                time.sleep(0.1)
                continue
            mask, obj = ct.find_object(frame, low, high)
            self.tracker.active = self.bridge.mode == "camera"
            target = self.tracker.update(obj)
            if self.tracker.active:
                self.bridge.link.set_target(round(target, 1))
            now = time.time()
            self.fps = 0.9 * self.fps + 0.1 / max(now - last, 1e-3)
            last = now
            ct.draw(frame, mask, obj, self.tracker, self.bridge.link.latest, self.fps)
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self.jpeg = buf.tobytes()

    def info(self):
        if not self.available:
            return {"available": False}
        return {"available": True, "found": self.tracker.found,
                "cam_x": round(self.tracker.cam_x, 3),
                "cam_target": round(self.tracker.target, 1), "fps": round(self.fps, 1)}

    def log_columns(self):
        if not self.available:
            return {}
        return {"cam_x": round(self.tracker.cam_x, 3), "found": int(self.tracker.found)}


def board_state(tel):
    if tel is None:
        return "?"
    if "state" in tel:
        return protocol.STATE_NAMES.get(tel["state"], "?")
    return "ARMED" if tel["armed"] else "DISARMED"     # older firmware


def clamp_number(text, limit):
    value = float(text)
    if value != value or abs(value) == float("inf"):    # NaN / inf
        raise ValueError
    return max(-limit, min(limit, value))


def make_client():
    """Works with paho-mqtt 1.x (Raspberry Pi OS Bookworm) and 2.x."""
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def make_web_handler(bridge):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=WEB_DIR, **kw)

        def log_message(self, *args):
            pass                        # keep the terminal quiet

        def do_GET(self):
            if self.path.startswith("/camera.mjpg"):
                return self._camera()
            if self.path.rstrip("/") == "/logs":
                return self._log_list()
            if self.path.startswith("/logs/"):
                return self._log_file()
            return super().do_GET()

        def _camera(self):
            cam = bridge.camera
            if not (cam and cam.available):
                self.send_error(404, "no camera")
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    jpg = cam.jpeg
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                         + jpg + b"\r\n")
                    time.sleep(0.1)     # 10 pictures a second is enough
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _log_list(self):
            os.makedirs(LOG_DIR, exist_ok=True)
            names = sorted((n for n in os.listdir(LOG_DIR) if n.endswith(".csv")), reverse=True)
            rows = "".join('<li><a href="/logs/%s">%s</a></li>' % (n, n) for n in names)
            body = ("<html><body style='font-family:sans-serif'><h2>Saved logs</h2>"
                    "<ul>%s</ul></body></html>" % (rows or "<li>none yet</li>")).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _log_file(self):
            name = os.path.basename(unquote(self.path[len("/logs/"):]))
            path = os.path.join(LOG_DIR, name)
            if not name.endswith(".csv") or not os.path.isfile(path):
                self.send_error(404)
                return
            with open(path, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/csv")
            self.send_header("Content-Disposition", 'attachment; filename="%s"' % name)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port (default: find the board)")
    ap.add_argument("--broker", default="localhost", help="MQTT broker address")
    ap.add_argument("--http", type=int, default=8080, help="web dashboard port (0 = off)")
    ap.add_argument("--no-camera", action="store_true")
    args = ap.parse_args()

    bridge = Bridge(args)
    print("Board on %s. MQTT broker %s." % (bridge.link.port, args.broker))
    if args.http:
        server = ThreadingHTTPServer(("0.0.0.0", args.http), make_web_handler(bridge))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print("Dashboard: http://<this Pi's address>:%d  (Ctrl+C to stop)" % args.http)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    bridge.close()


if __name__ == "__main__":
    main()
