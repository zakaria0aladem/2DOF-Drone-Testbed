"""Serial link to the Hiwonder (STM32) board over its UART2 USB-C port."""

import glob
import threading
import time

import serial

import protocol

BAUD = 115200
SEND_PERIOD_S = 0.1     # 10 Hz commands, also the board's heartbeat


def find_port():
    """The board shows up on the Pi as /dev/ttyACM0 (or ttyUSB0 on some cables)."""
    ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        raise RuntimeError("Board not found: is the USB cable in the board's middle (UART2) port?")
    return ports[0]


class BoardLink:
    """Reads telemetry and keeps sending the target in two background threads."""

    def __init__(self, port=None):
        self.port = port or find_port()
        self.ser = serial.Serial(self.port, BAUD, timeout=0.1)
        self.target = 0.0
        self.sending = True         # False = stop sending, to test link loss
        self.latest = None          # newest valid telemetry (dict)
        self.rx_good = 0            # valid $TEL lines received
        self.rx_bad = 0             # $ lines with a wrong checksum or format
        self.tx_count = 0
        self.listeners = []         # functions called with every telemetry dict
        self._running = True
        threading.Thread(target=self._read_loop, daemon=True).start()
        threading.Thread(target=self._send_loop, daemon=True).start()

    def set_target(self, target_deg):
        self.target = float(target_deg)

    def _read_loop(self):
        while self._running:
            raw = self.ser.readline()
            if not raw:
                continue
            text = raw.decode("ascii", errors="replace")
            if not text.startswith("$"):
                continue                # the board's normal printout, not for us
            tel = protocol.parse_tel(text)
            if tel is None:
                self.rx_bad += 1
                continue
            tel["pi_time"] = time.time()
            self.latest = tel
            self.rx_good += 1
            for listener in self.listeners:
                listener(tel)

    def _send_loop(self):
        while self._running:
            if self.sending:
                self.ser.write(protocol.build_set(self.target).encode("ascii"))
                self.tx_count += 1
            time.sleep(SEND_PERIOD_S)

    def close(self):
        self._running = False
        time.sleep(0.2)
        self.ser.close()
