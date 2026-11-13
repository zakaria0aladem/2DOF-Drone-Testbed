"""Saves every telemetry line from the board to a CSV file (for the report plots).

Each row = one $TEL message (50 per second) plus what the Pi was doing at that
moment: the target it was sending, and whether it was sending at all.
Scripts can add their own columns (the camera adds cam_y, found, ...).
"""

import csv
import os
import time

import protocol

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


class DataLog:
    def __init__(self, link, name, extra=None):
        """link  = BoardLink to record from
        name  = start of the file name, e.g. "step"
        extra = optional function returning a dict of extra columns"""
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(LOG_DIR, "%s_%s.csv" % (name, stamp))
        self.link = link
        self.extra = extra
        self.t0 = time.time()
        self.rows = 0
        self._file = open(self.path, "w", newline="")
        self._writer = None
        link.listeners.append(self._on_tel)

    def _on_tel(self, tel):
        row = {"t": round(tel["pi_time"] - self.t0, 3)}
        row.update({k: tel[k] for k in protocol.TEL_FIELDS})
        row["pi_target"] = self.link.target
        row["pi_sending"] = int(self.link.sending)
        if self.extra:
            row.update(self.extra())
        if self._writer is None:        # header from the first row
            self._writer = csv.DictWriter(self._file, fieldnames=list(row))
            self._writer.writeheader()
        self._writer.writerow(row)
        self.rows += 1

    def close(self):
        if self._on_tel in self.link.listeners:
            self.link.listeners.remove(self._on_tel)
        self._file.close()
        return self.path
