"""Message format shared with the board (same as bsp_comm.h).

Every message is one text line:  $<fields>*<checksum>\\r\\n
checksum = XOR of all characters between '$' and '*', as 2 hex digits.

Board -> Pi (50 Hz):
  $TEL,ms,pitch,enc_pitch,enc_yaw,target,output,left_us,right_us,armed,link_lost,bad,
       yaw_target,yaw_out,m1,m2,m3,m4,state*CS
  The last 7 fields (yaw_target ... state) were added for the yaw controller and the
  IoT dashboard. Older firmware sends only the first 11; both are accepted.
  state: 0 = disarmed, 1 = arming countdown, 2 = armed, 3 = IMU fault
Pi -> board (10 Hz, also the heartbeat):
  $SET,<pitch target deg>*CS
  $YAW,<yaw target deg>*CS     (only sent once someone has set a yaw target)
  $DIS*CS                      (disarm the motors now)
"""

TEL_FIELDS_BASE = ["ms", "pitch", "enc_pitch", "enc_yaw", "target", "output",
                   "left_us", "right_us", "armed", "link_lost", "bad"]
TEL_FIELDS_NEW = ["yaw_target", "yaw_out", "m1", "m2", "m3", "m4", "state"]
TEL_FIELDS = TEL_FIELDS_BASE + TEL_FIELDS_NEW
INT_FIELDS = ("ms", "left_us", "right_us", "armed", "link_lost", "bad",
              "m1", "m2", "m3", "m4", "state")
STATE_NAMES = {0: "DISARMED", 1: "ARMING", 2: "ARMED", 3: "IMU FAULT"}


def checksum(text):
    """XOR of every character of text."""
    total = 0
    for ch in text:
        total ^= ord(ch)
    return total


def build_line(body):
    """Adds the $ start, the checksum and the line end to a message body."""
    return "$%s*%02X\r\n" % (body, checksum(body))


def build_set(target_deg):
    """Command line telling the board a new pitch target angle."""
    return build_line("SET,%.1f" % target_deg)


def build_yaw(target_deg):
    """Command line telling the board a new yaw target angle."""
    return build_line("YAW,%.1f" % target_deg)


def build_disarm():
    """Command line that disarms the motors."""
    return build_line("DIS")


def parse_tel(line):
    """Turns one $TEL line into a dict, or returns None if it is not valid.
    Lines that don't start with '$' (the board's normal printout) are skipped."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None
    body, _, sent = line[1:].partition("*")
    try:
        if int(sent, 16) != checksum(body):
            return None
    except ValueError:
        return None

    parts = body.split(",")
    count = len(parts) - 1
    if parts[0] != "TEL" or count not in (len(TEL_FIELDS_BASE), len(TEL_FIELDS)):
        return None
    try:
        values = [float(p) for p in parts[1:]]
    except ValueError:
        return None
    tel = dict(zip(TEL_FIELDS, values))     # old firmware: only the first 11 keys
    for key in INT_FIELDS:
        if key in tel:
            tel[key] = int(tel[key])
    return tel
