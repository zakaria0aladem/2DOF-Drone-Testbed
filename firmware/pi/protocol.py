"""Message format shared with the board (same as bsp_comm.h).

Every message is one text line:  $<fields>*<checksum>\\r\\n
checksum = XOR of all characters between '$' and '*', as 2 hex digits.

Board -> Pi (50 Hz):
  $TEL,ms,pitch,enc_pitch,enc_yaw,target,output,left_us,right_us,armed,link_lost,bad*CS
Pi -> board (10 Hz, also the heartbeat):
  $SET,<target deg>*CS
"""

TEL_FIELDS = ["ms", "pitch", "enc_pitch", "enc_yaw", "target", "output",
              "left_us", "right_us", "armed", "link_lost", "bad"]


def checksum(text):
    """XOR of every character of text."""
    total = 0
    for ch in text:
        total ^= ord(ch)
    return total


def build_set(target_deg):
    """Command line telling the board a new target angle."""
    body = "SET,%.1f" % target_deg
    return "$%s*%02X\r\n" % (body, checksum(body))


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
    if parts[0] != "TEL" or len(parts) != len(TEL_FIELDS) + 1:
        return None
    try:
        values = [float(p) for p in parts[1:]]
    except ValueError:
        return None
    tel = dict(zip(TEL_FIELDS, values))
    for key in ("ms", "left_us", "right_us", "armed", "link_lost", "bad"):
        tel[key] = int(tel[key])
    return tel
