"""tbproto.py - PC side of the testbed protocol. Mirrors firmware/tlm_proto.h.

Standard library only. Frame: A5 5A | type | len | payload | crc16 (LE),
CRC-16/CCITT-FALSE over type, len and payload.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

PROTO_VERSION = 1
SYNC = b"\xA5\x5A"
MAX_PAYLOAD = 96

# board -> PC
T_TELEMETRY = 0x01
T_HELLO = 0x02
T_TEXT = 0x03
# PC -> board
T_MOTOR_CMD = 0x10
T_ARM = 0x11
T_DISARM = 0x12
T_SET_RATE = 0x13
T_PING = 0x14

F_ARMED = 1 << 0
F_CMD_TIMEOUT = 1 << 1
F_ENC_FAULT = 1 << 2
F_IMU_FAULT = 1 << 3
F_TX_OVERRUN = 1 << 4
F_CLAMPED = 1 << 5
F_KILLED = 1 << 6

TELEM_FMT = struct.Struct("<II2f2f3f3f4HHBB")   # 60 bytes
CMD_FMT = struct.Struct("<HH4H")                # 12 bytes
HELLO_FMT = struct.Struct("<BBHHH")             # 8 bytes
assert TELEM_FMT.size == 60 and CMD_FMT.size == 12 and HELLO_FMT.size == 8


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(ftype: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes((ftype, len(payload))) + payload
    return SYNC + body + struct.pack("<H", crc16(body))


@dataclass
class Telemetry:
    seq: int
    t_us: int
    q: tuple[float, float]
    dq: tuple[float, float]
    acc: tuple[float, float, float]
    gyro: tuple[float, float, float]
    motor_us: tuple[int, int, int, int]
    cmd_seq: int
    flags: int

    @classmethod
    def unpack(cls, p: bytes) -> "Telemetry":
        v = TELEM_FMT.unpack(p)
        return cls(v[0], v[1], v[2:4], v[4:6], v[6:9], v[9:12], v[12:16], v[16], v[17])

    def pack(self) -> bytes:
        return TELEM_FMT.pack(self.seq, self.t_us, *self.q, *self.dq, *self.acc,
                              *self.gyro, *self.motor_us, self.cmd_seq, self.flags, 0)

    @property
    def armed(self) -> bool:
        return bool(self.flags & F_ARMED)


@dataclass
class Hello:
    proto_version: int
    rate_hz: int
    us_min: int
    us_max: int

    @classmethod
    def unpack(cls, p: bytes) -> "Hello":
        ver, _, rate, lo, hi = HELLO_FMT.unpack(p)
        return cls(ver, rate, lo, hi)

    def pack(self) -> bytes:
        return HELLO_FMT.pack(self.proto_version, 0, self.rate_hz, self.us_min, self.us_max)


def motor_cmd(seq: int, us) -> bytes:
    us = [int(round(x)) for x in us]
    if len(us) != 4:
        raise ValueError("need 4 motor values")
    return encode(T_MOTOR_CMD, CMD_FMT.pack(seq & 0xFFFF, 0, *us))


def set_rate(hz: int) -> bytes:
    return encode(T_SET_RATE, struct.pack("<H", hz))


class Parser:
    """Streaming parser. feed() returns a list of (type, payload) for valid frames."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.frames_ok = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # keep a trailing 0xA5 in case the 0x5A comes next
                del self.buf[: max(0, len(self.buf) - 1)]
                return out
            if i:
                del self.buf[:i]
            if len(self.buf) < 4:
                return out
            ln = self.buf[3]
            if ln > MAX_PAYLOAD:
                del self.buf[:1]
                continue
            n = ln + 6
            if len(self.buf) < n:
                return out
            body = bytes(self.buf[2 : 4 + ln])
            (crc_rx,) = struct.unpack_from("<H", self.buf, 4 + ln)
            if crc16(body) == crc_rx:
                out.append((body[0], body[2:]))
                self.frames_ok += 1
                del self.buf[:n]
            else:
                self.crc_errors += 1
                del self.buf[:1]   # resync from the next byte
