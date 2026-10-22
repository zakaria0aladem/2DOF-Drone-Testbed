"""Read firmware images: .bin (raw, at 0x08000000), .hex (Intel HEX), .elf."""
from __future__ import annotations

import struct
from pathlib import Path

FLASH = 0x08000000


def load_firmware(path):
    """Returns (segments [(addr, bytes)], symbols {addr: name})."""
    p = Path(path)
    data = p.read_bytes()
    if data[:4] == b"\x7fELF":
        return _elf(data)
    if p.suffix.lower() in (".hex", ".ihex") or data[:1] == b":":
        return _ihex(data.decode("ascii", "replace")), {}
    return [(FLASH, data)], {}


def _ihex(text):
    segs = {}
    base = 0
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith(":"):
            continue
        rec = bytes.fromhex(line[1:])
        n, addr, typ = rec[0], (rec[1] << 8) | rec[2], rec[3]
        payload = rec[4:4 + n]
        if (sum(rec) & 0xFF) != 0:
            raise ValueError(f"bad checksum in HEX line: {line}")
        if typ == 0:
            segs[base + addr] = payload
        elif typ == 2:
            base = int.from_bytes(payload, "big") << 4
        elif typ == 4:
            base = int.from_bytes(payload, "big") << 16
        elif typ == 1:
            break
    # merge contiguous records
    out = []
    for a in sorted(segs):
        d = segs[a]
        if out and out[-1][0] + len(out[-1][1]) == a:
            out[-1] = (out[-1][0], out[-1][1] + d)
        else:
            out.append((a, d))
    return out


def _elf(data):
    if data[4] != 1 or data[5] != 1:
        raise ValueError("expected a 32-bit little-endian ELF (ARM)")
    (e_phoff, e_shoff) = struct.unpack_from("<II", data, 28)
    (e_phentsize, e_phnum, e_shentsize, e_shnum, e_shstrndx) = struct.unpack_from("<HHHHH", data, 42)
    segs = []
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        p_type, p_offset, p_vaddr, p_paddr, p_filesz = struct.unpack_from("<IIIII", data, off)
        if p_type == 1 and p_filesz:
            segs.append((p_paddr, data[p_offset:p_offset + p_filesz]))
    syms = {}
    sections = []
    for i in range(e_shnum):
        off = e_shoff + i * e_shentsize
        sections.append(struct.unpack_from("<IIIIIIIIII", data, off))
    for sh in sections:
        if sh[1] == 2:                                    # SHT_SYMTAB
            strtab = sections[sh[6]]
            so = strtab[4]
            for k in range(sh[5] // 16):
                st_name, st_value, st_size, st_info = struct.unpack_from("<IIIB", data, sh[4] + 16 * k)
                if st_info & 0xF == 2 and st_name:          # functions
                    end = data.index(b"\0", so + st_name)
                    syms[st_value & ~1] = data[so + st_name:end].decode("ascii", "replace")
    return segs, syms
