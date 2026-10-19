"""smbus falso para testes: registra escritas, devolve leituras programaveis."""
import math
import os
import time
WRITES = []
READ_VALUES = {}


class SMBus(object):
    def __init__(self, n):
        self.n = n

    def read_i2c_block_data(self, addr, off, n):
        hf = os.environ.get("FAKE_HEADING_FILE")
        if hf and addr == 0x1e and off == 0x28:
            # mag 2D ficticio: "rot" = girando; numero = rumo de bussola fixo
            with open(hf) as f:
                txt = f.read().strip()
            comp = (time.time() * float(os.environ.get("FAKE_ROT_DPS", "40"))) % 360 if txt == "rot" else float(txt)
            a = math.radians(37 - comp)  # sign = -1 na calibracao
            vals = [int(3400 + 800 * math.cos(a)), int(-2900 + 900 * math.sin(a)), 8600]
            out = []
            for v in vals:
                v &= 0xFFFF
                out += [v & 0xFF, v >> 8]
            return out
        if addr == 0x6b and off == 0x28:  # acc parado, z para baixo
            v = (-4000) & 0xFFFF
            return [0, 0, 0, 0, v & 0xFF, v >> 8]
        return list(READ_VALUES.get((addr, off), [0] * n))[:n]

    def read_byte_data(self, addr, off):
        return 0

    def write_i2c_block_data(self, addr, cmd, vals):
        WRITES.append((addr, cmd, list(vals)))
