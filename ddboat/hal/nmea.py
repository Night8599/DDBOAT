# -*- coding: utf-8 -*-
"""Parser NMEA 0183 (RMC e GGA) com checksum e status."""
from ddboat.core.geo import ddmm_to_deg


def checksum_ok(line):
    """True se a frase tem '*hh' e o XOR entre '$' e '*' bate."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    try:
        expected = int(cs[:2], 16)
    except ValueError:
        return False
    x = 0
    for ch in body:
        x ^= ord(ch)
    return x == expected


def parse_rmc(line):
    """Retorna dict(lat, lon, sog_kn, cog_deg, utc) se for RMC valido (status A)
    com checksum correto; None caso contrario. Aceita $GPRMC e $GNRMC."""
    line = line.strip()
    if len(line) < 6 or line[3:6] != "RMC" or not checksum_ok(line):
        return None
    f = line.split("*")[0].split(",")
    if len(f) < 10 or f[2] != "A":
        return None
    try:
        lat = ddmm_to_deg(f[3], f[4])
        lon = ddmm_to_deg(f[5], f[6])
    except ValueError:
        return None
    sog = float(f[7]) if f[7] else None
    cog = float(f[8]) if f[8] else None
    return {"lat": lat, "lon": lon, "sog_kn": sog, "cog_deg": cog, "utc": f[1]}


def parse_gga(line):
    """Retorna dict(quality, sats, hdop) se for GGA com checksum correto."""
    line = line.strip()
    if len(line) < 6 or line[3:6] != "GGA" or not checksum_ok(line):
        return None
    f = line.split("*")[0].split(",")
    if len(f) < 9:
        return None
    try:
        return {"quality": int(f[6] or 0), "sats": int(f[7] or 0),
                "hdop": float(f[8]) if f[8] else None}
    except ValueError:
        return None
