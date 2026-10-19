# -*- coding: utf-8 -*-
"""Geodesia local: NMEA, lat/lon <-> ENU local.

Mesma aproximacao do ddboatlib.py do curso (equiretangular em torno de um
ponto de referencia, raio R0 = 6378137 m). Boa para distancias de alguns km.
psi0 permite girar o referencial (piscina da ENSTA: psi0 = -3.88 graus).
Com psi0 = 0 o referencial e ENU puro (x leste, y norte).
"""
import math

R0 = 6378137.0


def ddmm_to_deg(value, hemi):
    """NMEA ddmm.mmmm (ou dddmm.mmmm) + hemisferio -> graus decimais."""
    v = float(value)
    deg = int(v / 100)
    minutes = v - deg * 100
    dd = deg + minutes / 60.0
    if hemi in ("S", "W"):
        dd = -dd
    return dd


class LocalFrame(object):
    """Referencial local com origem em (lat0, lon0) em graus."""

    def __init__(self, lat0_deg, lon0_deg, psi0_deg=0.0):
        self.lat0 = float(lat0_deg)
        self.lon0 = float(lon0_deg)
        self.psi0 = math.radians(psi0_deg)
        self._lat0r = math.radians(self.lat0)
        self._lon0r = math.radians(self.lon0)
        self._c = math.cos(self.psi0)
        self._s = math.sin(self.psi0)

    def to_xy(self, lat_deg, lon_deg):
        x1 = R0 * math.cos(self._lat0r) * (math.radians(lon_deg) - self._lon0r)
        y1 = R0 * (math.radians(lat_deg) - self._lat0r)
        x = self._c * x1 - self._s * y1
        y = self._s * x1 + self._c * y1
        return x, y

    def to_latlon(self, x, y):
        x1 = self._c * x + self._s * y
        y1 = -self._s * x + self._c * y
        lat = math.degrees(self._lat0r + y1 / R0)
        lon = math.degrees(self._lon0r + x1 / (R0 * math.cos(self._lat0r)))
        return lat, lon


def distance(p, q):
    return math.hypot(q[0] - p[0], q[1] - p[1])


def bearing_theta(p, q):
    """Direcao de p para q como theta (rad, anti-horario a partir de x)."""
    return math.atan2(q[1] - p[1], q[0] - p[0])


def point_in_polygon(pt, poly):
    """Teste par-impar. poly = lista de (x, y)."""
    x, y = pt
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            xc = xi + (y - yi) * (xj - xi) / (yj - yi)
            if x < xc:
                inside = not inside
        j = i
    return inside
