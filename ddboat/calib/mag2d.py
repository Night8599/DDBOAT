# -*- coding: utf-8 -*-
"""Calibracao 2D do magnetometro: exatamente o metodo do cap_nord.py testado no
barco (min/max em x e y + apontamento norte e leste com sinal automatico).

O JSON gerado tem as mesmas chaves do calib_cap.json antigo
(offx, offy, sx, sy, north, sign), entao o arquivo antigo tambem serve.
Bussola resultante: 0 = direcao apontada como "norte" na fase 2, horario.
"""
import json
import math

from ddboat.core.angles import wrap_180, circular_mean

KEYS = ("offx", "offy", "sx", "sy", "north", "sign")


class CalibError(Exception):
    pass


def fit_minmax(xs, ys):
    if max(xs) == min(xs) or max(ys) == min(ys):
        raise CalibError("o magnetometro nao variou")
    return {"offx": (max(xs) + min(xs)) / 2.0, "offy": (max(ys) + min(ys)) / 2.0,
            "sx": 2.0 / (max(xs) - min(xs)), "sy": 2.0 / (max(ys) - min(ys)),
            "north": 0.0, "sign": 1}


def check_coverage(xs, ys, c, sectors=8):
    """Erro se a rotacao nao cobriu todos os setores de 360/sectors graus
    (volta incompleta -> min/max errado -> rumo errado)."""
    hit = set()
    for mx, my in zip(xs, ys):
        hit.add(int((raw_angle_deg(mx, my, c) % 360.0) // (360.0 / sectors)))
    missing = sectors - len(hit)
    if missing:
        raise CalibError("volta incompleta: %d de %d setores de %d graus sem medida"
                         % (missing, sectors, 360 // sectors))


def raw_angle_deg(mx, my, c):
    x = (mx - c["offx"]) * c["sx"]
    y = (my - c["offy"]) * c["sy"]
    return math.degrees(math.atan2(y, x))


def mean_raw_angle_deg(samples, c):
    """samples: lista de (mx, my)."""
    return math.degrees(circular_mean([math.radians(raw_angle_deg(mx, my, c)) for mx, my in samples]))


def set_north_east(c, north_samples, east_samples):
    """Fases 2 e 3 do cap_nord. Erro se norte->leste nao der ~90 graus."""
    c = dict(c)
    c["north"] = mean_raw_angle_deg(north_samples, c)
    d = wrap_180(mean_raw_angle_deg(east_samples, c) - c["north"])
    if 60.0 <= d <= 120.0:
        c["sign"] = 1
    elif -120.0 <= d <= -60.0:
        c["sign"] = -1
    else:
        raise CalibError("diferenca norte->leste = %.1f graus, longe de 90" % d)
    return c, d


def heading_compass_deg(mx, my, c):
    """Rumo de bussola (0 = norte calibrado, horario), em [0, 360)."""
    return (c["sign"] * wrap_180(raw_angle_deg(mx, my, c) - c["north"])) % 360.0


def save(c, path, extra=None):
    d = dict((k, c[k]) for k in KEYS)
    if extra:
        d.update(extra)
    with open(path, "w") as f:
        json.dump(d, f, indent=2)


def load(path):
    with open(path) as f:
        c = json.load(f)
    for k in KEYS:
        if k not in c:
            raise CalibError("arquivo %s sem a chave %s" % (path, k))
    return c
