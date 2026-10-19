# -*- coding: utf-8 -*-
"""Convencao do heading do Unity (desconhecida), medida com uma manobra:
anda reto, gira com (-V, V), anda reto de novo (mesma ideia do teleop_sim).

Resultado: theta = sign * radians(heading_deg) + offset, com theta medido no
plano X-Y do Unity (atan2(dY, dX)); turn_sign = +1 se (-V, V) aumenta theta.
"""
import math

from ddboat.core.angles import wrap_pi, circular_mean


class CalibError(Exception):
    pass


def from_two_legs(leg1, leg2, h_turn_start, h_turn_end, min_move, min_turn_deg):
    """leg = ((X0, Y0, h0), (X1, Y1, h1)) poses no inicio e fim de cada reta."""
    thetas = []
    heads = []
    for (p0, p1) in (leg1, leg2):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        if math.hypot(dx, dy) < min_move:
            raise CalibError("o robo quase nao andou (%.3f)" % math.hypot(dx, dy))
        thetas.append(math.atan2(dy, dx))
        heads.append(math.radians(circular_mean_deg2(p0[2], p1[2])))
    dth = wrap_pi(thetas[1] - thetas[0])
    dh = wrap_pi(heads[1] - heads[0])
    if abs(math.degrees(dth)) < min_turn_deg or abs(math.degrees(dh)) < min_turn_deg:
        raise CalibError("giro pequeno demais (%.1f graus)" % math.degrees(dth))
    sign = 1 if dth * dh > 0 else -1
    offset = circular_mean([wrap_pi(t - sign * h) for t, h in zip(thetas, heads)])
    dturn = wrap_pi(sign * math.radians(h_turn_end - h_turn_start))
    turn_sign = 1 if dturn > 0 else -1
    return {"sign": sign, "offset_deg": math.degrees(offset), "turn_sign": turn_sign}


def circular_mean_deg2(a, b):
    return math.degrees(circular_mean([math.radians(a), math.radians(b)]))


def theta_from_heading(h_deg, conv):
    return wrap_pi(conv["sign"] * math.radians(h_deg) + math.radians(conv["offset_deg"]))
