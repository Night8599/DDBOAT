# -*- coding: utf-8 -*-
"""Guiagem (polycopie DD-Boats, cap. 1 e relatorios da turma anterior).

Todas as funcoes trabalham em ENU (m) e devolvem theta de referencia em rad.

line_follow: e = det(u, m - a) (e > 0: barco a esquerda da reta)
             theta_ref = phi - gamma * tanh(e / r)        (campo de vetores)
lookahead:   mira um ponto L metros a frente da projecao do barco na reta
passed_b:    (b - a).(m - b) > -margin*|b - a|  (semiplano de validacao)
"""
import math

from ddboat.core.geo import distance


def goto_theta(m, target):
    return math.atan2(target[1] - m[1], target[0] - m[0])


def cross_track(a, b, m):
    L = distance(a, b)
    if L == 0:
        raise ValueError("segmento de comprimento zero")
    ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
    e = ux * (m[1] - a[1]) - uy * (m[0] - a[0])
    s = ux * (m[0] - a[0]) + uy * (m[1] - a[1])
    return e, s, L, (ux, uy)


def line_follow_theta(a, b, m, r, gamma):
    e, _, _, _ = cross_track(a, b, m)
    phi = math.atan2(b[1] - a[1], b[0] - a[0])
    return phi - gamma * math.tanh(e / r), e


def lookahead_theta(a, b, m, L_ahead):
    e, s, L, (ux, uy) = cross_track(a, b, m)
    s_t = min(s + L_ahead, L + L_ahead)
    tx, ty = a[0] + ux * s_t, a[1] + uy * s_t
    return math.atan2(ty - m[1], tx - m[0]), e


def passed_b(a, b, m, margin=0.0):
    dx, dy = b[0] - a[0], b[1] - a[1]
    return dx * (m[0] - b[0]) + dy * (m[1] - b[1]) > -margin * math.hypot(dx, dy)


def approach_thrust(dist, thrust, slow_radius, thrust_min):
    """Reduz o empuxo linearmente dentro de slow_radius (evita passar do ponto)."""
    if slow_radius <= 0 or dist >= slow_radius:
        return thrust
    return thrust_min + (thrust - thrust_min) * dist / slow_radius
