# -*- coding: utf-8 -*-
"""Angulos e convencoes. UNICO lugar do codigo onde convencao de rumo existe.

theta (interno): rad, anti-horario a partir do leste (ENU).
compass (bussola): graus, horario a partir do norte.
Relacao: theta = 90deg - compass  (em rad, com wrap).
"""
import math

TWO_PI = 2.0 * math.pi


def wrap_pi(a):
    """Angulo em rad no intervalo [-pi, pi)."""
    return (a + math.pi) % TWO_PI - math.pi


# Nome usado no curso do Jaulin
sawtooth = wrap_pi


def wrap_180(a):
    """Angulo em graus no intervalo [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def wrap_360(a):
    """Angulo em graus no intervalo [0, 360)."""
    return a % 360.0


def compass_to_theta(compass_deg):
    """Bussola (graus, 0=N, horario) -> theta (rad, 0=leste, anti-horario)."""
    return wrap_pi(math.radians(90.0 - compass_deg))


def theta_to_compass(theta):
    """theta (rad, 0=leste, anti-horario) -> bussola (graus, 0=N, horario)."""
    return wrap_360(90.0 - math.degrees(theta))


def diff(a, b):
    """Menor diferenca a - b em rad, em [-pi, pi)."""
    return wrap_pi(a - b)


def circular_mean(angles):
    """Media circular de uma lista de angulos em rad."""
    s = 0.0
    c = 0.0
    for a in angles:
        s += math.sin(a)
        c += math.cos(a)
    if s == 0.0 and c == 0.0:
        raise ValueError("media circular indefinida")
    return math.atan2(s, c)


def circular_mean_deg(angles_deg):
    return math.degrees(circular_mean([math.radians(a) for a in angles_deg]))
