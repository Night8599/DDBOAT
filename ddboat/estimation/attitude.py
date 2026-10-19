# -*- coding: utf-8 -*-
"""Atitude a partir de acelerometro + magnetometro (algoritmo EulerAngles do
polycopie DD-Boats do Jaulin, secao 2.4).

Referencial do barco: x frente, y esquerda, z cima (FLU).
Referencial do mundo (polycopie): NWU, g0 = (0,0,1), campo y0 = (cos I, 0, -sin I).
psi devolvido: rotacao em torno de z (cima), anti-horario a partir do NORTE.
Para o theta interno (anti-horario a partir do LESTE): theta = psi + pi/2.
"""
import math

from ddboat.core import linalg as la


def r_uv(u, v):
    """Rotacao minima que leva o vetor unitario u em v."""
    c = max(-1.0, min(1.0, la.dot(u, v)))
    axis = la.cross(u, v)
    n = la.norm(axis)
    if n < 1e-12:
        if c > 0:
            return la.eye(3)
        raise ValueError("u e v opostos: barco de cabeca para baixo?")
    ang = math.acos(c)
    return la.rodrigues([a / n * ang for a in axis])


def euler_angles(acc_boat, mag_boat):
    """acc_boat: forca especifica medida (aponta para CIMA em repouso), no
    referencial do barco. mag_boat: campo calibrado no referencial do barco.
    Retorna (phi roll, theta_pitch, psi) em rad, como no polycopie."""
    g1 = la.normalize(acc_boat)
    y1 = la.normalize(mag_boat)
    phi = math.asin(max(-1.0, min(1.0, g1[1])))
    tht = -math.asin(max(-1.0, min(1.0, g1[0])))
    rh = r_uv(g1, [0.0, 0.0, 1.0])
    yh = la.matvec(rh, y1)
    psi = -math.atan2(yh[1], yh[0])
    return phi, tht, psi


def heading_tilt_compensated(acc_boat, mag_boat):
    """Rumo exato da PROA projetada no plano horizontal, anti-horario a partir
    do norte magnetico (mesma convencao do psi acima), em rad.

    Diferenca para euler_angles: o polycopie usa a rotacao minima que leva o
    'cima' medido para a vertical; com roll E pitch juntos isso difere do rumo
    real da proa em ~0.4 graus para (8, 6) graus. Aqui projetamos a proa
    (x do barco) e o campo no plano horizontal e medimos o angulo entre eles.
    """
    u = la.normalize(acc_boat)
    y = list(mag_boat)
    yu = la.dot(y, u)
    n_h = [a - yu * b for a, b in zip(y, u)]
    xu = u[0]
    x_h = [1.0 - xu * u[0], -xu * u[1], -xu * u[2]]
    if la.norm(n_h) < 1e-9 or la.norm(x_h) < 1e-9:
        raise ValueError("geometria degenerada (barco vertical?)")
    return math.atan2(la.dot(la.cross(n_h, x_h), u), la.dot(n_h, x_h))
