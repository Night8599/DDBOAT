# -*- coding: utf-8 -*-
"""Calibracao 3D do magnetometro (polycopie DD-Boats, secoes 2.2 e 2.2.2).

1) Elipsoide por minimos quadrados: X p = 1, p em R^9
   (x1^2, x2^2, x3^2, x1x2, x1x3, x2x3, x1, x2, x3)
2) xbar = -1/2 P^-1 c ; Q = P / (1 + xbar^T P xbar) ; esfera: sqrt(Q)(x - xbar)
3) Rotacao pelo apontamento Norte e Oeste com o barco na horizontal:
   yN = (cos I, 0, -sin I), yW = (0, -cos I, -sin I) no referencial do barco (FLU)
   R = Y V^-1, projetada na rotacao mais proxima.
Corretor final: y_barco = Ainv (x - xbar), com Ainv = R sqrt(Q).
"""
import json
import math

from ddboat.core import linalg as la


class CalibError(Exception):
    pass


def fit_ellipsoid(points):
    """points: lista de (x1, x2, x3). Retorna (Q, xbar)."""
    if len(points) < 9:
        raise CalibError("pontos insuficientes (%d)" % len(points))
    # normaliza escala para a equacao normal ficar bem condicionada
    s = max(max(abs(v) for v in p) for p in points)
    if s == 0:
        raise CalibError("magnetometro zerado")
    pts = [[v / s for v in p] for p in points]
    rows = [[a * a, b * b, c * c, a * b, a * c, b * c, a, b, c] for a, b, c in pts]
    XtX = [[sum(r[i] * r[j] for r in rows) for j in range(9)] for i in range(9)]
    Xt1 = [sum(r[i] for r in rows) for i in range(9)]
    p = la.solve(XtX, Xt1)
    P = [[p[0], p[3] / 2, p[4] / 2], [p[3] / 2, p[1], p[5] / 2], [p[4] / 2, p[5] / 2, p[2]]]
    c = [p[6], p[7], p[8]]
    xbar = [-0.5 * v for v in la.solve(P, c)]
    # den < 0 e normal quando a origem fica FORA do elipsoide (hard iron maior
    # que o campo, caso do barco): P sai negativa e Q = P/den continua positiva.
    den = 1.0 + la.dot(xbar, la.matvec(P, xbar))
    if abs(den) < 1e-12:
        raise CalibError("ajuste degenerado (den ~ 0)")
    Q = la.scale(P, 1.0 / den)
    vals, _ = la.eig_sym(Q)
    if min(vals) <= 0:
        raise CalibError("ajuste nao e elipsoide (Q nao e definida positiva): "
                         "gire o barco em mais orientacoes")
    # desfaz a normalizacao: x = s * x_n
    return la.scale(Q, 1.0 / (s * s)), [v * s for v in xbar]


def rotation_from_nw(sqrtQ, xbar, x_north, x_west, incl_deg):
    I = math.radians(incl_deg)
    yN = [math.cos(I), 0.0, -math.sin(I)]
    yW = [0.0, -math.cos(I), -math.sin(I)]
    vN = la.matvec(sqrtQ, [a - b for a, b in zip(x_north, xbar)])
    vW = la.matvec(sqrtQ, [a - b for a, b in zip(x_west, xbar)])
    V = la.transpose([vN, vW, la.cross(vN, vW)])
    Y = la.transpose([yN, yW, la.cross(yN, yW)])
    R_raw = la.matmul(Y, la.inv(V))
    R = la.nearest_rotation(R_raw)
    # diagnostico: angulo medido entre N e W comparado com o teorico
    ang_meas = math.degrees(math.acos(max(-1, min(1, la.dot(la.normalize(vN), la.normalize(vW))))))
    ang_theo = math.degrees(math.acos(max(-1, min(1, la.dot(yN, yW)))))
    return R, {"angle_NW_measured_deg": ang_meas, "angle_NW_theory_deg": ang_theo,
               "norm_vN": la.norm(vN), "norm_vW": la.norm(vW)}


def calibrate(points, x_north, x_west, incl_deg):
    Q, xbar = fit_ellipsoid(points)
    sq = la.sqrtm_spd(Q)
    R, diag = rotation_from_nw(sq, xbar, x_north, x_west, incl_deg)
    Ainv = la.matmul(R, sq)
    norms = [la.norm(la.matvec(Ainv, [a - b for a, b in zip(p, xbar)])) for p in points]
    mean = sum(norms) / len(norms)
    std = math.sqrt(sum((n - mean) ** 2 for n in norms) / len(norms))
    diag.update({"norm_mean": mean, "norm_std": std, "n_points": len(points)})
    return {"xbar": xbar, "Ainv": Ainv, "incl_deg": incl_deg}, diag


def apply(calib, m):
    return la.matvec(calib["Ainv"], [a - b for a, b in zip(m, calib["xbar"])])


def save(calib, diag, path):
    d = dict(calib)
    d["diagnostics"] = diag
    with open(path, "w") as f:
        json.dump(d, f, indent=2)


def load(path):
    with open(path) as f:
        d = json.load(f)
    for k in ("xbar", "Ainv"):
        if k not in d:
            raise CalibError("arquivo %s sem a chave %s" % (path, k))
    return d
