# -*- coding: utf-8 -*-
"""Algebra linear minima em Python puro (matrizes pequenas, listas de listas).

Existe para o lado do barco nao depender de numpy (versao do barco desconhecida).
Os testes comparam estas funcoes com numpy no PC.
"""
import math


def zeros(n, m):
    return [[0.0] * m for _ in range(n)]


def eye(n):
    a = zeros(n, n)
    for i in range(n):
        a[i][i] = 1.0
    return a


def transpose(a):
    return [list(r) for r in zip(*a)]


def matmul(a, b):
    bt = transpose(b)
    return [[sum(x * y for x, y in zip(r, c)) for c in bt] for r in a]


def matvec(a, v):
    return [sum(x * y for x, y in zip(r, v)) for r in a]


def add(a, b):
    return [[x + y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def sub(a, b):
    return [[x - y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def scale(a, k):
    return [[x * k for x in r] for r in a]


def dot(u, v):
    return sum(x * y for x, y in zip(u, v))


def norm(v):
    return math.sqrt(dot(v, v))


def normalize(v):
    n = norm(v)
    if n == 0.0:
        raise ValueError("vetor nulo")
    return [x / n for x in v]


def cross(u, v):
    return [u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0]]


def solve(a, b):
    """Resolve a x = b (b vetor) por eliminacao de Gauss com pivoteamento."""
    n = len(a)
    m = [list(a[i]) + [b[i]] for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-300:
            raise ValueError("matriz singular")
        m[col], m[piv] = m[piv], m[col]
        for r in range(col + 1, n):
            f = m[r][col] / m[col][col]
            if f != 0.0:
                for c in range(col, n + 1):
                    m[r][c] -= f * m[col][c]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        s = m[i][n] - sum(m[i][j] * x[j] for j in range(i + 1, n))
        x[i] = s / m[i][i]
    return x


def inv(a):
    n = len(a)
    cols = [solve(a, [1.0 if i == j else 0.0 for i in range(n)]) for j in range(n)]
    return transpose(cols)


def det3(a):
    return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))


def eig_sym(a, tol=1e-14, max_sweeps=100):
    """Autovalores/autovetores de matriz simetrica (Jacobi).
    Retorna (valores, V) com colunas de V = autovetores."""
    n = len(a)
    m = [list(r) for r in a]
    v = eye(n)
    for _ in range(max_sweeps):
        off = sum(m[i][j] ** 2 for i in range(n) for j in range(n) if i != j)
        if off < tol * tol:
            break
        for p in range(n - 1):
            for q in range(p + 1, n):
                if abs(m[p][q]) < 1e-300:
                    continue
                theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q])
                t = (1.0 if theta >= 0 else -1.0) / (abs(theta) + math.sqrt(theta * theta + 1.0))
                c = 1.0 / math.sqrt(t * t + 1.0)
                s = t * c
                for k in range(n):
                    mkp = m[k][p]
                    mkq = m[k][q]
                    m[k][p] = c * mkp - s * mkq
                    m[k][q] = s * mkp + c * mkq
                for k in range(n):
                    mpk = m[p][k]
                    mqk = m[q][k]
                    m[p][k] = c * mpk - s * mqk
                    m[q][k] = s * mpk + c * mqk
                for k in range(n):
                    vkp = v[k][p]
                    vkq = v[k][q]
                    v[k][p] = c * vkp - s * vkq
                    v[k][q] = s * vkp + c * vkq
    return [m[i][i] for i in range(n)], v


def sqrtm_spd(a):
    """Raiz quadrada de matriz simetrica definida positiva."""
    vals, v = eig_sym(a)
    if min(vals) <= 0.0:
        raise ValueError("matriz nao e definida positiva: autovalores %r" % (vals,))
    d = [[math.sqrt(vals[i]) if i == j else 0.0 for j in range(len(vals))]
         for i in range(len(vals))]
    return matmul(matmul(v, d), transpose(v))


def nearest_rotation(m, iters=50):
    """Projeta uma matriz 3x3 na rotacao mais proxima (fator polar).
    Iteracao de Newton R <- (R + R^-T)/2. Erro se det <= 0."""
    if det3(m) <= 0.0:
        raise ValueError("det <= 0: medidas inconsistentes (espelhamento)")
    r = [list(x) for x in m]
    for _ in range(iters):
        rit = transpose(inv(r))
        nr = scale(add(r, rit), 0.5)
        d = max(abs(nr[i][j] - r[i][j]) for i in range(3) for j in range(3))
        r = nr
        if d < 1e-14:
            break
    return r


def rodrigues(axis_angle):
    """Matriz de rotacao Exp(w^) para w = eixo*angulo."""
    th = norm(axis_angle)
    if th < 1e-15:
        return eye(3)
    k = [x / th for x in axis_angle]
    kx = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]]
    kx2 = matmul(kx, kx)
    return add(add(eye(3), scale(kx, math.sin(th))), scale(kx2, 1.0 - math.cos(th)))
