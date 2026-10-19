# -*- coding: utf-8 -*-
"""Filtros simples de sinal."""
from collections import deque

from ddboat.core.angles import wrap_pi


def median(values):
    s = sorted(values)
    n = len(s)
    if n == 0:
        raise ValueError("lista vazia")
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


class EMA(object):
    """Media movel exponencial: y += alpha (x - y)."""

    def __init__(self, alpha):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha em (0, 1]")
        self.alpha = alpha
        self.y = None

    def update(self, x):
        self.y = x if self.y is None else self.y + self.alpha * (x - self.y)
        return self.y


class CircularEMA(object):
    """EMA para angulos em rad (sem salto em +-pi)."""

    def __init__(self, alpha):
        self.alpha = alpha
        self.y = None

    def update(self, a):
        self.y = a if self.y is None else wrap_pi(self.y + self.alpha * wrap_pi(a - self.y))
        return self.y


class Hampel(object):
    """Rejeita outliers: se |x - mediana| > k * 1.4826 * MAD da janela, devolve a mediana."""

    def __init__(self, window=7, k=3.0):
        self.buf = deque(maxlen=window)
        self.k = k
        self.rejected = 0

    def update(self, x):
        self.buf.append(x)
        if len(self.buf) < 3:
            return x
        med = median(self.buf)
        mad = median([abs(v - med) for v in self.buf])
        if mad > 0 and abs(x - med) > self.k * 1.4826 * mad:
            self.rejected += 1
            return med
        return x


class RateLimiter(object):
    """Limita a variacao por segundo de um sinal."""

    def __init__(self, max_rate):
        self.max_rate = max_rate
        self.y = None

    def update(self, x, dt):
        if self.y is None:
            self.y = x
            return x
        step = self.max_rate * dt
        self.y += max(-step, min(step, x - self.y))
        return self.y
