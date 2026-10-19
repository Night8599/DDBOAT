# -*- coding: utf-8 -*-
"""Relogios e laco de frequencia fixa.

RealClock usa time.monotonic (nao anda para tras se o relogio do sistema mudar).
VirtualClock e usado nos simuladores para rodar mais rapido que o tempo real.
"""
import time


class RealClock(object):
    def now(self):
        return time.monotonic()

    def sleep(self, dt):
        if dt > 0:
            time.sleep(dt)


class VirtualClock(object):
    def __init__(self, t0=0.0):
        self.t = float(t0)

    def now(self):
        return self.t

    def sleep(self, dt):
        if dt > 0:
            self.t += dt


class Rate(object):
    """Laco de frequencia fixa sem deriva: dorme ate o proximo tick.

    Se uma volta passar do periodo (overrun), nao tenta "recuperar" o atraso:
    reagenda a partir de agora e conta o overrun.
    """

    def __init__(self, hz, clock):
        if hz <= 0:
            raise ValueError("frequencia deve ser > 0")
        self.period = 1.0 / hz
        self.clock = clock
        self.next_t = clock.now() + self.period
        self.last_t = clock.now()
        self.overruns = 0
        self.ticks = 0
        self.dt = self.period
        self.max_dt = 0.0

    def sleep(self):
        now = self.clock.now()
        remaining = self.next_t - now
        if remaining > 0:
            self.clock.sleep(remaining)
            self.next_t += self.period
        else:
            self.overruns += 1
            self.next_t = now + self.period
        t = self.clock.now()
        self.dt = t - self.last_t
        self.last_t = t
        self.ticks += 1
        if self.dt > self.max_dt:
            self.max_dt = self.dt
        return self.dt
