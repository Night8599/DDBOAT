# -*- coding: utf-8 -*-
"""PID discreto.

- derivada na MEDIDA (nao no erro): sem pico quando a consigna muda
- se a derivada da medida for conhecida (ex.: giro para o rumo), passe em
  meas_rate e ela e usada direto, sem derivar ruido
- filtro passa-baixa de 1a ordem no termo D (tau_d)
- anti-windup por clamping: o integrador nao cresce quando a saida satura no
  mesmo sentido do erro
- angular=True: erro com wrap em [-pi, pi)
- freeze(): congela o integrador (ex.: RC assumiu o barco)
"""
from ddboat.core.angles import wrap_pi


class PID(object):
    def __init__(self, kp, ki=0.0, kd=0.0, out_min=None, out_max=None,
                 i_min=None, i_max=None, tau_d=0.0, angular=False):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.i_min, self.i_max = i_min, i_max
        self.tau_d = tau_d
        self.angular = angular
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.d_filt = 0.0
        self.prev_meas = None
        self.frozen = False
        self.last = {}

    def freeze(self, on=True):
        self.frozen = on

    def _err(self, sp, meas):
        e = sp - meas
        return wrap_pi(e) if self.angular else e

    def update(self, setpoint, meas, dt, meas_rate=None):
        e = self._err(setpoint, meas)
        p = self.kp * e
        # derivada da medida
        if meas_rate is not None:
            d_raw = meas_rate
        elif self.prev_meas is None or dt <= 0:
            d_raw = 0.0
        else:
            dm = meas - self.prev_meas
            if self.angular:
                dm = wrap_pi(dm)
            d_raw = dm / dt
        self.prev_meas = meas
        if self.tau_d > 0 and dt > 0:
            a = dt / (self.tau_d + dt)
            self.d_filt += a * (d_raw - self.d_filt)
        else:
            self.d_filt = d_raw
        d = -self.kd * self.d_filt
        u_unsat = p + self.ki * self.integral + d
        u = u_unsat
        if self.out_max is not None and u > self.out_max:
            u = self.out_max
        if self.out_min is not None and u < self.out_min:
            u = self.out_min
        saturating = (u != u_unsat) and ((u_unsat > u and e > 0) or (u_unsat < u and e < 0))
        if not self.frozen and self.ki != 0.0 and dt > 0 and not saturating:
            self.integral += e * dt
            if self.i_max is not None:
                self.integral = min(self.integral, self.i_max)
            if self.i_min is not None:
                self.integral = max(self.integral, self.i_min)
        self.last = {"e": e, "p": p, "i": self.ki * self.integral, "d": d, "u": u}
        return u
