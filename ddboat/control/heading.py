# -*- coding: utf-8 -*-
"""Controlador de rumo: PID angular com o giro como termo D (quando disponivel).

Saida: comando de giro D em unidades do Arduino (vai para o Mixer).
"""
from ddboat.control.pid import PID


class HeadingController(object):
    def __init__(self, kp, ki, kd, turn_max, i_max, tau_d):
        self.pid = PID(kp, ki, kd, out_min=-turn_max, out_max=turn_max,
                       i_min=-i_max, i_max=i_max, tau_d=tau_d, angular=True)

    @classmethod
    def from_config(cls, cfg, prefix="control.heading"):
        g = lambda k: float(cfg.require(prefix + "." + k))
        return cls(g("kp"), g("ki"), g("kd"), g("turn_max"), g("i_max"), g("tau_d_s"))

    def update(self, theta_ref, theta, dt, r=None):
        return self.pid.update(theta_ref, theta, dt, meas_rate=r)

    def reset(self):
        self.pid.reset()
