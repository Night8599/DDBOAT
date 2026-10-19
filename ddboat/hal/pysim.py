# -*- coding: utf-8 -*-
"""Simulador Python do DDBoat (modelo de 8 estados do polycopie do Jaulin,
arquivos 05/07 do curso) gerando sensores no MESMO formato do barco real.

Estado: x, y (ENU, m), theta (rad), vx, vy (m/s, no barco), w (rad/s),
        w1, w2 (rotacao das helices).
Entrada do modelo u (tensao). Mapa comando->tensao: u = cmd * u_per_cmd
(PARAMETRO DO SIMULADOR, nao medido no barco real).

Sensores gerados (para testar o pipeline inteiro, inclusive a calibracao 2D):
  mag cru: coerente com uma calibracao 2D ficticia (pysim.mag2d) + ruido
  acc cru: IMU com eixo z para baixo, como medido nos logs reais (~-4000 em z)
  gyr cru: z para baixo -> gyro_z = -taxa de guinada (70 mdps/LSB) + bias
  GPS: lat/lon em torno de pysim.origin, ruido gaussiano, a gps_hz
"""
import math
import random

from ddboat.core.clock import VirtualClock, RealClock
from ddboat.core.geo import LocalFrame
from ddboat.core.angles import theta_to_compass
from ddboat.hal.base import Backend, Fix, Sample, clamp_cmd

GYRO_DPS_PER_LSB = 0.070
ACC_1G_LSB = 4096.0


class PySim(Backend):
    name = "pysim"
    has_imu = True
    has_gps = True

    def __init__(self, cfg, seed=0, realtime=False):
        self.cfg = cfg
        g = lambda k: cfg.require("pysim." + k)
        # realtime=True: relogio real (teleop interativo); False: tempo virtual (rapido)
        self.clock = RealClock() if realtime else VirtualClock()
        self._t0 = self.clock.now()
        self.p = [float(v) for v in g("params_p1_p7")]
        self.u_per_cmd = float(g("u_per_cmd"))
        cmd_max = int(g("cmd_max"))
        self.limits = (-cmd_max if g("allow_reverse") else 0, cmd_max)
        self.dt_int = float(g("dt_integration_s"))
        self.current = [float(v) for v in g("current_xy_mps")]
        self.gps_period = 1.0 / float(g("gps_hz"))
        self.gps_sigma = float(g("gps_sigma_m"))
        self.mag_sigma = float(g("mag_noise_lsb"))
        self.gyro_bias = float(g("gyro_bias_lsb"))
        self.gyro_sigma = float(g("gyro_noise_lsb"))
        self.acc_sigma = float(g("acc_noise_lsb"))
        self.mag2d = g("mag2d")
        o = g("origin")
        self.frame = LocalFrame(o[0], o[1])
        x0 = g("start_x_y_theta")
        self.s = [float(x0[0]), float(x0[1]), float(x0[2]), 0.0, 0.0, 0.0, 0.0, 0.0]
        self.u = (0.0, 0.0)
        self.cmd = (0, 0)
        self.rng = random.Random(seed)
        self._next_gps = 0.0
        self._sim_t = 0.0

    # ------------------------------------------------------------- dinamica
    def _f(self, s, u):
        p1, p2, p3, p4, p5, p6, p7 = self.p
        _, _, th, vx, vy, w, w1, w2 = s
        return [vx * math.cos(th) - vy * math.sin(th) + self.current[0],
                vx * math.sin(th) + vy * math.cos(th) + self.current[1],
                w,
                w * vy - p5 * vx * abs(vx) + p3 * (w1 * abs(w1) + w2 * abs(w2)),
                -w * vx - p6 * vy * abs(vy),
                -p7 * w * abs(w) + p4 * (w2 * abs(w2) - w1 * abs(w1)),
                -p1 * w1 * abs(w1) + p2 * u[0],
                -p1 * w2 * abs(w2) + p2 * u[1]]

    def _advance(self, t_target):
        while self._sim_t < t_target - 1e-12:
            h = min(self.dt_int, t_target - self._sim_t)
            k1 = self._f(self.s, self.u)  # Runge-Kutta 2 (ponto medio)
            mid = [a + 0.5 * h * b for a, b in zip(self.s, k1)]
            k2 = self._f(mid, self.u)
            self.s = [a + h * b for a, b in zip(self.s, k2)]
            self._sim_t += h

    # ------------------------------------------------------------- sensores
    def _mag_raw(self, theta):
        c = self.mag2d
        compass = theta_to_compass(theta)
        raw = math.radians(c["north"] + c["sign"] * compass)
        mx = c["offx"] + math.cos(raw) / c["sx"] + self.rng.gauss(0, self.mag_sigma)
        my = c["offy"] + math.sin(raw) / c["sy"] + self.rng.gauss(0, self.mag_sigma)
        return [int(round(mx)), int(round(my)), int(round(c["mz"]))]

    def read(self):
        t = self.clock.now()
        self._advance(t - self._t0)
        th = self.s[2]
        w = self.s[5]
        n = self.rng.gauss
        mag = self._mag_raw(th)
        acc = [int(round(n(0, self.acc_sigma))), int(round(n(0, self.acc_sigma))),
               int(round(-ACC_1G_LSB + n(0, self.acc_sigma)))]
        gz = -math.degrees(w) / GYRO_DPS_PER_LSB + self.gyro_bias + n(0, self.gyro_sigma)
        gyr = [int(round(n(0, self.gyro_sigma))), int(round(n(0, self.gyro_sigma))),
               int(round(gz))]
        fix = None
        if t - self._t0 >= self._next_gps:
            self._next_gps = t - self._t0 + self.gps_period
            x = self.s[0] + n(0, self.gps_sigma)
            y = self.s[1] + n(0, self.gps_sigma)
            lat, lon = self.frame.to_latlon(x, y)
            vx, vy = self.s[3], self.s[4]
            sog = math.hypot(vx, vy) / 0.514444
            fix = Fix(t, lat, lon, sog, theta_to_compass(th), None)
        return Sample(t, mag, acc, gyr, fix)

    def set_motors(self, left, right):
        l = clamp_cmd(left, self.limits[0], self.limits[1])
        r = clamp_cmd(right, self.limits[0], self.limits[1])
        self._advance(self.clock.now() - self._t0)
        self.cmd = (l, r)
        self.u = (l * self.u_per_cmd, r * self.u_per_cmd)
        return l, r

    def nav_overrides(self):
        return {"mag2d": self.mag2d}

    def truth(self):
        """Estado verdadeiro (so para testes)."""
        return list(self.s)
