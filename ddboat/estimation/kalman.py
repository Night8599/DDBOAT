# -*- coding: utf-8 -*-
"""Filtro de Kalman linear generico (Python puro) e dois filtros prontos.

KF.predict(A, B, u, Q) / KF.update(y, C, R) com gate de Mahalanobis opcional.

HeadingKF: estado [theta, b]
  predicao: theta += dt (r_med - b), b constante (passeio aleatorio)
  medida:   theta_mag (inovacao com wrap em [-pi, pi))
PositionKF: estado [x, y, vx, vy] (velocidade constante)
  medida:   GPS (x, y), com gate para descartar saltos
"""

from ddboat.core import linalg as la
from ddboat.core.angles import wrap_pi

# chi2 com 1 e 2 graus de liberdade (99,9%)
CHI2_999 = {1: 10.83, 2: 13.82}


class KF(object):
    def __init__(self, x0, P0):
        self.x = [float(v) for v in x0]
        self.P = [list(map(float, r)) for r in P0]
        self.last_nis = None

    def predict(self, A, Q, B=None, u=None):
        x = la.matvec(A, self.x)
        if B is not None:
            bu = la.matvec(B, u)
            x = [a + b for a, b in zip(x, bu)]
        self.x = x
        self.P = la.add(la.matmul(la.matmul(A, self.P), la.transpose(A)), Q)

    def update(self, y, C, R, innov=None, gate=None):
        """y: lista; innov: inovacao ja calculada (ex.: angulo com wrap).
        gate: limiar chi2 do NIS; se passar, a medida e descartada (retorna False)."""
        if innov is None:
            yp = la.matvec(C, self.x)
            innov = [a - b for a, b in zip(y, yp)]
        S = la.add(la.matmul(la.matmul(C, self.P), la.transpose(C)), R)
        Si = la.inv(S)
        nis = la.dot(innov, la.matvec(Si, innov))
        self.last_nis = nis
        if gate is not None and nis > gate:
            return False
        K = la.matmul(la.matmul(self.P, la.transpose(C)), Si)
        self.x = [a + b for a, b in zip(self.x, la.matvec(K, innov))]
        n = len(self.x)
        IKC = la.sub(la.eye(n), la.matmul(K, C))
        # forma de Joseph (numericamente estavel)
        self.P = la.add(la.matmul(la.matmul(IKC, self.P), la.transpose(IKC)),
                        la.matmul(la.matmul(K, R), la.transpose(K)))
        return True


class HeadingKF(object):
    def __init__(self, theta0, bias0, sigma_theta0, sigma_bias0, q_rate, q_bias, r_mag):
        """q_rate: densidade do ruido do giro (rad/s/sqrt(Hz)), q_bias: deriva do
        bias (rad/s^2/sqrt(Hz)), r_mag: desvio da medida de rumo do mag (rad)."""
        self.kf = KF([theta0, bias0], [[sigma_theta0 ** 2, 0.0], [0.0, sigma_bias0 ** 2]])
        self.q_rate = q_rate
        self.q_bias = q_bias
        self.r_mag = r_mag

    def predict(self, r_meas, dt):
        A = [[1.0, -dt], [0.0, 1.0]]
        B = [[dt], [0.0]]
        Q = [[(self.q_rate ** 2) * dt, 0.0], [0.0, (self.q_bias ** 2) * dt]]
        self.kf.predict(A, Q, B, [r_meas])
        self.kf.x[0] = wrap_pi(self.kf.x[0])

    def update(self, theta_mag):
        innov = [wrap_pi(theta_mag - self.kf.x[0])]
        ok = self.kf.update([theta_mag], [[1.0, 0.0]], [[self.r_mag ** 2]], innov=innov)
        self.kf.x[0] = wrap_pi(self.kf.x[0])
        return ok

    @property
    def theta(self):
        return self.kf.x[0]

    @property
    def bias(self):
        return self.kf.x[1]


class PositionKF(object):
    def __init__(self, x0, y0, sigma_pos0, sigma_vel0, q_acc, r_gps, gate=CHI2_999[2]):
        P0 = [[0.0] * 4 for _ in range(4)]
        P0[0][0] = P0[1][1] = sigma_pos0 ** 2
        P0[2][2] = P0[3][3] = sigma_vel0 ** 2
        self.kf = KF([x0, y0, 0.0, 0.0], P0)
        self.q_acc = q_acc
        self.r_gps = r_gps
        self.gate = gate
        self.rejected = 0

    def predict(self, dt):
        A = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
        q = self.q_acc ** 2
        d2, d3 = dt * dt, dt ** 3
        Q = [[q * d3 / 3, 0, q * d2 / 2, 0], [0, q * d3 / 3, 0, q * d2 / 2],
             [q * d2 / 2, 0, q * dt, 0], [0, q * d2 / 2, 0, q * dt]]
        self.kf.predict(A, Q)

    def update(self, x, y):
        C = [[1, 0, 0, 0], [0, 1, 0, 0]]
        R = [[self.r_gps ** 2, 0], [0, self.r_gps ** 2]]
        ok = self.kf.update([x, y], C, R, gate=self.gate)
        if not ok:
            self.rejected += 1
        return ok

    @property
    def state(self):
        return list(self.kf.x)
