import json
import math
import os
import random
import tempfile
import unittest

import helpers
from ddboat.core import linalg as la
from ddboat.core.angles import compass_to_theta, theta_to_compass, wrap_180
from ddboat.calib import mag2d, mag3d, imu as imucal, unity_conv
from ddboat.estimation.attitude import euler_angles, heading_tilt_compensated


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


I_DEG = 64.0
B_ENU = [0.0, math.cos(math.radians(I_DEG)), -math.sin(math.radians(I_DEG))]


class World(object):
    """Barco FLU com rumo (bussola), pitch e roll; IMU montada com rotacao R_ib."""

    def __init__(self, seed=0):
        rnd = random.Random(seed)
        # montagem: z da IMU para baixo (como nos logs) + pequena rotacao
        self.R_ib = la.matmul(rx(math.pi), la.matmul(rz(0.3), ry(0.05)))  # imu -> barco
        self.A = [[480, 40, 25], [40, 450, 10], [25, 10, 520]]  # soft iron (LSB por unidade de campo)
        self.xbar = [3200.0, -2900.0, 8000.0]  # hard iron enorme (como no barco)
        self.rnd = rnd

    def R_wb(self, compass_deg, pitch, roll):
        return la.matmul(rz(compass_to_theta(compass_deg)), la.matmul(ry(pitch), rx(roll)))

    def raw(self, compass_deg, pitch=0.0, roll=0.0, noise=0.0):
        Rwb = self.R_wb(compass_deg, pitch, roll)
        b_boat = la.matvec(la.transpose(Rwb), B_ENU)
        f_boat = la.matvec(la.transpose(Rwb), [0, 0, 1.0])
        Rbi = la.transpose(self.R_ib)
        b_imu = la.matvec(Rbi, b_boat)
        f_imu = la.matvec(Rbi, f_boat)
        m = [a + b + self.rnd.gauss(0, noise) for a, b in zip(la.matvec(self.A, b_imu), self.xbar)]
        acc = [4096 * v for v in f_imu]
        return m, acc


class TestMag2D(unittest.TestCase):
    def test_procedure_both_signs(self):
        for sign_flip in (False, True):
            def raw(comp):
                a = math.radians(37 + (-comp if sign_flip else comp))
                return (3400 + 800 * math.cos(a), -2900 + 900 * math.sin(a))
            pts = [raw(c) for c in range(0, 360, 5)]
            c = mag2d.fit_minmax([p[0] for p in pts], [p[1] for p in pts])
            c, d = mag2d.set_north_east(c, [raw(0)] * 5, [raw(90)] * 5)
            for comp in (0, 45, 90, 200, 300):
                h = mag2d.heading_compass_deg(raw(comp)[0], raw(comp)[1], c)
                self.assertAlmostEqual(wrap_180(h - comp), 0.0, 6)

    def test_bad_east(self):
        c = mag2d.fit_minmax([0, 2], [0, 2])
        with self.assertRaises(mag2d.CalibError):
            mag2d.set_north_east(c, [(2, 1)] * 3, [(2, 1)] * 3)

    def test_load_old_calib_cap(self):
        d = {"offx": 1.0, "offy": 2.0, "sx": 0.1, "sy": 0.1, "north": 3.0, "sign": -1}
        p = os.path.join(tempfile.mkdtemp(), "calib_cap.json")
        with open(p, "w") as f:
            json.dump(d, f)
        self.assertEqual(mag2d.load(p)["sign"], -1)


class TestMag3D(unittest.TestCase):
    def setUp(self):
        self.w = World(1)
        self.noise = getattr(self, "noise", 3)
        rnd = random.Random(2)
        self.cloud = []
        for _ in range(600):
            m, _ = self.w.raw(rnd.uniform(0, 360), rnd.uniform(-1.5, 1.5), rnd.uniform(-3.1, 3.1), noise=self.noise)
            self.cloud.append(m)
        xn, _ = self.w.raw(0.0)
        xw, _ = self.w.raw(270.0)
        self.cal, self.diag = mag3d.calibrate(self.cloud, xn, xw, I_DEG)

    def test_fit(self):
        self.assertAlmostEqual(self.diag["norm_mean"], 1.0, 2)
        self.assertLess(self.diag["norm_std"], 0.01)
        for a, b in zip(self.cal["xbar"], self.w.xbar):
            self.assertLess(abs(a - b), 5)

    def test_heading_flat(self):
        for comp in (0, 33, 90, 181, 270, 359):
            m, _ = self.w.raw(comp)
            y = mag3d.apply(self.cal, m)
            psi = -math.atan2(y[1], y[0])
            est = theta_to_compass(psi + math.pi / 2)
            self.assertLess(abs(wrap_180(est - comp)), 0.5, (comp, est))

    def test_tilt_compensation(self):
        # eixos do acelerometro pelo procedimento horizontal + proa levantada
        _, a_flat = self.w.raw(10.0)
        _, a_bow = self.w.raw(10.0, pitch=math.radians(-25))  # nariz para cima
        R, tilt = imucal.axes_from_flat_bow(a_flat, a_bow)
        self.assertAlmostEqual(tilt, 25.0, 3)
        worst_tc, worst_flat = 0.0, 0.0
        for comp in (0, 90, 135, 270):
            for p, r in ((10, 0), (0, 10), (-8, 6)):
                m, acc = self.w.raw(comp, math.radians(p), math.radians(r))
                y = mag3d.apply(self.cal, m)
                psi = heading_tilt_compensated(la.matvec(R, acc), y)
                worst_tc = max(worst_tc, abs(wrap_180(theta_to_compass(psi + math.pi / 2) - comp)))
                psi_j = euler_angles(la.matvec(R, acc), y)[2]  # polycopie: perto, nao exato
                self.assertLess(abs(wrap_180(theta_to_compass(psi_j + math.pi / 2) - comp)), 1.5)
                psi_f = -math.atan2(y[1], y[0])
                worst_flat = max(worst_flat, abs(wrap_180(theta_to_compass(psi_f + math.pi / 2) - comp)))
        self.assertLess(worst_tc, 0.05 if self.noise == 0 else 1.0)
        self.assertGreater(worst_flat, 10.0)  # sem compensacao o erro e grande


class TestMag3DNoNoise(TestMag3D):
    noise = 0


class TestImu(unittest.TestCase):
    def test_yaw_rate_sign_z_down(self):
        # IMU z para baixo: acc em repouso ~ (0,0,-4096); giro anti-horario visto de
        # cima = rotacao em torno de -z da IMU => gyro_z negativo
        g = [[1, -2, 5]] * 50
        a = [[0, 0, -4096]] * 50
        cal = imucal.gyro_still(g, a, 30)
        r = imucal.yaw_rate([1, -2, 5 - 100], cal)  # gz 100 LSB abaixo do bias
        self.assertAlmostEqual(r, math.radians(100 * 0.07), 9)

    def test_not_still(self):
        g = [[0, 0, 0], [0, 0, 200]] * 10
        with self.assertRaises(imucal.CalibError):
            imucal.gyro_still(g, [[0, 0, -4096]] * 20, 30)


class TestUnityConv(unittest.TestCase):
    def test_conventions(self):
        for sign, off, turn_ccw in ((1, 0, True), (-1, 90, True), (1, 37, False), (-1, -120, False)):
            def head(theta):
                return math.degrees(sign * (theta - math.radians(off)))
            th1, th2 = 0.3, 0.3 + (0.8 if turn_ccw else -0.8)
            leg = lambda th, x0: ((x0, 0, head(th)), (x0 + 2 * math.cos(th), 2 * math.sin(th), head(th)))
            c = unity_conv.from_two_legs(leg(th1, 0), leg(th2, 5), head(th1), head(th2), 0.05, 10)
            self.assertEqual(c["sign"], sign)
            self.assertAlmostEqual(wrap_180(c["offset_deg"] - off), 0, 6)
            self.assertEqual(c["turn_sign"], 1 if turn_ccw else -1)
            self.assertAlmostEqual(unity_conv.theta_from_heading(head(1.0), c), 1.0, 9)


if __name__ == "__main__":
    unittest.main()
