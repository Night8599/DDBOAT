import math
import random
import unittest

import helpers  # noqa: F401
from ddboat.control.pid import PID
from ddboat.control.mixer import Mixer
from ddboat.guidance import core as gd
from ddboat.estimation.kalman import HeadingKF, PositionKF
from ddboat.estimation.filters import Hampel, EMA, CircularEMA
from ddboat.core.angles import wrap_pi


class TestPID(unittest.TestCase):
    def test_antiwindup(self):
        pid = PID(1.0, ki=1.0, out_min=-1, out_max=1)
        for _ in range(100):
            pid.update(10.0, 0.0, 0.1)  # saturado o tempo todo
        self.assertAlmostEqual(pid.integral, 0.0)

    def test_derivative_on_measurement(self):
        pid = PID(0.0, kd=1.0)
        pid.update(0.0, 0.0, 0.1)
        u = pid.update(100.0, 0.0, 0.1)  # degrau na consigna: sem pico
        self.assertEqual(u, 0.0)
        u = pid.update(0.0, 0.0, 0.1, meas_rate=2.0)
        self.assertEqual(u, -2.0)

    def test_angular_wrap(self):
        pid = PID(1.0, angular=True)
        u = pid.update(math.radians(179), math.radians(-179), 0.1)
        self.assertAlmostEqual(u, math.radians(-2), 9)

    def test_closed_loop_first_order(self):
        pid = PID(2.0, ki=1.0, out_min=-5, out_max=5)
        y = 0.0
        for _ in range(400):
            u = pid.update(1.0, y, 0.05)
            y += 0.05 * (-y + u)
        self.assertAlmostEqual(y, 1.0, 3)


class TestMixer(unittest.TestCase):
    def test_no_reverse_and_priority(self):
        m = Mixer(0, 191)
        self.assertEqual(m.mix(100, 0), (100, 100))
        self.assertEqual(m.mix(100, 30), (70, 130))
        l, r = m.mix(191, 50)  # empuxo no maximo: giro preservado
        self.assertEqual(r - l, 100)
        self.assertEqual((l, r), (91, 191))
        l, r = m.mix(20, 50)  # empuxo baixo: sem negativo, giro preservado
        self.assertEqual((l, r), (0, 100))
        l, r = m.mix(100, 500)  # giro maior que possivel
        self.assertEqual((l, r), (0, 191))
        for T in range(0, 192, 7):
            for D in range(-200, 201, 13):
                l, r = m.mix(T, D)
                self.assertTrue(0 <= l <= 191 and 0 <= r <= 191)

    def test_reverse_and_sign(self):
        m = Mixer(-190, 190, turn_sign=-1)
        self.assertEqual(m.mix(0, 50), (50, -50))


class TestGuidance(unittest.TestCase):
    def test_cross_track_sign(self):
        e, s, L, _ = gd.cross_track((0, 0), (10, 0), (5, 2))
        self.assertAlmostEqual(e, 2.0)  # a esquerda = positivo
        th, e = gd.line_follow_theta((0, 0), (10, 0), (5, 2), 10, 1.0)
        self.assertLess(th, 0)  # deve virar para a direita (theta menor)

    def test_passed_b(self):
        self.assertFalse(gd.passed_b((0, 0), (10, 0), (9.9, 3)))
        self.assertTrue(gd.passed_b((0, 0), (10, 0), (10.1, -3)))

    def test_field_converges_kinematic(self):
        for mode in ("field", "lookahead"):
            x, y, th = 0.0, 15.0, 0.0
            a, b = (0, 0), (200, 0)
            for _ in range(2000):
                if mode == "field":
                    ref, e = gd.line_follow_theta(a, b, (x, y), 10, 1.0)
                else:
                    ref, e = gd.lookahead_theta(a, b, (x, y), 8.0)
                th += 0.05 * 2.0 * wrap_pi(ref - th)
                x += 0.05 * math.cos(th)
                y += 0.05 * math.sin(th)
            self.assertLess(abs(y), 0.3, mode)


class TestKalman(unittest.TestCase):
    def test_heading_bias(self):
        rnd = random.Random(0)
        kf = HeadingKF(0.0, 0.0, 0.2, 0.02, 0.01, 0.0005, 0.05)
        th, bias, dt = 0.0, 0.01, 0.1
        for k in range(3000):
            w = 0.3 * math.sin(k * 0.01)
            th = wrap_pi(th + w * dt)
            kf.predict(w + bias + rnd.gauss(0, 0.003), dt)
            kf.update(wrap_pi(th + rnd.gauss(0, 0.05)))
        self.assertAlmostEqual(kf.bias, bias, 3)
        self.assertLess(abs(wrap_pi(kf.theta - th)), 0.05)

    def test_position_gate(self):
        rnd = random.Random(1)
        kf = PositionKF(0, 0, 5, 1, 0.3, 2.0)
        x = 0.0
        for k in range(200):
            x += 1.0 * 1.0
            kf.predict(1.0)
            kf.update(x + rnd.gauss(0, 2), rnd.gauss(0, 2))
        self.assertLess(abs(kf.state[2] - 1.0), 0.3)
        ok = kf.update(x + 300, 0)  # salto absurdo
        self.assertFalse(ok)


class TestFilters(unittest.TestCase):
    def test_hampel(self):
        h = Hampel(7, 3)
        out = [h.update(v) for v in [10, 11, 10, 12, 11, 266, 11, 10]]
        self.assertLess(max(out), 20)

    def test_circular_ema(self):
        f = CircularEMA(0.5)
        f.update(math.radians(179))
        y = f.update(math.radians(-179))
        self.assertAlmostEqual(abs(y), math.pi, 1)


if __name__ == "__main__":
    unittest.main()
