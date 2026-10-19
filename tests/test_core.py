import math
import unittest

import helpers  # noqa: F401
from ddboat.core import angles as A
from ddboat.core import geo as G
from ddboat.core import linalg as la
from ddboat.core.clock import Rate, VirtualClock

try:
    import numpy as np
except ImportError:
    np = None


class TestAngles(unittest.TestCase):
    def test_compass_theta(self):
        cases = [(0, 90), (90, 0), (180, -90), (270, 180), (45, 45)]
        for comp, th_deg in cases:
            th = A.compass_to_theta(comp)
            self.assertAlmostEqual(A.wrap_180(math.degrees(th) - th_deg), 0.0, 9)
            self.assertAlmostEqual(A.wrap_180(A.theta_to_compass(th) - comp), 0.0, 9)

    def test_wrap(self):
        self.assertAlmostEqual(A.wrap_pi(3 * math.pi), -math.pi)
        self.assertAlmostEqual(A.wrap_180(190), -170)
        self.assertAlmostEqual(A.diff(math.radians(179), math.radians(-179)), math.radians(-2))

    def test_circular_mean(self):
        self.assertAlmostEqual(A.wrap_180(A.circular_mean_deg([350, 10]) - 0), 0, 9)


class TestGeo(unittest.TestCase):
    def test_ddmm(self):
        self.assertAlmostEqual(G.ddmm_to_deg("4825.154359", "N"), 48 + 25.154359 / 60)
        self.assertAlmostEqual(G.ddmm_to_deg(428.162682, "W"), -(4 + 28.162682 / 60))

    def test_roundtrip_and_axes(self):
        f = G.LocalFrame(48.19924, -3.0145649)
        lat, lon = f.to_latlon(100.0, 0.0)  # 100 m a leste
        self.assertAlmostEqual(lat, 48.19924, 9)
        self.assertGreater(lon, -3.0145649)
        x, y = f.to_xy(lat, lon)
        self.assertAlmostEqual(x, 100.0, 6)
        self.assertAlmostEqual(y, 0.0, 6)
        x, y = f.to_xy(48.19924 + 0.001, -3.0145649)  # ao norte
        self.assertAlmostEqual(x, 0.0, 6)
        self.assertAlmostEqual(y, 111.32, 1)

    def test_same_as_ddboatlib_pool(self):
        # ddboatlib: psi0 = 356.12-360 graus, lat0/long0 em rad
        lat0, lon0 = math.degrees(0.84507466), math.degrees(-0.07800405)
        f = G.LocalFrame(lat0, lon0, 356.12 - 360)
        psi0 = math.radians(356.12 - 360)
        x, y = 5.0, 12.0
        x1 = math.cos(psi0) * x + math.sin(psi0) * y
        y1 = -math.sin(psi0) * x + math.cos(psi0) * y
        lat = 0.84507466 + y1 / 6378137.0
        lon = -0.07800405 + x1 / (6378137.0 * math.cos(0.84507466))
        la_, lo_ = f.to_latlon(x, y)
        self.assertAlmostEqual(la_, math.degrees(lat), 10)
        self.assertAlmostEqual(lo_, math.degrees(lon), 10)

    def test_polygon(self):
        sq = [(0, 0), (10, 0), (10, 10), (0, 10)]
        self.assertTrue(G.point_in_polygon((5, 5), sq))
        self.assertFalse(G.point_in_polygon((15, 5), sq))


@unittest.skipIf(np is None, "numpy ausente")
class TestLinalg(unittest.TestCase):
    def setUp(self):
        self.rng = np.random.RandomState(0)

    def test_solve_inv(self):
        for n in (2, 3, 4, 9):
            a = self.rng.randn(n, n) + n * np.eye(n)
            b = self.rng.randn(n)
            np.testing.assert_allclose(la.solve(a.tolist(), b.tolist()), np.linalg.solve(a, b), atol=1e-10)
            np.testing.assert_allclose(la.inv(a.tolist()), np.linalg.inv(a), atol=1e-10)

    def test_eig_sqrtm(self):
        m = self.rng.randn(3, 3)
        s = m @ m.T + 3 * np.eye(3)
        r = np.array(la.sqrtm_spd(s.tolist()))
        np.testing.assert_allclose(r @ r, s, atol=1e-9)
        np.testing.assert_allclose(r, r.T, atol=1e-12)

    def test_nearest_rotation(self):
        q, _ = np.linalg.qr(self.rng.randn(3, 3))
        if np.linalg.det(q) < 0:
            q[:, 0] *= -1
        m = q + 0.05 * self.rng.randn(3, 3)
        u, _, vt = np.linalg.svd(m)
        ref = u @ vt
        r = np.array(la.nearest_rotation(m.tolist()))
        np.testing.assert_allclose(r, ref, atol=1e-9)

    def test_rodrigues(self):
        r = np.array(la.rodrigues([0, 0, math.pi / 2]))
        np.testing.assert_allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-12)


class TestRate(unittest.TestCase):
    def test_virtual(self):
        c = VirtualClock()
        r = Rate(10, c)
        for _ in range(10):
            r.sleep()
        self.assertAlmostEqual(c.now(), 1.0, 9)
        self.assertEqual(r.overruns, 0)
        c.sleep(0.5)  # atraso grande
        r.sleep()
        self.assertEqual(r.overruns, 1)


if __name__ == "__main__":
    unittest.main()
