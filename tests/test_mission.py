import math
import os
import unittest

import helpers
from ddboat.core.config import REPO_ROOT
from ddboat.core.geo import LocalFrame
from ddboat.hal.pysim import PySim
from ddboat.mission.runner import run_mission
from ddboat.io.logger import read_csv

ORIGIN = [48.19924, -3.0145649]
F = LocalFrame(ORIGIN[0], ORIGIN[1])


def ll(x, y):
    lat, lon = F.to_latlon(x, y)
    return {"lat": lat, "lon": lon}


def base_cfg(**kw):
    d = {"nav.origin_latlon": ORIGIN}
    d.update(kw)
    return helpers.cfg(**d)


def run(cfg, steps, ticks=30000, on_tick=None, seed=3):
    b = PySim(cfg, seed=seed)
    traj = []

    def cb(be, st):
        traj.append(be.truth()[:3])
        if on_tick:
            on_tick(be, st, traj)
    r = run_mission(cfg, b, {"steps": steps}, tag="test", quiet=True, max_ticks=ticks, on_tick=cb)
    return r, traj


class TestMissionPySim(unittest.TestCase):
    def test_goto_latlon_and_polygon(self):
        cfg = base_cfg()
        steps = [{"type": "gyro_bias", "duration_s": 3}, {"type": "wait_fix", "timeout_s": 5},
                 {"type": "goto", "point": ll(40, 30), "radius_m": 3, "thrust": 150},
                 {"type": "polygon", "points": [ll(40, 30), ll(40, -20), ll(-10, -20)], "laps": 1,
                  "thrust": 150}]
        r, traj = run(cfg, steps)
        self.assertTrue(r["ok"], r)
        x, y, _ = traj[-1]
        self.assertLess(math.hypot(x - 40, y - 30), 6)
        cols, rows = read_csv(r["log"])
        self.assertIn("theta_ref_deg", cols)
        self.assertGreater(len(rows), 100)

    def test_heading_step_compass(self):
        cfg = base_cfg()
        steps = [{"type": "heading", "compass_deg": 90, "thrust": 120, "duration_s": 40}]
        r, traj = run(cfg, steps)
        self.assertTrue(r["ok"])
        th = traj[-1][2]
        self.assertLess(abs(math.degrees(th)), 8)  # bussola 90 = leste = theta 0

    def test_fusions_and_kf(self):
        for fusion in ("complementary", "kalman"):
            cfg = base_cfg(**{"nav.gyro_fusion": fusion, "nav.position_filter_kalman": True})
            steps = [{"type": "gyro_bias", "duration_s": 3},
                     {"type": "goto", "point": {"x": 30, "y": -25}, "radius_m": 3, "thrust": 150}]
            r, traj = run(cfg, steps)
            self.assertTrue(r["ok"], (fusion, r))

    def test_current_line(self):
        cfg = base_cfg(**{"pysim.current_xy_mps": [0.0, 0.3]})
        steps = [{"type": "line", "a": {"x": 0, "y": 0}, "b": {"x": 80, "y": 0}, "thrust": 150}]
        errs = []
        r, traj = run(cfg, steps, on_tick=lambda be, st, tr: errs.append(abs(tr[-1][1])) if tr[-1][0] > 30 else None)
        self.assertTrue(r["ok"])
        self.assertLess(max(errs), 4.0)  # campo de vetores ainda segura com corrente lateral

    def test_geofence_rth(self):
        fence = [ll(-20, -20), ll(30, -20), ll(30, 30), ll(-20, 30)]
        cfg = base_cfg(**{"safety.geofence_latlon": [[p["lat"], p["lon"]] for p in fence],
                          "safety.home_latlon": ORIGIN})
        steps = [{"type": "goto", "point": {"x": 100, "y": 0}, "radius_m": 3, "thrust": 150}]
        r, traj = run(cfg, steps)
        self.assertTrue(r["ok"], r)
        self.assertIn("RTH", r["reason"])
        self.assertLess(math.hypot(traj[-1][0], traj[-1][1]), 8)
        self.assertLess(max(p[0] for p in traj), 40)

    def test_geofence_without_home_stops(self):
        fence = [ll(-20, -20), ll(30, -20), ll(30, 30), ll(-20, 30)]
        cfg = base_cfg(**{"safety.geofence_latlon": [[p["lat"], p["lon"]] for p in fence]})
        steps = [{"type": "goto", "point": {"x": 100, "y": 0}, "radius_m": 3, "thrust": 150}]
        r, traj = run(cfg, steps)
        self.assertFalse(r["ok"])
        self.assertIn("home_latlon = null", r["reason"])

    def test_stop_file(self):
        cfg = base_cfg()
        stop = os.path.join(REPO_ROOT, "data", "STOP")

        def cb(be, st, tr):
            if len(tr) == 50:
                open(stop, "w").close()
        steps = [{"type": "heading", "compass_deg": 0, "thrust": 120, "duration_s": 100}]
        r, traj = run(cfg, steps, on_tick=cb)
        self.assertFalse(r["ok"])
        self.assertIn("STOP", r["reason"])
        self.assertLess(len(traj), 55)
        os.remove(stop)

    def test_gps_loss_hold(self):
        cfg = base_cfg()
        cmds = []

        def cb(be, st, tr):
            if len(tr) == 100:
                be.gps_period = 1e9  # GPS some
            cmds.append(be.cmd)
        steps = [{"type": "goto", "point": {"x": 500, "y": 0}, "radius_m": 3, "thrust": 150}]
        r, traj = run(cfg, steps, ticks=300, on_tick=cb)
        self.assertEqual(r["reason"], "max_ticks")
        self.assertEqual(cmds[-1], (0, 0))  # parado depois do timeout

    def test_latlon_without_origin_errors(self):
        cfg = helpers.cfg()
        steps = [{"type": "goto", "point": ll(10, 10), "radius_m": 3, "thrust": 100}]
        r, _ = run(cfg, steps)
        self.assertFalse(r["ok"])
        self.assertIn("origem ENU", r["reason"])

    def test_real_requires_geofence(self):
        from ddboat.mission.steps import MissionError

        class FakeReal(object):
            name = "real"
        with self.assertRaises(MissionError):
            run_mission(helpers.cfg(), FakeReal(), {"steps": []})


if __name__ == "__main__":
    unittest.main()
