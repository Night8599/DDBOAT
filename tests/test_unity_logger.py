import json
import math
import os
import socket
import threading
import time
import unittest

import helpers
from ddboat.hal.unity import UnitySim
from ddboat.mission.runner import run_mission
from ddboat.io.logger import read_csv


class FakeUnity(threading.Thread):
    """Unity falso: convencao espelhada e com defasagem (pior caso)."""

    def __init__(self, cmd_port, pose_port, robot_id, sign=-1, off_deg=37.0):
        threading.Thread.__init__(self)
        self.daemon = True
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", cmd_port))
        self.rx.settimeout(0.01)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pose_port = pose_port
        self.id = robot_id
        self.sign, self.off = sign, math.radians(off_deg)
        self.x = self.y = 0.0
        self.th = 0.0
        self.l = self.r = 0
        self.run_flag = True

    def run(self):
        last = time.time()
        while self.run_flag:
            try:
                d, _ = self.rx.recvfrom(1024)
                m = json.loads(d.decode())
                if m.get("id") == self.id and "cmd_left" in m:
                    self.l, self.r = m["cmd_left"], m["cmd_right"]
            except socket.timeout:
                pass
            now = time.time()
            dt = now - last
            last = now
            v = 0.02 * (self.l + self.r) / 2.0
            self.th += dt * 0.01 * (self.r - self.l)
            self.x += dt * v * math.cos(self.th)
            self.y += dt * v * math.sin(self.th)
            h = math.degrees((self.th - self.off) * self.sign)
            for msg in ({"id": 99, "X": 1, "Y": 1, "heading_deg": 0},
                        {"id": self.id, "X": self.x, "Y": self.y, "heading_deg": h}):
                self.tx.sendto(json.dumps(msg).encode(), ("127.0.0.1", self.pose_port))
            self.tx.sendto(b"lixo", ("127.0.0.1", self.pose_port))
            time.sleep(0.01)


class TestUnity(unittest.TestCase):
    def test_conv_then_goto(self):
        from ddboat.calib import unity_conv
        cfg = helpers.cfg(**{"unity.ip": "127.0.0.1", "unity.cmd_port": 15005, "unity.pose_port": 15006})
        fu = FakeUnity(15005, 15006, 4)
        fu.start()
        b = UnitySim(cfg)
        b.start()
        # manobra de convencao (mesma do comando unity-conv), versao curta
        def leg(cmd, dur):
            t0 = time.time()
            first = None
            while time.time() - t0 < dur:
                b.read()
                if first is None and b.last_pose is not None:
                    first = b.last_pose
                b.set_motors(*cmd)
                time.sleep(0.02)
            b.read()
            return first, b.last_pose
        time.sleep(0.2)
        l1 = leg((100, 100), 1.0)
        tr = leg((-100, 100), 1.0)
        l2 = leg((100, 100), 1.0)
        b.close()
        conv = unity_conv.from_two_legs(l1, l2, tr[0][2], tr[1][2], 0.05, 10)
        self.assertEqual(conv["sign"], -1)
        self.assertLess(abs((conv["offset_deg"] - 37 + 180) % 360 - 180), 2.0)
        self.assertEqual(conv["turn_sign"], 1)
        p = os.path.join(helpers.ROOT, "data", "test_unity_conv.json")
        with open(p, "w") as f:
            json.dump(conv, f)
        cfg.data["nav"]["unity_conv_file"] = "data/test_unity_conv.json"
        tx, ty = fu.x + 3.0, fu.y - 4.0
        b2 = UnitySim(cfg)
        r = run_mission(cfg, b2, {"steps": [{"type": "goto", "point": {"x": tx, "y": ty},
                                              "radius_m": 0.5, "thrust": 120}]},
                        tag="test_unity", quiet=True, max_ticks=600)
        time.sleep(0.3)  # deixa o Unity falso consumir os 3 pacotes de parada
        fu.run_flag = False
        os.remove(p)
        self.assertTrue(r["ok"], r)
        self.assertLess(math.hypot(fu.x - tx, fu.y - ty), 1.0)
        self.assertEqual((fu.l, fu.r), (0, 0))


class TestOldLogs(unittest.TestCase):
    def test_read_old_formats(self):
        proj = "/mnt/project"
        for name in ("log_001.csv", "log_003.csv"):
            p = os.path.join(proj, name)
            if not os.path.exists(p):
                self.skipTest("logs antigos ausentes")
            cols, rows = read_csv(p)
            self.assertIn("rumo_deg", cols)
            self.assertIsInstance(rows[0]["mag_x"], float)
        cols, rows = read_csv(os.path.join(proj, "log_003.csv"))
        self.assertAlmostEqual(rows[0]["lat"], 48.4193567, 7)


if __name__ == "__main__":
    unittest.main()
