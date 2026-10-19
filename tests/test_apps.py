"""Comandos do main.py em subprocesso, com hardware falso (tests/fakes)."""
import json
import os
import pty
import select
import subprocess
import sys
import time
import unittest

import helpers

FAKES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fakes")
MAIN = os.path.join(helpers.ROOT, "main.py")


def env(**kw):
    e = dict(os.environ, PYTHONPATH=FAKES)
    e.update(kw)
    return e


class TestApps(unittest.TestCase):
    def test_selftest_and_gps_and_imu_raw(self):
        out = subprocess.run([sys.executable, MAIN, "selftest", "--gps-s", "1"], env=env(),
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60).stdout.decode()
        self.assertIn("[OK ] imu-acc", out)
        self.assertIn("[OK ] arduino", out)
        out = subprocess.run([sys.executable, MAIN, "imu-raw", "--dur", "1"], env=env(),
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60).stdout.decode()
        self.assertIn("picos de ~256: 0", out)

    def test_calib_mag2d_then_heading_value(self):
        hf = os.path.join(helpers.ROOT, "data", "fake_heading")
        cfgp = os.path.join(helpers.ROOT, "data", "test_cfg.json")
        d = helpers.cfg(**{"calib.mag2d_rotation_s": 4,
                           "nav.mag2d_file": "data/test_mag2d.json"}).data
        with open(cfgp, "w") as f:
            json.dump(d, f)
        with open(hf, "w") as f:
            f.write("rot")
        p = subprocess.Popen([sys.executable, MAIN, "--config", cfgp, "calib-mag2d"], env=env(FAKE_HEADING_FILE=hf),
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        time.sleep(1.0)
        p.stdin.write(b"\n"); p.stdin.flush()          # fase 1 (gira 4 s, so 160 graus)
        out = p.communicate(timeout=30)[0].decode()
        self.assertIn("volta incompleta", out, out)  # 4 s a 40 graus/s = 160 graus
        # agora com volta completa
        os.environ["FAKE_ROT_DPS"] = "120"
        with open(hf, "w") as f:
            f.write("rot")
        p = subprocess.Popen([sys.executable, MAIN, "--config", cfgp, "calib-mag2d"],
                             env=env(FAKE_HEADING_FILE=hf, FAKE_ROT_DPS="120"),
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        time.sleep(1.0)
        p.stdin.write(b"\n"); p.stdin.flush()
        time.sleep(5.0)
        with open(hf, "w") as f:
            f.write("0")
        p.stdin.write(b"\n"); p.stdin.flush()
        time.sleep(1.0)
        with open(hf, "w") as f:
            f.write("90")
        p.stdin.write(b"\n"); p.stdin.flush()
        out = p.communicate(timeout=30)[0].decode()
        self.assertIn("sign = -1", out, out)
        from ddboat.calib import mag2d
        c = mag2d.load(os.path.join(helpers.ROOT, "data", "test_mag2d.json"))
        import math
        a = math.radians(37 - 200)
        h = mag2d.heading_compass_deg(int(3400 + 800 * math.cos(a)), int(-2900 + 900 * math.sin(a)), c)
        self.assertLess(abs((h - 200 + 180) % 360 - 180), 2.0)
        for f_ in (hf, cfgp, os.path.join(helpers.ROOT, "data", "test_mag2d.json")):
            os.remove(f_)

    def test_teleop_pysim_keys(self):
        pid, fd = pty.fork()
        if pid == 0:
            os.execve(sys.executable, [sys.executable, MAIN, "teleop", "--backend", "pysim"], env())
        out = b""

        def rd(t):
            nonlocal_out = []
            end = time.time() + t
            while time.time() < end:
                r, _, _ = select.select([fd], [], [], 0.05)
                if r:
                    try:
                        nonlocal_out.append(os.read(fd, 4096))
                    except OSError:
                        break
            return b"".join(nonlocal_out)
        out += rd(1.5)
        os.write(fd, b"150\n")
        out += rd(0.5)
        os.write(fd, b"w")
        out += rd(3.0)
        os.write(fd, b"a")
        out += rd(1.0)
        os.write(fd, b"q")
        out += rd(2.0)
        os.waitpid(pid, 0)
        txt = out.decode(errors="replace")
        self.assertIn("Sem re", txt)
        self.assertIn("cmd=( 150, 150)", txt)
        self.assertIn("cmd=(   0, 150)", txt)  # 'a' sem re vira (0, V)
        self.assertIn("Motores parados", txt)

    def test_run_pysim_cli(self):
        m = os.path.join(helpers.ROOT, "missions", "pysim_demo.json")
        r = subprocess.run([sys.executable, MAIN, "run", m, "--backend", "pysim", "--quiet"], env=env(),
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
        out = r.stdout.decode()
        self.assertEqual(r.returncode, 0, out)
        self.assertIn('"ok": true', out)


if __name__ == "__main__":
    unittest.main()
