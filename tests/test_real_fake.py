"""Backend real com smbus/pyserial FALSOS (tests/fakes): testa a integracao com os
drivers da ENSTA do repositorio, sem hardware."""
import os
import signal
import subprocess
import sys
import time
import unittest

import helpers

FAKES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fakes")
sys.path.insert(0, FAKES)
import serial as fake_serial  # noqa: E402
import smbus as fake_smbus  # noqa: E402

from ddboat.hal.real import RealBoat  # noqa: E402


def nmea(body):
    x = 0
    for c in body:
        x ^= ord(c)
    return "$%s*%02X\r\n" % (body, x)


class TestRealBackendFake(unittest.TestCase):
    def setUp(self):
        fake_serial.ARDUINO_WRITES[:] = []
        fake_serial.GPS_LINES[:] = []

    def make(self, **kw):
        cfg = helpers.cfg(**kw)
        b = RealBoat(cfg)
        return b

    def test_imu_shim_and_median(self):
        # mag_x: bytes (0x19, 0x0B) = 2841 ; um pico nao passa pela mediana
        fake_smbus.READ_VALUES[(0x1e, 0x28)] = [0x19, 0x0B, 0, 0, 0, 0]
        b = self.make()
        b.start(use_imu=True, use_gps=False, use_motors=False)
        m, a, g = b.read_imu()
        self.assertEqual(m[0], 2841)
        b.close()

    def test_bdu_flag_writes(self):
        fake_smbus.WRITES[:] = []
        b = self.make(**{"real.imu.bdu": True})
        b.start(use_imu=True, use_gps=False, use_motors=False)
        self.assertIn((0x6b, 0x12, [0x44]), fake_smbus.WRITES)
        self.assertIn((0x1e, 0x24, [0x40]), fake_smbus.WRITES)
        b.close()

    def test_motor_clamp_no_reverse(self):
        b = self.make()
        b.start(use_imu=False, use_gps=False, use_motors=True)
        self.assertEqual(b.set_motors(-50, 300), (0, 191))
        self.assertEqual(fake_serial.ARDUINO_WRITES[-1], "M 000 191;")
        b.close()
        self.assertEqual(fake_serial.ARDUINO_WRITES[-1], "M 000 000;")

    def test_watchdog(self):
        b = self.make(**{"real.motors.watchdog_s": 0.2})
        b.start(use_imu=False, use_gps=False, use_motors=True)
        b.set_motors(100, 100)
        time.sleep(0.5)
        self.assertEqual(b.watchdog_trips, 1)
        self.assertEqual(fake_serial.ARDUINO_WRITES[-1], "M 000 000;")
        b.close()

    def test_gps_driver_mode_rejects_nofix(self):
        fake_serial.GPS_LINES[:] = [nmea("GPRMC,123519,V,,,,,,,230394,,,N"),
                                    nmea("GPRMC,123520,A,4811.9544,N,00300.8739,W,0.5,20.9,230394,,,A")]
        b = self.make()
        b.start(use_imu=False, use_gps=True, use_motors=False)
        time.sleep(0.3)
        f = b.gps_th.pop_new()
        self.assertIsNotNone(f)
        self.assertAlmostEqual(f.lat, 48 + 11.9544 / 60, 7)
        self.assertAlmostEqual(f.lon, -(3 + 0.8739 / 60), 7)
        self.assertEqual(b.gps_th.rejected, 1)
        b.close()

    def test_gps_raw_mode_checksum(self):
        good = nmea("GPRMC,123520,A,4811.9544,N,00300.8739,W,0.5,20.9,230394,,,A")
        bad = good.replace("4811.9544", "4811.9545")  # checksum nao bate mais
        fake_serial.GPS_LINES[:] = [bad, nmea("GPGGA,123520,4811.9544,N,00300.8739,W,1,08,0.9,545.4,M,46.9,M,,"), good]
        b = self.make(**{"real.gps.mode": "raw"})
        b.start(use_imu=False, use_gps=True, use_motors=False)
        time.sleep(0.3)
        f = b.gps_th.pop_new()
        self.assertIsNotNone(f)
        self.assertEqual(b.gps_th.rejected, 1)
        self.assertEqual(b.gps_th.gga["sats"], 8)
        b.close()


class TestSignalsStopMotors(unittest.TestCase):
    """main.py motors num subprocesso; SIGTERM/SIGHUP/SIGINT devem zerar os motores."""

    def _run(self, sig):
        log = os.path.join(helpers.ROOT, "data", "fake_ard_%d.log" % sig)
        if os.path.exists(log):
            os.remove(log)
        env = dict(os.environ, PYTHONPATH=FAKES, FAKE_ARDUINO_LOG=log)
        p = subprocess.Popen([sys.executable, os.path.join(helpers.ROOT, "main.py"), "motors", "120", "80",
                              "--dur", "20"], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        time.sleep(1.5)
        p.send_signal(sig)
        out = p.communicate(timeout=10)[0].decode()
        with open(log) as f:
            lines = [l for l in f.read().splitlines() if l.startswith("M")]
        os.remove(log)
        return lines, out

    def test_signals(self):
        for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
            lines, out = self._run(sig)
            self.assertIn("M 120 080;", lines, out)
            self.assertEqual(lines[-1], "M 000 000;", out)


if __name__ == "__main__":
    unittest.main()
