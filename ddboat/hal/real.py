# -*- coding: utf-8 -*-
"""Backend do barco real, em cima dos drivers da ENSTA (sem edita-los).

Chamadas usadas = as que ja funcionaram no barco 04:
  Imu9IO().read_mag_raw() / read_accel_raw() / read_gyro_raw()
  GpsIO().set_filter_speed("0") e read_rmc_non_blocking() em thread
  ArduinoIO() (pela /dev/ttyV0) e send_arduino_cmd_motor(l, r)

Opcoes ainda NAO testadas no barco ficam desligadas no config:
  real.imu.bdu = true    -> liga Block Data Update (provavel causa dos picos de 256)
  real.gps.mode = "raw"  -> le as frases cruas e checa checksum e status 'A'
"""
import os
import sys
import threading
import time
import types

from ddboat.core.clock import RealClock
from ddboat.hal.base import Backend, Fix, Sample, clamp_cmd
from ddboat.hal import nmea


def load_drivers(drivers_dir):
    """Importa os drivers da pasta indicada.

    O imu9_driver_v2.py faz 'import drivers.i2creal', mas no repositorio o
    i2creal.py fica solto na mesma pasta. Se esse import nao existir, criamos
    o pacote 'drivers' em memoria apontando para o i2creal da pasta.
    """
    drivers_dir = os.path.abspath(drivers_dir)
    if not os.path.isdir(drivers_dir):
        raise IOError("pasta dos drivers nao existe: %s" % drivers_dir)
    if drivers_dir not in sys.path:
        sys.path.insert(0, drivers_dir)
    try:
        import drivers.i2creal  # noqa: F401  (clone original da ENSTA)
    except ImportError:
        import i2creal
        pkg = types.ModuleType("drivers")
        pkg.__path__ = []
        pkg.i2creal = i2creal
        sys.modules["drivers"] = pkg
        sys.modules["drivers.i2creal"] = i2creal
    import imu9_driver_v2
    import gps_driver_v2
    import arduino_driver_v2
    return imu9_driver_v2, gps_driver_v2, arduino_driver_v2


def _median3axis(samples):
    n = len(samples)
    return [sorted(s[k] for s in samples)[n // 2] for k in range(3)]


class GpsThread(threading.Thread):
    """Le o GPS em paralelo (leitura serial nao trava o laco de controle)."""

    def __init__(self, gps, mode, clock):
        threading.Thread.__init__(self)
        self.daemon = True
        self.gps = gps
        self.mode = mode
        self.clock = clock
        self.lock = threading.Lock()
        self.new = None
        self.last = None
        self.gga = None
        self.running = True
        self.rejected = 0

    def run(self):
        while self.running:
            try:
                fix = self._read_one()
                if fix is not None:
                    with self.lock:
                        self.new = fix
                        self.last = fix
            except Exception:
                self.rejected += 1
            time.sleep(0.01)

    def _read_one(self):
        if self.mode == "driver":
            ok, d = self.gps.read_rmc_non_blocking()
            if not ok:
                return None
            # d = [lat ddmm, N/S, lon dddmm, E/W, hora, sog, cog]
            # Sem fix o driver deixa 0.0 nos campos vazios (foi o caso do
            # log_001: lat 0, lon -0). Rejeitamos esse caso.
            if float(d[0]) == 0.0 and float(d[2]) == 0.0:
                self.rejected += 1
                return None
            from ddboat.core.geo import ddmm_to_deg
            return Fix(self.clock.now(), ddmm_to_deg(d[0], d[1]), ddmm_to_deg(d[2], d[3]),
                       float(d[5]), float(d[6]), str(d[4]))
        ok, line = self.gps.read_non_blocking()
        if not ok:
            return None
        g = nmea.parse_gga(line)
        if g is not None:
            with self.lock:
                self.gga = g
            return None
        r = nmea.parse_rmc(line)
        if r is None:
            if line[3:6] == "RMC":
                self.rejected += 1
            return None
        return Fix(self.clock.now(), r["lat"], r["lon"], r["sog_kn"], r["cog_deg"], r["utc"])

    def pop_new(self):
        with self.lock:
            f = self.new
            self.new = None
            return f


class RealBoat(Backend):
    name = "real"
    has_imu = True
    has_gps = True

    def __init__(self, cfg):
        self.cfg = cfg
        self.clock = RealClock()
        cmd_max = int(cfg.require("real.motors.cmd_max"))
        allow_rev = bool(cfg.require("real.motors.allow_reverse"))
        self.limits = (-cmd_max if allow_rev else 0, cmd_max)
        self.median_n = int(cfg.require("real.imu.median_n"))
        self.median_sleep = float(cfg.require("real.imu.median_sleep_s"))
        self.watchdog_s = float(cfg.require("real.motors.watchdog_s"))
        self.imu = None
        self.gps_th = None
        self.ard = None
        self._lock = threading.Lock()
        self._last_cmd_t = None
        self._last_cmd = (0, 0)
        self._wd = None
        self.watchdog_trips = 0

    # ------------------------------------------------------------------ start
    def start(self, use_imu=True, use_gps=True, use_motors=True):
        imu_mod, gps_mod, ard_mod = load_drivers(self.cfg.path_of("real.drivers_dir"))
        if use_imu:
            self.imu = imu_mod.Imu9IO()
            if self.cfg.require("real.imu.bdu"):
                # Block Data Update: LSM6DS33 CTRL3_C = BDU | IF_INC = 0x44
                # LIS3MDL CTRL_REG5 = BDU = 0x40. EXPERIMENTAL (nao testado no barco).
                self.imu._Imu9IO__dev_i2c_ag.write(0x12, [0x44])
                self.imu._Imu9IO__dev_i2c_mg.write(0x24, [0x40])
        else:
            self.has_imu = False
        if use_gps:
            try:
                gps = gps_mod.GpsIO(int(self.cfg.require("real.gps.tty_dev")))
                gps.set_filter_speed(str(self.cfg.require("real.gps.filter_speed")))
                self.gps_th = GpsThread(gps, self.cfg.require("real.gps.mode"), self.clock)
                self.gps_th.start()
            except Exception as e:
                print("AVISO: GPS nao inicializado (%r)" % (e,))
                self.has_gps = False
        else:
            self.has_gps = False
        if use_motors:
            self.ard = ard_mod.ArduinoIO()
            if self.ard.arduino is None:
                raise IOError("Arduino nao inicializado")
            self.ard.send_arduino_cmd_motor(0, 0)
            self._last_cmd_t = self.clock.now()
            if self.watchdog_s > 0:
                self._wd = threading.Thread(target=self._watchdog)
                self._wd.daemon = True
                self._wd.start()

    # ------------------------------------------------------------------ IMU/GPS
    def read_imu(self):
        s = []
        for i in range(self.median_n):
            s.append((self.imu.read_mag_raw(), self.imu.read_accel_raw(), self.imu.read_gyro_raw()))
            if i < self.median_n - 1 and self.median_sleep > 0:
                time.sleep(self.median_sleep)
        if self.median_n == 1:
            return list(s[0][0]), list(s[0][1]), list(s[0][2])
        return (_median3axis([x[0] for x in s]), _median3axis([x[1] for x in s]),
                _median3axis([x[2] for x in s]))

    def read(self):
        mag = acc = gyr = None
        if self.imu is not None:
            mag, acc, gyr = self.read_imu()
        fix = self.gps_th.pop_new() if self.gps_th is not None else None
        return Sample(self.clock.now(), mag, acc, gyr, fix)

    # ------------------------------------------------------------------ motores
    def set_motors(self, left, right):
        if self.ard is None:
            raise IOError("motores nao inicializados")
        l = clamp_cmd(left, self.limits[0], self.limits[1])
        r = clamp_cmd(right, self.limits[0], self.limits[1])
        with self._lock:
            self.ard.send_arduino_cmd_motor(l, r)
            self._last_cmd = (l, r)
            self._last_cmd_t = self.clock.now()
        return l, r

    def _watchdog(self):
        # Protege contra o laco principal travado (ex.: print bloqueado com o
        # Wi-Fi caindo). NAO protege contra o processo morto: isso depende do
        # firmware do Arduino (teste pendente).
        while self.ard is not None:
            time.sleep(self.watchdog_s / 4.0)
            with self._lock:
                if (self._last_cmd != (0, 0) and self._last_cmd_t is not None
                        and self.clock.now() - self._last_cmd_t > self.watchdog_s):
                    try:
                        self.ard.send_arduino_cmd_motor(0, 0)
                    except Exception:
                        pass
                    self._last_cmd = (0, 0)
                    self.watchdog_trips += 1

    def stop(self):
        if self.ard is not None:
            with self._lock:
                self.ard.send_arduino_cmd_motor(0, 0)
                self._last_cmd = (0, 0)
                self._last_cmd_t = self.clock.now()

    def close(self):
        try:
            self.stop()
        finally:
            if self.gps_th is not None:
                self.gps_th.running = False
            self.ard = None
