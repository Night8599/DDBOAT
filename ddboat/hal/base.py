# -*- coding: utf-8 -*-
"""Interface comum dos backends (barco real, Unity, simulador Python, replay).

Todo backend implementa:
  start()                 abre o hardware/rede
  read() -> Sample        uma leitura (campos ausentes = None)
  set_motors(l, r)        comando dos propulsores (unidades do Arduino)
  stop()                  zera motores
  close()                 libera recursos (sempre chamar no finally)
  clock                   objeto com now() e sleep(dt)
  limits                  (cmd_min, cmd_max) aceitos pelo backend
"""


class Fix(object):
    """Um fix de GPS valido."""
    __slots__ = ("t", "lat", "lon", "sog_kn", "cog_deg", "utc")

    def __init__(self, t, lat, lon, sog_kn=None, cog_deg=None, utc=None):
        self.t = t
        self.lat = lat
        self.lon = lon
        self.sog_kn = sog_kn
        self.cog_deg = cog_deg
        self.utc = utc


class Sample(object):
    """Uma leitura do backend.

    mag, acc, gyr: listas de 3 inteiros CRUS da IMU (unidades do sensor), ou None
    gps: Fix NOVO desde a ultima leitura, ou None
    pose: (X, Y, heading_cru_graus) do simulador Unity, ou None
    """
    __slots__ = ("t", "mag", "acc", "gyr", "gps", "pose")

    def __init__(self, t, mag=None, acc=None, gyr=None, gps=None, pose=None):
        self.t = t
        self.mag = mag
        self.acc = acc
        self.gyr = gyr
        self.gps = gps
        self.pose = pose


def clamp_cmd(v, cmd_min, cmd_max):
    v = int(round(v))
    if v < cmd_min:
        return cmd_min
    if v > cmd_max:
        return cmd_max
    return v


class Backend(object):
    name = "base"
    has_imu = False
    has_gps = False
    has_pose = False

    def start(self):
        pass

    def nav_overrides(self):
        """Calibracoes que o backend impoe ao navegador (so o simulador usa)."""
        return {}

    def read(self):
        raise NotImplementedError

    def set_motors(self, left, right):
        raise NotImplementedError

    def stop(self):
        self.set_motors(0, 0)

    def close(self):
        pass
