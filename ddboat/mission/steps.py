# -*- coding: utf-8 -*-
"""Passos de missao. Missao = arquivo JSON com uma lista de passos.

Pontos: {"lat": .., "lon": ..} (GPS) ou {"x": .., "y": ..} (ENU local / Unity).
Rumo de entrada em graus de BUSSOLA (0 = N, horario), convertido para theta.

Tipos:
  {"type": "wait", "duration_s": 5}
  {"type": "gyro_bias", "duration_s": 10}            barco PARADO
  {"type": "wait_fix", "timeout_s": 120}
  {"type": "motors", "left": 50, "right": 50, "duration_s": 2}
  {"type": "heading", "compass_deg": 90, "thrust": 120, "duration_s": 20}
  {"type": "goto", "point": P, "radius_m": 3, "thrust": 120}
  {"type": "line", "a": P, "b": P, "thrust": 120}
  {"type": "polygon", "points": [P, ...], "laps": 1, "thrust": 120}
  {"type": "home", "radius_m": 3, "thrust": 120}      usa safety.home_latlon
"""

from ddboat.core.angles import compass_to_theta
from ddboat.core.geo import distance
from ddboat.guidance import core as gd
from ddboat.calib import imu as imucal


class MissionError(Exception):
    pass


class Cmd(object):
    __slots__ = ("kind", "theta_ref", "thrust", "left", "right", "info")

    def __init__(self, kind, theta_ref=None, thrust=0.0, left=0, right=0, info=""):
        self.kind = kind  # "stop", "heading", "raw"
        self.theta_ref = theta_ref
        self.thrust = thrust
        self.left = left
        self.right = right
        self.info = info


STOP = Cmd("stop")


def _req(d, k):
    if k not in d:
        raise MissionError("passo %r sem o campo '%s'" % (d.get("type"), k))
    return d[k]


class Context(object):
    """O que os passos enxergam."""

    def __init__(self, cfg, nav):
        self.cfg = cfg
        self.nav = nav
        self.sample = None
        g = lambda k: cfg.require("guidance." + k)
        self.g_mode = g("mode")
        self.g_r = float(g("field_r_m"))
        self.g_gamma = float(g("field_gamma_rad"))
        self.g_L = float(g("lookahead_m"))
        self.g_margin = float(g("pass_margin"))
        self.g_slow = float(g("slow_radius_m"))
        self.g_tmin = float(g("slow_thrust_min"))

    def point(self, p):
        try:
            return self._point(p)
        except (TypeError, ValueError):
            raise MissionError("ponto invalido (preencha as coordenadas): %r" % (p,))

    def _point(self, p):
        if "x" in p and "y" in p:
            return float(p["x"]), float(p["y"])
        if "lat" in p and "lon" in p:
            if self.nav.frame is None:
                raise MissionError("ponto em lat/lon, mas a origem ENU ainda nao existe "
                                   "(defina nav.origin_latlon ou use wait_fix antes)")
            return self.nav.frame.to_xy(float(p["lat"]), float(p["lon"]))
        raise MissionError("ponto invalido: %r" % (p,))

    def line_theta(self, a, b, m):
        if self.g_mode == "field":
            return gd.line_follow_theta(a, b, m, self.g_r, self.g_gamma)
        if self.g_mode == "lookahead":
            return gd.lookahead_theta(a, b, m, self.g_L)
        raise MissionError("guidance.mode invalido: %r" % self.g_mode)


class Step(object):
    needs_position = False

    def __init__(self, d):
        self.d = d
        self.t0 = None

    def start(self, ctx, t):
        self.t0 = t

    def update(self, ctx, st, t):
        """Retorna Cmd, ou None quando o passo terminou."""
        raise NotImplementedError

    def describe(self):
        return self.d.get("type")


class Wait(Step):
    def update(self, ctx, st, t):
        return None if t - self.t0 >= float(_req(self.d, "duration_s")) else STOP


class Motors(Step):
    def update(self, ctx, st, t):
        if t - self.t0 >= float(_req(self.d, "duration_s")):
            return None
        return Cmd("raw", left=int(_req(self.d, "left")), right=int(_req(self.d, "right")))


class GyroBias(Step):
    def start(self, ctx, t):
        Step.start(self, ctx, t)
        self.gyr = []
        self.acc = []

    def update(self, ctx, st, t):
        s = ctx.sample
        if s is not None and s.gyr is not None:
            self.gyr.append(s.gyr)
            self.acc.append(s.acc)
        if t - self.t0 < float(_req(self.d, "duration_s")):
            return STOP
        if not self.gyr:
            raise MissionError("gyro_bias: backend sem giroscopio")
        cal = imucal.gyro_still(self.gyr, self.acc,
                                float(ctx.cfg.require("calib.gyro_still_max_std_lsb")))
        ctx.nav.set_gyro_cal(cal)
        self.result = cal
        return None


class WaitFix(Step):
    def update(self, ctx, st, t):
        if st.pos_valid:
            return None
        if t - self.t0 > float(_req(self.d, "timeout_s")):
            raise MissionError("wait_fix: sem GPS apos %s s" % self.d["timeout_s"])
        return STOP


class Heading(Step):
    def update(self, ctx, st, t):
        if t - self.t0 >= float(_req(self.d, "duration_s")):
            return None
        th = compass_to_theta(float(_req(self.d, "compass_deg")))
        return Cmd("heading", th, float(_req(self.d, "thrust")))


class Goto(Step):
    needs_position = True

    def __init__(self, d, target=None):
        Step.__init__(self, d)
        self.target = target

    def start(self, ctx, t):
        Step.start(self, ctx, t)
        if self.target is None:
            self.target = ctx.point(_req(self.d, "point"))

    def update(self, ctx, st, t):
        m = (st.x, st.y)
        dist = distance(m, self.target)
        if dist <= float(_req(self.d, "radius_m")):
            return None
        thrust = gd.approach_thrust(dist, float(_req(self.d, "thrust")), ctx.g_slow, ctx.g_tmin)
        return Cmd("heading", gd.goto_theta(m, self.target), thrust, info="d=%.1f" % dist)


class Line(Step):
    needs_position = True

    def start(self, ctx, t):
        Step.start(self, ctx, t)
        self.a = ctx.point(_req(self.d, "a"))
        self.b = ctx.point(_req(self.d, "b"))

    def update(self, ctx, st, t):
        m = (st.x, st.y)
        if gd.passed_b(self.a, self.b, m, ctx.g_margin):
            return None
        th, e = ctx.line_theta(self.a, self.b, m)
        return Cmd("heading", th, float(_req(self.d, "thrust")), info="e=%.1f" % e)


class Polygon(Step):
    needs_position = True

    def start(self, ctx, t):
        Step.start(self, ctx, t)
        pts = [ctx.point(p) for p in _req(self.d, "points")]
        if len(pts) < 2:
            raise MissionError("polygon precisa de >= 2 pontos")
        laps = int(_req(self.d, "laps"))
        segs = []
        for _ in range(laps):
            for i in range(len(pts)):
                segs.append((pts[i], pts[(i + 1) % len(pts)]))
        self.segs = segs
        self.k = 0

    def update(self, ctx, st, t):
        m = (st.x, st.y)
        while self.k < len(self.segs) and gd.passed_b(self.segs[self.k][0], self.segs[self.k][1], m, ctx.g_margin):
            self.k += 1
        if self.k >= len(self.segs):
            return None
        a, b = self.segs[self.k]
        th, e = ctx.line_theta(a, b, m)
        return Cmd("heading", th, float(_req(self.d, "thrust")),
                   info="seg=%d/%d e=%.1f" % (self.k + 1, len(self.segs), e))


def home_xy(ctx):
    h = ctx.cfg.get("safety.home_latlon")
    if h is None:
        raise MissionError("safety.home_latlon = null: retorno ao pontao indisponivel")
    return ctx.point({"lat": h[0], "lon": h[1]})


class Home(Goto):
    def start(self, ctx, t):
        self.target = home_xy(ctx)
        Goto.start(self, ctx, t)


TYPES = {"wait": Wait, "motors": Motors, "gyro_bias": GyroBias, "wait_fix": WaitFix,
         "heading": Heading, "goto": Goto, "line": Line, "polygon": Polygon, "home": Home}


def build(step_dicts):
    steps = []
    for i, d in enumerate(step_dicts):
        t = d.get("type")
        if t not in TYPES:
            raise MissionError("passo %d: tipo desconhecido %r" % (i, t))
        steps.append(TYPES[t](d))
    return steps
