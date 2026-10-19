# -*- coding: utf-8 -*-
"""Navegacao: transforma leituras cruas (Sample) no estado usado pelo controle.

Saida (NavState): x, y (ENU, m), theta (rad, anti-horario a partir do leste),
r (taxa de guinada, rad/s, ou None), flags de validade e diagnosticos.

Fontes de rumo (config nav.heading_source):
  "mag2d": calibracao 2D do cap_nord (metodo testado no barco)
  "mag3d": elipsoide 3D + rotacao N/O, com compensacao de inclinacao opcional
  "pose":  pose do Unity, convertida pela convencao medida (unity_conv)
Fusao com o giro (nav.gyro_fusion): "none", "complementary" ou "kalman".
So funciona depois da calibracao do giro parado (set_gyro_cal); sem ela,
cai para "none" e avisa uma vez.
"""
import math

from ddboat.core.angles import wrap_pi, compass_to_theta, theta_to_compass
from ddboat.core.geo import LocalFrame
from ddboat.core import linalg as la
from ddboat.calib import mag2d, mag3d, imu as imucal
from ddboat.calib.unity_conv import theta_from_heading
from ddboat.estimation.attitude import heading_tilt_compensated
from ddboat.estimation.kalman import HeadingKF, PositionKF


class NavState(object):
    __slots__ = ("t", "x", "y", "theta", "theta_mag", "r", "vx", "vy",
                 "pos_valid", "heading_valid", "gps_age", "fix", "lat", "lon", "warnings")

    def __init__(self):
        for k in self.__slots__:
            setattr(self, k, None)
        self.pos_valid = False
        self.heading_valid = False
        self.warnings = []

    @property
    def compass_deg(self):
        return None if self.theta is None else theta_to_compass(self.theta)


class Navigator(object):
    def __init__(self, cfg, backend_has_pose=False, overrides=None):
        overrides = overrides or {}
        n = lambda k: cfg.get("nav." + k)
        self.cfg = cfg
        self.source = "pose" if backend_has_pose else n("heading_source")
        self.fusion = n("gyro_fusion")
        decl = n("declination_deg")
        self.decl = math.radians(decl) if decl is not None else 0.0
        self.warnings = []
        if decl is None and self.source != "pose":
            self.warnings.append("nav.declination_deg = null: usando 0")
        if self.source == "mag2d":
            if "mag2d" in overrides:  # simulador: usa a calibracao que ele mesmo gerou
                self.c2 = overrides["mag2d"]
            else:
                self.c2 = mag2d.load(cfg.path_of("nav.mag2d_file"))
        elif self.source == "mag3d":
            self.c3 = mag3d.load(cfg.path_of("nav.mag3d_file"))
            self.tilt = bool(n("tilt_compensation"))
            self.axes = imucal.load(cfg.path_of("nav.axes_file"))["R_imu_to_boat"] if self.tilt else None
        elif self.source == "pose":
            self.conv = imucal.load(cfg.path_of("nav.unity_conv_file"))
        else:
            raise ValueError("nav.heading_source invalido: %r" % self.source)
        self.k_comp = float(n("complementary_gain"))
        self.kcfg = n("heading_kf")
        self.pcfg = n("position_kf")
        self.use_pos_kf = bool(n("position_filter_kalman"))
        org = n("origin_latlon")
        self.frame = LocalFrame(org[0], org[1]) if org is not None else None
        self.gyro_cal = None
        self.theta_hat = None
        self.hkf = None
        self.pkf = None
        self._pkf_reject_run = 0
        self.last_t = None
        self.last_fix_t = None
        self.state = NavState()
        self._warned_fusion = False

    # ------------------------------------------------------------ giro
    def set_gyro_cal(self, cal):
        self.gyro_cal = cal
        self.theta_hat = None
        self.hkf = None

    # ------------------------------------------------------------ rumo
    def _theta_mag(self, s):
        if self.source == "mag2d":
            comp = mag2d.heading_compass_deg(s.mag[0], s.mag[1], self.c2)
            return wrap_pi(compass_to_theta(comp) - self.decl)
        y = mag3d.apply(self.c3, s.mag)
        if self.tilt:
            acc_b = la.matvec(self.axes, s.acc)
            psi = heading_tilt_compensated(acc_b, y)
        else:
            psi = -math.atan2(y[1], y[0])
        # psi: anti-horario a partir do norte magnetico; theta: a partir do leste
        return wrap_pi(psi + math.pi / 2.0 - self.decl)

    def update(self, s):
        st = self.state
        dt = 0.0 if self.last_t is None else max(0.0, s.t - self.last_t)
        self.last_t = s.t
        st.t = s.t
        st.warnings = list(self.warnings)

        # ---------------- pose do Unity
        if self.source == "pose":
            if s.pose is not None:
                st.x, st.y = s.pose[0], s.pose[1]
                st.theta = theta_from_heading(s.pose[2], self.conv)
                st.theta_mag = st.theta
                st.pos_valid = st.heading_valid = True
                self.last_fix_t = s.t
            st.gps_age = None if self.last_fix_t is None else s.t - self.last_fix_t
            return st

        # ---------------- rumo
        if s.mag is not None:
            th_m = self._theta_mag(s)
            st.theta_mag = th_m
            r = imucal.yaw_rate(s.gyr, self.gyro_cal) if (self.gyro_cal and s.gyr) else None
            st.r = r
            mode = self.fusion
            if mode != "none" and r is None:
                if not self._warned_fusion:
                    self.warnings.append("fusao com giro pedida, mas giro nao calibrado: usando so o mag")
                    self._warned_fusion = True
                mode = "none"
            if mode == "none":
                st.theta = th_m
            elif mode == "complementary":
                if self.theta_hat is None:
                    self.theta_hat = th_m
                else:
                    self.theta_hat = wrap_pi(self.theta_hat + dt * r)
                    self.theta_hat = wrap_pi(self.theta_hat + self.k_comp * wrap_pi(th_m - self.theta_hat))
                st.theta = self.theta_hat
            elif mode == "kalman":
                k = self.kcfg
                if self.hkf is None:
                    self.hkf = HeadingKF(th_m, 0.0, k["sigma_theta0_rad"], k["sigma_bias0_rad_s"],
                                         k["q_rate"], k["q_bias"], k["r_mag_rad"])
                else:
                    self.hkf.predict(r, dt)
                    self.hkf.update(th_m)
                st.theta = self.hkf.theta
            else:
                raise ValueError("nav.gyro_fusion invalido: %r" % mode)
            st.heading_valid = True

        # ---------------- posicao
        if self.pkf is not None and dt > 0:
            self.pkf.predict(dt)
        if s.gps is not None:
            f = s.gps
            if self.frame is None:
                self.frame = LocalFrame(f.lat, f.lon)
                self.warnings.append("origem ENU = primeiro fix (%.7f, %.7f)" % (f.lat, f.lon))
            gx, gy = self.frame.to_xy(f.lat, f.lon)
            st.lat, st.lon, st.fix = f.lat, f.lon, f
            self.last_fix_t = s.t
            if self.use_pos_kf:
                p = self.pcfg
                if self.pkf is None:
                    self.pkf = PositionKF(gx, gy, p["sigma_pos0_m"], p["sigma_vel0_mps"],
                                          p["q_acc"], p["r_gps_m"])
                elif not self.pkf.update(gx, gy):
                    self._pkf_reject_run += 1
                    if self._pkf_reject_run >= int(p["reset_after_rejections"]):
                        self.pkf = PositionKF(gx, gy, p["sigma_pos0_m"], p["sigma_vel0_mps"],
                                              p["q_acc"], p["r_gps_m"])
                        self._pkf_reject_run = 0
                else:
                    self._pkf_reject_run = 0
            else:
                st.x, st.y = gx, gy
                st.pos_valid = True
        if self.pkf is not None:
            st.x, st.y, st.vx, st.vy = self.pkf.state
            st.pos_valid = True
        st.gps_age = None if self.last_fix_t is None else s.t - self.last_fix_t
        return st
