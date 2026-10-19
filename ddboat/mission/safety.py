# -*- coding: utf-8 -*-
"""Supervisao de seguranca, avaliada a cada volta do laco.

Acoes (em ordem de prioridade):
  "stop"  arquivo data/STOP, ou RTH pedido sem pontao definido
  "rth"   arquivo data/RTH, saiu da geofence, ou tempo maximo de missao
  "hold"  motores em 0 enquanto falta rumo, ou GPS velho num passo que usa posicao
  None    tudo certo
"""
from ddboat.core.geo import point_in_polygon


class Safety(object):
    def __init__(self, cfg, flags):
        self.cfg = cfg
        self.flags = flags
        self.geofence_ll = cfg.get("safety.geofence_latlon")
        self.home_ll = cfg.get("safety.home_latlon")
        self.gps_timeout = float(cfg.require("safety.gps_timeout_s"))
        self.max_s = cfg.get("safety.max_mission_s")
        self._fence_xy = None
        self.reason = ""

    def fence_xy(self, nav):
        if self.geofence_ll is None or nav.frame is None:
            return None
        if self._fence_xy is None:
            self._fence_xy = [nav.frame.to_xy(p[0], p[1]) for p in self.geofence_ll]
        return self._fence_xy

    def check(self, nav, st, elapsed, step_needs_pos):
        if self.flags is not None and self.flags.stop():
            self.reason = "arquivo STOP"
            return "stop"
        rth = None
        if self.flags is not None and self.flags.rth():
            rth = "arquivo RTH"
        if self.max_s is not None and elapsed > float(self.max_s):
            rth = "tempo maximo (%s s)" % self.max_s
        fence = self.fence_xy(nav)
        if fence is not None and st.pos_valid and not point_in_polygon((st.x, st.y), fence):
            rth = "fora da geofence"
        if rth is not None:
            if self.home_ll is None:
                self.reason = rth + " e home_latlon = null"
                return "stop"
            self.reason = rth
            return "rth"
        if not st.heading_valid:
            self.reason = "sem rumo"
            return "hold"
        if step_needs_pos and (st.gps_age is None or st.gps_age > self.gps_timeout or not st.pos_valid):
            self.reason = "GPS velho/ausente"
            return "hold"
        self.reason = ""
        return None
