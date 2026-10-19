# -*- coding: utf-8 -*-
"""Fabrica de backends."""


def make_backend(name, cfg, realtime=False):
    if name == "real":
        from ddboat.hal.real import RealBoat
        return RealBoat(cfg)
    if name == "unity":
        from ddboat.hal.unity import UnitySim
        return UnitySim(cfg)
    if name == "pysim":
        from ddboat.hal.pysim import PySim
        return PySim(cfg, realtime=realtime)
    raise ValueError("backend desconhecido: %r (use real, unity ou pysim)" % name)
