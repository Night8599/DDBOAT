# -*- coding: utf-8 -*-
"""Teleoperacao pelo teclado (barco real, Unity ou PySim). Junta o teleop.py
(barco) e o teleop_sim.py (Unity) testados, com log da biblioteca.

  w = frente (V, V)    a = esquerda (-V, V)    d = direita (V, -V)
  s = para             v = para e pede nova V   q = sai (Ctrl+C tambem)

Sem re no barco real (config real.motors.allow_reverse = false), o backend
limita o negativo a 0: 'a' vira (0, V) e 'd' vira (V, 0).
"""

from ddboat.apps.common import LOGS, ensure_dirs, ask_int
from ddboat.core import shutdown
from ddboat.core.clock import Rate
from ddboat.core.config import ConfigError
from ddboat.core.keyboard import KeyReader
from ddboat.estimation.navigator import Navigator
from ddboat.hal import make_backend
from ddboat.io.logger import CsvLogger
from ddboat.io.status import Status
from ddboat.mission.runner import sample_row, nav_row


def cmd_teleop(cfg, args):
    ensure_dirs()
    b = make_backend(args.backend, cfg, realtime=True)
    b.start()
    shutdown.install()
    status = Status(float(cfg.require("loop.status_period_s")))
    logger = None
    try:
        try:
            nav = Navigator(cfg, b.has_pose, b.nav_overrides())
        except (IOError, OSError, ConfigError, ValueError) as e:
            nav = None
            print("AVISO: sem navegacao (%s). So o dado cru sera mostrado." % e)
        lo, hi = b.limits
        print("Limites do backend '%s': %d a %d" % (b.name, lo, hi))
        if lo == 0:
            print("Sem re: 'a' = (0, V) e 'd' = (V, 0).")
        V = ask_int("Velocidade", 0, hi)
        logger = CsvLogger(LOGS, "teleop_" + b.name, {"config": cfg.data, "backend": b.name})
        print("Log: %s" % logger.path)
        print("w=frente a=esquerda d=direita s=para v=velocidade q=sair")
        cmd = (0, 0)
        rate = Rate(float(cfg.require("loop.hz")), b.clock)
        with KeyReader() as kb:
            while True:
                k = kb.read(0.0)
                if k == "w":
                    cmd = (V, V)
                elif k == "a":
                    cmd = (-V, V)
                elif k == "d":
                    cmd = (V, -V)
                elif k == "s":
                    cmd = (0, 0)
                elif k == "q":
                    break
                elif k == "v":
                    cmd = (0, 0)
                    b.set_motors(0, 0)
                    kb.restore()
                    print()
                    V = ask_int("Velocidade", 0, hi)
                    kb.resume()
                s = b.read()
                l, r = b.set_motors(cmd[0], cmd[1])
                row = sample_row(s)
                txt = ""
                if nav is not None:
                    st = nav.update(s)
                    row.update(nav_row(st))
                    if st.theta is not None:
                        txt += "rumo=%5.1f " % st.compass_deg
                    if st.pos_valid:
                        txt += "x=%6.1f y=%6.1f " % (st.x, st.y)
                if s.pose is not None:
                    txt += "X=%.2f Y=%.2f h=%.1f " % s.pose
                if s.gps is not None:
                    txt += "lat=%.6f lon=%.6f " % (s.gps.lat, s.gps.lon)
                row.update(cmd_l=l, cmd_r=r, loop_dt=rate.dt, step="teleop")
                logger.write(row)
                status.show(s.t, "%scmd=(%4d,%4d) V=%3d" % (txt, l, r, V))
                rate.sleep()
    except (shutdown.StopRequested, KeyboardInterrupt):
        pass
    finally:
        try:
            b.close()
        finally:
            if logger is not None:
                logger.close()
    print("\nMotores parados.%s" % ("" if logger is None else " Log: " + logger.path))
