# -*- coding: utf-8 -*-
"""Laco de missao: backend -> navegacao -> seguranca -> passo -> controle -> motores -> log."""
import math
import os

from ddboat.core import shutdown
from ddboat.core.clock import Rate
from ddboat.core.config import REPO_ROOT
from ddboat.control.heading import HeadingController
from ddboat.control.mixer import Mixer
from ddboat.estimation.navigator import Navigator
from ddboat.io.logger import CsvLogger
from ddboat.io.status import Status
from ddboat.io.telemetry import Telemetry
from ddboat.mission import steps as S
from ddboat.mission.safety import Safety

NEEDS_HEADING = (S.Heading, S.Goto, S.Line, S.Polygon, S.Home)


def make_mixer(cfg, backend):
    m = lambda k: cfg.require("control.mixer." + k)
    return Mixer(backend.limits[0], backend.limits[1], int(cfg.require(backend.name + ".turn_sign")),
                 float(m("gain_left")), float(m("gain_right")), float(m("deadzone")))


def sample_row(s):
    r = {"t": s.t}
    if s.mag is not None:
        r.update(mag_x=s.mag[0], mag_y=s.mag[1], mag_z=s.mag[2])
    if s.acc is not None:
        r.update(acc_x=s.acc[0], acc_y=s.acc[1], acc_z=s.acc[2])
    if s.gyr is not None:
        r.update(gyr_x=s.gyr[0], gyr_y=s.gyr[1], gyr_z=s.gyr[2])
    if s.gps is not None:
        f = s.gps
        r.update(gps_new=1, lat="%.8f" % f.lat, lon="%.8f" % f.lon, sog_kn=f.sog_kn, cog_deg=f.cog_deg)
    if s.pose is not None:
        r.update(pose_X=s.pose[0], pose_Y=s.pose[1], pose_h=s.pose[2])
    return r


def nav_row(st):
    deg = lambda a: None if a is None else math.degrees(a)
    return {"x": st.x, "y": st.y, "vx": st.vx, "vy": st.vy, "theta_deg": deg(st.theta),
            "compass_deg": st.compass_deg, "theta_mag_deg": deg(st.theta_mag), "r_dps": deg(st.r)}


def run_mission(cfg, backend, mission, tag="mission", quiet=False, allow_no_geofence=False,
                max_ticks=None, on_tick=None):
    """Executa a missao. Retorna dict com o resumo. on_tick(backend, st) e para testes."""
    if backend.name == "real" and cfg.get("safety.geofence_latlon") is None and not allow_no_geofence:
        raise S.MissionError("safety.geofence_latlon = null. Defina a geofence ou rode com "
                             "--sem-geofence (so em teste segurando o barco)")
    steps = S.build(mission["steps"])
    data_dir = os.path.join(REPO_ROOT, "data")
    status = Status(float(cfg.require("loop.status_period_s")), quiet)
    summary = {"ok": False, "reason": "", "steps_done": 0}
    logger = None
    tel = None
    backend.start()
    shutdown.install()  # depois do ArduinoIO, que instala o proprio SIGINT
    try:
        nav = Navigator(cfg, backend.has_pose, backend.nav_overrides())
        ctx = S.Context(cfg, nav)
        mixer = make_mixer(cfg, backend)
        hc = HeadingController.from_config(cfg)
        flags = shutdown.FileFlags(data_dir)
        safety = Safety(cfg, flags)
        tel = Telemetry.from_config(cfg)
        meta = {"config": cfg.data, "mission": mission, "backend": backend.name}
        logger = CsvLogger(os.path.join(data_dir, "logs"), tag, meta)
        status.line("Log: %s" % logger.path)
        for w in nav.warnings:
            status.line("AVISO: " + w)
        rate = Rate(float(cfg.require("loop.hz")), backend.clock)
        t_start = backend.clock.now()
        k = 0
        cur = steps[0]
        cur.start(ctx, t_start)
        status.line("Passo 1/%d: %s" % (len(steps), cur.describe()))
        rth = None
        ticks = 0
        while True:
            s = backend.read()
            ctx.sample = s
            st = nav.update(s)
            t = s.t
            action = safety.check(nav, st, t - t_start, cur.needs_position)
            if action == "stop":
                summary["reason"] = "seguranca: " + safety.reason
                break
            if action == "rth" and rth is None:
                rth = S.Home({"type": "home", "radius_m": cfg.require("safety.rth_radius_m"),
                              "thrust": cfg.require("safety.rth_thrust")})
                rth.start(ctx, t)
                cur = rth
                status.line("RETORNO AO PONTAO: " + safety.reason)
                action = safety.check(nav, st, t - t_start, cur.needs_position)
                if action in ("stop", "rth"):
                    action = None if action == "rth" else action
            if action == "hold" and (cur.needs_position or isinstance(cur, NEEDS_HEADING)):
                cmd = S.STOP
            else:
                cmd = cur.update(ctx, st, t)
                if action == "hold" and cmd is not None:
                    cmd = S.STOP
            if cmd is None:
                summary["steps_done"] += 1
                if cur is rth:
                    summary.update(ok=True, reason="chegou ao pontao (RTH)")
                    break
                k += 1
                if k >= len(steps):
                    summary.update(ok=True, reason="missao concluida")
                    break
                cur = steps[k]
                cur.start(ctx, t)
                hc.reset()
                status.line("Passo %d/%d: %s" % (k + 1, len(steps), cur.describe()))
                cmd = S.STOP
            turn = None
            if cmd.kind == "stop":
                l, r = 0, 0
                hc.reset()
            elif cmd.kind == "raw":
                l, r = cmd.left, cmd.right
            else:
                turn = hc.update(cmd.theta_ref, st.theta, rate.dt, st.r)
                l, r = mixer.mix(cmd.thrust, turn)
            l, r = backend.set_motors(l, r)
            row = sample_row(s)
            row.update(nav_row(st))
            row.update(step=type(cur).__name__, safety=action or "", thrust=cmd.thrust,
                       turn=turn, cmd_l=l, cmd_r=r, loop_dt=rate.dt, info=cmd.info,
                       theta_ref_deg=None if cmd.theta_ref is None else math.degrees(cmd.theta_ref))
            logger.write(row)
            if tel is not None:
                tel.maybe_send(t, {"t": t, "x": st.x, "y": st.y, "lat": st.lat, "lon": st.lon,
                                   "compass": st.compass_deg, "step": type(cur).__name__,
                                   "safety": action, "cmd": [l, r], "info": cmd.info})
            comp = "---" if st.theta is None else "%5.1f" % st.compass_deg
            pos = "x=%7.1f y=%7.1f" % (st.x, st.y) if st.pos_valid else "sem posicao"
            status.show(t, "%-8s rumo=%s %s cmd=(%4d,%4d) %s %s" % (
                type(cur).__name__, comp, pos, l, r, cmd.info, action or ""))
            if on_tick is not None:
                on_tick(backend, st)
            ticks += 1
            if max_ticks is not None and ticks >= max_ticks:
                summary["reason"] = "max_ticks"
                break
            rate.sleep()
    except (shutdown.StopRequested, KeyboardInterrupt) as e:
        summary["reason"] = "interrompido (%s)" % e
    except S.MissionError as e:
        summary["reason"] = "erro de missao: %s" % e
    finally:
        try:
            backend.close()
        finally:
            if logger is not None:
                logger.close()
                summary["log"] = logger.path
            if tel is not None:
                tel.close()
    status.line("Motores parados. %s" % summary["reason"])
    return summary
