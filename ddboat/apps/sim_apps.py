# -*- coding: utf-8 -*-
"""Comandos de simulacao/missao: unity-conv, run, nav-replay."""
import json
import os
import time

from ddboat.apps.common import LOGS, ensure_dirs, ask_int
from ddboat.calib import unity_conv
from ddboat.core import shutdown
from ddboat.hal import make_backend


def cmd_unity_conv(cfg, args):
    """Mede a convencao do heading do Unity: reta 2 s, giro (-V, V) 1 s, reta 2 s."""
    ensure_dirs()
    b = make_backend("unity", cfg)
    b.start()
    shutdown.install()

    def run_for(cmd, dur):
        t0 = time.time()
        first = None
        while time.time() - t0 < dur:
            b.read()
            if first is None and b.last_pose is not None:
                first = b.last_pose
            b.set_motors(cmd[0], cmd[1])
            time.sleep(0.05)
        b.read()
        return first, b.last_pose

    try:
        V = ask_int("Velocidade para a manobra", 1, b.limits[1])
        t0 = time.time()
        while b.last_pose is None and time.time() - t0 < 3.0:
            b.read()
            time.sleep(0.05)
        if b.last_pose is None:
            print("ERRO: nenhuma pose do robo %r chegou em 3 s." % b.robot_id)
            return
        leg1 = run_for((V, V), args.fwd_s)
        turn = run_for((-V, V), args.turn_s)
        leg2 = run_for((V, V), args.fwd_s)
        b.stop()
        conv = unity_conv.from_two_legs(leg1, leg2, turn[0][2], turn[1][2],
                                        args.min_move, args.min_turn_deg)
        p = cfg.path_of("nav.unity_conv_file")
        with open(p, "w") as f:
            json.dump(conv, f, indent=2)
        print("Convencao: %r  -> salvo em %s" % (conv, p))
        if conv["turn_sign"] != int(cfg.require("unity.turn_sign")):
            print("ATENCAO: (-V, V) gira no sentido %s no plano X-Y. Ajuste unity.turn_sign = %d no config."
                  % ("anti-horario" if conv["turn_sign"] > 0 else "horario", conv["turn_sign"]))
    except unity_conv.CalibError as e:
        print("ERRO: %s" % e)
    except (shutdown.StopRequested, KeyboardInterrupt):
        print("Interrompido.")
    finally:
        b.close()


def cmd_run(cfg, args):
    from ddboat.mission.runner import run_mission
    with open(args.mission) as f:
        mission = json.load(f)
    b = make_backend(args.backend, cfg, realtime=args.realtime)
    tag = os.path.splitext(os.path.basename(args.mission))[0] + "_" + b.name
    res = run_mission(cfg, b, mission, tag=tag, quiet=args.quiet,
                      allow_no_geofence=args.sem_geofence)
    print(json.dumps(res, indent=2))
    return 0 if res.get("ok") else 1


def cmd_nav_replay(cfg, args):
    """Reprocessa um log (com colunas cruas) pelo navegador com o config atual.
    Serve para comparar fusoes/filtros offline com dado real."""
    from ddboat.io.logger import read_csv, CsvLogger
    from ddboat.hal.base import Sample, Fix
    from ddboat.estimation.navigator import Navigator
    from ddboat.calib import imu as imucal
    from ddboat.mission.runner import sample_row, nav_row
    cols, rows = read_csv(args.log)
    need = ["t", "mag_x", "mag_y", "mag_z", "acc_x", "acc_y", "acc_z", "gyr_x", "gyr_y", "gyr_z"]
    miss = [c for c in need if c not in cols]
    if miss:
        print("ERRO: log sem as colunas %s (os logs antigos do teleop nao servem)" % miss)
        return 1
    nav = Navigator(cfg)
    if args.gyro_still_s > 0:
        t0 = rows[0]["t"]
        g = [[r["gyr_x"], r["gyr_y"], r["gyr_z"]] for r in rows if r["t"] - t0 < args.gyro_still_s]
        a = [[r["acc_x"], r["acc_y"], r["acc_z"]] for r in rows if r["t"] - t0 < args.gyro_still_s]
        nav.set_gyro_cal(imucal.gyro_still(g, a, float(cfg.require("calib.gyro_still_max_std_lsb"))))
    out = CsvLogger(LOGS, "replay", {"source": os.path.abspath(args.log), "config": cfg.data})
    for r in rows:
        fix = None
        if r.get("gps_new") == 1 and r.get("lat") is not None:
            fix = Fix(r["t"], r["lat"], r["lon"], r.get("sog_kn"), r.get("cog_deg"))
        s = Sample(r["t"], [r["mag_x"], r["mag_y"], r["mag_z"]], [r["acc_x"], r["acc_y"], r["acc_z"]],
                   [r["gyr_x"], r["gyr_y"], r["gyr_z"]], fix)
        st = nav.update(s)
        row = sample_row(s)
        row.update(nav_row(st))
        out.write(row)
    out.close()
    print("Replay salvo em %s" % out.path)
