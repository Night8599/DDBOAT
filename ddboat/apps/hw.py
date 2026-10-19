# -*- coding: utf-8 -*-
"""Comandos de hardware: motors, selftest, imu-raw, gps, heading."""
import math
import os
import time

from ddboat.apps.common import LOGS, ensure_dirs
from ddboat.core import shutdown
from ddboat.hal.real import RealBoat


def cmd_motors(cfg, args):
    """Equivalente ao comPropulseurs.py: aplica (esq, dir) por N s e para."""
    b = RealBoat(cfg)
    b.start(use_imu=False, use_gps=False, use_motors=True)
    shutdown.install()
    try:
        l, r = b.set_motors(args.left, args.right)
        print("Comando enviado (apos limites): (%d, %d) por %.1f s" % (l, r, args.dur))
        t0 = time.time()
        while time.time() - t0 < args.dur:
            b.set_motors(args.left, args.right)  # reenvia (watchdog)
            time.sleep(0.1)
    except (shutdown.StopRequested, KeyboardInterrupt):
        print("Interrompido.")
    finally:
        b.close()
        print("Propulsores parados.")


def cmd_selftest(cfg, args):
    """Checagem sem ligar motores: drivers, IMU, GPS, Arduino, calibracoes."""
    res = []

    def rep(ok, name, msg):
        res.append(ok)
        print("[%s] %-10s %s" % ("OK " if ok is True else ("AVS" if ok is None else "ERR"), name, msg))

    b = RealBoat(cfg)
    try:
        b.start(use_imu=True, use_gps=True, use_motors=True)
        rep(True, "drivers", "importados de %s" % cfg.path_of("real.drivers_dir"))
    except Exception as e:
        rep(False, "start", repr(e))
        return 1
    shutdown.install()
    try:
        mags, accs = [], []
        for _ in range(10):
            m, a, g = b.read_imu()
            mags.append(m)
            accs.append(a)
        an = sum(math.sqrt(sum(v * v for v in a)) for a in accs) / len(accs)
        mn = sum(math.sqrt(sum(v * v for v in m)) for m in mags) / len(mags)
        rep(3500 < an < 4700, "imu-acc", "|acc| medio = %.0f LSB (1 g ~ 4096 em +-8 g)" % an)
        rep(mn > 0, "imu-mag", "|mag| medio = %.0f LSB, ultimo = %r" % (mn, mags[-1]))
        st = b.ard.get_arduino_status()
        rep(True, "arduino", "status: %r" % st)
        print("Lendo GPS por %d s..." % args.gps_s)
        fixes = 0
        t0 = time.time()
        last = None
        while time.time() - t0 < args.gps_s:
            f = b.gps_th.pop_new() if b.gps_th else None
            if f is not None:
                fixes += 1
                last = f
            time.sleep(0.05)
        if b.gps_th is None:
            rep(False, "gps", "nao inicializado")
        else:
            msg = "%d fixes validos, %d rejeitados" % (fixes, b.gps_th.rejected)
            if last is not None:
                msg += ", ultimo %.7f %.7f" % (last.lat, last.lon)
            if b.gps_th.gga is not None:
                msg += ", GGA %r" % (b.gps_th.gga,)
            rep(True if fixes > 0 else None, "gps", msg)
        for key in ("nav.mag2d_file", "nav.mag3d_file", "nav.axes_file"):
            p = cfg.path_of(key)
            if os.path.exists(p):
                age = (time.time() - os.path.getmtime(p)) / 3600.0
                rep(True, "calib", "%s (%.1f h atras)" % (os.path.relpath(p), age))
            else:
                rep(None, "calib", "%s ausente" % os.path.relpath(p))
        for key in ("safety.home_latlon", "safety.geofence_latlon", "nav.declination_deg"):
            rep(None if cfg.get(key) is None else True, "config", "%s = %r" % (key, cfg.get(key)))
    except (shutdown.StopRequested, KeyboardInterrupt):
        print("Interrompido.")
    finally:
        b.close()
    return 0 if all(r is not False for r in res) else 1


def _spike_count(series, tol=48):
    """Conta picos isolados de ~256: salto de ~256 que volta na amostra seguinte."""
    n = 0
    for i in range(1, len(series) - 1):
        d1 = series[i] - series[i - 1]
        d2 = series[i + 1] - series[i]
        if abs(abs(d1) - 256) <= tol and abs(abs(d2) - 256) <= tol and d1 * d2 < 0:
            n += 1
    return n


def cmd_imu_raw(cfg, args):
    """Le a IMU crua (sem mediana) na maior taxa possivel e conta os picos de 256.
    Para o teste do BDU: rode sem --bdu e depois com --bdu, barco PARADO."""
    ensure_dirs()
    b = RealBoat(cfg)
    b.start(use_imu=True, use_gps=False, use_motors=False)
    ag = b.imu._Imu9IO__dev_i2c_ag
    mg = b.imu._Imu9IO__dev_i2c_mg
    if args.bdu:
        ag.write(0x12, [0x44])
        mg.write(0x24, [0x40])
    else:  # valores de fabrica (o Imu9IO() ja escreveu 0x04 no CTRL3_C)
        ag.write(0x12, [0x04])
        mg.write(0x24, [0x00])
    shutdown.install()
    path = os.path.join(LOGS, "imu_raw_%s_%s.csv" % ("bdu" if args.bdu else "nobdu", time.strftime("%Y%m%d_%H%M%S")))
    rows = []
    try:
        t0 = time.time()
        while time.time() - t0 < args.dur:
            rows.append([time.time() - t0] + b.imu.read_mag_raw() + b.imu.read_accel_raw() + b.imu.read_gyro_raw())
    except (shutdown.StopRequested, KeyboardInterrupt):
        print("Interrompido.")
    with open(path, "w") as f:
        f.write("t,mag_x,mag_y,mag_z,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z\n")
        for r in rows:
            f.write(",".join(str(v) for v in r) + "\n")
    names = ["mag_x", "mag_y", "mag_z", "acc_x", "acc_y", "acc_z", "gyr_x", "gyr_y", "gyr_z"]
    print("BDU %s: %d leituras em %.1f s (%.0f Hz). Arquivo: %s" % (
        "LIGADO" if args.bdu else "desligado", len(rows), args.dur, len(rows) / args.dur, path))
    for k, nm in enumerate(names):
        print("  %-6s picos de ~256: %d" % (nm, _spike_count([r[k + 1] for r in rows])))


def cmd_gps(cfg, args):
    """Mostra os fixes do GPS (modo driver ou raw) por N s."""
    cfg.data["real"]["gps"]["mode"] = args.mode
    b = RealBoat(cfg)
    b.start(use_imu=False, use_gps=True, use_motors=False)
    shutdown.install()
    try:
        t0 = time.time()
        while time.time() - t0 < args.dur:
            f = b.gps_th.pop_new() if b.gps_th else None
            if f is not None:
                print("%6.1f s  lat=%.7f lon=%.7f sog=%s kn cog=%s utc=%s  rejeitados=%d gga=%r" % (
                    time.time() - t0, f.lat, f.lon, f.sog_kn, f.cog_deg, f.utc,
                    b.gps_th.rejected, b.gps_th.gga))
            time.sleep(0.05)
    except (shutdown.StopRequested, KeyboardInterrupt):
        print("Interrompido.")
    finally:
        b.close()


def cmd_heading(cfg, args):
    """Mostra o rumo (mag2d) sem ligar motores. Equivalente ao 'cap_nord check'."""
    from ddboat.calib import mag2d
    c = mag2d.load(cfg.path_of("nav.mag2d_file"))
    b = RealBoat(cfg)
    b.start(use_imu=True, use_gps=False, use_motors=False)
    shutdown.install()
    print("Gire o barco e compare com a bussola do celular. Ctrl+C para sair.")
    try:
        while True:
            m, a, g = b.read_imu()
            h = mag2d.heading_compass_deg(m[0], m[1], c)
            print("\rrumo(bussola) = %5.1f   mag=%r   " % (h, m), end="")
            time.sleep(0.05)
    except (shutdown.StopRequested, KeyboardInterrupt):
        print("\nFim.")
    finally:
        b.close()
