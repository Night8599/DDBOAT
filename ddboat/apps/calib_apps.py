# -*- coding: utf-8 -*-
"""Comandos de calibracao no barco (sem motores): mag2d, mag3d, gyro, axes, check-gyro."""
import math
import os
import time

from ddboat.apps.common import CALIB, LOGS, ensure_dirs, wait_enter
from ddboat.calib import mag2d, mag3d, imu as imucal
from ddboat.core import shutdown
from ddboat.core.angles import wrap_pi, compass_to_theta
from ddboat.hal.real import RealBoat


def _imu_backend(cfg):
    b = RealBoat(cfg)
    b.start(use_imu=True, use_gps=False, use_motors=False)
    shutdown.install()
    return b


def _mean_vec(b, n):
    vs = [b.read_imu() for _ in range(n)]
    m = [sum(v[0][k] for v in vs) / float(n) for k in range(3)]
    a = [sum(v[1][k] for v in vs) / float(n) for k in range(3)]
    return m, a, vs


def cmd_calib_mag2d(cfg, args):
    """Mesmo procedimento do cap_nord.py calib (testado no barco)."""
    ensure_dirs()
    out = cfg.path_of("nav.mag2d_file")
    if args.import_file:
        c = mag2d.load(args.import_file)
        mag2d.save(c, out, {"imported_from": os.path.abspath(args.import_file)})
        print("Importado %s -> %s" % (args.import_file, out))
        return
    T = float(cfg.require("calib.mag2d_rotation_s"))
    b = _imu_backend(cfg)
    try:
        wait_enter("Fase 1: segure o barco na horizontal. Aperte Enter e gire devagar,\n"
                   "dando pelo menos uma volta completa (360 graus), durante %d s." % T)
        xs, ys = [], []
        t0 = time.time()
        while time.time() - t0 < T:
            m, a, g = b.read_imu()
            xs.append(m[0])
            ys.append(m[1])
            print("\r%4.1f s  x=[%d, %d]  y=[%d, %d]   " % (time.time() - t0, min(xs), max(xs), min(ys), max(ys)), end="")
        print()
        c = mag2d.fit_minmax(xs, ys)
        mag2d.check_coverage(xs, ys, c)
        print("Amplitude x = %d, amplitude y = %d (devem ser parecidas)" % (max(xs) - min(xs), max(ys) - min(ys)))
        wait_enter("Fase 2: aponte a PROA para o NORTE, barco parado.")
        north = [tuple(b.read_imu()[0][:2]) for _ in range(20)]
        wait_enter("Fase 3: aponte a PROA para o LESTE, barco parado.")
        east = [tuple(b.read_imu()[0][:2]) for _ in range(20)]
        c, d = mag2d.set_north_east(c, north, east)
        print("Diferenca norte -> leste medida: %.1f graus, sign = %d" % (d, c["sign"]))
        mag2d.save(c, out, {"date": time.strftime("%Y-%m-%d %H:%M:%S"), "place": args.place})
        print("Calibracao salva em %s" % out)
    except mag2d.CalibError as e:
        print("ERRO: %s. Calibracao NAO salva." % e)
    except (shutdown.StopRequested, KeyboardInterrupt):
        print("\nInterrompido. Calibracao NAO salva.")
    finally:
        b.close()


def cmd_calib_gyro(cfg, args):
    """Bias do giro com o barco parado (so diagnostico; a missao refaz no passo gyro_bias)."""
    ensure_dirs()
    b = _imu_backend(cfg)
    try:
        print("Barco PARADO por %.0f s..." % args.dur)
        g, a = [], []
        t0 = time.time()
        while time.time() - t0 < args.dur:
            m, acc, gyr = b.read_imu()
            g.append(gyr)
            a.append(acc)
        cal = imucal.gyro_still(g, a, float(cfg.require("calib.gyro_still_max_std_lsb")))
        dps = [v * 0.070 for v in cal["gyro_bias_lsb"]]
        print("bias (LSB) = %s  ->  %s graus/s" % ([round(v, 1) for v in cal["gyro_bias_lsb"]],
                                                   [round(v, 2) for v in dps]))
        print("desvio (LSB) = %s, 'cima' na IMU = %s" % ([round(v, 1) for v in cal["gyro_std_lsb"]],
                                                         [round(v, 3) for v in cal["up_unit_imu"]]))
        p = os.path.join(CALIB, "gyro_still.json")
        imucal.save(cal, p)
        print("Salvo em %s" % p)
    except imucal.CalibError as e:
        print("ERRO: %s" % e)
    except (shutdown.StopRequested, KeyboardInterrupt):
        print("Interrompido.")
    finally:
        b.close()


def cmd_check_gyro(cfg, args):
    """Compara o giro com o mag2d para validar o sinal/escala da fusao.
    5 s parado (bias), depois gire o barco a mao na horizontal ~90 graus
    para um lado e para o outro durante o tempo pedido."""
    c = mag2d.load(cfg.path_of("nav.mag2d_file"))
    b = _imu_backend(cfg)
    try:
        wait_enter("Barco PARADO e na horizontal; vou medir o bias por 5 s.")
        g, a = [], []
        t0 = time.time()
        while time.time() - t0 < 5.0:
            m, acc, gyr = b.read_imu()
            g.append(gyr)
            a.append(acc)
        cal = imucal.gyro_still(g, a, float(cfg.require("calib.gyro_still_max_std_lsb")))
        wait_enter("Agora gire o barco devagar, na horizontal, para os dois lados, por %d s." % args.dur)
        th_g = 0.0
        th_m_unwrap = 0.0
        prev_m = None
        tl = time.time()
        t0 = tl
        absg = 0.0
        while time.time() - t0 < args.dur:
            m, acc, gyr = b.read_imu()
            now = time.time()
            dt = now - tl
            tl = now
            r = imucal.yaw_rate(gyr, cal)
            th_g += r * dt
            thm = compass_to_theta(mag2d.heading_compass_deg(m[0], m[1], c))
            if prev_m is not None:
                d = wrap_pi(thm - prev_m)
                th_m_unwrap += d
                absg += abs(d)
            prev_m = thm
            print("\rgiro: %7.1f graus   mag: %7.1f graus   " % (math.degrees(th_g), math.degrees(th_m_unwrap)), end="")
        print()
        print("Variacao total |mag| = %.0f graus" % math.degrees(absg))
        if abs(th_m_unwrap) < math.radians(30):
            print("Giro final pequeno demais para comparar. Termine com o barco virado ~90 graus.")
        else:
            ratio = th_g / th_m_unwrap
            print("Razao giro/mag = %.2f (esperado ~ +1.0). " % ratio +
                  ("OK: pode usar nav.gyro_fusion" if 0.8 < ratio < 1.2 else "NAO use a fusao: sinal/escala nao batem"))
    except (imucal.CalibError, mag2d.CalibError) as e:
        print("ERRO: %s" % e)
    except (shutdown.StopRequested, KeyboardInterrupt):
        print("Interrompido.")
    finally:
        b.close()


def cmd_calib_axes(cfg, args):
    """Eixos da IMU no barco: horizontal + proa levantada."""
    ensure_dirs()
    b = _imu_backend(cfg)
    try:
        wait_enter("1) Barco PARADO e na HORIZONTAL.")
        _, a_flat, _ = _mean_vec(b, 30)
        wait_enter("2) Levante a PROA (frente) uns 20 a 30 graus e segure parado.")
        _, a_bow, _ = _mean_vec(b, 30)
        R, tilt = imucal.axes_from_flat_bow(a_flat, a_bow)
        p = cfg.path_of("nav.axes_file")
        imucal.save({"R_imu_to_boat": R, "tilt_deg": tilt, "acc_flat": a_flat, "acc_bow": a_bow,
                     "date": time.strftime("%Y-%m-%d %H:%M:%S")}, p)
        print("Proa levantada %.1f graus. Linhas de R (frente, esquerda, cima no referencial da IMU):" % tilt)
        for row, nm in zip(R, ("frente", "esquerda", "cima")):
            print("  %-8s %s" % (nm, [round(v, 3) for v in row]))
        print("Salvo em %s" % p)
    except imucal.CalibError as e:
        print("ERRO: %s" % e)
    except (shutdown.StopRequested, KeyboardInterrupt):
        print("Interrompido.")
    finally:
        b.close()


def cmd_calib_mag3d(cfg, args):
    """Elipsoide 3D + apontamento Norte e Oeste (polycopie DD-Boats)."""
    ensure_dirs()
    b = _imu_backend(cfg)
    try:
        wait_enter("Fase 1: longe de metal. Aperte Enter e gire o barco em TODAS as orientacoes\n"
                   "(voltas inclinadas, de lado, de cabeca para baixo) durante %d s." % args.dur)
        pts = []
        t0 = time.time()
        while time.time() - t0 < args.dur:
            pts.append(b.read_imu()[0])
            print("\r%4.1f s  %d pontos   " % (time.time() - t0, len(pts)), end="")
        print()
        cloud = os.path.join(LOGS, "mag3d_cloud_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        with open(cloud, "w") as f:
            f.write("mag_x,mag_y,mag_z\n")
            for p in pts:
                f.write("%d,%d,%d\n" % tuple(p))
        print("Nuvem salva em %s" % cloud)
        wait_enter("Fase 2: barco na HORIZONTAL, PROA para o NORTE, parado.")
        xn, _, _ = _mean_vec(b, 20)
        wait_enter("Fase 3: barco na HORIZONTAL, PROA para o OESTE, parado.")
        xw, _, _ = _mean_vec(b, 20)
        calib, diag = mag3d.calibrate(pts, xn, xw, float(cfg.require("calib.mag3d_inclination_deg")))
        calib["date"] = time.strftime("%Y-%m-%d %H:%M:%S")
        calib["cloud_file"] = cloud
        mag3d.save(calib, diag, cfg.path_of("nav.mag3d_file"))
        print("Diagnostico:")
        for k in sorted(diag):
            print("  %-24s %s" % (k, diag[k]))
        print("Esperado: norm_mean ~ 1, norm_std pequeno (< 0.05), angulo N-W medido ~ teorico.")
        print("Salvo em %s" % cfg.path_of("nav.mag3d_file"))
    except mag3d.CalibError as e:
        print("ERRO: %s" % e)
    except (shutdown.StopRequested, KeyboardInterrupt):
        print("\nInterrompido. Nada salvo.")
    finally:
        b.close()
