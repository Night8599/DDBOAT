# -*- coding: utf-8 -*-
"""Ponto de entrada unico da biblioteca DDBoat.

Uso:  python3 main.py <comando> [opcoes]       (python3 main.py -h para a lista)
Config padrao: config/boat04.json  (troque com --config)
"""
from __future__ import print_function
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ddboat.core.config import Config  # noqa: E402


def build_parser():
    p = argparse.ArgumentParser(prog="main.py", description="Biblioteca DDBoat - barco 04")
    p.add_argument("--config", default=None, help="arquivo de config (padrao config/boat04.json)")
    sub = p.add_subparsers(dest="cmd")

    s = sub.add_parser("motors", help="aplica (esq, dir) por N s e para (antigo comPropulseurs)")
    s.add_argument("left", type=int)
    s.add_argument("right", type=int)
    s.add_argument("--dur", type=float, default=2.0)

    s = sub.add_parser("selftest", help="checa drivers, IMU, GPS, Arduino e calibracoes (sem motor)")
    s.add_argument("--gps-s", type=int, default=5)

    s = sub.add_parser("imu-raw", help="IMU crua na taxa maxima + contagem de picos de 256 (teste BDU)")
    s.add_argument("--dur", type=float, default=10.0)
    s.add_argument("--bdu", action="store_true", help="liga o Block Data Update antes de ler")

    s = sub.add_parser("gps", help="mostra os fixes do GPS")
    s.add_argument("--dur", type=float, default=30.0)
    s.add_argument("--mode", choices=["driver", "raw"], default="driver")

    sub.add_parser("heading", help="mostra o rumo mag2d sem motores (antigo cap_nord check)")

    s = sub.add_parser("calib-mag2d", help="calibracao 2D (antigo cap_nord calib)")
    s.add_argument("--import-file", default=None, help="importa um calib_cap.json existente")
    s.add_argument("--place", default="", help="anotacao do local (vai para o JSON)")

    s = sub.add_parser("calib-gyro", help="bias do giro com o barco parado (diagnostico)")
    s.add_argument("--dur", type=float, default=10.0)

    s = sub.add_parser("check-gyro", help="valida sinal/escala do giro contra o mag2d")
    s.add_argument("--dur", type=int, default=20)

    sub.add_parser("calib-axes", help="eixos da IMU no barco (horizontal + proa levantada)")

    s = sub.add_parser("calib-mag3d", help="calibracao 3D (elipsoide + norte/oeste)")
    s.add_argument("--dur", type=float, default=60.0)

    s = sub.add_parser("teleop", help="teclado: w a s d v q")
    s.add_argument("--backend", choices=["real", "unity", "pysim"], default="real")

    s = sub.add_parser("unity-conv", help="mede a convencao do heading do Unity")
    s.add_argument("--fwd-s", type=float, default=2.0)
    s.add_argument("--turn-s", type=float, default=1.0)
    s.add_argument("--min-move", type=float, default=0.05)
    s.add_argument("--min-turn-deg", type=float, default=10.0)

    s = sub.add_parser("run", help="executa uma missao JSON")
    s.add_argument("mission")
    s.add_argument("--backend", choices=["real", "unity", "pysim"], default="real")
    s.add_argument("--realtime", action="store_true", help="pysim em tempo real (padrao: rapido)")
    s.add_argument("--quiet", action="store_true", help="sem linha de status")
    s.add_argument("--sem-geofence", action="store_true",
                   help="permite rodar no barco real sem geofence (so teste segurando o barco)")

    s = sub.add_parser("nav-replay", help="reprocessa um log pelo navegador (offline)")
    s.add_argument("log")
    s.add_argument("--gyro-still-s", type=float, default=0.0,
                   help="usa os N primeiros segundos (barco parado) para o bias do giro")
    return p


def main(argv=None):
    p = build_parser()
    args = p.parse_args(argv)
    if not args.cmd:
        p.print_help()
        return 1
    cfg = Config.load(args.config)
    from ddboat.apps import hw, calib_apps, teleop, sim_apps
    table = {
        "motors": hw.cmd_motors, "selftest": hw.cmd_selftest, "imu-raw": hw.cmd_imu_raw,
        "gps": hw.cmd_gps, "heading": hw.cmd_heading,
        "calib-mag2d": calib_apps.cmd_calib_mag2d, "calib-gyro": calib_apps.cmd_calib_gyro,
        "check-gyro": calib_apps.cmd_check_gyro, "calib-axes": calib_apps.cmd_calib_axes,
        "calib-mag3d": calib_apps.cmd_calib_mag3d,
        "teleop": teleop.cmd_teleop, "unity-conv": sim_apps.cmd_unity_conv,
        "run": sim_apps.cmd_run, "nav-replay": sim_apps.cmd_nav_replay,
    }
    rc = table[args.cmd](cfg, args)
    return rc or 0


if __name__ == "__main__":
    sys.exit(main())
