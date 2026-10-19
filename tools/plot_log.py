# -*- coding: utf-8 -*-
"""Painel de um log de missao (PC). Uso: python tools/plot_log.py data/logs/ARQ.csv [--png saida.png]"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ddboat.io.logger import read_csv  # noqa: E402


def col(rows, k):
    return [r.get(k) for r in rows]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--png", default=None)
    a = ap.parse_args()
    import matplotlib
    if a.png:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols, rows = read_csv(a.log)
    t = col(rows, "t")
    t0 = t[0]
    t = [v - t0 for v in t]
    fig, ax = plt.subplots(2, 2, figsize=(12, 9))
    xs = [(r["x"], r["y"]) for r in rows if r.get("x") is not None]
    if xs:
        ax[0][0].plot([p[0] for p in xs], [p[1] for p in xs], ".-", ms=2)
        ax[0][0].set_aspect("equal")
    ax[0][0].set_title("Trajetoria ENU (m)")
    ax[0][0].grid(True)
    ax[0][1].plot(t, col(rows, "theta_deg"), label="theta (estimado)")
    ax[0][1].plot(t, col(rows, "theta_mag_deg"), ".", ms=2, label="theta do mag")
    ax[0][1].plot(t, col(rows, "theta_ref_deg"), "--", label="theta_ref")
    ax[0][1].set_title("Rumo (graus, theta ENU)")
    ax[0][1].legend()
    ax[0][1].grid(True)
    ax[1][0].plot(t, col(rows, "cmd_l"), label="esq")
    ax[1][0].plot(t, col(rows, "cmd_r"), label="dir")
    ax[1][0].set_title("Comandos dos motores")
    ax[1][0].legend()
    ax[1][0].grid(True)
    ax[1][1].plot(t, col(rows, "loop_dt"))
    ax[1][1].set_title("Periodo real do laco (s)")
    ax[1][1].grid(True)
    for a_ in ax[1]:
        a_.set_xlabel("t (s)")
    fig.suptitle(os.path.basename(a.log))
    fig.tight_layout()
    if a.png:
        fig.savefig(a.png, dpi=110)
        print("salvo em", a.png)
    else:
        plt.show()


if __name__ == "__main__":
    main()
