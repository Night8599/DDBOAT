# -*- coding: utf-8 -*-
"""Utilidades dos comandos de linha."""
import os
import sys
import time

from ddboat.core.config import REPO_ROOT

DATA = os.path.join(REPO_ROOT, "data")
CALIB = os.path.join(DATA, "calib")
LOGS = os.path.join(DATA, "logs")


def ensure_dirs():
    for d in (DATA, CALIB, LOGS):
        if not os.path.isdir(d):
            os.makedirs(d)


def ask(msg):
    sys.stdout.write(msg)
    sys.stdout.flush()
    return sys.stdin.readline().strip()


def wait_enter(msg):
    ask(msg + " [Enter] ")


def ask_int(msg, lo, hi):
    while True:
        s = ask("%s (%d a %d): " % (msg, lo, hi))
        try:
            v = int(s)
        except ValueError:
            print("ERRO: digite um numero inteiro.")
            continue
        if lo <= v <= hi:
            return v
        print("ERRO: fora do intervalo.")


def collect(backend, duration_s, period_s=0.0, show=None):
    """Le o backend por duration_s e devolve a lista de Samples."""
    out = []
    t0 = time.time()
    while time.time() - t0 < duration_s:
        s = backend.read()
        out.append(s)
        if show is not None:
            show(time.time() - t0, s, out)
        if period_s > 0:
            time.sleep(period_s)
    return out
