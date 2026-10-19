# -*- coding: utf-8 -*-
"""Log de missao: uma linha por volta do laco (CSV com ',' e ponto decimal)
+ um .json com metadados (config, missao, calibracoes, versao).

Escrita em buffer, com flush a cada flush_s segundos (cartao SD lento).
Para Excel em frances: tools/fr_excel.py converte para ';' e virgula decimal.
"""
import json
import math
import os
import time

import ddboat

COLUMNS = ["t", "step", "safety", "mag_x", "mag_y", "mag_z", "acc_x", "acc_y", "acc_z",
           "gyr_x", "gyr_y", "gyr_z", "gps_new", "lat", "lon", "sog_kn", "cog_deg",
           "pose_X", "pose_Y", "pose_h", "x", "y", "vx", "vy", "theta_deg", "compass_deg",
           "theta_mag_deg", "r_dps", "theta_ref_deg", "thrust", "turn", "cmd_l", "cmd_r",
           "loop_dt", "info"]


def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, float):
        if math.isnan(v):
            return ""
        return "%.7g" % v
    s = str(v)
    return s.replace(",", " ")


class CsvLogger(object):
    def __init__(self, log_dir, tag, meta, flush_s=1.0):
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(log_dir, "%s_%s" % (stamp, tag))
        n = 1
        path = base
        while os.path.exists(path + ".csv"):
            n += 1
            path = "%s_%d" % (base, n)
        self.path = path + ".csv"
        meta = dict(meta)
        meta["ddboat_version"] = ddboat.__version__
        meta["started_local"] = time.strftime("%Y-%m-%d %H:%M:%S")
        meta["columns"] = COLUMNS
        with open(path + ".json", "w") as f:
            json.dump(meta, f, indent=2)
        self.f = open(self.path, "w")
        self.f.write(",".join(COLUMNS) + "\n")
        self.flush_s = flush_s
        self._last_flush = time.time()
        self.rows = 0

    def write(self, row):
        """row: dict coluna -> valor (colunas ausentes ficam vazias)."""
        self.f.write(",".join(_fmt(row.get(c)) for c in COLUMNS) + "\n")
        self.rows += 1
        now = time.time()
        if now - self._last_flush >= self.flush_s:
            self.f.flush()
            self._last_flush = now

    def close(self):
        if self.f is not None:
            self.f.flush()
            self.f.close()
            self.f = None


def read_csv(path):
    """Le um log (formato novo, ou os antigos log_00x.csv com ',' ou ';').
    Retorna (colunas, linhas como dict de float/None)."""
    with open(path) as f:
        lines = [l.rstrip("\n\r") for l in f if l.strip()]
    head = lines[0]
    if ";" in head:  # formato frances do teleop antigo
        sep, dec = ";", ","
    else:
        sep, dec = ",", "."
    cols = head.split(sep)
    rows = []
    for l in lines[1:]:
        vals = l.split(sep)
        r = {}
        for c, v in zip(cols, vals):
            if v == "":
                r[c] = None
                continue
            try:
                r[c] = float(v.replace(dec, ".")) if dec == "," else float(v)
            except ValueError:
                r[c] = v
        rows.append(r)
    return cols, rows
