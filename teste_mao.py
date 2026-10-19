# -*- coding: utf-8 -*-
"""Teste na mao do barco real: primeiro teste da biblioteca no hardware.

Uso no barco (ao ar livre, ceu aberto, longe de metal):
    cd ~/DDBOAT
    python3 teste_mao.py

Roda secao por secao, pedindo acoes (girar, inclinar, apontar, Enter) e
respostas (s/n). Cada secao pode ser pulada com 'p'. Ctrl+C a qualquer
momento zera os motores e salva o relatorio parcial.

Saidas em data/logs/:
    teste_mao_<data>.txt    relatorio legivel (OK / AVS / ERR / INFO / PULADO)
    teste_mao_<data>.json   todas as medidas
    *_teste_mao_*.csv       logs do laco e do controle de rumo
Calibracoes em data/calib/: mag2d.json, gyro_still.json, axes.json.

Nenhum valor do config/boat04.json e alterado: as recomendacoes ficam no
relatorio para voces decidirem.
"""
from __future__ import print_function

import copy
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

from ddboat.apps.common import CALIB, LOGS, ask, ask_int, ensure_dirs  # noqa: E402
from ddboat.apps.hw import _spike_count  # noqa: E402
from ddboat.calib import imu as imucal  # noqa: E402
from ddboat.calib import mag2d  # noqa: E402
from ddboat.control.heading import HeadingController  # noqa: E402
from ddboat.core import linalg as la  # noqa: E402
from ddboat.core import shutdown  # noqa: E402
from ddboat.core.angles import circular_mean_deg, compass_to_theta, wrap_180, wrap_pi  # noqa: E402
from ddboat.core.clock import Rate  # noqa: E402
from ddboat.core.config import Config, ConfigError  # noqa: E402
from ddboat.core.geo import LocalFrame  # noqa: E402
from ddboat.core.keyboard import KeyReader  # noqa: E402
from ddboat.estimation.navigator import Navigator  # noqa: E402
from ddboat.hal import nmea  # noqa: E402
from ddboat.hal.real import RealBoat, load_drivers  # noqa: E402
from ddboat.io.logger import CsvLogger  # noqa: E402
from ddboat.mission.runner import make_mixer, nav_row, run_mission, sample_row  # noqa: E402

PULSE_S = 2.0        # duracao de cada pulso de motor
HEADING_TOL = 10.0   # graus: erro de rumo aceito contra o celular (OK); ate 2x = AVS


class Skip(Exception):
    pass


# ====================================================================== relatorio
class Report(object):
    def __init__(self):
        self.items = []
        self.recs = []
        self.data = {}
        self.sec = "-"

    def add(self, status, name, msg):
        self.items.append({"secao": self.sec, "status": status, "item": name, "msg": msg})
        print("  [%-6s] %-18s %s" % (status, name, msg))

    def rec(self, msg):
        self.recs.append(msg)
        print("  >> RECOMENDACAO: %s" % msg)

    def put(self, key, value):
        self.data.setdefault(self.sec, {})[key] = value

    def save(self, stamp, interrupted):
        base = os.path.join(LOGS, "teste_mao_%s" % stamp)
        cnt = {}
        for it in self.items:
            cnt[it["status"]] = cnt.get(it["status"], 0) + 1
        lines = ["Teste na mao - %s" % time.strftime("%Y-%m-%d %H:%M:%S"),
                 "Interrompido: %s" % ("SIM" if interrupted else "nao"),
                 "Contagem: %s" % ", ".join("%s=%d" % (k, cnt[k]) for k in sorted(cnt)), ""]
        sec = None
        for it in self.items:
            if it["secao"] != sec:
                sec = it["secao"]
                lines.append("== %s" % sec)
            lines.append("  [%-6s] %-18s %s" % (it["status"], it["item"], it["msg"]))
        lines.append("")
        lines.append("== Recomendacoes (nada foi alterado no config)")
        for r in self.recs:
            lines.append("  - " + r)
        with open(base + ".txt", "w") as f:
            f.write("\n".join(lines) + "\n")
        with open(base + ".json", "w") as f:
            json.dump({"itens": self.items, "recomendacoes": self.recs, "medidas": self.data,
                       "interrompido": interrupted}, f, indent=2, default=str)
        return base


# ====================================================================== entrada
def yn(msg):
    while True:
        a = ask(msg + " (s/n): ").lower()
        if a in ("s", "n"):
            return a == "s"
        print("Responda s ou n.")


def choice(msg, opts):
    """opts: dict tecla -> descricao. Retorna a tecla."""
    txt = " / ".join("%s=%s" % (k, opts[k]) for k in sorted(opts))
    while True:
        a = ask("%s [%s]: " % (msg, txt)).lower()
        if a in opts:
            return a
        print("Opcao invalida.")


def ask_float(msg, lo, hi):
    while True:
        s = ask("%s (%g a %g): " % (msg, lo, hi)).replace(",", ".")
        try:
            v = float(s)
        except ValueError:
            print("ERRO: digite um numero.")
            continue
        if lo <= v <= hi:
            return v
        print("ERRO: fora do intervalo.")


def step(msg):
    """Enter continua, p pula a etapa (Skip)."""
    a = ask(msg + "\n   [Enter = fazer, p = pular] ").lower()
    if a == "p":
        raise Skip()


def countdown(seconds, label):
    end = time.time() + seconds
    while True:
        left = end - time.time()
        if left <= 0:
            break
        sys.stdout.write("\r   %s: %4.1f s   " % (label, left))
        sys.stdout.flush()
        time.sleep(0.1)
    print()


# ====================================================================== contexto
class Ctx(object):
    def __init__(self, cfg, rep):
        self.cfg = cfg
        self.rep = rep
        self.b = None
        self.V = None
        self.c2 = None
        self.gps_t0 = None
        self.gps_t0_mono = None
        self.gyro_cal = None

    def need_boat(self):
        if self.b is None:
            raise Skip("barco nao inicializado")

    def need_motors(self):
        self.need_boat()
        if self.b.ard is None:
            raise Skip("Arduino nao inicializado")

    def need_mag2d(self):
        if self.c2 is None:
            p = self.cfg.path_of("nav.mag2d_file")
            if not os.path.exists(p):
                raise Skip("sem calibracao mag2d (rode a secao de calibracao)")
            self.c2 = mag2d.load(p)
        return self.c2

    def need_V(self):
        if self.V is None:
            hi = int(self.cfg.require("real.motors.cmd_max"))
            print("   Lembrete: os motores so respondem depois da melodia tocar 3 vezes.")
            print("   Segure o barco firme, maos e fios longe das helices.")
            self.V = ask_int("Velocidade", 0, hi)
            self.rep.put("V", self.V)
        return self.V

    def stop(self):
        if self.b is not None and self.b.ard is not None:
            try:
                self.b.stop()
            except Exception:
                pass


def full_close(b):
    """Fecha o backend e as portas seriais (o close() do RealBoat so zera motores)."""
    if b is None:
        return
    ser = b.ard.arduino if b.ard is not None else None
    try:
        b.close()
    finally:
        if b.gps_th is not None:
            b.gps_th.running = False
            b.gps_th.join(1.0)
            try:
                b.gps_th.gps.ser.close()
            except Exception:
                pass
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


def read_n(b, n):
    return [b.read_imu() for _ in range(n)]


def mean3(vs):
    return [sum(v[k] for v in vs) / float(len(vs)) for k in range(3)]


def std3(vs, m):
    return [math.sqrt(sum((v[k] - m[k]) ** 2 for v in vs) / float(len(vs))) for k in range(3)]


def heading_now(b, c, n=20):
    """Rumo medio (bussola, graus) de n leituras com mediana."""
    hs = [mag2d.heading_compass_deg(s[0][0], s[0][1], c) for s in read_n(b, n)]
    return circular_mean_deg(hs) % 360.0


def angle_between(u, v):
    c = la.dot(u, v) / (la.norm(u) * la.norm(v))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def pulse(ctx, l, r, dur=PULSE_S, raw=False):
    """Aplica (l, r) por dur s, reenviando a 10 Hz (watchdog), e zera."""
    b = ctx.b
    try:
        t0 = time.time()
        while time.time() - t0 < dur:
            if raw:
                b.ard.send_arduino_cmd_motor(l, r)
            else:
                b.set_motors(l, r)
            time.sleep(0.1)
    finally:
        b.stop()


# ====================================================================== secoes
def sec_ambiente(ctx):
    rep, cfg = ctx.rep, ctx.cfg
    v = sys.version_info
    rep.add("OK" if v >= (3, 3) else "ERR", "python", sys.version.split()[0])
    rep.put("python", sys.version)
    rep.add("INFO", "config", cfg.path)
    try:
        out = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
                                      stderr=subprocess.STDOUT).decode().strip()
        dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT,
                                        stderr=subprocess.STDOUT).decode().strip()
        n = len([x for x in dirty.splitlines() if x.strip()])
        rep.add("INFO", "git", "commit %s, %d arquivo(s) modificado(s)" % (out, n))
    except Exception as e:
        rep.add("INFO", "git", "sem git (%s)" % e.__class__.__name__)
    year = time.localtime().tm_year
    if year < 2026:
        rep.add("AVS", "relogio", "%s: data do sistema errada, nomes dos logs vao sair com data errada"
                % time.strftime("%Y-%m-%d %H:%M"))
    else:
        rep.add("OK", "relogio", time.strftime("%Y-%m-%d %H:%M:%S"))
    try:
        st = os.statvfs(ROOT)
        mb = st.f_bavail * st.f_frsize / 1e6
        rep.add("OK" if mb > 100 else "AVS", "disco", "%.0f MB livres" % mb)
    except Exception as e:
        rep.add("INFO", "disco", repr(e))
    try:
        ensure_dirs()
        p = os.path.join(LOGS, ".teste_escrita")
        with open(p, "w") as f:
            f.write("ok")
        os.remove(p)
        rep.add("OK", "data/", "escrita em %s" % os.path.relpath(LOGS, ROOT))
    except Exception as e:
        rep.add("ERR", "data/", repr(e))
    for k in ("real.drivers_dir", "real.motors.cmd_max", "real.motors.allow_reverse", "real.turn_sign",
              "real.imu.median_n", "real.gps.mode", "loop.hz", "nav.heading_source",
              "control.heading.kp", "calib.mag2d_rotation_s"):
        try:
            rep.put(k, cfg.require(k))
        except ConfigError as e:
            rep.add("ERR", "config", str(e))
    for name in sorted(os.listdir(CALIB)) if os.path.isdir(CALIB) else []:
        age = (time.time() - os.path.getmtime(os.path.join(CALIB, name))) / 3600.0
        rep.add("INFO", "calib existente", "%s (%.1f h atras)" % (name, age))


def sec_arranque(ctx):
    rep, cfg = ctx.rep, ctx.cfg
    try:
        load_drivers(cfg.path_of("real.drivers_dir"))
        rep.add("OK", "drivers", "importados de %s" % cfg.require("real.drivers_dir"))
    except Exception as e:
        rep.add("ERR", "drivers", repr(e))
        raise Skip("sem drivers")
    b = RealBoat(cfg)
    try:
        b.start(use_imu=True, use_gps=True, use_motors=True)
    except Exception as e:
        rep.add("ERR", "start", repr(e))
        full_close(b)
        raise Skip("barco nao inicializou")
    shutdown.install()  # depois do ArduinoIO, que instala o proprio SIGINT
    ctx.b = b
    ctx.gps_t0 = time.time()
    ctx.gps_t0_mono = b.clock.now()
    m, a, g = b.read_imu()
    rep.add("OK", "imu", "mag=%r acc=%r gyr=%r" % (m, a, g))
    rep.add("OK" if b.has_gps else "ERR", "gps", "thread iniciada" if b.has_gps else "nao inicializado")
    try:
        st = b.ard.get_arduino_status()
        rep.add("OK", "arduino", "status: %r" % st)
    except Exception as e:
        rep.add("AVS", "arduino", "aberto, mas status falhou: %r" % e)
    if b.watchdog_s > 0:
        rep.add("INFO", "watchdog", "%.1f s" % b.watchdog_s)


def sec_imu_parado(ctx):
    ctx.need_boat()
    b, rep = ctx.b, ctx.rep
    dur = 10.0
    step("Apoie o barco PARADO e na HORIZONTAL (mesa ou chao) por %d s, sem tocar." % dur)
    rows = []
    t0 = time.time()
    while time.time() - t0 < dur:
        m, a, g = b.read_imu()
        rows.append((time.time() - t0, m, a, g))
    hz = len(rows) / dur
    rep.add("INFO", "taxa read_imu", "%.1f Hz com mediana de %d" % (hz, b.median_n))
    rep.put("read_imu_hz", hz)
    accs = [r[2] for r in rows]
    gyrs = [r[3] for r in rows]
    mags = [r[1] for r in rows]
    am = mean3(accs)
    an = la.norm(am)
    rep.add("OK" if 3500 < an < 4700 else "ERR", "acc |g|", "%.0f LSB (1 g ~ 4096 em +-8 g)" % an)
    zi = max(range(3), key=lambda k: abs(am[k]))
    axis = [0.0, 0.0, 0.0]
    axis[zi] = 1.0 if am[zi] > 0 else -1.0
    tilt = angle_between(am, axis)
    rep.add("INFO", "inclinacao IMU", "%.1f graus entre a vertical e o eixo %s%s da IMU"
            % (tilt, "+" if am[zi] > 0 else "-", "xyz"[zi]))
    rep.put("acc_mean", am)
    rep.put("tilt_rest_deg", tilt)
    try:
        cal = imucal.gyro_still(gyrs, accs, float(ctx.cfg.require("calib.gyro_still_max_std_lsb")))
        ctx.gyro_cal = cal
        imucal.save(cal, os.path.join(CALIB, "gyro_still.json"))
        r = abs(la.dot(cal["gyro_bias_lsb"], cal["up_unit_imu"])) * 0.070
        rep.add("OK", "giro bias", "%s LSB, desvio %s LSB" % ([round(x, 1) for x in cal["gyro_bias_lsb"]],
                                                            [round(x, 1) for x in cal["gyro_std_lsb"]]))
        rep.add("INFO", "deriva sem correcao", "%.1f graus/min no rumo" % (r * 60.0))
        rep.put("gyro_cal", cal)
    except imucal.CalibError as e:
        rep.add("AVS", "giro bias", str(e))
    mm = mean3(mags)
    ms = std3(mags, mm)
    rep.add("INFO", "mag", "media %s, |m| = %.0f, ruido (desvio) %s LSB"
            % ([int(x) for x in mm], la.norm(mm), [round(x, 1) for x in ms]))
    rep.put("mag_mean", mm)
    rep.put("mag_std", ms)
    # taxa real de atualizacao dos sensores (leituras cruas, sem mediana)
    ch_m = ch_a = n = 0
    pm = pa = None
    t0 = time.time()
    while time.time() - t0 < 2.0:
        m = b.imu.read_mag_raw()
        a = b.imu.read_accel_raw()
        n += 1
        if pm is not None and m != pm:
            ch_m += 1
        if pa is not None and a != pa:
            ch_a += 1
        pm, pa = list(m), list(a)
    rep.add("INFO", "ODR medido", "mag ~%.0f Hz, acc ~%.0f Hz (leitura crua a %.0f Hz)"
            % (ch_m / 2.0, ch_a / 2.0, n / 2.0))
    rep.put("odr", {"mag_hz": ch_m / 2.0, "acc_hz": ch_a / 2.0, "read_hz": n / 2.0})


def sec_bdu(ctx):
    ctx.need_boat()
    b, rep = ctx.b, ctx.rep
    ag = b.imu._Imu9IO__dev_i2c_ag
    mg = b.imu._Imu9IO__dev_i2c_mg
    step("Barco PARADO. Leio a IMU crua 5 s sem BDU e 5 s com BDU (picos de ~256).")
    names = ["mag_x", "mag_y", "mag_z", "acc_x", "acc_y", "acc_z", "gyr_x", "gyr_y", "gyr_z"]
    res = {}
    try:
        for bdu in (False, True):
            if bdu:
                ag.write(0x12, [0x44])
                mg.write(0x24, [0x40])
            else:
                ag.write(0x12, [0x04])
                mg.write(0x24, [0x00])
            rows = []
            t0 = time.time()
            while time.time() - t0 < 5.0:
                rows.append(b.imu.read_mag_raw() + b.imu.read_accel_raw() + b.imu.read_gyro_raw())
            sp = dict((nm, _spike_count([r[k] for r in rows])) for k, nm in enumerate(names))
            tot = sum(sp.values())
            res["bdu" if bdu else "sem_bdu"] = {"leituras": len(rows), "picos": sp, "total": tot}
            rep.add("INFO", "BDU %s" % ("ligado" if bdu else "desligado"),
                    "%d picos em %d leituras %s" % (tot, len(rows), dict((k, v) for k, v in sp.items() if v)))
    finally:
        if ctx.cfg.require("real.imu.bdu"):
            ag.write(0x12, [0x44])
            mg.write(0x24, [0x40])
        else:
            ag.write(0x12, [0x04])
            mg.write(0x24, [0x00])
    rep.put("bdu", res)
    a, c = res["sem_bdu"]["total"], res["bdu"]["total"]
    if a > 0 and c == 0:
        rep.add("OK", "BDU", "elimina os picos")
        rep.rec("real.imu.bdu = true (picos %d -> 0)" % a)
    elif a == 0 and c == 0:
        rep.add("OK", "BDU", "sem picos parado nos dois casos; repita com o barco se mexendo")
    else:
        rep.add("AVS", "BDU", "picos sem BDU = %d, com BDU = %d" % (a, c))


def sec_eixos(ctx):
    ctx.need_boat()
    b, rep = ctx.b, ctx.rep
    step("Eixos da IMU: 3 posturas, cada uma parada por ~1 s.")
    ask("   1) Barco na HORIZONTAL e parado. [Enter] ")
    a_flat = mean3([s[1] for s in read_n(b, 30)])
    ask("   2) Levante a PROA (frente) uns 20 a 30 graus e segure. [Enter] ")
    a_bow = mean3([s[1] for s in read_n(b, 30)])
    ask("   3) Volte a horizontal e abaixe o lado ESQUERDO (bombordo) uns 20 graus. [Enter] ")
    a_left = mean3([s[1] for s in read_n(b, 30)])
    try:
        R, tilt = imucal.axes_from_flat_bow(a_flat, a_bow)
    except imucal.CalibError as e:
        rep.add("ERR", "eixos", str(e))
        return
    vb = la.matvec(R, a_left)
    roll = math.degrees(math.asin(max(-1.0, min(1.0, -vb[1] / la.norm(vb)))))
    ok = roll > 5.0
    rep.add("OK" if ok else "ERR", "lado esquerdo",
            "inclinacao medida %.1f graus para a esquerda" % roll if ok else
            "esperado esquerda para baixo, medido %.1f graus: confira se abaixou o lado ESQUERDO" % roll)
    names = []
    for row, nm in zip(R, ("frente", "esquerda", "cima")):
        k = max(range(3), key=lambda i: abs(row[i]))
        names.append("%s = %s%s IMU (%.2f)" % (nm, "+" if row[k] > 0 else "-", "xyz"[k], abs(row[k])))
    rep.add("INFO", "eixos", "; ".join(names))
    rep.add("INFO", "proa levantada", "%.1f graus" % tilt)
    p = ctx.cfg.path_of("nav.axes_file")
    imucal.save({"R_imu_to_boat": R, "tilt_deg": tilt, "acc_flat": a_flat, "acc_bow": a_bow,
                 "acc_left_down": a_left, "date": time.strftime("%Y-%m-%d %H:%M:%S"),
                 "origem": "teste_mao"}, p)
    rep.add("OK" if ok else "AVS", "axes.json", "salvo em %s" % os.path.relpath(p, ROOT))
    rep.put("R", R)


def sec_calib_mag2d(ctx):
    ctx.need_boat()
    b, rep, cfg = ctx.b, ctx.rep, ctx.cfg
    out = cfg.path_of("nav.mag2d_file")
    if os.path.exists(out):
        age = (time.time() - os.path.getmtime(out)) / 3600.0
        if not yn("   Ja existe %s (%.1f h atras). Refazer?" % (os.path.relpath(out, ROOT), age)):
            ctx.c2 = mag2d.load(out)
            rep.add("INFO", "mag2d", "mantida a calibracao existente")
            return
    north = choice("   A bussola do celular esta em norte", {"m": "magnetico", "v": "verdadeiro",
                                                                "n": "nao sei"})
    north = {"m": "magnetico", "v": "verdadeiro", "n": "nao sei"}[north]
    T = float(cfg.require("calib.mag2d_rotation_s"))
    print("   Dica: deixe o celular a ~1 m do barco ao apontar (o celular perturba o magnetometro).")
    ask("   Fase 1: barco na horizontal. Enter e gire devagar, pelo menos 1 volta completa, "
        "durante %d s. [Enter] " % T)
    xs, ys = [], []
    t0 = time.time()
    while time.time() - t0 < T:
        m, a, g = b.read_imu()
        xs.append(m[0])
        ys.append(m[1])
        sys.stdout.write("\r   %4.1f s  x=[%d, %d]  y=[%d, %d]   " % (time.time() - t0, min(xs), max(xs),
                                                                     min(ys), max(ys)))
        sys.stdout.flush()
    print()
    try:
        c = mag2d.fit_minmax(xs, ys)
        mag2d.check_coverage(xs, ys, c)
        ax, ay = max(xs) - min(xs), max(ys) - min(ys)
        ratio = min(ax, ay) / float(max(ax, ay))
        rep.add("OK" if ratio > 0.7 else "AVS", "amplitudes", "x = %d, y = %d (razao %.2f, ideal ~1)"
                % (ax, ay, ratio))
        rep.add("INFO", "hard iron", "centro (%.0f, %.0f) LSB" % (c["offx"], c["offy"]))
        ask("   Fase 2: PROA para o NORTE, barco parado. [Enter] ")
        n_s = [tuple(s[0][:2]) for s in read_n(b, 20)]
        ask("   Fase 3: PROA para o LESTE, barco parado. [Enter] ")
        e_s = [tuple(s[0][:2]) for s in read_n(b, 20)]
        c, d = mag2d.set_north_east(c, n_s, e_s)
        rep.add("OK" if abs(abs(d) - 90) < 20 else "AVS", "norte->leste", "%.1f graus, sign = %d" % (d, c["sign"]))
        mag2d.save(c, out, {"date": time.strftime("%Y-%m-%d %H:%M:%S"), "place": "teste_mao",
                            "compass_north": north, "n_samples": len(xs)})
        ctx.c2 = mag2d.load(out)
        rep.add("OK", "mag2d.json", "salvo em %s" % os.path.relpath(out, ROOT))
        rep.put("mag2d", c)
        rep.put("compass_north", north)
        if north == "magnetico":
            rep.rec("calibrado em norte magnetico: para rumo verdadeiro preencha nav.declination_deg "
                    "com a declinacao local (hoje null = 0)")
        elif north == "nao sei":
            rep.rec("confirmar no app se a bussola mostra norte magnetico ou verdadeiro")
    except mag2d.CalibError as e:
        rep.add("ERR", "mag2d", "%s. Calibracao NAO salva" % e)


def sec_valida_rumo(ctx):
    ctx.need_boat()
    c = ctx.need_mag2d()
    b, rep = ctx.b, ctx.rep
    step("Compara o rumo da biblioteca com o celular em 4 direcoes.")
    errs = []
    for nome, alvo in (("LESTE", 90), ("SUL", 180), ("OESTE", 270), ("NORTE", 0)):
        ask("   Aponte a PROA para ~%s, barco na horizontal e parado. [Enter] " % nome)
        h = heading_now(b, c)
        ph = ask_float("   Rumo que o CELULAR mostra agora", 0, 360)
        e = wrap_180(h - ph)
        errs.append(e)
        st = "OK" if abs(e) <= HEADING_TOL else ("AVS" if abs(e) <= 2 * HEADING_TOL else "ERR")
        rep.add(st, "rumo %s" % nome, "biblioteca %.1f, celular %.1f, erro %+.1f graus" % (h, ph, e))
        rep.put(nome, {"lib": h, "celular": ph, "erro": e})
    m = sum(errs) / len(errs)
    rep.add("INFO", "erro medio", "%+.1f graus (vies), max |erro| %.1f" % (m, max(abs(x) for x in errs)))


def sec_inclinacao(ctx):
    ctx.need_boat()
    c = ctx.need_mag2d()
    b, rep = ctx.b, ctx.rep
    step("Sensibilidade do rumo a inclinacao (sem compensacao). Mantenha a proa no mesmo rumo.")
    ask("   Proa para um rumo fixo (ex.: NORTE), HORIZONTAL. [Enter] ")
    base = read_n(b, 20)
    h0 = circular_mean_deg([mag2d.heading_compass_deg(s[0][0], s[0][1], c) for s in base]) % 360.0
    a0 = mean3([s[1] for s in base])
    worst = 0.0
    for pose in ("PROA LEVANTADA ~15 graus", "PROA ABAIXADA ~15 graus",
                 "LADO ESQUERDO ABAIXADO ~15 graus", "LADO DIREITO ABAIXADO ~15 graus"):
        ask("   %s, mesmo rumo. [Enter] " % pose)
        s = read_n(b, 20)
        h = circular_mean_deg([mag2d.heading_compass_deg(x[0][0], x[0][1], c) for x in s]) % 360.0
        tilt = angle_between(a0, mean3([x[1] for x in s]))
        d = wrap_180(h - h0)
        worst = max(worst, abs(d))
        rep.add("INFO", pose.split(" ~")[0].lower(), "inclinacao %.1f graus -> rumo mudou %+.1f graus" % (tilt, d))
        rep.put(pose, {"tilt": tilt, "drumo": d})
    if worst > 10.0:
        rep.add("AVS", "inclinacao", "ate %.1f graus de erro" % worst)
        rep.rec("rumo sensivel a inclinacao (%.0f graus): testar mag3d + tilt_compensation" % worst)
    else:
        rep.add("OK", "inclinacao", "erro max %.1f graus" % worst)


def sec_giro(ctx):
    ctx.need_boat()
    c = ctx.need_mag2d()
    b, rep = ctx.b, ctx.rep
    step("Giro x magnetometro: 5 s parado, depois 20 s girando a mao na horizontal.")
    ask("   Barco PARADO e na horizontal. [Enter] ")
    g, a = [], []
    t0 = time.time()
    while time.time() - t0 < 5.0:
        m, acc, gyr = b.read_imu()
        g.append(gyr)
        a.append(acc)
    try:
        cal = imucal.gyro_still(g, a, float(ctx.cfg.require("calib.gyro_still_max_std_lsb")))
    except imucal.CalibError as e:
        rep.add("ERR", "bias", str(e))
        return
    ask("   Gire devagar para um lado e para o outro (~90 graus), terminando virado. [Enter] ")
    th_g = th_m = absm = 0.0
    prev = None
    tl = t0 = time.time()
    while time.time() - t0 < 20.0:
        m, acc, gyr = b.read_imu()
        now = time.time()
        dt = now - tl
        tl = now
        th_g += imucal.yaw_rate(gyr, cal) * dt
        thm = compass_to_theta(mag2d.heading_compass_deg(m[0], m[1], c))
        if prev is not None:
            d = wrap_pi(thm - prev)
            th_m += d
            absm += abs(d)
        prev = thm
        sys.stdout.write("\r   giro %7.1f   mag %7.1f graus   " % (math.degrees(th_g), math.degrees(th_m)))
        sys.stdout.flush()
    print()
    rep.put("giro_deg", math.degrees(th_g))
    rep.put("mag_deg", math.degrees(th_m))
    if abs(th_m) < math.radians(30):
        rep.add("AVS", "giro/mag", "terminou virado so %.0f graus: repita terminando ~90 graus" % math.degrees(th_m))
        return
    ratio = th_g / th_m
    ok = 0.8 < ratio < 1.2
    rep.add("OK" if ok else "ERR", "giro/mag", "razao %.2f (esperado ~1.0), percurso |mag| %.0f graus"
            % (ratio, math.degrees(absm)))
    if ok:
        rep.rec("giro validado (razao %.2f): nav.gyro_fusion pode ser testado na agua" % ratio)


def sec_gps(ctx):
    ctx.need_boat()
    b, rep = ctx.b, ctx.rep
    if b.gps_th is None:
        rep.add("ERR", "gps", "nao inicializado")
        return
    step("GPS: ceu aberto. Espera o 1o fix (ate 180 s, 'p' desiste), depois 30 s parado.")
    first = b.gps_th.last
    ja = first is not None
    with KeyReader() as kb:
        while first is None:
            el = time.time() - ctx.gps_t0
            f = b.gps_th.last
            if f is not None:
                first = f
                break
            if el > 180:
                break
            sys.stdout.write("\r   esperando fix: %3.0f s desde a inicializacao, rejeitados %d   "
                             % (el, b.gps_th.rejected))
            sys.stdout.flush()
            if kb.read(0.5) == "p":
                break
    print()
    if first is None:
        rep.add("ERR", "fix", "nenhum fix valido (rejeitados %d)" % b.gps_th.rejected)
    else:
        el = b.clock.now() - ctx.gps_t0_mono
        if ja:
            rep.add("OK", "fix", "ja havia fix ao chegar aqui (GPS ligado ha %.0f s): %.7f, %.7f"
                    % (el, first.lat, first.lon))
        else:
            rep.add("OK", "1o fix", "%.0f s apos ligar: %.7f, %.7f" % (first.t - ctx.gps_t0_mono,
                                                                     first.lat, first.lon))
        b.gps_th.pop_new()
        fixes = []
        t0 = time.time()
        while time.time() - t0 < 30.0:
            f = b.gps_th.pop_new()
            if f is not None:
                fixes.append(f)
            sys.stdout.write("\r   %4.1f s  %d fixes   " % (time.time() - t0, len(fixes)))
            sys.stdout.flush()
            time.sleep(0.05)
        print()
        hz = len(fixes) / 30.0
        rep.add("OK" if hz >= 0.5 else "AVS", "taxa", "%.2f fixes/s" % hz)
        if fixes:
            lat0 = sum(f.lat for f in fixes) / len(fixes)
            lon0 = sum(f.lon for f in fixes) / len(fixes)
            fr = LocalFrame(lat0, lon0)
            xy = [fr.to_xy(f.lat, f.lon) for f in fixes]
            sx = math.sqrt(sum(p[0] ** 2 for p in xy) / len(xy))
            sy = math.sqrt(sum(p[1] ** 2 for p in xy) / len(xy))
            dmax = max(math.hypot(p[0], p[1]) for p in xy)
            sog = [f.sog_kn for f in fixes if f.sog_kn is not None]
            rep.add("OK" if max(sx, sy) < 5 else "AVS", "dispersao parado",
                    "desvio x %.1f m, y %.1f m, max %.1f m" % (sx, sy, dmax))
            rep.add("INFO", "posicao media", "%.7f, %.7f" % (lat0, lon0))
            if sog:
                rep.add("INFO", "sog parado", "media %.2f nos" % (sum(sog) / len(sog)))
            rep.put("fixes", [[f.t, f.lat, f.lon, f.sog_kn, f.cog_deg, f.utc] for f in fixes])
        rep.add("INFO", "rejeitados", "%d (sem fix ou frase invalida)" % b.gps_th.rejected)
    # modo raw numa copia da porta (/dev/ttyGPS1), sem comandos PMTK
    try:
        _, gps_mod, _ = load_drivers(ctx.cfg.path_of("real.drivers_dir"))
        g = gps_mod.GpsIO(1)
    except Exception as e:
        rep.add("AVS", "gps raw", "nao abriu /dev/ttyGPS1: %r" % e)
        return
    cnt = {"linhas": 0, "checksum_ruim": 0, "rmc_A": 0, "rmc_V": 0, "gga": 0}
    gga = None
    try:
        t0 = time.time()
        while time.time() - t0 < 10.0:
            try:
                ok, line = g.read_non_blocking(timeout=0.2)
            except Exception:
                cnt["checksum_ruim"] += 1
                continue
            if not ok:
                continue
            cnt["linhas"] += 1
            if not nmea.checksum_ok(line):
                cnt["checksum_ruim"] += 1
                continue
            if line[3:6] == "RMC":
                if nmea.parse_rmc(line) is not None:
                    cnt["rmc_A"] += 1
                else:
                    cnt["rmc_V"] += 1
            r = nmea.parse_gga(line)
            if r is not None:
                cnt["gga"] += 1
                gga = r
    finally:
        try:
            g.close()
        except Exception:
            pass
    rep.put("raw", {"contagem": cnt, "gga": gga})
    rep.add("OK" if cnt["rmc_A"] > 0 else "AVS", "gps raw 10 s", "%s" % cnt)
    if gga is not None:
        rep.add("INFO", "GGA", "qualidade %s, satelites %s, HDOP %s" % (gga["quality"], gga["sats"], gga["hdop"]))


def sec_laco(ctx):
    ctx.need_boat()
    b, rep, cfg = ctx.b, ctx.rep, ctx.cfg
    hz = float(cfg.require("loop.hz"))
    step("Laco completo (ler, navegar, logar, motores em 0) a %.0f Hz por 15 s. Pode segurar o barco." % hz)
    try:
        nav = Navigator(cfg)
    except Exception as e:
        nav = None
        rep.add("AVS", "navegador", "sem navegacao (%s)" % e)
    if b.gps_th is not None:
        b.gps_th.pop_new()
    logger = CsvLogger(LOGS, "teste_mao_laco", {"config": cfg.data, "backend": "real", "teste": "laco"})
    rate = Rate(hz, b.clock)
    work = []
    t0 = time.time()
    try:
        while time.time() - t0 < 15.0:
            w0 = time.time()
            s = b.read()
            row = sample_row(s)
            txt = ""
            if nav is not None:
                st = nav.update(s)
                row.update(nav_row(st))
                if st.theta is not None:
                    txt = "rumo %5.1f " % st.compass_deg
            l, r = b.set_motors(0, 0)
            row.update(cmd_l=l, cmd_r=r, loop_dt=rate.dt, step="teste_laco")
            logger.write(row)
            work.append(time.time() - w0)
            sys.stdout.write("\r   %s dt %.3f s   " % (txt, rate.dt))
            sys.stdout.flush()
            rate.sleep()
    finally:
        logger.close()
    print()
    mw = sum(work) / len(work)
    over = rate.overruns / float(max(1, rate.ticks))
    rep.add("OK" if over < 0.05 else "AVS", "laco %.0f Hz" % hz,
            "%d voltas, overruns %.0f%%, dt max %.3f s" % (rate.ticks, 100 * over, rate.max_dt))
    rep.add("INFO", "custo da volta", "media %.1f ms, max %.1f ms -> ate ~%.0f Hz possivel"
            % (1000 * mw, 1000 * max(work), 1.0 / max(mw, 1e-6)))
    rep.add("INFO", "log", os.path.relpath(logger.path, ROOT))
    rep.put("loop", {"hz": hz, "ticks": rate.ticks, "overruns": rate.overruns, "max_dt": rate.max_dt,
                     "work_mean_s": mw, "work_max_s": max(work)})


def sec_motores(ctx):
    ctx.need_motors()
    b, rep, cfg = ctx.b, ctx.rep, ctx.cfg
    V = ctx.need_V()
    lo, hi = b.limits
    rep.add("INFO", "limites", "%d a %d, V = %d" % (lo, hi, V))
    lr = {"e": "esquerda", "d": "direita", "a": "as duas", "n": "nenhuma"}

    def sub(title, fn):
        try:
            print("\n -- %s" % title)
            fn()
        except Skip:
            rep.add("PULADO", title, "")
        finally:
            ctx.stop()

    def esq():
        step("   Esquerdo sozinho: (%d, 0) por %.0f s." % (V, PULSE_S))
        pulse(ctx, V, 0)
        a = choice("   Qual helice girou?", lr)
        rep.add("OK" if a == "e" else "ERR", "so esquerdo", "girou: %s" % lr[a])

    def dire():
        step("   Direito sozinho: (0, %d) por %.0f s." % (V, PULSE_S))
        pulse(ctx, 0, V)
        a = choice("   Qual helice girou?", lr)
        rep.add("OK" if a == "d" else "ERR", "so direito", "girou: %s" % lr[a])

    def ambos():
        step("   Os dois: (%d, %d) por %.0f s (leio tambem o comando aplicado no ESC)." % (V, V, PULSE_S))
        esc = []

        def query():
            # em outra thread: a leitura pode bloquear ate 1 s e o laco continua reenviando
            try:
                ser = b.ard.arduino
                if hasattr(ser, "reset_input_buffer"):
                    ser.reset_input_buffer()
                esc.append(b.ard.get_arduino_cmd_motor_esc())
            except Exception as e:
                esc.append("erro %r" % e)

        q = None
        try:
            t0 = time.time()
            while time.time() - t0 < PULSE_S:
                b.set_motors(V, V)
                if q is None and time.time() - t0 > 0.5:
                    q = threading.Thread(target=query)
                    q.daemon = True
                    q.start()
                time.sleep(0.1)
        finally:
            b.stop()
        if q is not None:
            q.join(1.5)
        esc = esc[0] if esc else None
        rep.add("INFO", "resposta ESC (Z;)", repr(esc))
        rep.add("OK" if yn("   As duas giraram com forca parecida?") else "AVS", "os dois", "")
        a = choice("   O ar sai para TRAS (sentido de avanco)?", {"s": "sim", "n": "nao", "x": "nao sei"})
        rep.add("INFO", "sentido", {"s": "para tras (ok)", "n": "para frente (helice invertida?)",
                                     "x": "nao deu para sentir"}[a])

    def mixer():
        mx = make_mixer(cfg, b)
        T = V / 2.0
        l, r = mx.mix(T, T)
        step("   Mixer 'virar a ESQUERDA' (giro > 0, turn_sign = %d) -> comando (%d, %d)."
             % (mx.turn_sign, l, r))
        pulse(ctx, l, r)
        a = choice("   Qual helice girou MAIS forte?", {"e": "esquerda", "d": "direita", "i": "iguais"})
        exp = "d" if r > l else ("e" if l > r else "i")
        ok = (a == exp == "d")
        rep.add("OK" if ok else "ERR", "mixer esquerda", "comando (%d, %d), mais forte: %s" % (l, r, a))
        if not ok:
            rep.rec("conferir real.turn_sign e a ligacao dos motores (giro a esquerda deveria "
                    "acelerar a helice DIREITA)")

    def re():
        step("   Re (direto no Arduino, fora dos limites da biblioteca): (%d, %d) por %.0f s." % (-V, -V, PULSE_S))
        pulse(ctx, -V, -V, raw=True)
        a = choice("   Giraram ao CONTRARIO do teste dos dois?", {"s": "sim", "n": "nao", "x": "nao sei"})
        rep.add("INFO", "re", {"s": "funciona", "n": "nao funciona", "x": "inconclusivo"}[a])
        if a == "s" and not cfg.require("real.motors.allow_reverse"):
            rep.rec("re funciona: real.motors.allow_reverse = true libera curvas mais fortes")

    def watchdog():
        step("   Watchdog: envio (%d, %d) UMA vez e paro de reenviar. Deve parar sozinho em ~%.1f s."
             % (V, V, b.watchdog_s))
        n0 = b.watchdog_trips
        b.set_motors(V, V)
        countdown(b.watchdog_s + 2.0, "observe")
        trips = b.watchdog_trips - n0
        a = yn("   Pararam sozinhos ~%.1f s depois?" % b.watchdog_s)
        rep.add("OK" if (trips > 0 and a) else "ERR", "watchdog", "disparou %d vez(es), visto parar: %s"
                % (trips, "sim" if a else "nao"))

    def interferencia():
        step("   Interferencia dos motores na bussola: segure o barco PARADO num rumo fixo,\n"
             "   3 s desligado, 3 s com (%d, %d), 3 s desligado." % (V, V))
        segs = []
        for on in (False, True, False):
            vals = []
            t0 = time.time()
            while time.time() - t0 < 3.0:
                if on:
                    b.set_motors(V, V)
                vals.append(b.read_imu()[0])
            b.stop()
            segs.append(mean3(vals))
        dm = la.norm([segs[1][k] - (segs[0][k] + segs[2][k]) / 2.0 for k in range(3)])
        msg = "|delta mag| = %.0f LSB" % dm
        st = "INFO"
        try:
            c = ctx.need_mag2d()
            hs = [mag2d.heading_compass_deg(s[0], s[1], c) for s in segs]
            dh = wrap_180(hs[1] - circular_mean_deg([hs[0], hs[2]]))
            msg += ", rumo desligado %.1f / ligado %.1f / desligado %.1f -> %+.1f graus" % (hs[0], hs[1], hs[2], dh)
            st = "OK" if abs(dh) < 3 else ("AVS" if abs(dh) < 10 else "ERR")
            if st != "OK":
                rep.rec("motores desviam a bussola %.0f graus em V=%d: afastar cabos da IMU ou "
                        "compensar com o comando" % (dh, V))
        except Skip:
            msg += " (sem mag2d para converter em rumo)"
        rep.add(st, "interferencia", msg)
        rep.put("interferencia", segs)

    sub("esquerdo", esq)
    sub("direito", dire)
    sub("os dois", ambos)
    sub("mixer", mixer)
    sub("re", re)
    sub("watchdog", watchdog)
    sub("interferencia", interferencia)


def sec_controle(ctx):
    ctx.need_motors()
    ctx.need_mag2d()
    b, rep, cfg = ctx.b, ctx.rep, ctx.cfg
    V = ctx.need_V()
    T = V / 2.0
    g = lambda k: float(cfg.require("control.heading." + k))
    turn_max = min(g("turn_max"), T)
    hc = HeadingController(g("kp"), g("ki"), g("kd"), turn_max, g("i_max"), g("tau_d_s"))
    mx = make_mixer(cfg, b)
    nav = Navigator(cfg)
    if ctx.gyro_cal is not None:
        nav.set_gyro_cal(ctx.gyro_cal)
    ref = compass_to_theta(90.0)
    step("Controle de rumo em malha fechada, alvo = LESTE (90), empuxo %.0f, giro max %.0f\n"
         "   (limitados por V para o teste na mao)." % (T, turn_max))
    logger = CsvLogger(LOGS, "teste_mao_rumo", {"config": cfg.data, "backend": "real", "teste": "rumo",
                                                "thrust": T, "turn_max": turn_max})
    hz = float(cfg.require("loop.hz"))

    def run(dur, label):
        hc.reset()
        rate = Rate(hz, b.clock)
        ls, rs = [], []
        t0 = time.time()
        try:
            while time.time() - t0 < dur:
                s = b.read()
                st = nav.update(s)
                turn = hc.update(ref, st.theta, rate.dt, st.r)
                l, r = b.set_motors(*mx.mix(T, turn))
                ls.append(l)
                rs.append(r)
                row = sample_row(s)
                row.update(nav_row(st))
                row.update(step=label, thrust=T, turn=turn, cmd_l=l, cmd_r=r, loop_dt=rate.dt,
                           theta_ref_deg=math.degrees(ref))
                logger.write(row)
                sys.stdout.write("\r   rumo %5.1f  giro %+6.1f  cmd (%3d, %3d)   " % (st.compass_deg, turn, l, r))
                sys.stdout.flush()
                rate.sleep()
        finally:
            b.stop()
            print()
        return sum(ls) / len(ls), sum(rs) / len(rs)

    try:
        for nome, exp in (("NORTE", "e"), ("SUL", "d")):
            ask("   Aponte a PROA para o %s e segure. Os motores ligam por 4 s. [Enter] " % nome)
            ml, mr = run(4.0, "rumo_" + nome.lower())
            a = choice("   Qual helice girou MAIS forte?", {"e": "esquerda", "d": "direita", "i": "iguais"})
            calc = "e" if ml > mr else ("d" if mr > ml else "i")
            ok = a == exp and calc == exp
            rep.add("OK" if ok else "ERR", "proa %s" % nome,
                    "media cmd (%.0f, %.0f), esperado mais forte: %s, visto: %s"
                    % (ml, mr, "esquerda" if exp == "e" else "direita", a))
        if yn("   Fazer 15 s livres (gire devagar e veja os motores corrigirem)?"):
            ask("   [Enter] para comecar ")
            run(15.0, "rumo_livre")
            rep.add("OK" if yn("   Os motores corrigiam para o LESTE?") else "AVS", "rumo livre", "")
    finally:
        logger.close()
        rep.add("INFO", "log", os.path.relpath(logger.path, ROOT))


def sec_parada(ctx):
    rep, cfg = ctx.rep, ctx.cfg
    V = ctx.need_V()
    step("Parada do programa: rodo 'main.py motors %d %d' em outro processo e o derrubo\n"
         "   com SIGHUP (como queda do SSH) e depois com kill -9." % (V, V))
    full_close(ctx.b)
    ctx.b = None
    logp = os.path.join(LOGS, "teste_mao_parada_%s.txt" % time.strftime("%Y%m%d_%H%M%S"))
    main_py = os.path.join(ROOT, "main.py")

    def launch():
        f = open(logp, "a")
        p = subprocess.Popen([sys.executable, main_py, "motors", str(V), str(V), "--dur", "15"],
                             cwd=ROOT, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        return p, f

    def zero():
        z = RealBoat(cfg)
        z.start(use_imu=False, use_gps=False, use_motors=True)
        shutdown.install()
        full_close(z)

    for sig, nome in ((signal.SIGHUP, "SIGHUP"), (signal.SIGKILL, "kill -9")):
        try:
            step("   %s: os motores ligam ~3 s, depois mando %s." % (nome, nome))
        except Skip:
            rep.add("PULADO", nome, "")
            continue
        p, f = launch()
        try:
            countdown(3.0, "motores ligando")
            p.send_signal(sig)
            t0 = time.time()
            while p.poll() is None and time.time() - t0 < 3.0:
                time.sleep(0.05)
            rc = p.poll()
            if sig == signal.SIGKILL:
                countdown(4.0, "observe se continuam girando")
        finally:
            if p.poll() is None:
                p.kill()
                p.wait()
            f.close()
            zero()
        if sig == signal.SIGHUP:
            a = yn("   Pararam assim que o programa recebeu SIGHUP?")
            rep.add("OK" if (a and rc is not None) else "ERR", "SIGHUP", "processo saiu: %s (rc %s), visto parar: %s"
                    % ("sim" if rc is not None else "nao", rc, "sim" if a else "nao"))
        else:
            a = yn("   Continuaram girando depois do kill -9 (ate eu mandar zero)?")
            if a:
                rep.add("AVS", "kill -9", "motores CONTINUAM com o processo morto")
                rep.rec("firmware nao para sozinho: nunca usar kill -9; parar com Ctrl+C, 'touch data/STOP' ou kill")
            else:
                rep.add("OK", "kill -9", "firmware parou sozinho com o processo morto")
    rep.add("INFO", "log", os.path.relpath(logp, ROOT))


def sec_missao(ctx):
    rep, cfg = ctx.rep, ctx.cfg
    ctx.need_mag2d()
    V = ctx.need_V()
    T = int(V / 2)
    step("Runner de missao (integracao): gyro_bias 5 s + rumo LESTE 20 s, sem geofence,\n"
         "   cmd_max = %d e giro max = %d so neste teste." % (V, T))
    full_close(ctx.b)
    ctx.b = None
    d = copy.deepcopy(cfg.data)
    d["real"]["motors"]["cmd_max"] = V
    d["control"]["heading"]["turn_max"] = min(float(d["control"]["heading"]["turn_max"]), float(T))
    cfg2 = Config(d, cfg.path)
    mission = {"steps": [{"type": "gyro_bias", "duration_s": 5},
                         {"type": "heading", "compass_deg": 90, "thrust": T, "duration_s": 20}]}
    print("   Primeiros 5 s: barco APOIADO e parado. Depois pegue e gire devagar;")
    print("   os motores devem tentar levar a proa para o LESTE. Ctrl+C para no meio.")
    ask("   [Enter] para comecar ")
    try:
        res = run_mission(cfg2, RealBoat(cfg2), mission, tag="teste_mao_missao", allow_no_geofence=True)
    except Exception as e:
        rep.add("ERR", "runner", "%s: %s" % (e.__class__.__name__, e))
        return
    print()
    rep.add("OK" if res.get("ok") else "ERR", "runner", "%s, passos %s" % (res.get("reason"), res.get("steps_done")))
    if res.get("log"):
        rep.add("INFO", "log", os.path.relpath(res["log"], ROOT))
    if res.get("ok"):
        rep.add("OK" if yn("   Os motores corrigiam para o LESTE?") else "ERR", "rumo no runner", "")


SECTIONS = [
    ("IMU parada", sec_imu_parado),
    ("Picos de 256 / BDU", sec_bdu),
    ("Eixos da IMU", sec_eixos),
    ("Calibracao mag2d", sec_calib_mag2d),
    ("Validacao do rumo x celular", sec_valida_rumo),
    ("Rumo x inclinacao", sec_inclinacao),
    ("Giro x magnetometro", sec_giro),
    ("GPS", sec_gps),
    ("Laco completo", sec_laco),
    ("Motores", sec_motores),
    ("Controle de rumo na mao", sec_controle),
    ("Parada (SIGHUP, kill -9)", sec_parada),
    ("Runner de missao", sec_missao),
]


def choose():
    print("\nSecoes:")
    for i, (n, _) in enumerate(SECTIONS):
        print("  %2d  %s" % (i + 1, n))
    while True:
        s = ask("Enter = todas, ou numeros separados por virgula (ex.: 1,4,10): ").replace(" ", "")
        if not s:
            return list(range(len(SECTIONS)))
        try:
            idx = sorted(set(int(x) - 1 for x in s.split(",") if x))
        except ValueError:
            print("Entrada invalida.")
            continue
        if all(0 <= i < len(SECTIONS) for i in idx):
            return idx
        print("Numero fora da lista.")


def run_section(ctx, name, fn):
    ctx.rep.sec = name
    print("\n==================== %s" % name)
    try:
        fn(ctx)
    except Skip as e:
        ctx.rep.add("PULADO", name, str(e))
    except (shutdown.StopRequested, KeyboardInterrupt):
        raise
    except Exception as e:
        ctx.rep.add("ERR", "excecao", "%s: %s" % (e.__class__.__name__, e))
    finally:
        ctx.stop()


def main():
    cfg = Config.load(sys.argv[1] if len(sys.argv) > 1 else None)
    ensure_dirs()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    rep = Report()
    ctx = Ctx(cfg, rep)
    interrupted = False
    print("Teste na mao do DDBoat. Ctrl+C a qualquer momento zera os motores e salva o relatorio.")
    try:
        run_section(ctx, "Ambiente", sec_ambiente)
        run_section(ctx, "Arranque", sec_arranque)
        for i in choose():
            run_section(ctx, SECTIONS[i][0], SECTIONS[i][1])
    except (shutdown.StopRequested, KeyboardInterrupt):
        interrupted = True
        print("\nInterrompido.")
    finally:
        ctx.stop()
        full_close(ctx.b)
        base = rep.save(stamp, interrupted)
    cnt = {}
    for it in rep.items:
        cnt[it["status"]] = cnt.get(it["status"], 0) + 1
    print("\nMotores parados. Resumo: %s" % ", ".join("%s=%d" % (k, cnt[k]) for k in sorted(cnt)))
    for it in rep.items:
        if it["status"] in ("ERR", "AVS"):
            print("  [%s] %s / %s: %s" % (it["status"], it["secao"], it["item"], it["msg"]))
    if rep.recs:
        print("Recomendacoes:")
        for r in rep.recs:
            print("  - " + r)
    print("Relatorio: %s.txt e .json" % os.path.relpath(base, ROOT))
    return 1 if interrupted else 0


if __name__ == "__main__":
    sys.exit(main())
