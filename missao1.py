# -*- coding: utf-8 -*-
"""missao1.py - DDBoat 04: linha reta + uma volta sobre si mesmo.

Script AUTONOMO: nao importa o pacote ddboat/ (biblioteca ainda nao validada).
Usa so a biblioteca padrao do Python (3.5+) e os drivers da ENSTA, com as
mesmas chamadas e a mesma calibracao do cap_nord.py, que ja funcionaram no barco.

Convencao (a mesma aprovada para a biblioteca):
  ENU: x = leste, y = norte. theta em rad, anti-horario a partir do leste.
  theta = 90 graus - rumo_bussola.  cos(theta) = componente leste da proa,
  sin(theta) = componente norte. Bussola (0 = N, horario) so na tela e no log.

Automato da missao:   RETA -> GIRO -> FIM
  RETA: segura o rumo medido na partida por T segundos (controle P + mixer).
  GIRO: um so motor (sem re no lago). Sub-automato de 4 estados pelos sinais
        de (cos theta, sin theta):
            Q1 = (+,+) proa NE     Q2 = (-,+) proa NO
            Q3 = (-,-) proa SO     Q4 = (+,-) proa SE
        Giro a esquerda (anti-horario): Q1 -> Q2 -> Q3 -> Q4 -> Q1
        Giro a direita  (horario):      Q1 -> Q4 -> Q3 -> Q2 -> Q1
        Troca de estado so quando a componente que muda de sinal passa de
        sin(histerese) do outro lado (ruido na fronteira nao troca estado).
        Transicao no sentido contrario desconta 1 (balanco nao conta volta).
        Com 4 transicoes liquidas o barco voltou ao quadrante inicial; dai ele
        gira ate passar pelo rumo inicial e para (volta >= 360 graus).
  FIM:  motores em 0.

Uso no barco (cd ~/DDBOAT):
  python3 missao1.py calib            calibracao 2D (identica ao cap_nord)
  python3 missao1.py check            SEM motor: gire o barco com a mao e veja
                                      quadrante e transicoes do automato
  python3 missao1.py run [opcoes]     missao com motores
No PC (sem barco):
  python3 missao1.py sim [opcoes]     mesma logica no modelo de 8 estados do Jaulin

Opcoes: --reta-s 20  --empuxo 120  --kp 60  --giro-max 90  --giro-cmd 120
        --sentido esq|dir  --hist-deg 10  --giro-timeout-s 60  --hz 10
        --calib ARQ  --drivers PASTA
Parar: Ctrl+C, ou por outro SSH: touch data/STOP
"""
from __future__ import print_function

import json
import math
import os
import random
import signal
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
LOGS = os.path.join(DATA, "logs")
STOP_FILE = os.path.join(DATA, "STOP")

CMD_MAX = 191           # maximo efetivo no lago (apresentacao de 15/10)
CMD_MIN = 0             # sem marcha a re no lago

DRIVER_DIRS = [os.path.join(HERE, "src", "DriversDDBOAT_2026"),
               os.path.expanduser("~/GKKDossier/drivers-ddboat-v2")]
CALIB_FILES = [os.path.join(DATA, "calib", "mag2d.json"),
               os.path.join(HERE, "calib_cap.json")]

DEFAULTS = {"reta-s": 20.0, "empuxo": 120.0, "kp": 60.0, "giro-max": 90.0,
            "giro-cmd": 120.0, "sentido": "esq", "hist-deg": 10.0,
            "giro-timeout-s": 60.0, "hz": 10.0, "calib": None, "drivers": None}


# ----------------------------------------------------------------- angulos
def wrap_pi(a):
    """Angulo em rad no intervalo [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def compass_to_theta(h_deg):
    return wrap_pi(math.radians(90.0 - h_deg))


def theta_to_compass(theta):
    return (90.0 - math.degrees(theta)) % 360.0


# ------------------------------------------------- automato dos quadrantes
QUAD_NAMES = {1: "Q1(+,+)", 2: "Q2(-,+)", 3: "Q3(-,-)", 4: "Q4(+,-)"}


def quadrant_of(c, s):
    """Quadrante pelos sinais de (cos, sin). Zero conta como positivo."""
    if c >= 0.0 and s >= 0.0:
        return 1
    if c < 0.0 and s >= 0.0:
        return 2
    if c < 0.0:
        return 3
    return 4


class QuadrantAutomaton(object):
    """4 estados Q1..Q4. direction = +1 (esquerda, anti-horario) ou -1 (direita).

    Saidas de cada estado (s = sin(histerese)):
        Q1 -> Q2 se cos < -s      Q1 -> Q4 se sin < -s
        Q2 -> Q3 se sin < -s      Q2 -> Q1 se cos >  s
        Q3 -> Q4 se cos >  s      Q3 -> Q2 se sin >  s
        Q4 -> Q1 se sin >  s      Q4 -> Q3 se cos < -s
    Anti-horario = proximo numero (Q4 -> Q1). Horario = numero anterior.
    """

    def __init__(self, theta0, direction, hyst_deg):
        if direction not in (1, -1):
            raise ValueError("direction deve ser +1 ou -1")
        self.direction = direction
        self.s = math.sin(math.radians(hyst_deg))
        self.q0 = quadrant_of(math.cos(theta0), math.sin(theta0))
        self.q = self.q0
        self.net = 0           # transicoes liquidas no sentido pedido
        self.history = [self.q0]

    def _ccw_exit(self, c, s):
        h = self.s
        return {1: c < -h, 2: s < -h, 3: c > h, 4: s > h}[self.q]

    def _cw_exit(self, c, s):
        h = self.s
        return {1: s < -h, 2: c > h, 3: s > h, 4: c < -h}[self.q]

    def update(self, theta):
        """Devolve +1/-1 se trocou de estado no sentido pedido/contrario, senao 0."""
        c, s = math.cos(theta), math.sin(theta)
        step = 0
        if self._ccw_exit(c, s):
            self.q = self.q % 4 + 1
            step = 1
        elif self._cw_exit(c, s):
            self.q = (self.q - 2) % 4 + 1
            step = -1
        if step == 0:
            return 0
        self.history.append(self.q)
        d = step * self.direction
        self.net += d
        return d

    def back_in_start(self):
        return self.net >= 4 and self.q == self.q0


def passed_start(theta, theta0, direction):
    """Depois das 4 transicoes: True quando a proa alcanca/passa theta0."""
    e = wrap_pi(theta - theta0)
    return direction * e >= 0.0 and math.cos(e) > 0.0


# ------------------------------------------------------------------ mixer
def mix(thrust, turn):
    """(empuxo, giro) -> (esq, dir) em [0, 191]. giro > 0 = esquerda (dir mais forte).
    Prioridade para o giro: a saturacao corta empuxo, nunca a curva."""
    half = (CMD_MAX - CMD_MIN) / 2.0
    d = max(-half, min(half, turn))
    t = max(CMD_MIN + abs(d), min(CMD_MAX - abs(d), thrust))
    return int(round(t - d)), int(round(t + d))


# ------------------------------------------------------------- calibracao
def raw_angle(mx, my, c):
    x = (mx - c["offx"]) * c["sx"]
    y = (my - c["offy"]) * c["sy"]
    return math.degrees(math.atan2(y, x))


def compass_heading(m, c):
    """Rumo de bussola em graus [0, 360), igual ao cap_nord.heading()."""
    return (c["sign"] * wrap180(raw_angle(m[0], m[1], c) - c["north"])) % 360.0


def find_file(user_path, candidates, what):
    paths = [user_path] if user_path else candidates
    for p in paths:
        if p and os.path.isfile(os.path.expanduser(p)):
            return os.path.expanduser(p)
    print("ERRO: %s nao encontrado. Procurei em:" % what)
    for p in paths:
        print("   ", p)
    sys.exit(1)


def load_calib(path):
    with open(path) as f:
        c = json.load(f)
    for k in ("offx", "offy", "sx", "sy", "north", "sign"):
        if k not in c:
            print("ERRO: %s sem a chave %s" % (path, k))
            sys.exit(1)
    return c


# ------------------------------------------------------------ hardware real
def load_drivers(user_dir):
    """Importa os drivers da ENSTA sem edita-los. O imu9_driver_v2 faz
    'import drivers.i2creal'; se esse pacote nao existir, ele e criado em
    memoria apontando para o i2creal.py da mesma pasta."""
    cands = [user_dir] if user_dir else DRIVER_DIRS
    d = None
    for p in cands:
        p = os.path.abspath(os.path.expanduser(p))
        if os.path.isfile(os.path.join(p, "imu9_driver_v2.py")):
            d = p
            break
    if d is None:
        print("ERRO: drivers nao encontrados em:", cands)
        sys.exit(1)
    if d not in sys.path:
        sys.path.insert(0, d)
    try:
        import drivers.i2creal  # noqa: F401
    except ImportError:
        import i2creal
        pkg = types.ModuleType("drivers")
        pkg.__path__ = []
        pkg.i2creal = i2creal
        sys.modules["drivers"] = pkg
        sys.modules["drivers.i2creal"] = i2creal
    import imu9_driver_v2
    print("Drivers:", d)
    return imu9_driver_v2


class RealBoat(object):
    def __init__(self, args, motors):
        self.calib_path = find_file(args["calib"], CALIB_FILES, "arquivo de calibracao")
        self.c = load_calib(self.calib_path)
        imu_mod = load_drivers(args["drivers"])
        self.imu = imu_mod.Imu9IO()
        self.ard = None
        if motors:
            import arduino_driver_v2
            self.ard = arduino_driver_v2.ArduinoIO()
            if self.ard.arduino is None:
                print("ERRO: Arduino nao inicializado.")
                sys.exit(1)
        self.t0 = time.time()

    def now(self):
        return time.time() - self.t0

    def sleep(self, dt):
        if dt > 0:
            time.sleep(dt)

    def read_mag(self):
        """Mediana de 3 leituras por eixo (igual ao cap_nord)."""
        s = []
        for _ in range(3):
            s.append(self.imu.read_mag_raw())
            time.sleep(0.01)
        return [sorted([v[i] for v in s])[1] for i in range(3)]

    def heading(self):
        """Devolve (theta ENU rad, rumo bussola graus, mag cru)."""
        m = self.read_mag()
        h = compass_heading(m, self.c)
        return compass_to_theta(h), h, m

    def motors(self, left, right):
        if self.ard is not None:
            self.ard.send_arduino_cmd_motor(int(left), int(right))

    def stop(self):
        if self.ard is not None:
            for _ in range(3):
                try:
                    self.ard.send_arduino_cmd_motor(0, 0)
                except Exception:
                    pass
                time.sleep(0.05)


# --------------------------------------------------------------- simulador
class SimBoat(object):
    """Modelo de 8 estados do polycopie (05/07__ddboat), p2 = 220,
    comando -> tensao u = cmd * 5/191 (mesmo mapa do config/boat04.json).
    O rumo medido tem ruido gaussiano. Nao representa o barco real com precisao:
    serve para validar a logica do automato, nao os ganhos."""

    P = (0.07, 220.0, 3e-05, 15e-05, 0.4, 5.0, 5.0)
    U_PER_CMD = 0.02618

    def __init__(self, theta0_deg=30.0, noise_deg=3.0, asym=0.9, seed=1):
        self.x = [0.0, 0.0, math.radians(theta0_deg), 0.0, 0.0, 0.0, 0.0, 0.0]
        self.u = (0.0, 0.0)
        self.t = 0.0
        self.noise = noise_deg
        self.asym = asym          # motor esquerdo mais fraco: reta em malha aberta curva
        self.rng = random.Random(seed)
        self.calib_path = "(simulador)"
        self.traj = []

    def now(self):
        return self.t

    def _f(self, x):
        p1, p2, p3, p4, p5, p6, p7 = self.P
        _, _, th, vx, vy, w, w1, w2 = x
        return [vx * math.cos(th) - vy * math.sin(th),
                vx * math.sin(th) + vy * math.cos(th),
                w,
                w * vy - p5 * vx * abs(vx) + p3 * (w1 * abs(w1) + w2 * abs(w2)),
                -w * vx - p6 * vy * abs(vy),
                -p7 * w * abs(w) + p4 * (w2 * abs(w2) - w1 * abs(w1)),
                -p1 * w1 * abs(w1) + p2 * self.u[0],
                -p1 * w2 * abs(w2) + p2 * self.u[1]]

    def sleep(self, dt):
        n = max(1, int(round(dt / 0.005)))
        h = dt / n
        for _ in range(n):
            d = self._f(self.x)
            self.x = [a + h * b for a, b in zip(self.x, d)]
        self.x[2] = wrap_pi(self.x[2])
        self.t += dt
        self.traj.append((self.t, self.x[0], self.x[1], self.x[2]))

    def heading(self):
        th = wrap_pi(self.x[2] + math.radians(self.rng.gauss(0.0, self.noise)))
        return th, theta_to_compass(th), [0, 0, 0]

    def motors(self, left, right):
        self.u = (self.asym * left * self.U_PER_CMD, right * self.U_PER_CMD)

    def stop(self):
        self.u = (0.0, 0.0)


# ----------------------------------------------------------------- parada
class StopRequested(Exception):
    pass


def install_signals():
    def handler(sig, frame):
        raise StopRequested("sinal %d" % sig)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, handler)


def stop_file_present():
    return os.path.isfile(STOP_FILE)


# ------------------------------------------------------------------- log
def new_log_path(prefix):
    if not os.path.isdir(LOGS):
        os.makedirs(LOGS)
    i = 1
    while True:
        p = os.path.join(LOGS, "%s_%03d.csv" % (prefix, i))
        if not os.path.exists(p):
            return p
        i += 1


LOG_COLS = ["t_s", "estado", "quadrante", "transicoes", "rumo_deg", "theta_deg",
            "cos", "sin", "alvo_rumo_deg", "erro_deg", "giro_acum_deg",
            "cmd_esq", "cmd_dir", "mag_x", "mag_y", "mag_z"]


# ---------------------------------------------------------------- missao
def mean_theta(boat, n=10, dt=0.05):
    sc = ss = 0.0
    for _ in range(n):
        th, _, _ = boat.heading()
        sc += math.cos(th)
        ss += math.sin(th)
        boat.sleep(dt)
    return math.atan2(ss, sc)


def run_mission(boat, a, log_path, verbose=True):
    """Executa RETA -> GIRO -> FIM. Devolve um dicionario com o resultado."""
    period = 1.0 / a["hz"]
    direction = 1 if a["sentido"] == "esq" else -1
    res = {"fim": None, "giro_acum_deg": 0.0, "historico": [], "log": log_path}

    f = open(log_path, "w")
    f.write("# missao1 calib=%s args=%s\n" % (boat.calib_path, json.dumps(a, sort_keys=True)))
    f.write(",".join(LOG_COLS) + "\n")

    def log(row):
        f.write(",".join(str(row[k]) for k in LOG_COLS) + "\n")

    state = "RETA"
    theta_ref = mean_theta(boat)
    if verbose:
        print("Rumo alvo da reta: %.1f graus (bussola)" % theta_to_compass(theta_ref))
    auto = None
    theta_prev = None
    acc = 0.0
    t_state = boat.now()
    try:
        while True:
            t_loop = boat.now()
            if stop_file_present():
                res["fim"] = "arquivo STOP"
                break
            theta, h, m = boat.heading()
            err = wrap_pi(theta_ref - theta)
            left = right = 0

            if state == "RETA":
                turn = max(-a["giro-max"], min(a["giro-max"], a["kp"] * err))
                left, right = mix(a["empuxo"], turn)
                if boat.now() - t_state >= a["reta-s"]:
                    state = "GIRO"
                    t_state = boat.now()
                    theta_ref = theta          # rumo no inicio do giro
                    err = 0.0
                    auto = QuadrantAutomaton(theta_ref, direction, a["hist-deg"])
                    theta_prev = theta
                    acc = 0.0
                    if verbose:
                        print("\n-> GIRO, rumo inicial %.1f, estado %s"
                              % (h, QUAD_NAMES[auto.q0]))

            if state == "GIRO":
                acc += math.degrees(wrap_pi(theta - theta_prev))
                theta_prev = theta
                d = auto.update(theta)
                if d != 0 and verbose:
                    print("\n   %s  transicoes liquidas = %d  giro acumulado = %.0f"
                          % (QUAD_NAMES[auto.q], auto.net, acc))
                if auto.back_in_start() and passed_start(theta, theta_ref, direction):
                    state = "FIM"
                    res["fim"] = "volta completa"
                elif boat.now() - t_state > a["giro-timeout-s"]:
                    state = "FIM"
                    res["fim"] = "TIMEOUT do giro"
                else:
                    g = int(a["giro-cmd"])
                    left, right = (0, g) if direction == 1 else (g, 0)

            if state == "FIM":
                left = right = 0
            boat.motors(left, right)
            log({"t_s": "%.3f" % boat.now(), "estado": state,
                 "quadrante": auto.q if auto else quadrant_of(math.cos(theta), math.sin(theta)),
                 "transicoes": auto.net if auto else 0,
                 "rumo_deg": "%.1f" % h, "theta_deg": "%.1f" % math.degrees(theta),
                 "cos": "%.3f" % math.cos(theta), "sin": "%.3f" % math.sin(theta),
                 "alvo_rumo_deg": "%.1f" % theta_to_compass(theta_ref),
                 "erro_deg": "%.1f" % math.degrees(err), "giro_acum_deg": "%.1f" % acc,
                 "cmd_esq": left, "cmd_dir": right,
                 "mag_x": m[0], "mag_y": m[1], "mag_z": m[2]})
            if verbose:
                sys.stdout.write("\r%-4s t=%5.1f rumo=%5.1f erro=%6.1f cmd=(%3d,%3d)   "
                                 % (state, boat.now(), h, math.degrees(err), left, right))
                sys.stdout.flush()
            if state == "FIM":
                break
            boat.sleep(period - (boat.now() - t_loop))
    except StopRequested as e:
        res["fim"] = "interrompido (%s)" % e
    finally:
        boat.stop()
        f.close()
    res["giro_acum_deg"] = acc
    res["historico"] = [QUAD_NAMES[q] for q in auto.history] if auto else []
    if verbose:
        print("\nFim: %s. Giro acumulado = %.0f graus. Estados: %s"
              % (res["fim"], acc, " -> ".join(res["historico"])))
        print("Motores parados. Log:", log_path)
    return res


# ---------------------------------------------------------------- modos
def mode_check(a):
    """Sem motores: gire o barco com a mao. Mostra quadrante e automato."""
    boat = RealBoat(a, motors=False)
    direction = 1 if a["sentido"] == "esq" else -1
    theta0, h0, _ = boat.heading()
    auto = QuadrantAutomaton(theta0, direction, a["hist-deg"])
    print("Calibracao:", boat.calib_path)
    print("Gire o barco para a %s. Motores NAO sao ligados. Ctrl+C para sair."
          % ("ESQUERDA (anti-horario)" if direction == 1 else "DIREITA (horario)"))
    print("Inicio: rumo %.1f, estado %s" % (h0, QUAD_NAMES[auto.q0]))
    install_signals()
    try:
        while True:
            th, h, _ = boat.heading()
            d = auto.update(th)
            if d != 0:
                print("\n   %s  transicoes liquidas = %d" % (QUAD_NAMES[auto.q], auto.net))
            if auto.back_in_start() and passed_start(th, theta0, direction):
                print("\nVOLTA COMPLETA detectada. Reiniciando o automato.")
                theta0 = th
                auto = QuadrantAutomaton(theta0, direction, a["hist-deg"])
            sys.stdout.write("\rrumo=%5.1f theta=%6.1f cos=%+.2f sin=%+.2f %s   "
                             % (h, math.degrees(th), math.cos(th), math.sin(th),
                                QUAD_NAMES[auto.q]))
            sys.stdout.flush()
            time.sleep(0.05)
    except StopRequested:
        print("\nSaindo.")


def mode_run(a):
    if os.path.isfile(STOP_FILE):
        print("ERRO: existe %s. Apague antes (rm data/STOP)." % STOP_FILE)
        sys.exit(1)
    boat = RealBoat(a, motors=True)
    install_signals()            # depois do ArduinoIO, que instala o proprio SIGINT
    boat.stop()
    print("Calibracao:", boat.calib_path)
    print("Reta %.0f s (empuxo %d), depois volta para a %s com comando %d."
          % (a["reta-s"], a["empuxo"], a["sentido"], a["giro-cmd"]))
    run_mission(boat, a, new_log_path("missao1"))


def mode_sim(a):
    boat = SimBoat()
    install_signals()
    res = run_mission(boat, a, new_log_path("missao1_sim"))
    return boat, res


def mode_calib(a):
    """Calibracao 2D identica ao cap_nord.py (min/max + norte/leste)."""
    imu_mod = load_drivers(a["drivers"])
    imu = imu_mod.Imu9IO()
    out = a["calib"] or CALIB_FILES[0]

    def read_mag():
        s = []
        for _ in range(3):
            s.append(imu.read_mag_raw())
            time.sleep(0.01)
        return [sorted([v[i] for v in s])[1] for i in range(3)]

    def mean_raw(c, n=20):
        ss = cc = 0.0
        for _ in range(n):
            m = read_mag()
            r = math.radians(raw_angle(m[0], m[1], c))
            ss += math.sin(r)
            cc += math.cos(r)
        return math.degrees(math.atan2(ss, cc))

    def wait(msg):
        print(msg)
        sys.stdout.flush()
        sys.stdin.readline()

    wait("Fase 1: barco na horizontal. Enter e gire devagar, pelo menos uma volta, por 20 s.")
    xs, ys = [], []
    t0 = time.time()
    while time.time() - t0 < 20.0:
        m = read_mag()
        xs.append(m[0])
        ys.append(m[1])
        sys.stdout.write("\r%4.1f s  x=[%d, %d]  y=[%d, %d]   "
                         % (time.time() - t0, min(xs), max(xs), min(ys), max(ys)))
        sys.stdout.flush()
    print()
    if max(xs) == min(xs) or max(ys) == min(ys):
        print("ERRO: o magnetometro nao variou. NAO salvo.")
        return
    c = {"offx": (max(xs) + min(xs)) / 2.0, "offy": (max(ys) + min(ys)) / 2.0,
         "sx": 2.0 / (max(xs) - min(xs)), "sy": 2.0 / (max(ys) - min(ys)),
         "north": 0.0, "sign": 1}
    print("Amplitude x = %d, amplitude y = %d (devem ser parecidas)"
          % (max(xs) - min(xs), max(ys) - min(ys)))
    wait("Fase 2: PROA para o NORTE, parado, Enter.")
    c["north"] = mean_raw(c)
    wait("Fase 3: PROA para o LESTE, parado, Enter.")
    d = wrap180(mean_raw(c) - c["north"])
    print("Diferenca norte -> leste: %.1f graus" % d)
    if 60.0 <= d <= 120.0:
        c["sign"] = 1
    elif -120.0 <= d <= -60.0:
        c["sign"] = -1
    else:
        print("ERRO: longe de 90 graus. NAO salvo.")
        return
    if not os.path.isdir(os.path.dirname(out)):
        os.makedirs(os.path.dirname(out))
    with open(out, "w") as f:
        json.dump(c, f, indent=2)
    print("Calibracao salva em", out)


def parse_args(argv):
    a = dict(DEFAULTS)
    i = 0
    while i < len(argv):
        k = argv[i]
        if not k.startswith("--") or k[2:] not in a or i + 1 >= len(argv):
            print("Opcao invalida: %s" % k)
            print(__doc__)
            sys.exit(1)
        v = argv[i + 1]
        key = k[2:]
        if key in ("sentido", "calib", "drivers"):
            a[key] = v
        else:
            try:
                a[key] = float(v)
            except ValueError:
                print("ERRO: %s precisa de numero" % k)
                sys.exit(1)
        i += 2
    if a["sentido"] not in ("esq", "dir"):
        print("ERRO: --sentido esq ou dir")
        sys.exit(1)
    for k in ("empuxo", "giro-cmd"):
        if not 0 <= a[k] <= CMD_MAX:
            print("ERRO: --%s entre 0 e %d" % (k, CMD_MAX))
            sys.exit(1)
    if a["hz"] <= 0 or a["reta-s"] < 0 or a["giro-timeout-s"] <= 0:
        print("ERRO: --hz e --giro-timeout-s > 0, --reta-s >= 0")
        sys.exit(1)
    if not 0 < a["hist-deg"] < 45:
        print("ERRO: --hist-deg entre 0 e 45")
        sys.exit(1)
    return a


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("calib", "check", "run", "sim"):
        print(__doc__)
        sys.exit(1)
    args = parse_args(sys.argv[2:])
    {"calib": mode_calib, "check": mode_check, "run": mode_run, "sim": mode_sim}[sys.argv[1]](args)
