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

CMD_MAX = 191
CMD_MIN = 0
ARDUINO_TTY = "/dev/ttyV0"

DRIVER_DIRS = [os.path.join(HERE, "src", "DriversDDBOAT_2026"),
               os.path.expanduser("~/GKKDossier/drivers-ddboat-v2")]
CALIB_FILES = [os.path.join(DATA, "calib", "mag2d.json"),
               os.path.join(HERE, "calib_cap.json")]

DEFAULTS = {"speed": 120, "kp": 60.0, "turn_max": 90.0, "direction": "left",
            "hysteresis_deg": 10.0, "turn_timeout_s": 60.0, "hz": 10.0,
            "manual_reverse": "yes", "filter": 1, "turn_slew": 0.0,
            "calib": None, "drivers": None, "keys": None}

STOPPED = "PARADO"
LINE = "RETA"
TURN = "GIRO"
MANUAL_STOP = "MANUAL"
FORWARD = "FRENTE"
SPIN_LEFT = "VIRA_E"
SPIN_RIGHT = "VIRA_D"

USAGE = (
    "Uso: python3 mission1.py calib | check | run | sim [opcoes]\n"
    "Opcoes: --speed N  --kp X  --turn-max X  --hysteresis-deg X  --turn-timeout-s X  --hz X\n"
    "        --manual-reverse yes|no  --calib ARQUIVO  --drivers PASTA\n"
    "        --filter N (mediana do rumo em N leituras, N impar, padrao 1 = desligado)\n"
    "        --turn-slew X (variacao maxima da correcao da reta por ciclo, padrao 0 = desligado)\n"
    "        --direction left|right (so no check)  --keys ROTEIRO (so no sim)\n"
    "Teclas do run: r reta | t volta (depois h ou a) | v velocidade | q para\n"
    "               m modo manual (w a d s v, m sai) | Ctrl+C sai")


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def compass_to_theta(h_deg):
    return wrap_pi(math.radians(90.0 - h_deg))


def theta_to_compass(theta):
    return (90.0 - math.degrees(theta)) % 360.0


QUAD_NAMES = {1: "Q1(+,+)", 2: "Q2(-,+)", 3: "Q3(-,-)", 4: "Q4(+,-)"}


def quadrant_of(c, s):
    if c >= 0.0 and s >= 0.0:
        return 1
    if c < 0.0 and s >= 0.0:
        return 2
    if c < 0.0:
        return 3
    return 4


class QuadrantAutomaton(object):
    def __init__(self, theta0, direction, hysteresis_deg):
        if direction not in (1, -1):
            raise ValueError("direction deve ser +1 ou -1")
        self.direction = direction
        self.s = math.sin(math.radians(hysteresis_deg))
        self.q0 = quadrant_of(math.cos(theta0), math.sin(theta0))
        self.q = self.q0
        self.net = 0
        self.history = [self.q0]

    def _ccw_exit(self, c, s):
        h = self.s
        return {1: c < -h, 2: s < -h, 3: c > h, 4: s > h}[self.q]

    def _cw_exit(self, c, s):
        h = self.s
        return {1: s < -h, 2: c > h, 3: s > h, 4: c < -h}[self.q]

    def update(self, theta):
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
    e = wrap_pi(theta - theta0)
    return direction * e >= 0.0 and math.cos(e) > 0.0


def mix(thrust, turn):
    outer = int(round(thrust))
    diff = min(2.0 * abs(turn), float(thrust))
    inner = int(round(thrust - diff))
    return (inner, outer) if turn > 0 else (outer, inner)


def raw_angle(mx, my, c):
    x = (mx - c["offx"]) * c["sx"]
    y = (my - c["offy"]) * c["sy"]
    return math.degrees(math.atan2(y, x))


def compass_heading(m, c):
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


def load_drivers(user_dir):
    candidates = [user_dir] if user_dir else DRIVER_DIRS
    d = None
    for p in candidates:
        p = os.path.abspath(os.path.expanduser(p))
        if os.path.isfile(os.path.join(p, "imu9_driver_v2.py")):
            d = p
            break
    if d is None:
        print("ERRO: drivers nao encontrados em:", candidates)
        sys.exit(1)
    if d not in sys.path:
        sys.path.insert(0, d)
    try:
        __import__("drivers.i2creal")
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
        s = []
        for _ in range(3):
            s.append(self.imu.read_mag_raw())
            time.sleep(0.01)
        return [sorted([v[i] for v in s])[1] for i in range(3)]

    def heading(self):
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


class SimBoat(object):
    P = (0.07, 220.0, 3e-05, 15e-05, 0.4, 5.0, 5.0)
    U_PER_CMD = 0.02618

    def __init__(self, theta0_deg=30.0, noise_deg=3.0, asym=0.9, seed=1):
        self.x = [0.0, 0.0, math.radians(theta0_deg), 0.0, 0.0, 0.0, 0.0, 0.0]
        self.u = (0.0, 0.0)
        self.t = 0.0
        self.noise = noise_deg
        self.asym = asym
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
        if dt <= 0:
            return
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


class StopRequested(Exception):
    pass


def install_signals():
    def handler(sig, frame):
        raise StopRequested("sinal %d" % sig)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, handler)


def ignore_signals():
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, signal.SIG_IGN)


def find_port_users(port):
    target = os.path.realpath(port)
    users = []
    try:
        pids = [p for p in os.listdir("/proc") if p.isdigit()]
    except OSError:
        return users
    for pid in pids:
        if int(pid) == os.getpid():
            continue
        fd_dir = "/proc/%s/fd" % pid
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                link = os.readlink(os.path.join(fd_dir, fd))
            except OSError:
                continue
            if link == target or os.path.realpath(link) == target:
                try:
                    with open("/proc/%s/cmdline" % pid) as f:
                        cmd = f.read().replace("\0", " ").strip()
                except (IOError, OSError):
                    cmd = "?"
                users.append((int(pid), cmd))
                break
    return users


def stop_file_present():
    return os.path.isfile(STOP_FILE)


def new_log_path(prefix):
    if not os.path.isdir(LOGS):
        os.makedirs(LOGS)
    i = 1
    while True:
        p = os.path.join(LOGS, "%s_%03d.csv" % (prefix, i))
        if not os.path.exists(p):
            return p
        i += 1


LOG_COLS = ["t_s", "estado", "quadrante", "transicoes", "rumo_deg", "rumo_filt_deg",
            "theta_deg", "cos", "sin", "alvo_rumo_deg", "erro_deg", "giro_acum_deg", "vel",
            "tecla", "cmd_esq", "cmd_dir", "loop_ms", "read_ms", "mag_x", "mag_y", "mag_z"]


ESC = "\x1b"
DEFAULT_SIM_KEYS = "1:r,10:t,10.2:a,26:r,40:v100<enter>,42:t,42.2:h,58:q"


def key_name(ch):
    if ch.isalnum():
        return ch
    return {"\n": "enter", "\r": "enter", ESC: "esc", "\x7f": "bksp",
            "\x08": "bksp"}.get(ch, "x%02x" % ord(ch))


def parse_keys(buf):
    out = []
    i = 0
    n = len(buf)
    while i < n:
        ch = buf[i]
        if ch == ESC:
            if i + 1 < n and buf[i + 1] in "[O":
                i += 2
                while i < n and (buf[i] in "0123456789;"):
                    i += 1
                i += 1
                continue
            out.append(ESC)
            i += 1
            continue
        out.append(ch.lower())
        i += 1
    return out


class TtyKeys(object):
    def __init__(self):
        import termios
        import tty
        self._termios = termios
        self.fd = sys.stdin.fileno()
        try:
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        except termios.error as e:
            print("ERRO: o terminal nao aceita leitura de teclas (%s)." % e)
            sys.exit(1)
        self.held = ""
        self.closed = False

    def read(self, now):
        import select
        new = ""
        while select.select([self.fd], [], [], 0)[0]:
            chunk = os.read(self.fd, 64)
            if not chunk:
                break
            new += chunk.decode("latin-1")
        data = self.held + new
        self.held = ""
        if new and (data.endswith(ESC) or data.endswith(ESC + "[") or data.endswith(ESC + "O")):
            cut = data.rfind(ESC)
            self.held = data[cut:]
            data = data[:cut]
        return parse_keys(data)

    def finished(self, now):
        return False

    def close(self):
        if not self.closed:
            self.closed = True
            try:
                self._termios.tcsetattr(self.fd, self._termios.TCSADRAIN, self.old)
            except (OSError, self._termios.error):
                pass


class ScriptKeys(object):
    def __init__(self, text):
        self.items = []
        for tok in text.split(","):
            tok = tok.strip()
            if not tok:
                continue
            t, _, chars = tok.partition(":")
            try:
                time_s = float(t)
            except ValueError:
                print("ERRO: roteiro invalido em '%s' (use tempo:teclas)" % tok)
                sys.exit(1)
            chars = chars.replace("<enter>", "\n").replace("<esc>", ESC)
            self.items.append((time_s, chars))
        self.items.sort(key=lambda x: x[0])
        self.last = self.items[-1][0] if self.items else 0.0

    def read(self, now):
        out = []
        while self.items and self.items[0][0] <= now:
            out.extend(parse_keys(self.items.pop(0)[1]))
        return out

    def finished(self, now):
        return not self.items and now >= self.last + 1.0

    def close(self):
        pass


class HeadingFilter(object):
    def __init__(self, window):
        self.window = max(1, int(window))
        self.cos = []
        self.sin = []

    def update(self, theta):
        self.cos.append(math.cos(theta))
        self.sin.append(math.sin(theta))
        if len(self.cos) > self.window:
            del self.cos[0]
            del self.sin[0]
        c = sorted(self.cos)[len(self.cos) // 2]
        s = sorted(self.sin)[len(self.sin) // 2]
        if c == 0.0 and s == 0.0:
            return theta
        return math.atan2(s, c)


class KeyMission(object):
    def __init__(self, args, verbose=True):
        self.args = args
        self.theta_ref = None
        self.verbose = verbose
        self.speed = int(args["speed"])
        self.manual_mode = False
        self.state = STOPPED
        self.prompt = None
        self.buf = ""
        self.auto = None
        self.direction = 0
        self.theta = 0.0
        self.theta_start = 0.0
        self.theta_prev = 0.0
        self.now = 0.0
        self.turn_start_time = 0.0
        self.turned_deg = 0.0
        self.turn_prev = 0.0
        self.events = []

    def say(self, msg):
        self.events.append((self.now, msg))
        if self.verbose:
            print("\n" + msg)

    def update(self, theta, now):
        self.theta = theta
        self.now = now

    def line_err_deg(self):
        return -math.degrees(wrap_pi(self.theta_ref - self.theta))

    def turn_remaining_deg(self):
        return max(0.0, 360.0 - abs(self.turned_deg))

    def err_info(self):
        if self.state == LINE and self.theta_ref is not None:
            return theta_to_compass(self.theta_ref), self.line_err_deg()
        if self.state == TURN:
            return None, self.turn_remaining_deg()
        return None, None

    def halt(self, why):
        self.prompt = None
        self.state = MANUAL_STOP if self.manual_mode else STOPPED
        self.say(why)

    def one_motor_cmd(self, direction):
        g = int(self.speed)
        return (0, g) if direction == 1 else (g, 0)

    def manual_spin_cmd(self, direction):
        g = int(self.speed)
        if self.args["manual_reverse"] == "yes":
            return (-g, g) if direction == 1 else (g, -g)
        return self.one_motor_cmd(direction)

    def set_manual_state(self, state, what):
        self.prompt = None
        self.state = state
        self.say("%s, v=%d" % (what, self.speed))

    def enter_manual(self):
        self.prompt = None
        self.manual_mode = True
        self.state = MANUAL_STOP
        self.say("MODO MANUAL, motores parados: w=frente a=esq d=dir s=para v=vel m=sai")

    def exit_manual(self):
        self.prompt = None
        self.manual_mode = False
        self.state = STOPPED
        self.say("MODO AUTOMATICO, PARADO: r=reta t=volta v=vel q=para m=manual")

    def start_line(self):
        self.prompt = None
        self.theta_ref = self.theta
        self.turn_prev = 0.0
        self.state = LINE
        self.say("RETA: rumo setado em %.1f graus, v=%d"
                 % (theta_to_compass(self.theta_ref), self.speed))

    def start_turn(self, direction):
        self.prompt = None
        self.direction = direction
        self.auto = QuadrantAutomaton(self.theta, direction, self.args["hysteresis_deg"])
        self.theta_start = self.theta
        self.theta_prev = self.theta
        self.turned_deg = 0.0
        self.turn_start_time = self.now
        self.state = TURN
        self.say("GIRO para a %s, v=%d, rumo inicial %.1f, estado %s"
                 % ("ESQUERDA (anti-horario)" if direction == 1 else "DIREITA (horario)",
                    self.speed, theta_to_compass(self.theta), QUAD_NAMES[self.auto.q0]))
        if self.speed == 0:
            self.say("AVISO: v = 0, o barco nao vai girar")

    def on_key(self, ch):
        stop_keys = ("q", "s") if self.manual_mode else ("q",)
        if self.prompt == "direction":
            if ch == "h":
                self.start_turn(-1)
            elif ch == "a":
                self.start_turn(1)
            elif ch == ESC:
                self.prompt = None
                self.say("giro cancelado")
            elif ch in stop_keys:
                self.halt("PARADO (tecla %s)" % ch)
            elif ch == "r":
                self.start_line()
            return
        if self.prompt == "speed":
            if len(ch) == 1 and ch in "0123456789":
                if len(self.buf) < 3:
                    self.buf += ch
            elif ch in ("\x7f", "\x08"):
                self.buf = self.buf[:-1]
            elif ch in ("\n", "\r"):
                if self.buf == "":
                    self.say("v inalterado = %d" % self.speed)
                elif int(self.buf) <= CMD_MAX:
                    self.speed = int(self.buf)
                    self.say("v = %d" % self.speed)
                else:
                    self.say("ERRO: v deve ser de 0 a %d, mantido %d" % (CMD_MAX, self.speed))
                self.prompt = None
            elif ch == ESC:
                self.prompt = None
                self.say("v inalterado = %d" % self.speed)
            elif ch in stop_keys:
                self.halt("PARADO (tecla %s)" % ch)
            return
        if self.manual_mode:
            if ch == "w":
                self.set_manual_state(FORWARD, "FRENTE: motores constantes (v,v), sem correcao de rumo")
            elif ch == "a":
                self.set_manual_state(SPIN_LEFT, "VIRA A ESQUERDA (manual)")
            elif ch == "d":
                self.set_manual_state(SPIN_RIGHT, "VIRA A DIREITA (manual)")
            elif ch in ("s", "q"):
                self.halt("MANUAL: motores parados (tecla %s)" % ch)
            elif ch == "v":
                self.prompt = "speed"
                self.buf = ""
            elif ch == "m":
                self.exit_manual()
            elif ch in ("r", "t"):
                self.say("No modo manual r e t nao funcionam. Saia com m.")
            return
        if ch == "r":
            self.start_line()
        elif ch == "t":
            self.prompt = "direction"
        elif ch == "v":
            self.prompt = "speed"
            self.buf = ""
        elif ch == "q":
            self.halt("PARADO (tecla q)")
        elif ch == "m":
            self.enter_manual()
        elif ch in ("w", "a", "s", "d"):
            self.say("Tecla %s so vale no modo manual (m). Aqui, q para." % ch)

    def control(self):
        left = right = 0
        if self.state == LINE:
            if self.speed > 0:
                err = wrap_pi(self.theta_ref - self.theta)
                lim = min(self.args["turn_max"], float(self.speed))
                turn = max(-lim, min(lim, self.args["kp"] * err))
                slew = self.args["turn_slew"]
                if slew > 0:
                    turn = max(self.turn_prev - slew, min(self.turn_prev + slew, turn))
                self.turn_prev = turn
                left, right = mix(self.speed, turn)
        elif self.state == FORWARD:
            left = right = int(self.speed)
        elif self.state == SPIN_LEFT:
            left, right = self.manual_spin_cmd(1)
        elif self.state == SPIN_RIGHT:
            left, right = self.manual_spin_cmd(-1)
        elif self.state == TURN:
            self.turned_deg += math.degrees(wrap_pi(self.theta - self.theta_prev))
            self.theta_prev = self.theta
            if self.auto.update(self.theta) != 0:
                self.say("  %s  transicoes liquidas = %d  giro acumulado = %.0f"
                         % (QUAD_NAMES[self.auto.q], self.auto.net, self.turned_deg))
            if self.auto.back_in_start() and passed_start(self.theta, self.theta_start,
                                                          self.direction):
                self.state = STOPPED
                self.say("VOLTA COMPLETA, giro acumulado = %.0f graus. PARADO. Estados: %s"
                         % (abs(self.turned_deg), " -> ".join(QUAD_NAMES[q] for q in self.auto.history)))
            elif self.now - self.turn_start_time > self.args["turn_timeout_s"]:
                self.state = STOPPED
                self.say("TIMEOUT do giro (%.0f s), giro acumulado = %.0f graus. PARADO."
                         % (self.args["turn_timeout_s"], self.turned_deg))
            else:
                left, right = self.one_motor_cmd(self.direction)
        return left, right

    def status(self, h, left, right):
        target, err = self.err_info()
        if self.state == LINE:
            quad = QUAD_NAMES[quadrant_of(math.cos(self.theta), math.sin(self.theta))]
            full_text = "rumo=%5.1f alvo=%5.1f erro=%+6.1f %s" % (h, target, err, quad)
            brief = "rumo=%5.1f erro=%+6.1f" % (h, err)
        elif self.state == TURN:
            full_text = "rumo=%5.1f erro=%4.0f %s" % (h, err, QUAD_NAMES[self.auto.q])
            brief = "rumo=%5.1f erro=%4.0f" % (h, err)
        else:
            full_text = brief = "rumo=%5.1f" % h
        if self.prompt == "direction":
            s = "SENTIDO? h=horario a=anti Esc=cancela | %s %s (%4d,%4d)" % (
                self.state, brief, left, right)
        elif self.prompt == "speed":
            s = "v 0-%d: %s_ Enter=ok Esc=cancela | %s %s (%4d,%4d)" % (
                CMD_MAX, self.buf, self.state, brief, left, right)
        else:
            s = "%-6s %s v=%3d cmd=(%4d,%4d)" % (self.state, full_text, self.speed, left, right)
        return s[:79].ljust(79)


def run_keys(boat, keys, args, log_path, verbose=True):
    period = 1.0 / args["hz"]
    m = None
    f = open(log_path, "w")
    f.write("# mission1 calib=%s args=%s\n" % (boat.calib_path, json.dumps(args, sort_keys=True)))
    f.write(",".join(LOG_COLS) + "\n")
    reason = None
    last_flush = 0.0
    prev_loop_start = None
    heading_filter = HeadingFilter(args["filter"])
    try:
        m = KeyMission(args, verbose)
        if verbose:
            print("Auto:   r=reta no rumo atual  t=volta (depois h/a)  v=velocidade  q=para")
            print("Manual: m entra (w=frente a=esq d=dir s=para v=velocidade), m de novo sai")
            print("Ctrl+C sai.")
            print("Comecando PARADO, motores em 0.")
        while True:
            t_loop = boat.now()
            loop_ms = 0.0 if prev_loop_start is None else (t_loop - prev_loop_start) * 1000.0
            prev_loop_start = t_loop
            if stop_file_present():
                reason = "arquivo STOP"
                break
            theta_raw, h_raw, mag = boat.heading()
            read_ms = (boat.now() - t_loop) * 1000.0
            theta = heading_filter.update(theta_raw)
            h = theta_to_compass(theta)
            m.update(theta, boat.now())
            names = []
            for ch in keys.read(boat.now()):
                m.on_key(ch)
                names.append(key_name(ch))
            if keys.finished(boat.now()):
                reason = "roteiro de teclas terminou"
                break
            left, right = m.control()
            boat.motors(left, right)
            target, error = m.err_info()
            row = {"t_s": "%.3f" % boat.now(), "estado": m.state,
                   "quadrante": m.auto.q if (m.auto and m.state == TURN)
                   else quadrant_of(math.cos(theta), math.sin(theta)),
                   "transicoes": m.auto.net if (m.auto and m.state == TURN) else 0,
                   "rumo_deg": "%.1f" % h_raw, "rumo_filt_deg": "%.1f" % h,
                   "theta_deg": "%.1f" % math.degrees(theta),
                   "cos": "%.3f" % math.cos(theta), "sin": "%.3f" % math.sin(theta),
                   "alvo_rumo_deg": "%.1f" % target if target is not None else "",
                   "erro_deg": "%.1f" % error if error is not None else "",
                   "giro_acum_deg": "%.1f" % m.turned_deg, "vel": m.speed,
                   "tecla": "+".join(names), "cmd_esq": left, "cmd_dir": right,
                   "loop_ms": "%.0f" % loop_ms, "read_ms": "%.0f" % read_ms,
                   "mag_x": mag[0], "mag_y": mag[1], "mag_z": mag[2]}
            f.write(",".join(str(row[k]) for k in LOG_COLS) + "\n")
            if boat.now() - last_flush >= 5.0:
                f.flush()
                last_flush = boat.now()
            if verbose:
                sys.stdout.write("\r" + m.status(h, left, right))
                sys.stdout.flush()
            boat.sleep(period - (boat.now() - t_loop))
    except StopRequested as e:
        reason = "interrompido (%s)" % e
    finally:
        ignore_signals()
        boat.stop()
        f.close()
    if verbose:
        print("\nFim: %s. Motores parados. Log: %s" % (reason, log_path))
    return m


def mode_check(args):
    boat = RealBoat(args, motors=False)
    direction = 1 if args["direction"] == "left" else -1
    theta0, h0, _ = boat.heading()
    auto = QuadrantAutomaton(theta0, direction, args["hysteresis_deg"])
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
                auto = QuadrantAutomaton(theta0, direction, args["hysteresis_deg"])
            sys.stdout.write("\rrumo=%5.1f theta=%6.1f cos=%+.2f sin=%+.2f %s   "
                             % (h, math.degrees(th), math.cos(th), math.sin(th),
                                QUAD_NAMES[auto.q]))
            sys.stdout.flush()
            time.sleep(0.05)
    except StopRequested:
        print("\nSaindo.")


def ask_start_speed(default):
    while True:
        sys.stdout.write("Velocidade inicial v, 0 a %d (Enter = %d): " % (CMD_MAX, default))
        sys.stdout.flush()
        line = sys.stdin.readline()
        if line == "":
            print("\nERRO: sem entrada de teclado.")
            sys.exit(1)
        line = line.strip()
        if line == "":
            return default
        try:
            v = int(line)
        except ValueError:
            print("ERRO: digite um numero inteiro.")
            continue
        if 0 <= v <= CMD_MAX:
            return v
        print("ERRO: fora do intervalo 0 a %d." % CMD_MAX)


def mode_run(args):
    if os.path.isfile(STOP_FILE):
        print("ERRO: existe %s. Apague antes (rm data/STOP)." % STOP_FILE)
        sys.exit(1)
    if not sys.stdin.isatty():
        print("ERRO: o modo run precisa de um terminal (ssh ou screen). Nao rode com nohup.")
        sys.exit(1)
    users = find_port_users(ARDUINO_TTY)
    if users:
        print("ERRO: outro processo ja esta usando %s." % ARDUINO_TTY)
        print("Dois programas mandando comandos ao mesmo tempo fazem os motores tremer:")
        for pid, cmd in users:
            print("   PID %d: %s" % (pid, cmd))
        print("Encerre com: kill PID   (confira tambem: screen -ls)")
        sys.exit(1)
    args["speed"] = ask_start_speed(args["speed"])
    print("v inicial = %d. Manual a/d: %s" % (args["speed"], "um motor para tras"
                                              if args["manual_reverse"] == "yes" else "um motor so"))
    boat = RealBoat(args, motors=True)
    install_signals()
    boat.stop()
    keys = TtyKeys()
    print("Calibracao:", boat.calib_path)
    try:
        run_keys(boat, keys, args, new_log_path("mission1"))
    finally:
        keys.close()


def mode_sim(args):
    boat = SimBoat()
    install_signals()
    keys = ScriptKeys(args["keys"] or DEFAULT_SIM_KEYS)
    m = run_keys(boat, keys, args, new_log_path("mission1_sim"))
    return boat, m


def mode_calib(args):
    imu_mod = load_drivers(args["drivers"])
    imu = imu_mod.Imu9IO()
    out = args["calib"] or CALIB_FILES[0]

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
    folder = os.path.dirname(out)
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    with open(out, "w") as f:
        json.dump(c, f, indent=2)
    print("Calibracao salva em", out)


def parse_args(argv):
    args = dict(DEFAULTS)
    string_options = ("direction", "manual_reverse", "calib", "drivers", "keys")
    i = 0
    while i < len(argv):
        flag = argv[i]
        key = flag[2:].replace("-", "_")
        if not flag.startswith("--") or key not in args or i + 1 >= len(argv):
            print("Opcao invalida: %s" % flag)
            print(USAGE)
            sys.exit(1)
        value = argv[i + 1]
        if key in string_options:
            args[key] = value
        elif key in ("speed", "filter"):
            try:
                args[key] = int(value)
            except ValueError:
                print("ERRO: --%s precisa de um numero inteiro" % key)
                sys.exit(1)
        else:
            try:
                args[key] = float(value)
            except ValueError:
                print("ERRO: %s precisa de numero" % flag)
                sys.exit(1)
        i += 2
    if args["direction"] not in ("left", "right"):
        print("ERRO: --direction left ou right")
        sys.exit(1)
    if args["manual_reverse"] not in ("yes", "no"):
        print("ERRO: --manual-reverse yes ou no")
        sys.exit(1)
    if args["filter"] < 1 or args["filter"] % 2 == 0:
        print("ERRO: --filter deve ser impar e >= 1")
        sys.exit(1)
    if args["turn_slew"] < 0:
        print("ERRO: --turn-slew deve ser >= 0")
        sys.exit(1)
    if not 0 <= args["speed"] <= CMD_MAX:
        print("ERRO: --speed entre 0 e %d" % CMD_MAX)
        sys.exit(1)
    if args["hz"] <= 0 or args["turn_timeout_s"] <= 0:
        print("ERRO: --hz e --turn-timeout-s devem ser > 0")
        sys.exit(1)
    if not 0 < args["hysteresis_deg"] < 45:
        print("ERRO: --hysteresis-deg entre 0 e 45")
        sys.exit(1)
    return args


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("calib", "check", "run", "sim"):
        print(USAGE)
        sys.exit(1)
    args = parse_args(sys.argv[2:])
    {"calib": mode_calib, "check": mode_check, "run": mode_run, "sim": mode_sim}[sys.argv[1]](args)