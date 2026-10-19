import csv
import json
import math
import os
import re
import signal
import sys
import threading
import time
import types
from xml.sax.saxutils import escape

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
LOGS = os.path.join(DATA, "logs")
STOP_FILE = os.path.join(DATA, "STOP")

CMD_MAX = 191
ARDUINO_TTY = "/dev/ttyV0"
GPS_TTY_DEV = 0
EARTH_RADIUS = 6378137.0

DRIVER_DIRS = [os.path.join(HERE, "src", "DriversDDBOAT_2026"),
               os.path.expanduser("~/GKKDossier/drivers-ddboat-v2")]
CALIB_FILES = [os.path.join(DATA, "calib", "mag2d.json"),
               os.path.join(HERE, "calib_cap.json")]

POINT_A = (48.198863, -3.015070)
POINT_B = (48.199072, -3.016258)
POINT_C = (48.197408, -3.015060)
POINT_D = (48.1970689, -3.017328)
POINT_E = (48.199707, -3.016404)
POINT_F = (48.199814, -3.015137)
START_LATLON = POINT_B
GOAL_LATLON = POINT_A
SOUTH_LINE = (POINT_C, POINT_D)
NORTH_LINE = (POINT_E, POINT_F)
CORRIDOR_NAMES = ("linha C-D (sul)", "linha E-F (norte)")

DEFAULTS = {"speed": 120, "kp": 60.0, "turn_max": 90.0,
            "arrival_radius": 3.0,
            "point_intensity": 1.0, "segment_intensity": 2.0,
            "speed_gain": 6.0, "min_speed": 0,
            "gps_timeout_s": 3.0,
            "corridor": "arm", "corridor_margin": 3.0, "corridor_arm": 10.0,
            "turn_mode": "arc", "pivot_on": 60.0, "pivot_off": 20.0,
            "kf_acc_sigma": 0.3, "kf_gps_sigma": 2.0, "kf_gate": 9.21, "kf_reset_after": 5,
            "hz": 10.0, "manual_reverse": "yes",
            "start": None, "goal": None, "calib": None, "drivers": None}

STOPPED = "PARADO"
MISSION = "MISSAO"
NO_GPS = "SEM_GPS"
MANUAL_STOP = "MANUAL"
FORWARD = "FRENTE"
SPIN_LEFT = "VIRA_E"
SPIN_RIGHT = "VIRA_D"

USAGE = (
    "Uso: python3 mission2.py calib | gps | check | run [opcoes]\n"
    "  calib  calibra a bussola (igual a mission1)\n"
    "  gps    mostra lat/lon do GPS para copiar em START_LATLON e GOAL_LATLON\n"
    "  check  sem motor: posicao filtrada, distancia, rumo desejado, erro e velocidade\n"
    "  run    missao com motores. Teclas: g inicia | v velocidade | q para\n"
    "         m modo manual (w a d s v, m sai) | Ctrl+C sai. Gera o KML da rota no fim\n"
    "  kml    python3 mission2.py kml ARQUIVO.csv  converte um log da missao em KML\n"
    "Opcoes: --start LAT,LON --goal LAT,LON --arrival-radius M --speed N --min-speed N\n"
    "        --speed-gain X --point-intensity X --segment-intensity X --kp X --turn-max X\n"
    "        --gps-timeout-s X --kf-acc-sigma X --kf-gps-sigma X --kf-gate X\n"
    "        --kf-reset-after N --hz X --manual-reverse yes|no --calib ARQUIVO --drivers PASTA\n"
    "        --corridor off|arm|strict --corridor-margin M --corridor-arm M --turn-mode arc|pivot\n"
    "        --pivot-on GRAUS --pivot-off GRAUS")


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def compass_to_theta(h_deg):
    return wrap_pi(math.radians(90.0 - h_deg))


def theta_to_compass(theta):
    return (90.0 - math.degrees(theta)) % 360.0


def mix(thrust, turn):
    outer = int(round(thrust))
    diff = min(2.0 * abs(turn), float(thrust))
    inner = int(round(thrust - diff))
    return (inner, outer) if turn > 0 else (outer, inner)


def ddmm_to_deg(value, hemisphere):
    v = float(value)
    degrees = int(v / 100)
    minutes = v - degrees * 100
    result = degrees + minutes / 60.0
    return -result if hemisphere in ("S", "W") else result


class LocalFrame(object):
    def __init__(self, lat0_deg, lon0_deg):
        self.lat0_deg = lat0_deg
        self.lon0_deg = lon0_deg
        self.lat0 = math.radians(lat0_deg)
        self.lon0 = math.radians(lon0_deg)

    def to_xy(self, lat_deg, lon_deg):
        x = EARTH_RADIUS * math.cos(self.lat0) * (math.radians(lon_deg) - self.lon0)
        y = EARTH_RADIUS * (math.radians(lat_deg) - self.lat0)
        return x, y

    def to_latlon(self, x, y):
        lat = math.degrees(self.lat0 + y / EARTH_RADIUS)
        lon = math.degrees(self.lon0 + x / (EARTH_RADIUS * math.cos(self.lat0)))
        return lat, lon


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def vec_unit(v):
    n = math.hypot(v[0], v[1])
    return (v[0] / n, v[1] / n)


class PointField(object):
    def __init__(self, p0, intensity):
        self.p0 = p0
        self.intensity = intensity

    def potential(self, p):
        d = vec_sub(p, self.p0)
        return vec_dot(d, d) * self.intensity

    def gradient(self, p):
        d = vec_sub(p, self.p0)
        return (-2.0 * d[0] * self.intensity, -2.0 * d[1] * self.intensity)


class LineField(object):
    def __init__(self, p0, n, intensity):
        self.p0 = p0
        self.n = vec_unit(n)
        self.intensity = intensity

    def potential(self, p):
        return vec_dot(self.n, vec_sub(p, self.p0)) ** 2 * self.intensity

    def gradient(self, p):
        k = -2.0 * vec_dot(self.n, vec_sub(p, self.p0)) * self.intensity
        return (self.n[0] * k, self.n[1] * k)


class SegmentField(object):
    def __init__(self, p0, p1, intensity):
        self.p0 = p0
        self.p1 = p1
        self.intensity = intensity
        self.vector = vec_sub(p1, p0)
        self.length_squared = vec_dot(self.vector, self.vector)

    def closest(self, p):
        t = vec_dot(vec_sub(p, self.p0), self.vector) / self.length_squared
        t = max(0.0, min(1.0, t))
        return (self.p0[0] + t * self.vector[0], self.p0[1] + t * self.vector[1])

    def potential(self, p):
        d = vec_sub(p, self.closest(p))
        return vec_dot(d, d) * self.intensity

    def gradient(self, p):
        d = vec_sub(p, self.closest(p))
        return (-2.0 * d[0] * self.intensity, -2.0 * d[1] * self.intensity)


class UniformField(object):
    def __init__(self, v, intensity):
        self.v = vec_unit(v)
        self.intensity = intensity

    def potential(self, p):
        return -vec_dot(self.v, p) * self.intensity

    def gradient(self, p):
        return (self.v[0] * self.intensity, self.v[1] * self.intensity)


class PotentialField(object):
    def __init__(self):
        self.elements = []

    def add(self, element):
        self.elements.append(element)

    def potential(self, p):
        return sum(e.potential(p) for e in self.elements)

    def gradient(self, p):
        gx = gy = 0.0
        for e in self.elements:
            g = e.gradient(p)
            gx += g[0]
            gy += g[1]
        return (gx, gy)


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))]
            for i in range(len(a))]


def mat_t(a):
    return [list(row) for row in zip(*a)]


def mat_add(a, b):
    return [[x + y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def mat_sub(a, b):
    return [[x - y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def identity(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


class PositionKalman(object):
    def __init__(self, acc_sigma, gps_sigma, gate, reset_after):
        self.acc_sigma = acc_sigma
        self.gps_sigma = gps_sigma
        self.gate = gate
        self.reset_after = reset_after
        self.x = None
        self.p = None
        self.last_time = None
        self.rejections = 0

    def initialized(self):
        return self.x is not None

    def reset(self, px, py, t):
        s2 = self.gps_sigma ** 2
        self.x = [[px], [py], [0.0], [0.0]]
        self.p = [[s2, 0.0, 0.0, 0.0], [0.0, s2, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
        self.last_time = t
        self.rejections = 0

    def predict(self, t):
        if self.x is None or t <= self.last_time:
            return
        dt = t - self.last_time
        f = [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
        a2 = self.acc_sigma ** 2
        q4, q3, q2 = a2 * dt ** 4 / 4.0, a2 * dt ** 3 / 2.0, a2 * dt ** 2
        q = [[q4, 0.0, q3, 0.0], [0.0, q4, 0.0, q3], [q3, 0.0, q2, 0.0], [0.0, q3, 0.0, q2]]
        self.x = mat_mul(f, self.x)
        self.p = mat_add(mat_mul(mat_mul(f, self.p), mat_t(f)), q)
        self.last_time = t

    def update(self, zx, zy, t):
        if self.x is None:
            self.reset(zx, zy, t)
            return "init"
        r = self.gps_sigma ** 2
        s = [[self.p[0][0] + r, self.p[0][1]], [self.p[1][0], self.p[1][1] + r]]
        det = s[0][0] * s[1][1] - s[0][1] * s[1][0]
        if det <= 0.0:
            self.reset(zx, zy, t)
            return "reset"
        s_inv = [[s[1][1] / det, -s[0][1] / det], [-s[1][0] / det, s[0][0] / det]]
        y = [[zx - self.x[0][0]], [zy - self.x[1][0]]]
        d2 = mat_mul(mat_mul(mat_t(y), s_inv), y)[0][0]
        if d2 > self.gate:
            self.rejections += 1
            if self.rejections >= self.reset_after:
                self.reset(zx, zy, t)
                return "reset"
            return "rej"
        ph_t = [[self.p[i][0], self.p[i][1]] for i in range(4)]
        k = mat_mul(ph_t, s_inv)
        self.x = mat_add(self.x, mat_mul(k, y))
        h = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]
        ikh = mat_sub(identity(4), mat_mul(k, h))
        krk = mat_mul(mat_mul(k, [[r, 0.0], [0.0, r]]), mat_t(k))
        self.p = mat_add(mat_mul(mat_mul(ikh, self.p), mat_t(ikh)), krk)
        self.rejections = 0
        return "ok"

    def position(self):
        return (self.x[0][0], self.x[1][0])

    def velocity(self):
        return (self.x[2][0], self.x[3][0])


class Corridor(object):
    def __init__(self, south, north, reference):
        self.lines = []
        for p, q in (south, north):
            dx, dy = q[0] - p[0], q[1] - p[1]
            n = math.hypot(dx, dy)
            nx, ny = -dy / n, dx / n
            if (reference[0] - p[0]) * nx + (reference[1] - p[1]) * ny < 0:
                nx, ny = -nx, -ny
            self.lines.append((p, (nx, ny)))

    def distances(self, position):
        return [(position[0] - p[0]) * n[0] + (position[1] - p[1]) * n[1] for p, n in self.lines]

    def violated(self, position, margin):
        for name, value in zip(CORRIDOR_NAMES, self.distances(position)):
            if value < -margin:
                return name
        return None


def compute_guidance(position, theta, goal_xy, field, args, speed):
    gx, gy = field.gradient(position)
    norm = math.hypot(gx, gy)
    desired = math.atan2(gy, gx)
    speed_cmd = int(round(min(float(speed), max(float(args["min_speed"]), args["speed_gain"] * norm))))
    err = wrap_pi(desired - theta)
    lim = min(args["turn_max"], float(speed_cmd))
    turn = max(-lim, min(lim, args["kp"] * err))
    left, right = mix(speed_cmd, turn)
    return {"distance": math.hypot(goal_xy[0] - position[0], goal_xy[1] - position[1]),
            "gx": gx, "gy": gy, "norm": norm, "desired": desired, "err": err,
            "speed_cmd": speed_cmd, "left": left, "right": right}


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


class GpsReader(threading.Thread):
    def __init__(self, gps, clock):
        threading.Thread.__init__(self)
        self.daemon = True
        self.gps = gps
        self.clock = clock
        self.lock = threading.Lock()
        self.latest = None
        self.fresh = False
        self.running = True
        self.accepted = 0
        self.rejected = 0

    def run(self):
        while self.running:
            try:
                ok, d = self.gps.read_rmc_non_blocking()
                if ok:
                    self.handle(d)
            except Exception:
                self.rejected += 1
            time.sleep(0.01)

    def handle(self, d):
        if d[0] == 0.0 and d[2] == 0.0:
            self.rejected += 1
            return
        lat = ddmm_to_deg(d[0], d[1])
        lon = ddmm_to_deg(d[2], d[3])
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            self.rejected += 1
            return
        with self.lock:
            self.latest = (self.clock(), lat, lon, float(d[5]), float(d[6]))
            self.fresh = True
            self.accepted += 1

    def pop_new(self):
        with self.lock:
            if not self.fresh:
                return None
            self.fresh = False
            return self.latest

    def last_fix(self):
        with self.lock:
            return self.latest


class RealBoat(object):
    def __init__(self, args, motors, gps=False, imu=True):
        self.t0 = time.time()
        self.calib_path = "-"
        self.c = None
        self.imu = None
        self.ard = None
        self.gps_reader = None
        if imu:
            self.calib_path = find_file(args["calib"], CALIB_FILES, "arquivo de calibracao")
            self.c = load_calib(self.calib_path)
        imu_mod = load_drivers(args["drivers"])
        if imu:
            self.imu = imu_mod.Imu9IO()
        if gps:
            import gps_driver_v2
            try:
                device = gps_driver_v2.GpsIO(GPS_TTY_DEV)
                device.send_mtk_cmd("PMTK386,0")
            except Exception as e:
                print("ERRO: GPS nao inicializado (%r)" % (e,))
                sys.exit(1)
            self.gps_reader = GpsReader(device, self.now)
            self.gps_reader.start()
        if motors:
            import arduino_driver_v2
            self.ard = arduino_driver_v2.ArduinoIO()
            if self.ard.arduino is None:
                print("ERRO: Arduino nao inicializado.")
                sys.exit(1)

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

    def pop_gps(self):
        return self.gps_reader.pop_new() if self.gps_reader is not None else None

    def last_gps(self):
        return self.gps_reader.last_fix() if self.gps_reader is not None else None

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

    def close(self):
        if self.gps_reader is not None:
            self.gps_reader.running = False


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


def reset_signals():
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, signal.SIG_DFL)


LOG_COLS = ["t_s", "estado", "lat", "lon", "gps_age_s", "gps_new", "gps_x_m", "gps_y_m",
            "x_m", "y_m", "vx_mps", "vy_mps", "dist_m", "corredor", "dist_sul_m", "dist_norte_m",
            "pivo", "grad_x", "grad_y", "grad_norm",
            "alvo_rumo_deg", "rumo_deg", "erro_deg", "vel", "vel_cmd", "tecla",
            "cmd_esq", "cmd_dir", "kf", "loop_ms", "read_ms", "mag_x", "mag_y", "mag_z"]


ESC = "\x1b"


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


class Mission(object):
    def __init__(self, args, field, start_xy, goal_xy, corridor, verbose=True):
        self.args = args
        self.field = field
        self.start_xy = start_xy
        self.goal_xy = goal_xy
        self.corridor_obj = corridor
        self.corridor = corridor if args["corridor"] != "off" else None
        self.armed = False
        self.violated = None
        self.outside = False
        self.deep = False
        self.corridor_dist = None
        self.pivoting = False
        self.verbose = verbose
        self.speed = int(args["speed"])
        self.manual_mode = False
        self.state = STOPPED
        self.prompt = None
        self.buf = ""
        self.theta = 0.0
        self.now = 0.0
        self.position = None
        self.gps_age = None
        self.guidance = None
        self.finished = False
        self.events = []

    def say(self, msg):
        self.events.append((self.now, msg, self.position))
        if self.verbose:
            print("\n" + msg)

    def update(self, theta, now, position, gps_age):
        self.theta = theta
        self.now = now
        self.position = position
        self.gps_age = gps_age
        fresh = (position is not None and gps_age is not None
                 and gps_age <= self.args["gps_timeout_s"])
        self.guidance = (compute_guidance(position, theta, self.goal_xy, self.field,
                                          self.args, self.speed) if fresh else None)
        if self.corridor_obj is not None and position is not None:
            self.corridor_dist = self.corridor_obj.distances(position)
            self.violated = self.corridor_obj.violated(position, self.args["corridor_margin"])
            self.outside = min(self.corridor_dist) < 0.0
            self.deep = min(self.corridor_dist) >= self.args["corridor_arm"]
        else:
            self.corridor_dist = None
            self.violated = None
            self.outside = False
            self.deep = False

    def halt(self, why):
        self.prompt = None
        self.state = MANUAL_STOP if self.manual_mode else STOPPED
        self.say(why)

    def manual_spin_cmd(self, direction):
        g = int(self.speed)
        if self.args["manual_reverse"] == "yes":
            return (-g, g) if direction == 1 else (g, -g)
        return (0, g) if direction == 1 else (g, 0)

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
        self.say("MODO AUTOMATICO, PARADO: g=missao v=vel q=para m=manual")

    def start_mission(self):
        if self.position is None:
            self.say("Sem fix do GPS ainda. Aguarde.")
            return
        if self.guidance is None:
            self.say("GPS sem fix novo ha %.1f s. Aguarde." % self.gps_age)
            return
        if self.corridor is not None and self.args["corridor"] == "strict" and self.outside:
            self.say("Fora do corredor: no modo strict a missao nao inicia.")
            return
        self.prompt = None
        self.state = MISSION
        self.pivoting = False
        self.armed = self.corridor is not None and (self.deep or self.args["corridor"] == "strict")
        note = ""
        if self.corridor is not None:
            note = (" (corredor armado)" if self.armed
                    else " (fora do corredor: a checagem arma quando ele entrar %.0f m)" % self.args["corridor_arm"])
        self.say("MISSAO iniciada: %.1f m do ponto de retorno, v=%d%s"
                 % (self.guidance["distance"], self.speed, note))

    def on_key(self, ch):
        stop_keys = ("q", "s") if self.manual_mode else ("q",)
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
            elif ch in ("g", "r", "t"):
                self.say("No modo manual a missao nao inicia. Saia com m.")
            return
        if ch == "g":
            self.start_mission()
        elif ch == "v":
            self.prompt = "speed"
            self.buf = ""
        elif ch == "q":
            self.halt("PARADO (tecla q)")
        elif ch == "m":
            self.enter_manual()
        elif ch in ("r", "t"):
            self.say("A tecla %s nao existe no mission2. Use g para iniciar a missao." % ch)
        elif ch in ("w", "a", "s", "d"):
            self.say("Tecla %s so vale no modo manual (m). Aqui, q para." % ch)

    def drive(self, g):
        if self.args["turn_mode"] == "pivot":
            err_deg = abs(math.degrees(g["err"]))
            if self.pivoting and err_deg < self.args["pivot_off"]:
                self.pivoting = False
            elif not self.pivoting and err_deg > self.args["pivot_on"]:
                self.pivoting = True
            if self.pivoting:
                return (0, g["speed_cmd"]) if g["err"] > 0 else (g["speed_cmd"], 0)
        return g["left"], g["right"]

    def control(self):
        left = right = 0
        if self.state == MISSION or self.state == NO_GPS:
            if self.guidance is None:
                if self.state == MISSION:
                    self.state = NO_GPS
                    self.say("GPS sem fix novo ha mais de %.0f s: motores parados ate voltar"
                             % self.args["gps_timeout_s"])
            else:
                if self.state == NO_GPS:
                    self.state = MISSION
                    self.say("GPS voltou: missao retomada")
                g = self.guidance
                if g["distance"] <= self.args["arrival_radius"]:
                    self.finished = True
                    self.state = STOPPED
                    self.say("CHEGOU ao ponto de retorno: d=%.1f m (raio %.1f m)"
                             % (g["distance"], self.args["arrival_radius"]))
                else:
                    if self.corridor is not None:
                        if not self.armed and self.deep:
                            self.armed = True
                            self.say("Corredor armado: dentro por mais de %.0f m" % self.args["corridor_arm"])
                        if self.armed and self.violated is not None:
                            self.armed = False
                            self.state = STOPPED
                            self.say("SAIU DO CORREDOR (%s): motores parados. g retoma, m manual" % self.violated)
                            return 0, 0
                    left, right = self.drive(g)
        elif self.state == FORWARD:
            left = right = int(self.speed)
        elif self.state == SPIN_LEFT:
            left, right = self.manual_spin_cmd(1)
        elif self.state == SPIN_RIGHT:
            left, right = self.manual_spin_cmd(-1)
        return left, right

    def status(self, h, left, right):
        g = self.guidance
        gps_text = "---" if self.gps_age is None else "%.1f" % self.gps_age
        tag = ""
        if self.corridor is not None and self.outside:
            tag += " FORA"
        if self.pivoting and self.state == MISSION:
            tag += " PIVO"
        if self.prompt == "speed":
            brief = "rumo=%5.1f" % h if g is None else "rumo=%5.1f d=%5.1f" % (h, g["distance"])
            s = "v 0-%d: %s_ Enter=ok Esc=cancela | %s %s (%4d,%4d)" % (
                CMD_MAX, self.buf, self.state, brief, left, right)
        elif self.state == MISSION and g is not None:
            s = "%-7s rumo=%5.1f alvo=%5.1f erro=%+6.1f d=%5.1f cmd=(%3d,%3d) gps=%s%s" % (
                self.state, h, theta_to_compass(g["desired"]), -math.degrees(g["err"]),
                g["distance"], left, right, gps_text, tag)
        elif self.state == STOPPED:
            dist = "---" if g is None else "%.1f" % g["distance"]
            s = "%-7s rumo=%5.1f d=%s v=%3d cmd=(%4d,%4d) gps=%s%s" % (
                self.state, h, dist, self.speed, left, right, gps_text, tag)
        else:
            s = "%-7s rumo=%5.1f v=%3d cmd=(%4d,%4d) gps=%s%s" % (
                self.state, h, self.speed, left, right, gps_text, tag)
        return s[:79].ljust(79)


EVENT_NAMES = (("MISSAO iniciada", "Inicio (g)"), ("CHEGOU", "Chegada"),
               ("SAIU DO CORREDOR", "Saiu do corredor"), ("GPS sem fix novo", "GPS perdido"),
               ("GPS voltou", "GPS voltou"), ("PARADO (tecla q)", "Parada (q)"))


def kml_coords(points):
    return " ".join("%.7f,%.7f,0" % (lon, lat) for lat, lon in points)


def kml_marker(name, lat, lon, style, description=""):
    return ("<Placemark><name>%s</name><description>%s</description><styleUrl>#%s</styleUrl>"
            "<Point><coordinates>%.7f,%.7f,0</coordinates></Point></Placemark>"
            % (escape(name), escape(description), style, lon, lat))


def kml_line(name, points, style, visible=True, description=""):
    return ("<Placemark><name>%s</name><description>%s</description><visibility>%d</visibility>"
            "<styleUrl>#%s</styleUrl><LineString><tessellate>1</tessellate>"
            "<altitudeMode>clampToGround</altitudeMode><coordinates>%s</coordinates></LineString></Placemark>"
            % (escape(name), escape(description), 1 if visible else 0, style, kml_coords(points)))


def kml_styles():
    out = []
    for name, color, scale in (("ponto_retorno", "ff00ff00", 1.3), ("ponto_inicio", "ffff0000", 1.3),
                               ("ponto_borda", "ff0000ff", 1.1), ("evento", "ff00ffff", 0.9),
                               ("fim", "ffff00ff", 1.1)):
        out.append('<Style id="%s"><IconStyle><color>%s</color><scale>%.1f</scale><Icon>'
                   '<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>'
                   '</Icon></IconStyle><LabelStyle><scale>1.0</scale></LabelStyle></Style>'
                   % (name, color, scale))
    for name, color, width in (("linha_rota", "ff00ff00", 4), ("linha_gps", "ff0080ff", 2),
                               ("linha_plano", "ffffffff", 3), ("linha_corredor", "ff0000ff", 3)):
        out.append('<Style id="%s"><LineStyle><color>%s</color><width>%d</width></LineStyle></Style>'
                   % (name, color, width))
    return "".join(out)


def kml_marks(args):
    a = args["goal"] or GOAL_LATLON
    b = args["start"] or START_LATLON
    return [("A - Retorno", a[0], a[1], "ponto_retorno", "Ponto de retorno"),
            ("B - Inicio", b[0], b[1], "ponto_inicio", "Inicio da reta guia"),
            ("C", POINT_C[0], POINT_C[1], "ponto_borda", "Linha sul (C-D)"),
            ("D", POINT_D[0], POINT_D[1], "ponto_borda", "Linha sul (C-D)"),
            ("E", POINT_E[0], POINT_E[1], "ponto_borda", "Linha norte (E-F)"),
            ("F", POINT_F[0], POINT_F[1], "ponto_borda", "Linha norte (E-F)")]


def build_kml(title, description, marks, planned, route, raw, events):
    points = "".join(kml_marker(n, lat, lon, st, d) for n, lat, lon, st, d in marks)
    guides = (kml_line("Linha sul C-D", SOUTH_LINE, "linha_corredor")
              + kml_line("Linha norte E-F", NORTH_LINE, "linha_corredor")
              + kml_line("Reta guia B-A", planned, "linha_plano"))
    run = []
    if len(route) >= 2:
        run.append(kml_line("Rota do barco (posicao filtrada)", route, "linha_rota", True,
                            "Posicao filtrada pelo Kalman"))
    if len(raw) >= 2:
        run.append(kml_line("GPS cru", raw, "linha_gps", False, "Fixes do GPS sem filtro"))
    for name, lat, lon, text in events:
        run.append(kml_marker(name, lat, lon, "evento", text))
    if route:
        run.append(kml_marker("Fim da rota", route[-1][0], route[-1][1], "fim", "Ultima posicao registrada"))
    return ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<kml xmlns="http://www.opengis.net/kml/2.2"><Document><name>%s</name>'
            '<description>%s</description>%s'
            '<Folder><name>Pontos</name>%s</Folder>'
            '<Folder><name>Corredor e reta guia</name>%s</Folder>'
            '<Folder><name>Missao</name><open>1</open>%s</Folder></Document></kml>\n'
            % (escape(title), escape(description), kml_styles(), points, guides, "".join(run)))


def write_kml(path, text):
    with open(path, "w") as f:
        f.write(text)


def run_kml(args, frame, mission, track, raw_track, log_path):
    events = []
    for t, msg, position in mission.events:
        if position is None:
            continue
        for key, name in EVENT_NAMES:
            if msg.startswith(key):
                lat, lon = frame.to_latlon(position[0], position[1])
                events.append(("%s t=%.0fs" % (name, t), lat, lon, msg))
                break
    marks = kml_marks(args)
    planned = [(marks[1][1], marks[1][2]), (marks[0][1], marks[0][2])]
    description = "log %s | raio de chegada %.1f m | v=%d | corredor %s | modo de curva %s" % (
        os.path.basename(log_path), args["arrival_radius"], mission.speed, args["corridor"], args["turn_mode"])
    return build_kml("Missao 2 - " + os.path.basename(log_path), description, marks, planned,
                     track, raw_track, events)


def mode_kml(args):
    path = args["input"]
    origin = None
    log_args = {}
    with open(path) as f:
        first = f.readline()
        m = re.search(r"origin=([-0-9.]+),([-0-9.]+)", first)
        if m:
            origin = (float(m.group(1)), float(m.group(2)))
        m = re.search(r"args=(\{.*\})", first)
        if m:
            try:
                log_args = json.loads(m.group(1))
            except ValueError:
                log_args = {}
        f.seek(0)
        rows = list(csv.DictReader([line for line in f if not line.startswith("#")]))
    if origin is None or not rows:
        print("ERRO: %s nao parece um log do mission2 (sem origem ou sem linhas)." % path)
        sys.exit(1)
    frame = LocalFrame(origin[0], origin[1])
    merged = dict(DEFAULTS)
    merged.update(log_args)
    for key in ("start", "goal"):
        if merged.get(key):
            merged[key] = tuple(merged[key])
    track = []
    raw = []
    events = []
    previous = None
    last_distance = None
    for i, r in enumerate(rows):
        if r["x_m"] != "" and i % 5 == 0:
            track.append(frame.to_latlon(float(r["x_m"]), float(r["y_m"])))
        if r["gps_new"] == "1" and r["lat"] != "":
            raw.append((float(r["lat"]), float(r["lon"])))
        state = r["estado"]
        if r["dist_m"] != "":
            last_distance = float(r["dist_m"])
        if state != previous and r["x_m"] != "":
            name = None
            if state == MISSION:
                name = "Inicio (g)" if previous != NO_GPS else "GPS voltou"
            elif state == NO_GPS:
                name = "GPS perdido"
            elif state == STOPPED and previous == MISSION:
                name = ("Chegada" if last_distance is not None and last_distance <= merged["arrival_radius"] + 0.5
                        else "Parada")
            if name is not None:
                lat, lon = frame.to_latlon(float(r["x_m"]), float(r["y_m"]))
                events.append(("%s t=%.0fs" % (name, float(r["t_s"])), lat, lon, "estado " + state))
        previous = state
    if rows[-1]["x_m"] != "" and (not track or track[-1] != frame.to_latlon(float(rows[-1]["x_m"]), float(rows[-1]["y_m"]))):
        track.append(frame.to_latlon(float(rows[-1]["x_m"]), float(rows[-1]["y_m"])))
    marks = kml_marks(merged)
    planned = [(marks[1][1], marks[1][2]), (marks[0][1], marks[0][2])]
    description = "convertido de %s | raio de chegada %.1f m" % (os.path.basename(path), merged["arrival_radius"])
    out = path[:-4] + ".kml" if path.endswith(".csv") else path + ".kml"
    write_kml(out, build_kml("Missao 2 - " + os.path.basename(path), description, marks, planned,
                             track, raw, events))
    print("KML gerado: %s (%d pontos de rota, %d fixes de GPS, %d eventos)" % (out, len(track), len(raw), len(events)))


def fmt(value, spec):
    return spec % value if value is not None else ""


def run_mission(boat, keys, args, frame, field, start_xy, goal_xy, corridor, log_path,
                verbose=True, kml_path=None):
    period = 1.0 / args["hz"]
    kf = PositionKalman(args["kf_acc_sigma"], args["kf_gps_sigma"], args["kf_gate"],
                        args["kf_reset_after"])
    mission = Mission(args, field, start_xy, goal_xy, corridor, verbose)
    track = []
    raw_track = []
    last_track = -1.0
    f = open(log_path, "w")
    f.write("# mission2 calib=%s origin=%.7f,%.7f goal_xy=%.2f,%.2f args=%s\n" % (
        boat.calib_path, frame.lat0_deg, frame.lon0_deg, goal_xy[0], goal_xy[1],
        json.dumps(args, sort_keys=True)))
    f.write(",".join(LOG_COLS) + "\n")
    reason = None
    last_flush = 0.0
    prev_loop_start = None
    try:
        if verbose:
            print("Auto:   g=inicia a missao  v=velocidade  q=para")
            print("Manual: m entra (w=frente a=esq d=dir s=para v=velocidade), m de novo sai")
            print("Ctrl+C sai. Comecando PARADO, motores em 0.")
        while True:
            t_loop = boat.now()
            loop_ms = 0.0 if prev_loop_start is None else (t_loop - prev_loop_start) * 1000.0
            prev_loop_start = t_loop
            if stop_file_present():
                reason = "arquivo STOP"
                break
            theta, h, mag = boat.heading()
            read_ms = (boat.now() - t_loop) * 1000.0
            now = boat.now()
            fix = boat.pop_gps()
            kf_status = ""
            gps_x = gps_y = None
            if kf.initialized():
                kf.predict(now)
            if fix is not None:
                gps_x, gps_y = frame.to_xy(fix[1], fix[2])
                kf_status = kf.update(gps_x, gps_y, now)
            position = kf.position() if kf.initialized() else None
            if position is not None and now - last_track >= 0.5:
                track.append(frame.to_latlon(position[0], position[1]))
                last_track = now
            if fix is not None:
                raw_track.append((fix[1], fix[2]))
            last_fix = boat.last_gps()
            gps_age = None if last_fix is None else now - last_fix[0]
            mission.update(theta, now, position, gps_age)
            names = []
            for ch in keys.read(now):
                mission.on_key(ch)
                names.append(key_name(ch))
            if keys.finished(now):
                reason = "roteiro de teclas terminou"
                break
            left, right = mission.control()
            boat.motors(left, right)
            g = mission.guidance
            cd = mission.corridor_dist
            velocity = kf.velocity() if kf.initialized() else (None, None)
            row = {"t_s": "%.3f" % now, "estado": mission.state,
                   "lat": fmt(None if last_fix is None else last_fix[1], "%.7f"),
                   "lon": fmt(None if last_fix is None else last_fix[2], "%.7f"),
                   "gps_age_s": fmt(gps_age, "%.2f"), "gps_new": 1 if fix is not None else 0,
                   "gps_x_m": fmt(gps_x, "%.2f"), "gps_y_m": fmt(gps_y, "%.2f"),
                   "x_m": fmt(None if position is None else position[0], "%.2f"),
                   "y_m": fmt(None if position is None else position[1], "%.2f"),
                   "vx_mps": fmt(velocity[0], "%.2f"), "vy_mps": fmt(velocity[1], "%.2f"),
                   "dist_m": fmt(None if g is None else g["distance"], "%.2f"),
                   "corredor": "" if cd is None else (0 if mission.outside else 1),
                   "dist_sul_m": fmt(None if cd is None else cd[0], "%.1f"),
                   "dist_norte_m": fmt(None if cd is None else cd[1], "%.1f"),
                   "pivo": 1 if mission.pivoting else 0,
                   "grad_x": fmt(None if g is None else g["gx"], "%.3f"),
                   "grad_y": fmt(None if g is None else g["gy"], "%.3f"),
                   "grad_norm": fmt(None if g is None else g["norm"], "%.3f"),
                   "alvo_rumo_deg": fmt(None if g is None else theta_to_compass(g["desired"]), "%.1f"),
                   "rumo_deg": "%.1f" % h,
                   "erro_deg": fmt(None if g is None else -math.degrees(g["err"]), "%.1f"),
                   "vel": mission.speed,
                   "vel_cmd": "" if g is None else g["speed_cmd"],
                   "tecla": "+".join(names), "cmd_esq": left, "cmd_dir": right,
                   "kf": kf_status, "loop_ms": "%.0f" % loop_ms, "read_ms": "%.0f" % read_ms,
                   "mag_x": mag[0], "mag_y": mag[1], "mag_z": mag[2]}
            f.write(",".join(str(row[k]) for k in LOG_COLS) + "\n")
            if now - last_flush >= 5.0:
                f.flush()
                last_flush = now
            if verbose:
                sys.stdout.write("\r" + mission.status(h, left, right))
                sys.stdout.flush()
            if mission.finished:
                reason = "chegada"
                break
            boat.sleep(period - (boat.now() - t_loop))
    except StopRequested as e:
        reason = "interrompido (%s)" % e
    finally:
        ignore_signals()
        boat.stop()
        f.close()
        reset_signals()
        if kml_path is not None:
            try:
                write_kml(kml_path, run_kml(args, frame, mission, track, raw_track, log_path))
                if verbose:
                    print("\nKML da rota: %s" % kml_path)
            except Exception as e:
                print("\nAVISO: nao foi possivel gerar o KML (%r)" % (e,))
    if verbose:
        print("\nFim: %s. Motores parados. Log: %s" % (reason, log_path))
    return mission, reason


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


def parse_latlon(text, flag):
    try:
        lat, lon = [float(v) for v in text.split(",")]
    except ValueError:
        print("ERRO: %s precisa de LAT,LON em graus decimais (exemplo: 48.2000,-3.0100)" % flag)
        sys.exit(1)
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        print("ERRO: %s fora do intervalo valido" % flag)
        sys.exit(1)
    return (lat, lon)


def resolve_geometry(args):
    start = args["start"] or START_LATLON
    goal = args["goal"] or GOAL_LATLON
    if start is None or goal is None:
        print("ERRO: defina START_LATLON e GOAL_LATLON no topo do arquivo,")
        print("      ou use --start LAT,LON --goal LAT,LON")
        print("Para ler as coordenadas do GPS: python3 mission2.py gps")
        sys.exit(1)
    frame = LocalFrame(start[0], start[1])
    start_xy = (0.0, 0.0)
    goal_xy = frame.to_xy(goal[0], goal[1])
    distance = math.hypot(goal_xy[0], goal_xy[1])
    if distance < 2.0 * args["arrival_radius"]:
        print("ERRO: ponto inicial e ponto de retorno a %.1f m, menos que o dobro do raio de chegada (%.1f m)"
              % (distance, args["arrival_radius"]))
        sys.exit(1)
    field = PotentialField()
    field.add(PointField(goal_xy, args["point_intensity"]))
    field.add(SegmentField(start_xy, goal_xy, args["segment_intensity"]))
    reference = ((start_xy[0] + goal_xy[0]) / 2.0, (start_xy[1] + goal_xy[1]) / 2.0)
    corridor = Corridor(tuple(frame.to_xy(*q) for q in SOUTH_LINE),
                        tuple(frame.to_xy(*q) for q in NORTH_LINE), reference)
    if args["corridor"] != "off":
        for name, point in (("ponto inicial (B)", start_xy), ("ponto de retorno (A)", goal_xy)):
            outside = corridor.violated(point, 0.0)
            if outside is not None:
                print("ERRO: o %s esta fora do corredor (%s)." % (name, outside))
                print("Para testar em outro lugar: --corridor off")
                sys.exit(1)
    return frame, start_xy, goal_xy, field, corridor


def describe_geometry(frame, goal_xy, corridor, args):
    bearing = (90.0 - math.degrees(math.atan2(goal_xy[1], goal_xy[0]))) % 360.0
    print("Ponto inicial (B): %.7f,%.7f" % (frame.lat0_deg, frame.lon0_deg))
    print("Ponto de retorno (A): x=%.1f m y=%.1f m, a %.1f m, rumo %.1f graus"
          % (goal_xy[0], goal_xy[1], math.hypot(goal_xy[0], goal_xy[1]), bearing))
    south, north = corridor.distances(goal_xy)
    print("Corredor (%s): o retorno fica a %.0f m da linha sul e a %.0f m da linha norte"
          % (args["corridor"], south, north))


def mode_gps(args):
    boat = RealBoat(args, motors=False, gps=True, imu=False)
    start = args["start"] or START_LATLON
    frame = LocalFrame(start[0], start[1]) if start else None
    install_signals()
    print("Lendo o GPS. Ctrl+C sai. Se nada aparecer, ainda nao ha fix.")
    last = None
    try:
        while True:
            time.sleep(1.0)
            fix = boat.last_gps()
            reader = boat.gps_reader
            if fix is None:
                print("sem fix (linhas rejeitadas: %d)" % reader.rejected)
                continue
            last = fix
            text = "lat=%.7f lon=%.7f idade=%.1fs sog=%.1f nos aceitas=%d rejeitadas=%d" % (
                fix[1], fix[2], boat.now() - fix[0], fix[3], reader.accepted, reader.rejected)
            if frame is not None:
                x, y = frame.to_xy(fix[1], fix[2])
                text += " x=%.1f y=%.1f" % (x, y)
            print(text)
    except StopRequested:
        print("\nSaindo.")
    finally:
        boat.close()
    if last is not None:
        print("Ultima posicao para copiar: %.7f,%.7f" % (last[1], last[2]))


def mode_check(args):
    frame, start_xy, goal_xy, field, corridor = resolve_geometry(args)
    boat = RealBoat(args, motors=False, gps=True)
    kf = PositionKalman(args["kf_acc_sigma"], args["kf_gps_sigma"], args["kf_gate"],
                        args["kf_reset_after"])
    install_signals()
    describe_geometry(frame, goal_xy, corridor, args)
    print("Calibracao:", boat.calib_path)
    print("Modo check: motores NAO sao ligados. Ctrl+C para sair.")
    try:
        while True:
            theta, h, mag = boat.heading()
            now = boat.now()
            fix = boat.pop_gps()
            if kf.initialized():
                kf.predict(now)
            if fix is not None:
                x, y = frame.to_xy(fix[1], fix[2])
                kf.update(x, y, now)
            last_fix = boat.last_gps()
            if not kf.initialized():
                text = "rumo=%5.1f sem fix do GPS" % h
            else:
                position = kf.position()
                guidance = compute_guidance(position, theta, goal_xy, field, args, args["speed"])
                age = now - last_fix[0]
                south, north = corridor.distances(position)
                outside = min(corridor.distances(position)) < 0.0
                text = ("rumo=%5.1f alvo=%5.1f erro=%+6.1f d=%5.1f cmd=(%3d,%3d) x=%6.1f y=%6.1f gps=%.1f cor=%+.0f/%+.0f%s%s"
                        % (h, theta_to_compass(guidance["desired"]), -math.degrees(guidance["err"]),
                           guidance["distance"], guidance["left"], guidance["right"],
                           position[0], position[1], age, south, north,
                           " FORA" if outside else "",
                           " CHEGADA" if guidance["distance"] <= args["arrival_radius"] else ""))
            print(text)
            time.sleep(0.5)
    except StopRequested:
        print("\nSaindo.")
    finally:
        boat.close()


def mode_run(args):
    if os.path.isfile(STOP_FILE):
        print("ERRO: existe %s. Apague antes (rm data/STOP)." % STOP_FILE)
        sys.exit(1)
    if not sys.stdin.isatty():
        print("ERRO: o modo run precisa de um terminal (ssh ou screen). Nao rode com nohup.")
        sys.exit(1)
    frame, start_xy, goal_xy, field, corridor = resolve_geometry(args)
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
    boat = RealBoat(args, motors=True, gps=True)
    install_signals()
    boat.stop()
    keys = TtyKeys()
    print("Calibracao:", boat.calib_path)
    describe_geometry(frame, goal_xy, corridor, args)
    log_path = new_log_path("mission2")
    try:
        run_mission(boat, keys, args, frame, field, start_xy, goal_xy, corridor, log_path,
                    kml_path=log_path[:-4] + ".kml")
    finally:
        keys.close()
        boat.close()


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
    string_options = ("manual_reverse", "calib", "drivers", "corridor", "turn_mode")
    latlon_options = ("start", "goal")
    int_options = ("speed", "min_speed", "kf_reset_after")
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
        elif key in latlon_options:
            args[key] = parse_latlon(value, flag)
        elif key in int_options:
            try:
                args[key] = int(value)
            except ValueError:
                print("ERRO: --%s precisa de um numero inteiro" % key.replace("_", "-"))
                sys.exit(1)
        else:
            try:
                args[key] = float(value)
            except ValueError:
                print("ERRO: %s precisa de numero" % flag)
                sys.exit(1)
        i += 2
    if args["manual_reverse"] not in ("yes", "no"):
        print("ERRO: --manual-reverse yes ou no")
        sys.exit(1)
    if not 0 <= args["speed"] <= CMD_MAX or not 0 <= args["min_speed"] <= CMD_MAX:
        print("ERRO: --speed e --min-speed entre 0 e %d" % CMD_MAX)
        sys.exit(1)
    positive = ("arrival_radius", "point_intensity", "segment_intensity", "speed_gain", "gps_timeout_s",
                "kf_acc_sigma", "kf_gps_sigma", "kf_gate", "hz", "kp", "turn_max")
    for key in positive:
        if args[key] <= 0:
            print("ERRO: --%s deve ser > 0" % key.replace("_", "-"))
            sys.exit(1)
    if args["corridor"] not in ("off", "arm", "strict"):
        print("ERRO: --corridor off, arm ou strict")
        sys.exit(1)
    if args["turn_mode"] not in ("arc", "pivot"):
        print("ERRO: --turn-mode arc ou pivot")
        sys.exit(1)
    if args["corridor_margin"] < 0 or args["corridor_arm"] < 0:
        print("ERRO: --corridor-margin e --corridor-arm devem ser >= 0")
        sys.exit(1)
    if not 0 < args["pivot_off"] < args["pivot_on"] < 180:
        print("ERRO: use 0 < --pivot-off < --pivot-on < 180")
        sys.exit(1)
    if args["kf_reset_after"] < 1:
        print("ERRO: --kf-reset-after deve ser >= 1")
        sys.exit(1)
    return args


if __name__ == "__main__":
    modes = {"calib": mode_calib, "gps": mode_gps, "check": mode_check, "run": mode_run, "kml": mode_kml}
    if len(sys.argv) < 2 or sys.argv[1] not in modes:
        print(USAGE)
        sys.exit(1)
    if sys.argv[1] == "kml":
        if len(sys.argv) < 3:
            print(USAGE)
            sys.exit(1)
        kml_args = parse_args(sys.argv[3:])
        kml_args["input"] = sys.argv[2]
        mode_kml(kml_args)
    else:
        modes[sys.argv[1]](parse_args(sys.argv[2:]))
