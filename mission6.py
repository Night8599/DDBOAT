import json
import math
import os
import signal
import sys
import threading
import time
import types

from ddboats_client import get_boats, send_position

ROOT = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(ROOT, "data", "logs")
STOP_FILE = os.path.join(ROOT, "data", "STOP")
ARDUINO_TTY = "/dev/ttyV0"
ESC = "\x1b"
EARTH = 6378137.0
SERVER_IP = "172.20.26.99"
SERVER_PORT = 6800
BOAT_ID = 4
SEND_PERIOD = 1.0
POINTS = {"B": (48.199072, -3.016258), "BUOY": (48.198944, -3.015478)}
CONFIG = {"speed": 100, "kp": 60.0, "turn_max": 90.0, "hz": 10.0, "gps_timeout_s": 3.0,
          "kf_acc_sigma": 0.3, "kf_gps_sigma": 2.0, "kf_gate": 9.21, "kf_reset_after": 5,
          "min_cmd": 50, "boat_max_age_s": 5.0, "other_id": 7, "pred_max_s": 4.0, "pred_gain": 0.5,
          "turn_side": 1, "turn_radius": 8.0, "ref_speed": 0.4, "pass_m": 25.0, "out_shift_m": 25.0,
          "lead_m": 3.0, "k_along": 15.0, "kv": 0.0073, "hold_lag_m": 4.0, "sep_max": 17.0,
          "stable_m": 3.0, "min_sep": 6.0, "max_sep": 14.0, "follow_m": 3.0, "final_m": 8.0,
          "end_wait_s": 40.0, "ready_reads": 5, "done_radius": 3.0, "rope_m": 20.0,
          "comm_abort_s": 30.0, "linger_s": 20.0, "dry": 0, "comp_c0": 13.9, "comp_c1": -7.5, "comp_c2": 38.3,
          "comp_learn": 1, "comp_win_s": 12.0, "comp_win_m": 4.0, "comp_spread": 8.0, "comp_every_s": 4.0,
          "comp_half_s": 150.0, "comp_noise": 8.0, "comp_prior": 15.0, "comp_max_dev": 30.0, "comp_limit": 30.0}
COLUMNS = ["t_s", "state", "lat", "lon", "gps_age_s", "gps_new", "x_m", "y_m", "vx_mps", "vy_mps", "s_ref_m",
           "s_own_m", "s_par_m", "dist_ref_m", "sep_m", "heading_deg", "course_deg", "comp_off_deg", "error_deg",
           "speed_cmd", "key", "cmd_left", "cmd_right", "kf", "partner_n", "hold", "buoy_in_u", "mag_x", "mag_y", "mag_z"]
LAT0, LON0 = math.radians(POINTS["B"][0]), math.radians(POINTS["B"][1])

def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def to_theta(compass_deg):
    return wrap(math.radians(90.0 - compass_deg))


def to_compass(theta):
    return (90.0 - math.degrees(theta)) % 360.0


def mix(thrust, turn):
    outer = int(round(thrust))
    inner = int(round(thrust - min(2.0 * abs(turn), float(thrust))))
    return (inner, outer) if turn > 0 else (outer, inner)


def xy(lat, lon):
    return EARTH * math.cos(LAT0) * (math.radians(lon) - LON0), EARTH * (math.radians(lat) - LAT0)


def latlon(x, y):
    return math.degrees(LAT0 + y / EARTH), math.degrees(LON0 + x / (EARTH * math.cos(LAT0)))


XY = dict((k, xy(*v)) for k, v in POINTS.items() if v)


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class OtherBoats(object):
    def __init__(self):
        self.positions = []
        self.fresh = {}
        self.last_seen = {}
        self.hist = {}

    @property
    def count(self):
        return len(self.positions)

    def velocity(self, boat_id, t_pos, p):
        h = self.hist.setdefault(boat_id, [])
        if not h or t_pos > h[-1][0] + 0.5:
            h.append((t_pos, p[0], p[1]))
            del h[:-5]
        if len(h) < 3:
            return 0.0, 0.0
        t = [r[0] for r in h]
        tm = sum(t) / len(t)
        stt = sum((x - tm) ** 2 for x in t)
        if stt < 1e-6:
            return 0.0, 0.0
        ve = sum((r[0] - tm) * r[1] for r in h) / stt
        vn = sum((r[0] - tm) * r[2] for r in h) / stt
        speed = math.hypot(ve, vn)
        return (ve, vn) if speed <= 1.5 else (ve * 1.5 / speed, vn * 1.5 / speed)

    def update(self, boats, now):
        self.positions = []
        self.fresh = {}
        for boat_id, b in boats.items():
            if boat_id == BOAT_ID:
                continue
            self.last_seen[boat_id] = (b["lat"], b["lon"])
            if b["age"] > CONFIG["boat_max_age_s"]:
                continue
            p = xy(b["lat"], b["lon"])
            v = self.velocity(boat_id, now - b["age"], p)
            lead = CONFIG["pred_gain"] * min(b["age"], CONFIG["pred_max_s"])
            p = (p[0] + v[0] * lead, p[1] + v[1] * lead)
            self.positions.append(p)
            self.fresh[boat_id] = (p, b["cap"])


def guidance(p, theta, target, cmd):
    desired = math.atan2(target[1] - p[1], target[0] - p[0])
    err = wrap(desired - theta)
    lim = min(CONFIG["turn_max"], float(cmd))
    left, right = mix(cmd, max(-lim, min(lim, CONFIG["kp"] * err)))
    return {"dist": math.hypot(target[0] - p[0], target[1] - p[1]), "desired": desired, "err": err,
            "cmd": cmd, "left": left, "right": right}


class Tracker(object):
    def __init__(self):
        self.x = None

    def reset(self, px, py, t):
        self.x = [px, py, 0.0, 0.0]
        self.p = [CONFIG["kf_gps_sigma"] ** 2, 0.0, 1.0]
        self.t, self.rejected = t, 0

    def position(self):
        return None if self.x is None else (self.x[0], self.x[1])

    def predict(self, t):
        if self.x is None or t <= self.t:
            return
        dt, a2 = t - self.t, CONFIG["kf_acc_sigma"] ** 2
        p00, p01, p11 = self.p
        self.x[0] += self.x[2] * dt
        self.x[1] += self.x[3] * dt
        self.p = [p00 + 2.0 * dt * p01 + dt * dt * p11 + a2 * dt ** 4 / 4.0,
                  p01 + dt * p11 + a2 * dt ** 3 / 2.0, p11 + a2 * dt * dt]
        self.t = t

    def update(self, zx, zy, t):
        if self.x is None:
            self.reset(zx, zy, t)
            return "init"
        r, (p00, p01, p11) = CONFIG["kf_gps_sigma"] ** 2, self.p
        s = p00 + r
        if ((zx - self.x[0]) ** 2 + (zy - self.x[1]) ** 2) / s > CONFIG["kf_gate"]:
            self.rejected += 1
            if self.rejected >= CONFIG["kf_reset_after"]:
                self.reset(zx, zy, t)
                return "reset"
            return "rej"
        k0, k1 = p00 / s, p01 / s
        for i, z in ((0, zx), (1, zy)):
            innovation = z - self.x[i]
            self.x[i] += k0 * innovation
            self.x[i + 2] += k1 * innovation
        a = 1.0 - k0
        self.p = [a * a * p00 + r * k0 * k0, a * (p01 - k1 * p00) + r * k0 * k1,
                  k1 * k1 * p00 - 2.0 * k1 * p01 + p11 + r * k1 * k1]
        self.rejected = 0
        return "ok"


def compass_heading(m, c):
    if c.get("format") == 2:
        x, y = m[0] - c["cx"], m[1] - c["cy"]
        a = math.degrees(math.atan2(c["m10"] * x + c["m11"] * y, c["m00"] * x + c["m01"] * y))
    else:
        a = math.degrees(math.atan2((m[1] - c["offy"]) * c["sy"], (m[0] - c["offx"]) * c["sx"]))
    return (c["sign"] * ((a - c["north"] + 180.0) % 360.0 - 180.0)) % 360.0


def load_calibration():
    for path in (os.path.join(ROOT, "data", "calib", "mag2d.json"), os.path.join(ROOT, "calib_cap.json")):
        if os.path.isfile(path):
            with open(path) as f:
                return json.load(f)
    sys.exit("compass calibration file not found: data/calib/mag2d.json")


def load_drivers():
    for path in (os.path.join(ROOT, "src", "DriversDDBOAT_2026"),
                 os.path.expanduser("~/GKKDossier/drivers-ddboat-v2")):
        if os.path.isfile(os.path.join(path, "imu9_driver_v2.py")):
            sys.path.insert(0, path)
            break
    else:
        sys.exit("drivers not found")
    try:
        __import__("drivers.i2creal")
    except ImportError:
        import i2creal
        package = types.ModuleType("drivers")
        package.__path__ = []
        package.i2creal = i2creal
        sys.modules["drivers"] = package
        sys.modules["drivers.i2creal"] = i2creal


def ddmm(value, hemisphere):
    v = float(value)
    d = int(v / 100)
    result = d + (v - d * 100) / 60.0
    return -result if hemisphere in ("S", "W") else result


class Gps(threading.Thread):
    def __init__(self, device, clock):
        threading.Thread.__init__(self)
        self.daemon = True
        self.device, self.clock, self.lock = device, clock, threading.Lock()
        self.latest, self.fresh = None, False

    def run(self):
        while True:
            try:
                ok, d = self.device.read_rmc_non_blocking()
                if ok and not (d[0] == 0.0 and d[2] == 0.0):
                    lat, lon = ddmm(d[0], d[1]), ddmm(d[2], d[3])
                    if -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0:
                        with self.lock:
                            self.latest, self.fresh = (self.clock(), lat, lon), True
            except Exception:
                pass
            time.sleep(0.01)

    def pop(self):
        with self.lock:
            fix = self.latest if self.fresh else None
            self.fresh = False
            return fix


class Boat(object):
    def __init__(self):
        self.t0 = time.time()
        self.calibration = load_calibration()
        load_drivers()
        import imu9_driver_v2
        import gps_driver_v2
        import arduino_driver_v2
        self.imu = imu9_driver_v2.Imu9IO()
        try:
            device = gps_driver_v2.GpsIO(0)
            device.send_mtk_cmd("PMTK386,0")
        except Exception as e:
            sys.exit("GPS not initialized: %r" % (e,))
        self.gps = Gps(device, self.now)
        self.gps.start()
        self.arduino = arduino_driver_v2.ArduinoIO()
        if self.arduino.arduino is None:
            sys.exit("Arduino not initialized")

    def now(self):
        return time.time() - self.t0

    def sleep(self, dt):
        if dt > 0:
            time.sleep(dt)

    def heading(self):
        samples = []
        for _ in range(3):
            samples.append(self.imu.read_mag_raw())
            time.sleep(0.01)
        mag = [sorted(v[i] for v in samples)[1] for i in range(3)]
        return compass_heading(mag, self.calibration), mag

    def pop_gps(self):
        return self.gps.pop()

    def last_gps(self):
        return self.gps.latest

    def motors(self, left, right):
        self.arduino.send_arduino_cmd_motor(int(left), int(right))

    def stop(self):
        for _ in range(3):
            try:
                self.arduino.send_arduino_cmd_motor(0, 0)
            except Exception:
                pass
            time.sleep(0.05)


class Stop(Exception):
    pass


def on_signal(sig, frame):
    raise Stop("signal %d" % sig)


def set_signals(handler):
    for name in ("SIGINT", "SIGTERM", "SIGHUP"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), handler)


def port_users(port):
    target, users = os.path.realpath(port), []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            for fd in os.listdir("/proc/%s/fd" % pid):
                if os.path.realpath(os.readlink("/proc/%s/fd/%s" % (pid, fd))) == target:
                    with open("/proc/%s/cmdline" % pid) as f:
                        users.append("PID %s: %s" % (pid, f.read().replace("\0", " ").strip()))
                    break
        except (IOError, OSError):
            continue
    return users


def parse_keys(buf):
    keys, i = [], 0
    while i < len(buf):
        if buf[i] == ESC and buf[i + 1:i + 2] in ("[", "O"):
            i += 2
            while i < len(buf) and buf[i] in "0123456789;":
                i += 1
            i += 1
        else:
            keys.append(buf[i] if buf[i] == ESC else buf[i].lower())
            i += 1
    return keys


def key_name(k):
    names = {"\n": "enter", "\r": "enter", ESC: "esc", "\x7f": "bksp", "\x08": "bksp"}
    return k if k.isalnum() else names.get(k, "x%02x" % ord(k))


class Keys(object):
    def __init__(self):
        import termios
        import tty
        self.termios = termios
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def read(self):
        import select
        buf = ""
        while select.select([self.fd], [], [], 0)[0]:
            chunk = os.read(self.fd, 64)
            if not chunk:
                break
            buf += chunk.decode("latin-1")
        return parse_keys(buf)

    def close(self):
        try:
            self.termios.tcsetattr(self.fd, self.termios.TCSADRAIN, self.saved)
        except (OSError, self.termios.error):
            pass


class CourseFix(object):
    def __init__(self):
        self.c0 = [CONFIG["comp_c0"], CONFIG["comp_c1"], CONFIG["comp_c2"]]
        self.c = list(self.c0)
        self.hist, self.samples, self.next_t = [], [], 0.0

    def offset(self, h):
        r = math.radians(h)
        base = self.c0[0] + self.c0[1] * math.cos(r) + self.c0[2] * math.sin(r)
        value = self.c[0] + self.c[1] * math.cos(r) + self.c[2] * math.sin(r)
        return base + max(-CONFIG["comp_limit"], min(CONFIG["comp_limit"], value - base))

    def apply(self, h):
        return (h - self.offset(h)) % 360.0

    def sample(self, now, pos, h):
        if not self.hist or now - self.hist[-1][0] >= 0.5:
            self.hist.append((now, pos[0], pos[1], h))
        while self.hist and now - self.hist[0][0] > CONFIG["comp_win_s"]:
            self.hist.pop(0)
        if now < self.next_t or len(self.hist) < 4:
            return None
        dx, dy = self.hist[-1][1] - self.hist[0][1], self.hist[-1][2] - self.hist[0][2]
        if math.hypot(dx, dy) < CONFIG["comp_win_m"]:
            return None
        ref = self.hist[-1][3]
        dev = [wrap_deg(r[3] - ref) for r in self.hist]
        if max(dev) - min(dev) > CONFIG["comp_spread"]:
            return None
        hm = (ref + sum(dev) / len(dev)) % 360.0
        r = math.radians(hm)
        return [1.0, math.cos(r), math.sin(r)], wrap_deg(hm - to_compass(math.atan2(dy, dx)))

    def learn(self, now, pos, h):
        if pos is None or not CONFIG["comp_learn"]:
            return
        got = self.sample(now, pos, h)
        if got is None:
            return
        phi, z = got
        if abs(wrap_deg(z - self.offset(math.degrees(math.atan2(phi[2], phi[1]))))) > 50.0:
            return
        self.next_t = now + CONFIG["comp_every_s"]
        self.samples = (self.samples + [(now, phi, z)])[-300:]
        lam = 1.0 / CONFIG["comp_prior"] ** 2
        a = [[lam if i == j else 0.0 for j in range(3)] for i in range(3)]
        b = [lam * v for v in self.c0]
        for t, p, zz in self.samples:
            w = 0.5 ** ((now - t) / CONFIG["comp_half_s"]) / CONFIG["comp_noise"] ** 2
            for i in range(3):
                b[i] += w * p[i] * zz
                for j in range(3):
                    a[i][j] += w * p[i] * p[j]
        for i in range(3):
            k = max(range(i, 3), key=lambda r: abs(a[r][i]))
            a[i], a[k], b[i], b[k] = a[k], a[i], b[k], b[i]
            for r in range(i + 1, 3):
                f = a[r][i] / a[i][i]
                b[r] -= f * b[i]
                for j in range(i, 3):
                    a[r][j] -= f * a[i][j]
        c = [0.0, 0.0, 0.0]
        for i in (2, 1, 0):
            c[i] = (b[i] - sum(a[i][j] * c[j] for j in range(i + 1, 3))) / a[i][i]
        dev = CONFIG["comp_max_dev"]
        self.c = [max(self.c0[i] - dev, min(self.c0[i] + dev, c[i])) for i in range(3)]


class Plan(object):
    def __init__(self, start, other, buoy):
        mx, my = (start[0] + other[0]) / 2.0, (start[1] + other[1]) / 2.0
        d = math.hypot(buoy[0] - mx, buoy[1] - my)
        self.f = ((buoy[0] - mx) / d, (buoy[1] - my) / d)
        side = 1.0 if CONFIG["turn_side"] >= 0 else -1.0
        self.n = (self.f[1] * side, -self.f[0] * side)
        self.start, self.radius = start, CONFIG["turn_radius"]
        self.lat = abs((other[0] - start[0]) * self.n[0] + (other[1] - start[1]) * self.n[1])
        self.shift = 2.0 * self.radius
        self.l1 = d + CONFIG["pass_m"]
        self.arc = math.pi * self.radius
        self.e = (start[0] + self.l1 * self.f[0] - self.shift * self.n[0], start[1] + self.l1 * self.f[1] - self.shift * self.n[1])
        self.w = (start[0] + self.l1 * self.f[0], start[1] + self.l1 * self.f[1])
        self.back = self.l1
        self.total = self.l1 + self.arc + self.back

    def pos(self, s):
        s = max(0.0, min(self.total, s))
        if s <= self.l1:
            u = min(1.0, s / CONFIG["out_shift_m"])
            k = self.shift * u * u * (3.0 - 2.0 * u)
            return (self.start[0] + s * self.f[0] - k * self.n[0], self.start[1] + s * self.f[1] - k * self.n[1]), 1
        if s <= self.l1 + self.arc:
            phi = (s - self.l1) / self.radius
            k1, k2 = 1.0 - math.cos(phi), math.sin(phi)
            return (self.e[0] + self.radius * (self.n[0] * k1 + self.f[0] * k2),
                    self.e[1] + self.radius * (self.n[1] * k1 + self.f[1] * k2)), 2
        u = (s - self.l1 - self.arc) / self.back
        return (self.w[0] + u * (self.start[0] - self.w[0]), self.w[1] + u * (self.start[1] - self.w[1])), 3

    def direction(self, s):
        a, b = self.pos(s - 0.5)[0], self.pos(s + 0.5)[0]
        n = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
        return (b[0] - a[0]) / n, (b[1] - a[1]) / n

    def project(self, p, s0):
        best, found = 1e18, s0
        s, hi = max(0.0, s0 - 2.0), min(self.total, s0 + 12.0)
        while s <= hi + 1e-9:
            q = self.pos(s)[0]
            dd = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2
            if dd < best:
                best, found = dd, s
            s += 0.25
        return found


def in_u(a, b, buoy, forward):
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    rope = CONFIG["rope_m"]
    if d < 1.0 or d >= rope:
        return 0
    h = math.sqrt((rope / 2.0) ** 2 - (d / 2.0) ** 2)
    g = ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
    px, py = buoy[0] - (a[0] + b[0]) / 2.0, buoy[1] - (a[1] + b[1]) / 2.0
    along, side = px * forward[0] + py * forward[1], px * g[0] + py * g[1]
    return 1 if -h <= along <= 0.0 and abs(side) <= d / 2.0 * (1.0 + along / h) else 0


class Mission(object):
    def __init__(self):
        self.plan = self.plan_o = self.start = self.start_other = None
        self.own_hist, self.par_hist, self.warned, self.end_t = [], [], False, None
        self.state = "WAIT"
        self.finished = self.fresh = False
        self.theta = self.now = 0.0
        self.s_ref = self.s_own = 0.0
        self.s_par = None
        self.hold = self.in_u = 0
        self.pos = self.age = self.g = self.last = self.partner = self.sep = self.lost_t = None
        self.end = "done"
        self.others = OtherBoats()

    def say(self, text):
        print("\n" + text)

    def stage(self):
        return "WAIT" if self.state == "WAIT" else ("S%d" % self.plan.pos(self.s_ref)[1] if self.state == "RUN" else self.state)

    def update(self, theta, now, pos, age):
        dt = 0.0 if self.last is None else max(0.0, min(0.3, now - self.last))
        self.last = now
        self.theta, self.now, self.pos, self.age = theta, now, pos, age
        self.fresh = pos is not None and age is not None and age <= CONFIG["gps_timeout_s"]
        other = self.others.fresh.get(CONFIG["other_id"])
        self.partner = None if other is None else other[0]
        self.sep = None
        if self.fresh and self.partner is not None:
            self.sep = math.hypot(pos[0] - self.partner[0], pos[1] - self.partner[1])
        if self.state == "RUN" and self.fresh:
            self.progress(dt)
        self.compute()

    def progress(self, dt):
        self.s_own = self.plan.project(self.pos, self.s_own)
        if self.partner is None:
            self.hold = 1
            self.lost_t = self.now if self.lost_t is None else self.lost_t
            return
        self.lost_t = None
        self.s_par = self.plan_o.project(self.partner, 0.0 if self.s_par is None else self.s_par)
        final = self.s_ref >= self.plan.total - CONFIG["final_m"]
        self.hold = 1 if not final and (self.s_ref - min(self.s_own, self.s_par) > CONFIG["hold_lag_m"] or self.sep > CONFIG["sep_max"]) else 0
        if not self.hold:
            self.s_ref = min(self.plan.total, self.s_ref + CONFIG["ref_speed"] * dt)
        self.in_u = in_u(self.pos, self.partner, XY["BUOY"], self.plan.direction(self.s_ref))

    def compute(self):
        self.g = None
        if not self.fresh or self.state != "RUN":
            return
        target = self.plan.pos(self.s_ref + CONFIG["lead_m"])[0]
        u = self.plan.direction(self.s_ref)
        along = (target[0] - self.pos[0]) * u[0] + (target[1] - self.pos[1]) * u[1]
        dist = math.hypot(target[0] - self.pos[0], target[1] - self.pos[1])
        cmd = CONFIG["speed"] if dist > 6.0 else CONFIG["ref_speed"] / CONFIG["kv"] + CONFIG["k_along"] * along
        cmd = int(round(max(float(CONFIG["min_cmd"]), min(float(CONFIG["speed"]), cmd))))
        self.g = guidance(self.pos, self.theta, target, cmd)
        self.g["dist"] = dist

    def reading(self):
        if self.state != "WAIT":
            return
        n = CONFIG["ready_reads"]
        if not (self.fresh and self.partner is not None):
            self.own_hist, self.par_hist = [], []
            return
        self.own_hist = (self.own_hist + [self.pos])[-8:]
        self.par_hist = (self.par_hist + [self.partner])[-8:]
        if len(self.own_hist) < n:
            return

        def spread(pts):
            return max(math.hypot(p[0] - q[0], p[1] - q[1]) for p in pts for q in pts)

        own, par = self.own_hist[-n:], self.par_hist[-n:]
        if spread(own) > CONFIG["stable_m"]:
            return
        left = math.hypot(self.par_hist[-1][0] - self.par_hist[0][0], self.par_hist[-1][1] - self.par_hist[0][1]) > CONFIG["follow_m"]
        if spread(par) > CONFIG["stable_m"] and not left:
            return
        start = (sum(p[0] for p in own) / n, sum(p[1] for p in own) / n)
        other = self.par_hist[0] if left else (sum(p[0] for p in par) / n, sum(p[1] for p in par) / n)
        sep = math.hypot(start[0] - other[0], start[1] - other[1])
        plan = Plan(start, other, XY["BUOY"])
        if not left and not CONFIG["min_sep"] <= plan.lat <= CONFIG["max_sep"]:
            if not self.warned:
                self.say("WAITING: lateral separation %.1f m (distance %.1f m) is outside %.0f to %.0f m" % (
                    plan.lat, sep, CONFIG["min_sep"], CONFIG["max_sep"]))
                self.warned = True
            return
        self.start, self.start_other = start, other
        self.plan = plan
        self.plan_o = Plan(other, start, XY["BUOY"])
        self.state, self.s_ref, self.s_own, self.s_par = "RUN", 0.0, 0.0, None
        self.say("START: %s, separation %.1f m, buoy %.0f m ahead, plan %.0f m (reference %.2f m/s)" % (
            "boat %d already left" % CONFIG["other_id"] if left else "both boats still", sep,
            math.hypot(XY["BUOY"][0] - (start[0] + other[0]) / 2.0, XY["BUOY"][1] - (start[1] + other[1]) / 2.0),
            self.plan.total, CONFIG["ref_speed"]))

    def abort(self, text):
        self.state, self.finished, self.end = "STOPPED", True, "aborted"
        self.say(text)

    def on_key(self, k):
        if k == "q":
            self.abort("ABORTED (key q)")
        else:
            self.say("Only q aborts.")

    def control(self):
        if self.finished:
            return 0, 0
        if self.g is None:
            return 0, 0
        if self.state == "RUN":
            if self.lost_t is not None and self.now - self.lost_t > CONFIG["comm_abort_s"]:
                self.state, self.finished, self.end = "STOPPED", True, "partner lost"
                self.say("PARTNER LOST for over %.0f s: stopped" % CONFIG["comm_abort_s"])
                return 0, 0
            if self.s_ref >= self.plan.total - 0.01 and self.end_t is None:
                self.end_t = self.now
            if self.end_t is not None and (math.hypot(self.pos[0] - self.start[0], self.pos[1] - self.start[1]) <= CONFIG["done_radius"]
                                           or self.now - self.end_t > CONFIG["end_wait_s"]):
                self.state, self.finished = "STOPPED", True
                self.say("DONE: back at the start point, d=%.1f m" % math.hypot(self.pos[0] - self.start[0], self.pos[1] - self.start[1]))
                return 0, 0
        return self.g["left"], self.g["right"]

    def targets(self):
        if self.g is None:
            return None, None, None
        return self.g["desired"], self.g["err"], self.g["cmd"]

    def status(self, h, left, right):
        g = self.g
        age = "---" if self.age is None else "%.1f" % self.age
        cmd = "(%3d,%3d)" % (left, right)
        if g is not None and self.state in ("WAIT", "RUN"):
            text = "%-4s s=%5.1f/%5.1f hdg=%5.1f e=%+6.1f d=%5.1f sep=%s h=%d u=%d %s g=%s" % (
                self.stage(), self.s_ref, self.plan.total, h, -math.degrees(g["err"]), g["dist"],
                "---" if self.sep is None else "%.1f" % self.sep, self.hold, self.in_u, cmd, age)
        else:
            text = "%-7s hdg=%5.1f %s gps=%s" % (self.state, h, cmd, age)
        return text[:79].ljust(79)


def write_kml(path, track, raw, others, mission):
    def pt(label, p, style):
        return ("<Placemark><name>%s</name><styleUrl>#%s</styleUrl><Point><coordinates>%.7f,%.7f,0"
                "</coordinates></Point></Placemark>" % (label, style, p[1], p[0]))

    def ln(pts, style, visible=1):
        return ("<Placemark><visibility>%d</visibility><styleUrl>#%s</styleUrl><LineString><tessellate>1"
                "</tessellate><coordinates>%s</coordinates></LineString></Placemark>"
                % (visible, style, " ".join("%.7f,%.7f,0" % (lo, la) for la, lo in pts)))

    styles = "".join('<Style id="%s"><IconStyle><color>%s</color><scale>1.2</scale><Icon><href>'
                     'http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href></Icon>'
                     '</IconStyle></Style>' % s for s in (("a", "ff00ff00"), ("b", "ffff0000"), ("buoy", "ff00ffff"),
                                                          ("end", "ffff00ff"), ("other", "ff00a5ff")))
    styles += "".join('<Style id="%s"><LineStyle><color>%s</color><width>%d</width></LineStyle></Style>' % s
                      for s in (("route", "ff00ff00", 4), ("raw", "ff0080ff", 2), ("plan", "ffffffff", 3),
                                ("plano", "ff00ffff", 3)))
    items = [pt("buoy G", latlon(*XY["BUOY"]), "buoy")]
    if mission.plan is not None:
        items += [pt("start", latlon(*mission.start), "a"), pt("partner start", latlon(*mission.start_other), "b")]
        for plan, style in ((mission.plan, "plan"), (mission.plan_o, "plano")):
            items.append(ln([latlon(*plan.pos(i * 0.5)[0]) for i in range(int(plan.total / 0.5) + 1)], style))
    if len(track) > 1:
        items.append(ln(track, "route"))
    if len(raw) > 1:
        items.append(ln(raw, "raw", 0))
    if track:
        items += [pt("i", track[0], "end"), pt("f", track[-1], "end")]
    items += [pt("boat %d" % boat_id, others[boat_id], "other") for boat_id in sorted(others)]
    with open(path, "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2"><Document>'
                "%s%s</Document></kml>\n" % (styles, "".join(items)))


def run(boat, keys, mission, log_path, kml_path):
    kf, comp, track, raw, pos = Tracker(), CourseFix(), [], [], None
    period, reason, last_track, last_flush, next_send = 1.0 / CONFIG["hz"], None, -1.0, 0.0, None
    finish_t = None
    log = open(log_path, "w")
    log.write(",".join(COLUMNS) + "\n")
    boats_log = open(log_path[:-4] + "_boats.csv", "w")
    boats_log.write("t_s,id,lat,lon,cap_deg,age_s\n")
    try:
        print("Wait for boat %d, start from the current positions, go to the buoy at G, turn and come back. Only q aborts. Ctrl+C exits." % CONFIG["other_id"])
        while True:
            started = boat.now()
            if os.path.isfile(STOP_FILE):
                reason = "STOP file"
                break
            h_raw, mag = boat.heading()
            now = boat.now()
            fix = boat.pop_gps()
            kf.predict(now)
            result = ""
            if fix is not None:
                x, y = xy(fix[1], fix[2])
                result = kf.update(x, y, now)
                raw.append((fix[1], fix[2]))
            pos = kf.position()
            comp.learn(now, pos, h_raw)
            h = comp.apply(h_raw)
            theta = to_theta(h)
            if pos is not None and now - last_track >= 0.5:
                track.append(latlon(*pos))
                last_track = now
            cycle = False
            if pos is not None and (next_send is None or now >= next_send - period / 2.0):
                next_send = SEND_PERIOD + (now if next_send is None or now - next_send > SEND_PERIOD else next_send)
                lat, lon = latlon(*pos)
                try:
                    send_position(SERVER_IP, SERVER_PORT, BOAT_ID, lat, lon, h)
                except Exception:
                    pass
                try:
                    boats = get_boats()
                except Exception:
                    boats = {}
                mission.others.update(boats, now)
                cycle = True
                for boat_id, b in sorted(boats.items()):
                    boats_log.write("%.3f,%d,%.7f,%.7f,%.1f,%.1f\n" % (now, boat_id, b["lat"], b["lon"], b["cap"], b["age"]))
            last = boat.last_gps()
            mission.update(theta, now, pos, None if last is None else now - last[0])
            if cycle:
                mission.reading()
            pressed = keys.read()
            for k in pressed:
                mission.on_key(k)
            left, right = mission.control()
            boat.motors(0, 0) if CONFIG["dry"] else boat.motors(left, right)
            desired, err, speed_cmd = mission.targets()
            row = [now, mission.stage(), "" if last is None else "%.7f" % last[1], "" if last is None else "%.7f" % last[2],
                   mission.age, 1 if fix is not None else 0, pos and pos[0], pos and pos[1], pos and kf.x[2], pos and kf.x[3],
                   mission.s_ref, mission.s_own, mission.s_par, mission.g and mission.g["dist"], mission.sep, h_raw, h, comp.offset(h_raw),
                   None if err is None else -math.degrees(err), speed_cmd, "+".join(key_name(k) for k in pressed),
                   left, right, result, mission.others.count, mission.hold, mission.in_u] + list(mag)
            log.write(",".join("" if v is None else "%.3f" % v if isinstance(v, float) else str(v) for v in row) + "\n")
            if now - last_flush >= 5.0:
                log.flush()
                boats_log.flush()
                last_flush = now
            sys.stdout.write("\r" + mission.status(h, left, right))
            sys.stdout.flush()
            if mission.finished:
                reason = mission.end
                if finish_t is None:
                    finish_t = now
                if reason == "aborted" or now - finish_t >= CONFIG["linger_s"]:
                    break
            boat.sleep(period - (boat.now() - started))
    except Stop as e:
        reason = "interrupted (%s)" % e
    finally:
        set_signals(signal.SIG_IGN)
        boat.stop()
        log.close()
        boats_log.close()
        set_signals(signal.SIG_DFL)
        try:
            if pos is not None:
                track.append(latlon(*pos))
            write_kml(kml_path, track, raw, mission.others.last_seen, mission)
            print("\nKML: %s" % kml_path)
        except Exception as e:
            print("\nWARNING: KML not written (%r)" % (e,))
    print("\nEnd: %s. Motors stopped. Log: %s" % (reason, log_path))


def main():
    args = sys.argv[1:]
    if len(args) % 2:
        sys.exit("unknown option or missing value: %s (usage: python3 mission6.py [--option value ...])" % args[-1])
    for flag, value in zip(args[::2], args[1::2]):
        key = flag.lstrip("-").replace("-", "_")
        if not flag.startswith("--") or key not in CONFIG:
            sys.exit("unknown option %s (usage: python3 mission6.py [--option value ...])" % flag)
        CONFIG[key] = type(CONFIG[key])(value)
    if os.path.isfile(STOP_FILE):
        sys.exit("%s exists: remove it (rm data/STOP)" % STOP_FILE)
    if not sys.stdin.isatty():
        sys.exit("needs a terminal (ssh or screen), do not use nohup")
    users = port_users(ARDUINO_TTY)
    if users:
        sys.exit("%s is in use by another process (motors would shake): %s" % (ARDUINO_TTY, "; ".join(users)))
    boat = Boat()
    set_signals(on_signal)
    boat.stop()
    keys = Keys()
    os.makedirs(LOG_DIR, exist_ok=True)
    n = 1
    while os.path.exists(os.path.join(LOG_DIR, "mission6_%03d.csv" % n)):
        n += 1
    base = os.path.join(LOG_DIR, "mission6_%03d" % n)
    try:
        run(boat, keys, Mission(), base + ".csv", base + ".kml")
    finally:
        keys.close()


if __name__ == "__main__":
    main()
