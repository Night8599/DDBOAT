import json
import math
import os
import signal
import sys
import threading
import time
import types

ROOT = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(ROOT, "data", "logs")
STOP_FILE = os.path.join(ROOT, "data", "STOP")
ARDUINO_TTY = "/dev/ttyV0"
MAX_CMD = 191
ESC = "\x1b"
EARTH = 6378137.0
POINTS = {"A": (48.198863, -3.015070), "B": (48.199072, -3.016258), "C": (48.197408, -3.015060),
          "D": (48.1970689, -3.017328), "E": (48.199707, -3.016404), "F": (48.199814, -3.015137)}
NAMES = ("south line C-D", "north line E-F")
CONFIG = {"speed": 120, "kp": 60.0, "turn_max": 90.0, "hz": 10.0, "arrival_radius": 1.0,
          "near_radius": 3.0, "near_time_s": 4.0, "point_intensity": 1.0, "segment_intensity": 2.0,
          "speed_gain": 6.0, "min_speed": 50, "gps_timeout_s": 3.0, "corridor_margin": 3.0,
          "corridor_arm": 10.0, "barrier_overshoot": 5.0, "barrier_hold_s": 1.0,
          "barrier_timeout_s": 120.0, "slow_speed": 70, "slow_distance": 10.0, "kf_acc_sigma": 0.3,
          "kf_gps_sigma": 2.0, "kf_gate": 9.21, "kf_reset_after": 5}
COLUMNS = ["t_s", "state", "lat", "lon", "gps_age_s", "gps_new", "x_m", "y_m", "vx_mps", "vy_mps",
           "dist_m", "south_m", "north_m", "target_deg", "heading_deg", "error_deg", "speed",
           "speed_cmd", "key", "cmd_left", "cmd_right", "kf", "mag_x", "mag_y", "mag_z"]
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


GOAL = xy(*POINTS["A"])


def make_line(a, b):
    p, q = xy(*POINTS[a]), xy(*POINTS[b])
    n = math.hypot(q[0] - p[0], q[1] - p[1])
    nx, ny = (p[1] - q[1]) / n, (q[0] - p[0]) / n
    if (GOAL[0] / 2.0 - p[0]) * nx + (GOAL[1] / 2.0 - p[1]) * ny < 0:
        nx, ny = -nx, -ny
    return p, (nx, ny)


CORRIDOR = [make_line("C", "D"), make_line("E", "F")]


def gradient(p):
    t = max(0.0, min(1.0, (p[0] * GOAL[0] + p[1] * GOAL[1]) / (GOAL[0] * GOAL[0] + GOAL[1] * GOAL[1])))
    ip, isg = CONFIG["point_intensity"], CONFIG["segment_intensity"]
    return (-2.0 * (p[0] - GOAL[0]) * ip + -2.0 * (p[0] - t * GOAL[0]) * isg,
            -2.0 * (p[1] - GOAL[1]) * ip + -2.0 * (p[1] - t * GOAL[1]) * isg)


def guidance(p, theta, speed):
    gx, gy = gradient(p)
    cmd = int(round(min(float(speed), max(float(CONFIG["min_speed"]), CONFIG["speed_gain"] * math.hypot(gx, gy)))))
    desired = math.atan2(gy, gx)
    err = wrap(desired - theta)
    lim = min(CONFIG["turn_max"], float(cmd))
    left, right = mix(cmd, max(-lim, min(lim, CONFIG["kp"] * err)))
    return {"dist": math.hypot(GOAL[0] - p[0], GOAL[1] - p[1]), "desired": desired, "err": err,
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
        h = compass_heading(mag, self.calibration)
        return to_theta(h), h, mag

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


class Mission(object):
    def __init__(self):
        self.speed = CONFIG["speed"]
        self.state = "WAITING"
        self.finished = self.armed = self.deep = self.turn_done = self.slow = False
        self.theta = self.now = self.out_start = 0.0
        self.pos = self.age = self.g = self.dist = self.violated = self.paused = None
        self.ref = self.cross_since = self.near_since = None
        self.end = "arrived"
        self.recent = []

    def say(self, text):
        print("\n" + text)

    def limit(self):
        return min(self.speed, CONFIG["slow_speed"]) if self.slow else self.speed

    def update(self, theta, now, pos, age):
        self.theta, self.now, self.pos, self.age = theta, now, pos, age
        fresh = pos is not None and age is not None and age <= CONFIG["gps_timeout_s"]
        self.g = guidance(pos, theta, self.limit()) if fresh else None
        self.recent = (self.recent + [theta])[-10:]
        if pos is None:
            self.dist = self.violated = None
            self.deep = False
            return
        self.dist = [(pos[0] - q[0]) * n[0] + (pos[1] - q[1]) * n[1] for q, n in CORRIDOR]
        self.violated = next((name for name, d in zip(NAMES, self.dist) if d < -CONFIG["corridor_margin"]), None)
        self.deep = min(self.dist) >= CONFIG["corridor_arm"]

    def abort(self, text):
        self.state, self.finished, self.end = "STOPPED", True, "aborted"
        self.say(text)

    def on_key(self, k):
        if k == "q":
            self.abort("ABORTED (key q)")
        else:
            self.say("Only q aborts.")

    def waiting(self):
        if self.g is None or len(self.recent) < 5:
            return 0, 0
        self.ref = math.atan2(sum(math.sin(t) for t in self.recent), sum(math.cos(t) for t in self.recent))
        self.out_start, self.cross_since, self.state, self.armed = self.now, None, "OUTBOUND", False
        self.say("OUTBOUND: heading %.1f kept until %.0f m past line E-F, v=%d (barrier at %.1f m)" % (
            to_compass(self.ref), CONFIG["barrier_overshoot"], self.speed, self.dist[1]))
        return self.outbound()

    def lost_gps(self, paused):
        self.paused, self.state, self.near_since = paused, "NO_GPS", None
        self.say("GPS stale for over %.0f s: motors stopped until it returns" % CONFIG["gps_timeout_s"])

    def outbound(self):
        if self.g is None:
            self.lost_gps("OUTBOUND")
            return 0, 0
        south, north = self.dist
        if south < -CONFIG["corridor_margin"]:
            self.abort("ABORTED: LEFT CORRIDOR (%s) outbound" % NAMES[0])
            return 0, 0
        if not self.slow and self.speed > CONFIG["slow_speed"] and north <= CONFIG["slow_distance"]:
            self.slow = True
            self.say("SLOW: %.1f m from line E-F, speed limited to %d (v=%d)" % (north, CONFIG["slow_speed"], self.speed))
        if -north >= CONFIG["barrier_overshoot"]:
            if self.cross_since is None:
                self.cross_since = self.now
            if self.now - self.cross_since >= CONFIG["barrier_hold_s"]:
                return self.cross(False)
        else:
            self.cross_since = None
        timeout = CONFIG["barrier_timeout_s"]
        if timeout > 0 and self.now - self.out_start >= timeout:
            return self.cross(True)
        lim = min(CONFIG["turn_max"], float(self.limit()))
        return mix(self.limit(), max(-lim, min(lim, CONFIG["kp"] * wrap(self.ref - self.theta))))

    def cross(self, timed_out):
        north = self.dist[1]
        self.state, self.armed, self.cross_since, self.turn_done = "MISSION", False, None, False
        if timed_out:
            self.say("TIMEOUT: barrier not crossed in %.0f s (%.1f m to the line). Returning to point A"
                     % (CONFIG["barrier_timeout_s"], north))
        else:
            self.say("BARRIER crossed: %.1f m past line E-F. U-turn and return to point A" % -north)
        return self.follow()

    def follow(self):
        state, g = self.state, self.g
        if state not in ("MISSION", "NO_GPS"):
            return 0, 0
        if g is None:
            if state != "NO_GPS":
                self.lost_gps(state)
            return 0, 0
        if state == "NO_GPS":
            self.state = self.paused
            self.say("GPS back: mission resumed")
        if g["dist"] > CONFIG["near_radius"]:
            self.near_since = None
        elif self.near_since is None:
            self.near_since = self.now
        near = self.near_since is not None and self.now - self.near_since >= CONFIG["near_time_s"]
        if g["dist"] <= CONFIG["arrival_radius"] or near:
            self.finished, self.state = True, "STOPPED"
            self.say("ARRIVED at point A: d=%.1f m (%s)" % (g["dist"], "within %.1f m" % CONFIG["arrival_radius"]
                     if g["dist"] <= CONFIG["arrival_radius"] else "stayed within %.1f m for %.0f s" % (
                         CONFIG["near_radius"], CONFIG["near_time_s"])))
            return 0, 0
        if not self.armed and self.deep:
            self.armed = True
            self.say("Corridor armed: inside by more than %.0f m" % CONFIG["corridor_arm"])
        if self.armed and self.violated:
            self.armed, self.state = False, "STOPPED"
            self.say("LEFT CORRIDOR (%s): motors stopped" % self.violated)
            return 0, 0
        return g["left"], g["right"]

    def control(self):
        if self.state == "NO_GPS" and self.paused == "OUTBOUND":
            if self.g is None:
                return 0, 0
            self.state = "OUTBOUND"
            self.say("GPS back: resuming")
        if self.state == "WAITING":
            return self.waiting()
        if self.state == "OUTBOUND":
            return self.outbound()
        left, right = self.follow()
        if self.state == "STOPPED" and not self.finished:
            self.finished, self.end = True, "aborted"
        g = self.g
        if self.state == "MISSION" and not self.turn_done and g is not None and abs(math.degrees(g["err"])) <= 20.0:
            self.turn_done = True
            self.say("U-TURN done: aligned with point A (err %.0f deg)%s" % (
                abs(math.degrees(g["err"])), ", speed back to v=%d" % self.speed if self.slow else ""))
            self.slow = False
        return left, right

    def targets(self):
        if self.state == "OUTBOUND":
            return self.ref, wrap(self.ref - self.theta), self.limit()
        if self.g is None:
            return None, None, None
        return self.g["desired"], self.g["err"], self.g["cmd"]

    def status(self, h, left, right):
        g, state = self.g, self.state
        age = "---" if self.age is None else "%.1f" % self.age
        cmd = "(%3d,%3d)" % (left, right)
        tag = " SLOW" if self.slow else ""
        if state == "WAITING":
            text = "%-8s hdg=%5.1f waiting for GPS fix gps=%s" % (state, h, age)
        elif state == "OUTBOUND":
            text = "%-8s hdg=%5.1f ref=%5.1f err=%+6.1f bar=%+6.1f cmd=%s gps=%s%s" % (
                state, h, to_compass(self.ref), -math.degrees(wrap(self.ref - self.theta)), self.dist[1], cmd, age, tag)
        elif state == "MISSION" and g is not None:
            text = "%-8s hdg=%5.1f tgt=%5.1f err=%+6.1f d=%5.1f cmd=%s gps=%s%s" % (
                state, h, to_compass(g["desired"]), -math.degrees(g["err"]), g["dist"], cmd, age, tag)
        else:
            text = "%-8s hdg=%5.1f d=%s cmd=%s gps=%s" % (state, h, "---" if g is None else "%.1f" % g["dist"], cmd, age)
        return text[:79].ljust(79)


def write_kml(path, track, raw):
    def pt(label, p, style):
        return ("<Placemark><name>%s</name><styleUrl>#%s</styleUrl><Point><coordinates>%.7f,%.7f,0"
                "</coordinates></Point></Placemark>" % (label, style, p[1], p[0]))

    def ln(pts, style, visible=1):
        return ("<Placemark><visibility>%d</visibility><styleUrl>#%s</styleUrl><LineString><tessellate>1"
                "</tessellate><coordinates>%s</coordinates></LineString></Placemark>"
                % (visible, style, " ".join("%.7f,%.7f,0" % (lo, la) for la, lo in pts)))

    styles = "".join('<Style id="%s"><IconStyle><color>%s</color><scale>1.2</scale><Icon><href>'
                     'http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href></Icon>'
                     '</IconStyle></Style>' % s for s in (("a", "ff00ff00"), ("b", "ffff0000"), ("edge", "ff0000ff"),
                                                          ("end", "ffff00ff")))
    styles += "".join('<Style id="%s"><LineStyle><color>%s</color><width>%d</width></LineStyle></Style>' % s
                      for s in (("route", "ff00ff00", 4), ("raw", "ff0080ff", 2), ("guide", "ffffffff", 3),
                                ("wall", "ff0000ff", 3)))
    items = [pt("A", POINTS["A"], "a"), pt("B", POINTS["B"], "b")]
    items += [pt(k, POINTS[k], "edge") for k in "CDEF"]
    items += [ln([POINTS["C"], POINTS["D"]], "wall"), ln([POINTS["E"], POINTS["F"]], "wall"),
              ln([POINTS["B"], POINTS["A"]], "guide")]
    if len(track) > 1:
        items.append(ln(track, "route"))
    if len(raw) > 1:
        items.append(ln(raw, "raw", 0))
    if track:
        items += [pt("i", track[0], "end"), pt("f", track[-1], "end")]
    with open(path, "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2"><Document>'
                "%s%s</Document></kml>\n" % (styles, "".join(items)))


def run(boat, keys, mission, log_path, kml_path):
    kf, track, raw, pos = Tracker(), [], [], None
    period, reason, last_track, last_flush = 1.0 / CONFIG["hz"], None, -1.0, 0.0
    log = open(log_path, "w")
    log.write(",".join(COLUMNS) + "\n")
    try:
        print("Straight out to line E-F, U-turn, back to point A. Only q aborts. Ctrl+C exits.")
        while True:
            started = boat.now()
            if os.path.isfile(STOP_FILE):
                reason = "STOP file"
                break
            theta, h, mag = boat.heading()
            now = boat.now()
            fix = boat.pop_gps()
            kf.predict(now)
            result = ""
            if fix is not None:
                x, y = xy(fix[1], fix[2])
                result = kf.update(x, y, now)
                raw.append((fix[1], fix[2]))
            pos = kf.position()
            if pos is not None and now - last_track >= 0.5:
                track.append(latlon(*pos))
                last_track = now
            last = boat.last_gps()
            mission.update(theta, now, pos, None if last is None else now - last[0])
            pressed = keys.read()
            for k in pressed:
                mission.on_key(k)
            left, right = mission.control()
            boat.motors(left, right)
            desired, err, speed_cmd = mission.targets()
            dist = mission.dist or (None, None)
            row = [now, mission.state, "" if last is None else "%.7f" % last[1], "" if last is None else "%.7f" % last[2],
                   mission.age, 1 if fix is not None else 0, pos and pos[0], pos and pos[1],
                   pos and kf.x[2], pos and kf.x[3], mission.g and mission.g["dist"], dist[0], dist[1],
                   None if desired is None else to_compass(desired), h,
                   None if err is None else -math.degrees(err), mission.speed, speed_cmd,
                   "+".join(key_name(k) for k in pressed), left, right, result] + list(mag)
            log.write(",".join("" if v is None else "%.3f" % v if isinstance(v, float) else str(v) for v in row) + "\n")
            if now - last_flush >= 5.0:
                log.flush()
                last_flush = now
            sys.stdout.write("\r" + mission.status(h, left, right))
            sys.stdout.flush()
            if mission.finished:
                reason = mission.end
                break
            boat.sleep(period - (boat.now() - started))
    except Stop as e:
        reason = "interrupted (%s)" % e
    finally:
        set_signals(signal.SIG_IGN)
        boat.stop()
        log.close()
        set_signals(signal.SIG_DFL)
        try:
            if pos is not None:
                track.append(latlon(*pos))
            write_kml(kml_path, track, raw)
            print("\nKML: %s" % kml_path)
        except Exception as e:
            print("\nWARNING: KML not written (%r)" % (e,))
    print("\nEnd: %s. Motors stopped. Log: %s" % (reason, log_path))


def ask_speed(default):
    while True:
        text = input("Initial speed 0-%d (Enter = %d): " % (MAX_CMD, default)).strip()
        if not text:
            return default
        if text.isdigit() and int(text) <= MAX_CMD:
            return int(text)
        print("Invalid speed")


def main():
    args = sys.argv[1:]
    if args[:1] == ["barrier"]:
        args = args[1:]
    if len(args) % 2:
        sys.exit("unknown option or missing value: %s (usage: python3 mission2.py [--option value ...])" % args[-1])
    for flag, value in zip(args[::2], args[1::2]):
        key = flag.lstrip("-").replace("-", "_")
        if not flag.startswith("--") or key not in CONFIG:
            sys.exit("unknown option %s (usage: python3 mission2.py [--option value ...])" % flag)
        CONFIG[key] = type(CONFIG[key])(value)
    if os.path.isfile(STOP_FILE):
        sys.exit("%s exists: remove it (rm data/STOP)" % STOP_FILE)
    if not sys.stdin.isatty():
        sys.exit("needs a terminal (ssh or screen), do not use nohup")
    users = port_users(ARDUINO_TTY)
    if users:
        sys.exit("%s is in use by another process (motors would shake): %s" % (ARDUINO_TTY, "; ".join(users)))
    CONFIG["speed"] = ask_speed(CONFIG["speed"])
    boat = Boat()
    set_signals(on_signal)
    boat.stop()
    keys = Keys()
    os.makedirs(LOG_DIR, exist_ok=True)
    n = 1
    while os.path.exists(os.path.join(LOG_DIR, "mission2_%03d.csv" % n)):
        n += 1
    base = os.path.join(LOG_DIR, "mission2_%03d" % n)
    print("Point A is %.1f m from B, bearing %.1f deg" % (
        math.hypot(*GOAL), (90.0 - math.degrees(math.atan2(GOAL[1], GOAL[0]))) % 360.0))
    try:
        run(boat, keys, Mission(), base + ".csv", base + ".kml")
    finally:
        keys.close()


if __name__ == "__main__":
    main()
