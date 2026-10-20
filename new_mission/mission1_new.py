import json
import math
import os
import signal
import sys
import time
import types

ROOT = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(ROOT, "data", "logs")
STOP_FILE = os.path.join(ROOT, "data", "STOP")
ARDUINO_TTY = "/dev/ttyV0"
MAX_CMD = 191
ESC = "\x1b"
CONFIG = {"speed": 120, "kp": 60.0, "turn_max": 90.0, "hysteresis_deg": 10.0,
          "turn_timeout_s": 60.0, "hz": 10.0}
QUADRANTS = {1: "Q1(+,+)", 2: "Q2(-,+)", 3: "Q3(-,-)", 4: "Q4(+,-)"}
COLUMNS = ["t_s", "state", "quadrant", "transitions", "heading_deg", "theta_deg", "cos", "sin",
           "target_deg", "error_deg", "turned_deg", "speed", "key", "cmd_left", "cmd_right",
           "mag_x", "mag_y", "mag_z"]


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def to_theta(compass_deg):
    return wrap(math.radians(90.0 - compass_deg))


def to_compass(theta):
    return (90.0 - math.degrees(theta)) % 360.0


def quadrant(theta):
    c, s = math.cos(theta), math.sin(theta)
    return 1 if c >= 0 and s >= 0 else 2 if c < 0 and s >= 0 else 3 if c < 0 else 4


def mix(thrust, turn):
    outer = int(round(thrust))
    inner = int(round(thrust - min(2.0 * abs(turn), float(thrust))))
    return (inner, outer) if turn > 0 else (outer, inner)


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


class Boat(object):
    def __init__(self):
        self.t0 = time.time()
        self.calibration = load_calibration()
        load_drivers()
        import imu9_driver_v2
        import arduino_driver_v2
        self.imu = imu9_driver_v2.Imu9IO()
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


class Automaton(object):
    def __init__(self, theta, direction):
        self.direction = direction
        self.margin = math.sin(math.radians(CONFIG["hysteresis_deg"]))
        self.start = self.q = quadrant(theta)
        self.net = 0

    def update(self, theta):
        c, s, h = math.cos(theta), math.sin(theta), self.margin
        ccw = {1: c < -h, 2: s < -h, 3: c > h, 4: s > h}[self.q]
        cw = {1: s < -h, 2: c > h, 3: s > h, 4: c < -h}[self.q]
        step = 1 if ccw else -1 if cw else 0
        if step:
            self.q = self.q % 4 + 1 if step > 0 else (self.q - 2) % 4 + 1
            self.net += step * self.direction
        return step

    def done(self, theta, start_theta):
        e = wrap(theta - start_theta)
        return self.net >= 4 and self.q == self.start and self.direction * e >= 0 and math.cos(e) > 0


class Mission(object):
    def __init__(self):
        self.speed = CONFIG["speed"]
        self.state = "STOPPED"
        self.manual = False
        self.prompt = None
        self.buf = ""
        self.theta = self.now = self.turned = self.turn_start = self.start_theta = self.prev = 0.0
        self.ref = self.auto = None
        self.direction = 0

    def say(self, text):
        print("\n" + text)

    def go(self, state, text):
        self.prompt = None
        self.state = state
        self.say(text)

    def start_turn(self, direction):
        self.auto = Automaton(self.theta, direction)
        self.direction = direction
        self.start_theta = self.prev = self.theta
        self.turned = 0.0
        self.turn_start = self.now
        self.go("TURN", "TURN %s from heading %.1f, v=%d%s" % (
            "LEFT (counter-clockwise)" if direction == 1 else "RIGHT (clockwise)",
            to_compass(self.theta), self.speed, "" if self.speed else " (v=0: it will not turn)"))

    def on_key(self, k):
        stop_keys = ("q", "s") if self.manual else ("q",)
        if self.prompt == "direction":
            if k in ("h", "a"):
                self.start_turn(-1 if k == "h" else 1)
            elif k == ESC:
                self.go(self.state, "turn cancelled")
            elif k in stop_keys:
                self.go("MANUAL" if self.manual else "STOPPED", "STOPPED (key %s)" % k)
            elif k == "r":
                self.go("LINE", "LINE: heading set to %.1f, v=%d" % (to_compass(self.theta), self.speed))
                self.ref = self.theta
        elif self.prompt == "speed":
            if len(k) == 1 and k in "0123456789":
                self.buf += k if len(self.buf) < 3 else ""
            elif k in ("\x7f", "\x08"):
                self.buf = self.buf[:-1]
            elif k in ("\n", "\r"):
                if self.buf and int(self.buf) <= MAX_CMD:
                    self.speed = int(self.buf)
                elif self.buf:
                    self.say("v must be 0 to %d, kept %d" % (MAX_CMD, self.speed))
                self.prompt = None
            elif k == ESC:
                self.prompt = None
            elif k in stop_keys:
                self.go("MANUAL" if self.manual else "STOPPED", "STOPPED (key %s)" % k)
        elif self.manual:
            if k in ("w", "a", "d"):
                self.go({"w": "FORWARD", "a": "SPIN_L", "d": "SPIN_R"}[k], "MANUAL %s, v=%d" % (k, self.speed))
            elif k in ("s", "q"):
                self.go("MANUAL", "MANUAL stopped")
            elif k == "v":
                self.prompt, self.buf = "speed", ""
            elif k == "m":
                self.manual = False
                self.go("STOPPED", "AUTO mode, STOPPED")
            elif k in ("r", "t"):
                self.say("r and t do not work in manual mode. Leave with m.")
        elif k == "r":
            self.ref = self.theta
            self.go("LINE", "LINE: heading set to %.1f, v=%d" % (to_compass(self.theta), self.speed))
        elif k == "t":
            self.prompt = "direction"
        elif k == "v":
            self.prompt, self.buf = "speed", ""
        elif k == "q":
            self.go("STOPPED", "STOPPED (key q)")
        elif k == "m":
            self.manual = True
            self.go("MANUAL", "MANUAL mode, motors stopped: w=forward a/d=spin s=stop v=speed m=exit")
        elif k in ("w", "a", "s", "d"):
            self.say("Key %s only works in manual mode (m). Here, q stops." % k)

    def control(self):
        v, state = self.speed, self.state
        if state == "LINE" and v > 0:
            lim = min(CONFIG["turn_max"], float(v))
            return mix(v, max(-lim, min(lim, CONFIG["kp"] * wrap(self.ref - self.theta))))
        if state == "FORWARD":
            return v, v
        if state in ("SPIN_L", "SPIN_R"):
            return (-v, v) if state == "SPIN_L" else (v, -v)
        if state == "TURN":
            self.turned += math.degrees(wrap(self.theta - self.prev))
            self.prev = self.theta
            if self.auto.update(self.theta):
                self.say("  %s  net transitions = %d  turned = %.0f" % (QUADRANTS[self.auto.q], self.auto.net, self.turned))
            if self.auto.done(self.theta, self.start_theta):
                self.go("STOPPED", "TURN COMPLETE, turned %.0f degrees. STOPPED." % abs(self.turned))
            elif self.now - self.turn_start > CONFIG["turn_timeout_s"]:
                self.go("STOPPED", "TURN TIMEOUT (%.0f s), turned %.0f degrees. STOPPED."
                        % (CONFIG["turn_timeout_s"], self.turned))
            else:
                return (0, v) if self.direction == 1 else (v, 0)
        return 0, 0

    def status(self, h, left, right):
        cmd = "(%4d,%4d)" % (left, right)
        if self.state == "LINE":
            brief = "hdg=%5.1f err=%+6.1f" % (h, -math.degrees(wrap(self.ref - self.theta)))
            full = "hdg=%5.1f tgt=%5.1f err=%+6.1f %s" % (
                h, to_compass(self.ref), -math.degrees(wrap(self.ref - self.theta)), QUADRANTS[quadrant(self.theta)])
        elif self.state == "TURN":
            remaining = max(0.0, 360.0 - abs(self.turned))
            brief = "hdg=%5.1f left=%4.0f" % (h, remaining)
            full = "%s %s" % (brief, QUADRANTS[self.auto.q])
        else:
            brief = full = "hdg=%5.1f" % h
        if self.prompt == "direction":
            text = "DIRECTION? h=clockwise a=counter Esc=cancel | %s %s %s" % (self.state, brief, cmd)
        elif self.prompt == "speed":
            text = "v 0-%d: %s_ Enter=ok Esc=cancel | %s %s %s" % (MAX_CMD, self.buf, self.state, brief, cmd)
        else:
            text = "%-7s %s v=%3d cmd=%s" % (self.state, full, self.speed, cmd)
        return text[:79].ljust(79)


def key_name(k):
    names = {"\n": "enter", "\r": "enter", ESC: "esc", "\x7f": "bksp", "\x08": "bksp"}
    return k if k.isalnum() else names.get(k, "x%02x" % ord(k))


def run(boat, keys, log_path):
    mission = Mission()
    period = 1.0 / CONFIG["hz"]
    reason, last_flush = None, 0.0
    log = open(log_path, "w")
    log.write(",".join(COLUMNS) + "\n")
    try:
        print("Keys: r=hold heading  t=full turn (then h/a)  v=speed  q=stop  m=manual mode "
              "(w a d s v, m exits)")
        print("Ctrl+C exits. Starting STOPPED, motors at 0.")
        while True:
            started = boat.now()
            if os.path.isfile(STOP_FILE):
                reason = "STOP file"
                break
            theta, h, mag = boat.heading()
            mission.theta, mission.now = theta, boat.now()
            pressed = keys.read()
            for k in pressed:
                mission.on_key(k)
            left, right = mission.control()
            boat.motors(left, right)
            turning = mission.state == "TURN"
            line = mission.state == "LINE"
            row = [mission.now, mission.state, mission.auto.q if turning else quadrant(theta),
                   mission.auto.net if turning else 0, h, math.degrees(theta), math.cos(theta), math.sin(theta),
                   to_compass(mission.ref) if line else "",
                   -math.degrees(wrap(mission.ref - theta)) if line else "",
                   mission.turned, mission.speed, "+".join(key_name(k) for k in pressed), left, right] + list(mag)
            log.write(",".join("%.3f" % v if isinstance(v, float) else str(v) for v in row) + "\n")
            if mission.now - last_flush >= 5.0:
                log.flush()
                last_flush = mission.now
            sys.stdout.write("\r" + mission.status(h, left, right))
            sys.stdout.flush()
            boat.sleep(period - (boat.now() - started))
    except Stop as e:
        reason = "interrupted (%s)" % e
    finally:
        set_signals(signal.SIG_IGN)
        boat.stop()
        log.close()
        set_signals(signal.SIG_DFL)
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
    if args[:1] == ["run"]:
        args = args[1:]
    for flag, value in zip(args[::2], args[1::2]):
        key = flag.lstrip("-").replace("-", "_")
        if key not in CONFIG:
            sys.exit("unknown option " + flag)
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
    while os.path.exists(os.path.join(LOG_DIR, "mission1_%03d.csv" % n)):
        n += 1
    try:
        run(boat, keys, os.path.join(LOG_DIR, "mission1_%03d.csv" % n))
    finally:
        keys.close()


if __name__ == "__main__":
    main()
