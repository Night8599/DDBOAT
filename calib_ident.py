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
CALIB_DIR = os.path.join(ROOT, "data", "calib")
STOP_FILE = os.path.join(ROOT, "data", "STOP")
ARDUINO_TTY = "/dev/ttyV0"
ESC = "\x1b"
EARTH = 6378137.0
CONFIG = {"max_radius": 40.0, "still_s": 8.0, "fix_timeout_s": 120.0, "spin_cmd": 100, "spin_min_s": 12.0,
          "spin_max_s": 60.0, "spin_extra_s": 2.0, "spin_min_partial_s": 8.0, "partial_turn_deg": 150.0, "run_cmd": 100, "slow_cmd": 60, "settle_s": 8.0,
          "measure_s": 20.0, "period": 0.05, "kp": 2.5, "kd": 0.3, "ki": 0.2, "i_max": 40.0,
          "settle_max_s": 30.0, "tau_fuse": 2.0, "gyro_nominal": 0.07, "max_total_s": 900.0, "min_bins": 12}
COLUMNS = ["t_s", "phase", "lat", "lon", "e_m", "n_m", "gps_new", "mag_x", "mag_y", "mag_z", "acc_x", "acc_y",
           "acc_z", "gyro_x", "gyro_y", "gyro_z", "cmd_left", "cmd_right", "h_comp", "h_fuse", "target"]


class Abort(Exception):
    pass


class Skip(Exception):
    pass


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def circ_mean(values):
    return math.degrees(math.atan2(sum(math.sin(math.radians(v)) for v in values),
                                   sum(math.cos(math.radians(v)) for v in values))) % 360.0


def solve(a, b):
    n = len(b)
    m = [list(a[i]) + [b[i]] for i in range(n)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(m[r][i]))
        if abs(m[p][i]) < 1e-12:
            raise ValueError("singular system")
        m[i], m[p] = m[p], m[i]
        for r in range(i + 1, n):
            f = m[r][i] / m[i][i]
            for c in range(i, n + 1):
                m[r][c] -= f * m[i][c]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = (m[i][n] - sum(m[i][j] * x[j] for j in range(i + 1, n))) / m[i][i]
    return x


def linfit(t, y):
    n = len(t)
    tm, ym = sum(t) / n, sum(y) / n
    stt = sum((a - tm) ** 2 for a in t)
    slope = sum((a - tm) * (b - ym) for a, b in zip(t, y)) / stt
    resid = [b - (ym + slope * (a - tm)) for a, b in zip(t, y)]
    sigma = math.sqrt(sum(r * r for r in resid) / max(1, n - 2))
    return slope, sigma, sigma / math.sqrt(stt)


def fit_ellipse(points):
    n = len(points)
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    rows = [(p[0] - mx, p[1] - my) for p in points]
    scale = math.sqrt(sum(x * x + y * y for x, y in rows) / n)
    rows = [(x / scale, y / scale) for x, y in rows]
    feats = [(x * x, x * y, y * y, x, y) for x, y in rows]
    ata = [[sum(f[i] * f[j] for f in feats) for j in range(5)] for i in range(5)]
    atb = [sum(f[i] for f in feats) for i in range(5)]
    a, b, c, d, e = solve(ata, atb)
    det = 4.0 * a * c - b * b
    if det <= 0 or a + c <= 0:
        raise ValueError("points do not form an ellipse")
    x0 = (b * e - 2.0 * c * d) / det
    y0 = (b * d - 2.0 * a * e) / det
    f0 = a * x0 * x0 + b * x0 * y0 + c * y0 * y0 + d * x0 + e * y0
    k = 1.0 - f0
    q = [[a / k, b / (2.0 * k)], [b / (2.0 * k), c / k]]
    s = math.sqrt(q[0][0] * q[1][1] - q[0][1] * q[1][0])
    t = math.sqrt(q[0][0] + q[1][1] + 2.0 * s)
    root = [[(q[0][0] + s) / t, q[0][1] / t], [q[1][0] / t, (q[1][1] + s) / t]]
    m = [[root[i][j] / scale for j in range(2)] for i in range(2)]
    return (mx + x0 * scale, my + y0 * scale), m


def whiten(p, center, m):
    x, y = p[0] - center[0], p[1] - center[1]
    return m[0][0] * x + m[0][1] * y, m[1][0] * x + m[1][1] * y


def unwrap_total(angles):
    total, prev = 0.0, angles[0]
    for a in angles[1:]:
        total += wrap(a - prev)
        prev = a
    return total


def bins_covered(points, n_bins):
    if len(points) < 20:
        return 0
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    counts = [0] * n_bins
    for x, y in points:
        counts[int(((math.degrees(math.atan2(y - cy, x - cx)) + 360.0) % 360.0) // (360.0 / n_bins)) % n_bins] += 1
    return sum(1 for c in counts if c >= 3)


def compass_heading(m, c):
    if c.get("format") == 2:
        x, y = m[0] - c["cx"], m[1] - c["cy"]
        zx, zy = c["m00"] * x + c["m01"] * y, c["m10"] * x + c["m11"] * y
        a = math.degrees(math.atan2(zy, zx))
    else:
        a = math.degrees(math.atan2((m[1] - c["offy"]) * c["sy"], (m[0] - c["offx"]) * c["sx"]))
    return (c["sign"] * wrap(a - c["north"])) % 360.0


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


class Rig(object):
    def __init__(self):
        self.t0 = time.time()
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
        self.keys = Keys()

    def now(self):
        return time.time() - self.t0

    def sleep(self, dt):
        if dt > 0:
            time.sleep(dt)

    def sense(self):
        return {"mag": self.imu.read_mag_raw(), "gyro": self.imu.read_gyro_raw(), "acc": self.imu.read_accel_raw()}

    def pop_gps(self):
        return self.gps.pop()

    def motors(self, left, right):
        self.arduino.send_arduino_cmd_motor(int(left), int(right))

    def stop(self):
        for _ in range(3):
            try:
                self.arduino.send_arduino_cmd_motor(0, 0)
            except Exception:
                pass
            time.sleep(0.05)

    def abort_requested(self):
        if os.path.isfile(STOP_FILE):
            return "STOP file"
        return "key q" if "q" in self.keys.read() else None

    def close(self):
        self.keys.close()


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


def fit_runs(runs):
    def model(delta):
        rows, rhs = [], []
        for r in runs:
            h, w = math.radians(r["hbar"] + delta), 1.0 / r["v_sigma"]
            rows += [[w * math.sin(h), w, 0.0], [w * math.cos(h), 0.0, w]]
            rhs += [w * r["v"][0], w * r["v"][1]]
        ata = [[sum(row[i] * row[j] for row in rows) for j in range(3)] for i in range(3)]
        atb = [sum(row[i] * y for row, y in zip(rows, rhs)) for i in range(3)]
        x = solve(ata, atb)
        sse = sum((sum(row[i] * x[i] for i in range(3)) - y) ** 2 for row, y in zip(rows, rhs))
        return sse, x

    best = None
    for step, span, center in ((2.0, 180.0, 0.0), (0.2, 3.0, None), (0.02, 0.3, None)):
        c0 = center if center is not None else best[0]
        d = c0 - span
        while d <= c0 + span + 1e-9:
            sse, x = model(d)
            if x[0] > 0 and (best is None or sse < best[1]):
                best = (d, sse, x)
            d += step
    delta, sse, x = best
    n = len(runs)
    info_sum = sum((x[0] / r["v_sigma"]) ** 2 for r in runs) * (n - 1) / float(n)
    resid = [wrap(math.degrees(math.atan2(r["v"][0] - x[1], r["v"][1] - x[2])) - r["hbar"] - delta) for r in runs]
    excess = [max(0.0, abs(a) - 2.0 * math.degrees(r["v_sigma"] / x[0])) for a, r in zip(resid, runs)]
    return {"delta": wrap(delta), "speed": x[0], "current": (x[1], x[2]), "sigma": math.degrees(1.0 / math.sqrt(info_sum)),
            "chi2": sse / max(1, 2 * n - 3), "resid": resid, "excess": excess}


def mix_turn(base, u):
    u = max(-float(base), min(float(base), u))
    return (base, int(round(base - u))) if u > 0 else (int(round(base + u)), base)


def mean(values):
    return sum(values) / len(values)


def std(values):
    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / max(1, len(values) - 1))


class Session(object):
    def __init__(self, rig, log):
        self.rig, self.log = rig, log
        self.t0 = rig.now()
        self.phase, self.target = "init", ""
        self.origin = self.pos = self.cal = self.scale = self.hcomp = self.hfuse = self.last_t = None
        self.gps_new = 0
        self.mag = self.gyro = self.acc = [0, 0, 0]
        self.bias = [0.0, 0.0, 0.0]
        self.rate = 0.0
        self.cmd = (0, 0)
        self.delta_est = None
        self.current_est = (0.0, 0.0)
        self.gyro_noise = self.mag_noise = 0.0
        self.outside = 0
        self.fixes = []
        self.rows = 0

    def tick(self, left, right):
        rig = self.rig
        started = rig.now()
        if started - self.t0 > CONFIG["max_total_s"]:
            raise Abort("total time limit of %.0f s reached" % CONFIG["max_total_s"])
        why = rig.abort_requested()
        if why:
            raise Abort(why)
        rig.motors(left, right)
        self.cmd = (left, right)
        s = rig.sense()
        self.mag, self.gyro, self.acc = s["mag"], s["gyro"], s["acc"]
        now = rig.now()
        fix = rig.pop_gps()
        self.gps_new = 0
        if fix is not None:
            if self.origin is None:
                self.origin = (fix[1], fix[2])
            e = math.radians(fix[2] - self.origin[1]) * EARTH * math.cos(math.radians(self.origin[0]))
            n = math.radians(fix[1] - self.origin[0]) * EARTH
            self.pos, self.gps_new = (e, n), 1
            self.fixes.append((fix[0], e, n))
            self.outside = self.outside + 1 if math.hypot(e, n) > CONFIG["max_radius"] else 0
            if self.outside >= 2:
                raise Abort("went %.0f m from the start point (limit %.0f m)" % (math.hypot(e, n), CONFIG["max_radius"]))
        self.update_heading(now)
        lat = lon = ""
        if fix is not None:
            lat, lon = "%.7f" % fix[1], "%.7f" % fix[2]
        e, n = ("", "") if self.pos is None else ("%.2f" % self.pos[0], "%.2f" % self.pos[1])
        self.log.write(",".join(str(v) for v in [
            "%.3f" % now, self.phase, lat, lon, e, n, self.gps_new] + list(self.mag) + list(self.acc) + list(self.gyro) + [
            left, right, "" if self.hcomp is None else "%.1f" % self.hcomp,
            "" if self.hfuse is None else "%.1f" % self.hfuse, self.target]) + "\n")
        self.rows += 1
        if self.rows % 100 == 0:
            self.log.flush()
        rig.sleep(CONFIG["period"] - (rig.now() - started))
        return fix

    def update_heading(self, now):
        dt = 0.0 if self.last_t is None else now - self.last_t
        self.last_t = now
        if self.cal is None:
            return
        self.hcomp = compass_heading(self.mag, self.cal)
        if self.scale is not None:
            self.rate = 0.7 * self.rate + 0.3 * self.scale * (self.gyro[2] - self.bias[2])
        if self.hfuse is None or dt <= 0.0 or self.scale is None:
            self.hfuse = self.hcomp
            return
        pred = self.hfuse + self.rate * dt
        self.hfuse = (pred + (dt / CONFIG["tau_fuse"]) * wrap(self.hcomp - pred)) % 360.0

    def wait_fix(self):
        self.phase = "wait_fix"
        limit = self.rig.now() + CONFIG["fix_timeout_s"]
        while self.pos is None:
            if self.rig.now() > limit:
                raise Abort("no GPS fix in %.0f s" % CONFIG["fix_timeout_s"])
            self.tick(0, 0)

    def still(self):
        self.phase = "still"
        samples, mags, limit = [], [], self.rig.now() + CONFIG["still_s"]
        while self.rig.now() < limit:
            self.tick(0, 0)
            samples.append(list(self.gyro))
            mags.append(list(self.mag))
        self.bias = [mean([g[i] for g in samples]) for i in range(3)]
        self.gyro_noise = std([g[2] for g in samples])
        self.mag_noise = (std([m[0] for m in mags]) + std([m[1] for m in mags])) / 2.0

    def spin(self, left, right, name, ref=None):
        self.phase = name
        mags, gyro, fixes = [], [], []
        start, done, last_check = self.rig.now(), None, 0
        total, prev = 0.0, None
        while True:
            self.tick(left, right)
            now = self.rig.now()
            t = now - start
            mags.append((now, self.mag[0], self.mag[1]))
            gyro.append((now, self.gyro[2]))
            if self.gps_new:
                fixes.append(self.fixes[-1])
            if ref is not None:
                zx, zy = whiten((self.mag[0], self.mag[1]), ref[0], ref[1])
                a = math.degrees(math.atan2(zy, zx))
                total += 0.0 if prev is None else wrap(a - prev)
                prev = a
                if t >= CONFIG["spin_min_partial_s"] and abs(total) >= CONFIG["partial_turn_deg"]:
                    break
                if t >= CONFIG["spin_max_s"]:
                    raise Abort("%s: boat turned only %.0f deg in %.0f s (not rotating?)" % (name, abs(total), t))
                continue
            if t >= CONFIG["spin_min_s"] and int(t) != last_check:
                last_check = int(t)
                if done is None and bins_covered([(m[1], m[2]) for m in mags], CONFIG["min_bins"]) >= CONFIG["min_bins"]:
                    done = t
            if done is not None and t - done >= CONFIG["spin_extra_s"]:
                break
            if t >= CONFIG["spin_max_s"]:
                break
        for _ in range(int(2.0 / CONFIG["period"])):
            self.tick(0, 0)
        activity = mean([abs(v - self.bias[2]) for t, v in gyro])
        if activity < max(25.0, 10.0 * self.gyro_noise):
            raise Abort("%s: the gyro shows no rotation (mean %.0f counts): check the IMU and the motors" % (name, activity))
        cx, cy = mean([m[1] for m in mags]), mean([m[2] for m in mags])
        spread = math.sqrt(mean([(m[1] - cx) ** 2 + (m[2] - cy) ** 2 for m in mags]))
        if spread < (6.0 if ref is None else 3.0) * self.mag_noise:
            raise Abort("%s: the compass barely changed (spread %.0f vs noise %.0f counts): check the magnetometer" % (
                name, spread, self.mag_noise))
        if ref is None:
            covered = bins_covered([(m[1], m[2]) for m in mags], CONFIG["min_bins"])
            if covered < CONFIG["min_bins"] - 1:
                raise Abort("%s: compass cloud covers only %d of %d sectors in %.0f s (boat not rotating?)" % (
                    name, covered, CONFIG["min_bins"], CONFIG["spin_max_s"]))
        return {"mags": mags, "gyro": gyro, "fixes": fixes, "start": start, "cmd": (left, right)}

    def calibrate(self, sl, sr):
        points = [(m[1], m[2]) for m in sl["mags"] + sr["mags"]]
        center, m = fit_ellipse(points)
        totals, angles = [], []
        for sp in (sl, sr):
            a = [math.degrees(math.atan2(*reversed(whiten((p[1], p[2]), center, m)))) for p in sp["mags"]]
            angles.append(a)
            totals.append(unwrap_total(a))
        if abs(totals[0]) < 250.0 or abs(totals[1]) < 100.0 or totals[0] * totals[1] > 0:
            raise Abort("spins do not give opposite full turns (left %.0f, right %.0f deg): check motors and wiring" % (
                totals[0], totals[1]))
        sign = -1 if totals[0] > 0 else 1
        radii = [math.hypot(*whiten((p[0], p[1]), center, m)) for p in points]
        eig_sum, eig_det = m[0][0] + m[1][1], m[0][0] * m[1][1] - m[0][1] * m[1][0]
        disc = math.sqrt(max(0.0, eig_sum * eig_sum / 4.0 - eig_det))
        info = {"center": center, "m": m, "sign": sign, "rms_radius": std(radii) / mean(radii),
                "axis_ratio": (eig_sum / 2.0 + disc) / (eig_sum / 2.0 - disc), "turns": totals, "n": len(points)}
        self.cal = {"format": 2, "cx": center[0], "cy": center[1], "m00": m[0][0], "m01": m[0][1],
                    "m10": m[1][0], "m11": m[1][1], "north": 0.0, "sign": sign}
        integrals, turns_cw = [], []
        for sp, tot in zip((sl, sr), totals):
            g, total = sp["gyro"], 0.0
            for (t0, v0), (t1, v1) in zip(g[:-1], g[1:]):
                total += 0.5 * (v0 + v1 - 2.0 * self.bias[2]) * (t1 - t0)
            integrals.append(total)
            turns_cw.append(sign * tot)
        if abs(integrals[0] - integrals[1]) < 1e-6:
            raise Abort("gyro did not respond during the spins")
        self.scale = (turns_cw[0] - turns_cw[1]) / (integrals[0] - integrals[1])
        info["gyro_scale"] = self.scale
        info["gyro_scale_each"] = [turns_cw[0] / integrals[0], turns_cw[1] / integrals[1]]
        rates, resid = [], []
        for sp, a in zip((sl, sr), angles):
            t0 = sp["mags"][0][0]
            steady = [self.scale * (v - self.bias[2]) for t, v in sp["gyro"] if t - t0 >= 5.0]
            rates.append(mean(steady))
            heading = [sign * x for x in a]
            acc, hg = 0.0, [0.0]
            for (ta, va), (tb, vb) in zip(sp["gyro"][:-1], sp["gyro"][1:]):
                acc += self.scale * 0.5 * (va + vb - 2.0 * self.bias[2]) * (tb - ta)
                hg.append(acc)
            hc, base = [heading[0]], heading[0]
            for x in heading[1:]:
                base += wrap(x - base)
                hc.append(base)
            diff = [c - g for c, g in zip(hc, hg)]
            k = 20
            sm = [mean(diff[max(0, i - k):i + k + 1]) for i in range(len(diff))]
            trend = linfit(list(range(len(sm))), sm)[0]
            resid.append(std([s - trend * i for i, s in enumerate(sm)]))
        info["heading_rms_vs_gyro"] = max(resid)
        info["yaw_left_dps"], info["yaw_right_dps"] = rates
        info["yaw_gain"] = (rates[1] - rates[0]) / 200.0
        info["yaw_bias"] = (rates[0] + rates[1]) / 2.0
        r = sl["gyro"]
        final = rates[0]
        info["tau_yaw"] = None
        series = [self.scale * (v - self.bias[2]) for t, v in r]
        for i in range(2, len(series)):
            if series[i] != 0 and final != 0 and (mean(series[i - 2:i + 1]) / final) >= 0.632:
                info["tau_yaw"] = r[i][0] - r[0][0]
                break
        cum, k = 0.0, None
        for i in range(1, len(angles[0])):
            cum += wrap(angles[0][i] - angles[0][i - 1])
            if abs(cum) >= 330.0:
                k = i
                break
        info["current"] = (0.0, 0.0)
        info["current_known"] = False
        if k is not None:
            t0, t1 = sl["mags"][0][0], sl["mags"][k][0]
            window = [f for f in sl["fixes"] if t0 <= f[0] <= t1]
            if len(window) >= 8:
                t = [f[0] for f in window]
                info["current"] = (linfit(t, [f[1] for f in window])[0], linfit(t, [f[2] for f in window])[0])
                info["current_known"] = True
        return info

    def init_heading(self):
        self.phase = "init_heading"
        hs = []
        for _ in range(int(1.0 / CONFIG["period"])):
            self.tick(0, 0)
            hs.append(self.hcomp)
        self.hfuse = circ_mean(hs)

    def leg(self, name, target, cmd, settle=None, measure=None):
        settle = CONFIG["settle_s"] if settle is None else settle
        measure = CONFIG["measure_s"] if measure is None else measure
        self.phase, self.target = name, "%.0f" % target
        integ, last = 0.0, self.rig.now()
        start = steady_since = window_start = None
        fixes, hs, us = [], [], []
        start = last
        while True:
            now = self.rig.now()
            dt, last = now - last, now
            err = wrap(target - self.hfuse)
            u = CONFIG["kp"] * err - CONFIG["kd"] * self.rate + integ
            if abs(err) < 20.0:
                integ = max(-CONFIG["i_max"], min(CONFIG["i_max"], integ + CONFIG["ki"] * err * dt))
            left, right = mix_turn(cmd, u)
            self.tick(left, right)
            if window_start is None:
                if now - start >= settle:
                    if abs(err) <= 6.0 and abs(self.rate) <= 3.0:
                        steady_since = now if steady_since is None else steady_since
                    else:
                        steady_since = None
                    if (steady_since is not None and now - steady_since >= 3.0) or now - start >= CONFIG["settle_max_s"]:
                        window_start = now
                continue
            hs.append(self.hfuse)
            us.append(max(-float(cmd), min(float(cmd), u)))
            if self.gps_new:
                fixes.append(self.fixes[-1])
            if now - window_start >= measure:
                break
        self.target = ""
        if len(fixes) < 6:
            raise Skip("%s: only %d GPS fixes in the measurement window" % (name, len(fixes)))
        t = [f[0] for f in fixes]
        ve, se, sve = linfit(t, [f[1] for f in fixes])
        vn, sn, svn = linfit(t, [f[2] for f in fixes])
        hbar = circ_mean(hs)
        return {"v": (ve, vn), "v_sigma": max(0.02, (sve + svn) / 2.0), "gps_sigma": (se + sn) / 2.0, "hbar": hbar, "trim": mean(us), "cmd": cmd,
                "target": target, "spread": max(abs(wrap(h - hbar)) for h in hs), "name": name}

    def anchor(self):
        ax, ay = -self.current_est[0] * 60.0, -self.current_est[1] * 60.0
        limit = 0.4 * CONFIG["max_radius"]
        norm = math.hypot(ax, ay)
        return (ax, ay) if norm <= limit else (ax * limit / norm, ay * limit / norm)

    def go_home(self):
        if self.delta_est is None or self.pos is None:
            return
        if math.hypot(*self.pos) < 0.4 * CONFIG["max_radius"]:
            return
        self.phase = "home"
        limit = self.rig.now() + 60.0
        ax, ay = self.anchor()
        while math.hypot(self.pos[0] - ax, self.pos[1] - ay) > 0.15 * CONFIG["max_radius"] and self.rig.now() < limit:
            bearing = math.degrees(math.atan2(ax - self.pos[0], ay - self.pos[1])) % 360.0
            err = wrap((bearing - self.delta_est) % 360.0 - self.hfuse)
            left, right = mix_turn(CONFIG["run_cmd"], CONFIG["kp"] * err - CONFIG["kd"] * self.rate)
            self.tick(left, right)

    def toward_origin(self, headings):
        ax, ay = self.anchor()
        bearing = math.degrees(math.atan2(ax - self.pos[0], ay - self.pos[1])) % 360.0
        best = min(headings, key=lambda h: abs(wrap((h + self.delta_est) % 360.0 - bearing)))
        return (best, (best + 180.0) % 360.0)

    def run_all(self):
        self.wait_fix()
        self.still()
        sl = self.spin(0, CONFIG["spin_cmd"], "spin_left")
        try:
            ref = fit_ellipse([(m[1], m[2]) for m in sl["mags"]])
        except ValueError as e:
            raise Abort("compass cloud from the left spin is not usable: %s" % e)
        sr = self.spin(CONFIG["spin_cmd"], 0, "spin_right", ref)
        info = self.calibrate(sl, sr)
        self.current_est = info["current"]
        self.init_heading()
        h0 = self.hfuse % 360.0
        try:
            first = self.leg("leg1", h0, CONFIG["run_cmd"], settle=5.0, measure=9.0)
        except Skip as e:
            raise Abort(str(e))
        c = info["current"]
        d = (first["v"][0] - c[0], first["v"][1] - c[1]) if info["current_known"] else first["v"]
        if math.hypot(*d) < 0.15:
            d = first["v"]
        if math.hypot(*d) < 0.15:
            raise Abort("first run shows only %.2f m/s over the ground (motors or GPS problem)" % math.hypot(*d))
        self.delta_est = wrap(math.degrees(math.atan2(d[0], d[1])) - first["hbar"])
        legs = [first]
        self.go_home()
        order = self.toward_origin(((h0 + 120.0) % 360.0, (h0 + 240.0) % 360.0))
        for k, target in enumerate(order):
            try:
                legs.append(self.leg("leg%d" % (k + 2), target, CONFIG["run_cmd"]))
            except Skip as e:
                raise Abort(str(e))
            self.go_home()
        fit = fit_runs(legs)
        self.delta_est = fit["delta"]
        self.current_est = fit["current"]
        slow = None
        try:
            slow = self.leg("leg4", self.toward_origin((h0, (h0 + 120.0) % 360.0, (h0 + 240.0) % 360.0))[0], CONFIG["slow_cmd"])
        except Skip:
            slow = None
        self.phase = "end"
        for _ in range(int(2.0 / CONFIG["period"])):
            self.tick(0, 0)
        info["legs"], info["slow"], info["fit"] = legs, slow, fit
        return info


def write_json(path, data):
    with open(path, "w") as f:
        json.dump(data, f, indent=2, sort_keys=True)


def finish(info):
    sign, fit, legs, slow = info["sign"], info["fit"], info["legs"], info["slow"]
    north = wrap(-sign * fit["delta"])
    spread = max(fit["excess"])
    cal = {"format": 2, "cx": info["center"][0], "cy": info["center"][1], "m00": info["m"][0][0],
           "m01": info["m"][0][1], "m10": info["m"][1][0], "m11": info["m"][1][1], "north": north, "sign": sign,
           "created": time.strftime("%Y-%m-%d %H:%M:%S"), "points": info["n"],
           "rms_radius": info["rms_radius"], "heading_rms_vs_gyro_deg": info["heading_rms_vs_gyro"],
           "north_sigma_deg": fit["sigma"], "heading_dependent_error_deg": spread}
    cur_gap = math.hypot(fit["current"][0] - info["current"][0], fit["current"][1] - info["current"][1])
    ident = {"gyro_scale_dps_per_count": info["gyro_scale"], "gyro_scale_each_spin": info["gyro_scale_each"],
             "yaw_left_spin_dps": info["yaw_left_dps"], "yaw_right_spin_dps": info["yaw_right_dps"],
             "yaw_gain_dps_per_unit": info["yaw_gain"], "yaw_bias_dps": info["yaw_bias"], "tau_yaw_s": info["tau_yaw"],
             "trim_u_right_100": mean([l["trim"] for l in legs]),
             "trim_u_right_60": None if slow is None else slow["trim"],
             "speed_100_mps": fit["speed"],
             "speed_60_mps": None if slow is None else math.hypot(slow["v"][0] - fit["current"][0], slow["v"][1] - fit["current"][1]),
             "current_from_spins_en_mps": info["current"], "current_from_runs_en_mps": fit["current"],
             "gps_sigma_m": mean([l["gps_sigma"] for l in legs]), "north_delta_deg": fit["delta"],
             "per_run_residual_deg": fit["resid"]}
    problems = []
    if info["rms_radius"] > 0.12:
        problems.append("ellipse fit is loose (rms radius %.0f%%)" % (100 * info["rms_radius"]))
    if info["axis_ratio"] > 2.0:
        problems.append("compass cloud is very elongated (axis ratio %.2f)" % info["axis_ratio"])
    if info["heading_rms_vs_gyro"] > 8.0:
        problems.append("compass disagrees with the gyro by %.1f deg rms" % info["heading_rms_vs_gyro"])
    if spread > 15.0:
        problems.append("heading error changes by about %.0f deg between the three headings (compass distortion)" % spread)
    if fit["sigma"] > 6.0:
        problems.append("north offset uncertainty is %.1f deg (GPS too noisy)" % fit["sigma"])
    if fit["chi2"] > 9.0:
        problems.append("the three runs are not consistent with one heading offset (chi2 %.1f): unsteady heading or changing current" % fit["chi2"])
    if cur_gap > 0.3:
        problems.append("current estimates disagree by %.2f m/s (current changed during the test)" % cur_gap)
    unsteady = [l["name"] for l in legs if l["spread"] > 15.0]
    if unsteady:
        problems.append("heading was not steady in %s (spread over 15 deg)" % ", ".join(unsteady))
    return cal, ident, problems


def report(cal, ident, problems, info, bias):
    lines = ["", "CALIBRATION REPORT",
             "  compass sign %+d, north offset %+.1f deg (uncertainty %.1f deg), heading-dependent error %.1f deg" % (
                 cal["sign"], cal["north"], cal["north_sigma_deg"], cal["heading_dependent_error_deg"]),
             "  ellipse: axis ratio %.2f, radius rms %.1f%%, compass vs gyro %.1f deg rms, %d points" % (
                 info["axis_ratio"], 100 * cal["rms_radius"], cal["heading_rms_vs_gyro_deg"], cal["points"]),
             "  gyro: scale %.4f deg/s per count (spins: %.4f / %.4f), bias z %.1f counts" % (
                 ident["gyro_scale_dps_per_count"], ident["gyro_scale_each_spin"][0], ident["gyro_scale_each_spin"][1], bias[2]),
             "  yaw: left spin %+.1f deg/s, right spin %+.1f deg/s, gain %.3f deg/s per unit, bias %+.1f deg/s, tau %s" % (
                 ident["yaw_left_spin_dps"], ident["yaw_right_spin_dps"], ident["yaw_gain_dps_per_unit"],
                 ident["yaw_bias_dps"], "n/a" if ident["tau_yaw_s"] is None else "%.1f s" % ident["tau_yaw_s"]),
             "  trim to go straight (right-turn units): %+.1f at 100, %s at 60" % (
                 ident["trim_u_right_100"], "n/a" if ident["trim_u_right_60"] is None else "%+.1f" % ident["trim_u_right_60"]),
             "  speed through water: %.2f m/s at 100, %s at 60" % (
                 ident["speed_100_mps"], "n/a" if ident["speed_60_mps"] is None else "%.2f m/s" % ident["speed_60_mps"]),
             "  current (E,N): spins %.2f,%.2f  runs %.2f,%.2f m/s | GPS noise %.1f m" % (
                 ident["current_from_spins_en_mps"][0], ident["current_from_spins_en_mps"][1],
                 ident["current_from_runs_en_mps"][0], ident["current_from_runs_en_mps"][1], ident["gps_sigma_m"])]
    if problems:
        lines.append("  WARNINGS:")
        lines += ["   - " + p for p in problems]
    return "\n".join(lines)


def main():
    args = sys.argv[1:]
    if len(args) % 2:
        sys.exit("missing value for %s (usage: python3 calib_ident.py [--option value ...])" % args[-1])
    for flag, value in zip(args[::2], args[1::2]):
        key = flag.lstrip("-").replace("-", "_")
        if not flag.startswith("--") or key not in CONFIG:
            sys.exit("unknown option %s (usage: python3 calib_ident.py [--option value ...])" % flag)
        CONFIG[key] = type(CONFIG[key])(value)
    if os.path.isfile(STOP_FILE):
        sys.exit("%s exists: remove it (rm data/STOP)" % STOP_FILE)
    if not sys.stdin.isatty():
        sys.exit("needs a terminal (ssh or screen), do not use nohup")
    users = port_users(ARDUINO_TTY)
    if users:
        sys.exit("%s is in use by another process (motors would shake): %s" % (ARDUINO_TTY, "; ".join(users)))
    rig = Rig()
    set_signals(on_signal)
    rig.stop()
    os.makedirs(LOG_DIR, exist_ok=True)
    os.makedirs(CALIB_DIR, exist_ok=True)
    n = 1
    while os.path.exists(os.path.join(LOG_DIR, "ident_%03d.csv" % n)):
        n += 1
    log_path = os.path.join(LOG_DIR, "ident_%03d.csv" % n)
    log = open(log_path, "w")
    log.write(",".join(COLUMNS) + "\n")
    session, result, reason = Session(rig, log), None, "done"
    print("Calibration and identification: about 5 min, keep 40 m of free water around. Only q aborts.")
    try:
        result = session.run_all()
    except Abort as e:
        reason = "aborted: %s" % e
    except Stop as e:
        reason = "interrupted (%s)" % e
    except Exception as e:
        reason = "error: %r" % (e,)
    finally:
        set_signals(signal.SIG_IGN)
        rig.stop()
        log.close()
        rig.close()
        set_signals(signal.SIG_DFL)
    print("\nEnd: %s. Motors stopped. Log: %s" % (reason, log_path))
    if result is None:
        print("No calibration files were written.")
        if "from the start point" in reason:
            c = session.current_est
            print("Hint: the current is about %.2f m/s toward %.0f deg. Start the boat about 15 m up-current of the open area "
                  "or run with --max-radius 55 if there is room." % (math.hypot(*c), math.degrees(math.atan2(c[0], c[1])) % 360.0))
        return
    info = result
    cal, ident, problems = finish(info)
    print(report(cal, ident, problems, info, session.bias))
    ident["gyro_bias_counts"] = session.bias
    write_json(os.path.join(CALIB_DIR, "ident.json"), ident)
    target = os.path.join(CALIB_DIR, "mag2d.json")
    if problems:
        write_json(os.path.join(CALIB_DIR, "mag2d_candidate.json"), cal)
        print("Problems found: wrote data/calib/mag2d_candidate.json and kept the current mag2d.json.")
        return
    if os.path.isfile(target):
        os.rename(target, os.path.join(CALIB_DIR, "mag2d_old_%s.json" % time.strftime("%Y%m%d_%H%M%S")))
    write_json(target, cal)
    print("Wrote data/calib/mag2d.json (old one kept as mag2d_old_*.json) and data/calib/ident.json.")


if __name__ == "__main__":
    main()
