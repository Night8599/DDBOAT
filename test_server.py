import json
import math
import os
import sys
import time
import types

from ddboats_client import SERVER_IP, get_boats, send_position

ROOT = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(ROOT, "data", "logs")
EARTH = 6378137.0
CONFIG = {"id": 4, "server": SERVER_IP, "port": 6800, "period": 1.0, "align_tol": 15.0,
          "align_reads": 5, "max_age": 5.0}
COLUMNS = ["t_s", "lat", "lon", "heading_deg", "spread_m", "boat_id", "boat_lat", "boat_lon",
           "boat_cap_deg", "boat_age_s", "dist_m", "err_deg", "streak"]


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def distance(a, b):
    lat0 = math.radians((a[0] + b[0]) / 2.0)
    return EARTH * math.hypot(math.radians(b[1] - a[1]) * math.cos(lat0), math.radians(b[0] - a[0]))


def ddmm(value, hemisphere):
    v = float(value)
    d = int(v / 100)
    result = d + (v - d * 100) / 60.0
    return -result if hemisphere in ("S", "W") else result


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


def read_heading(imu, calibration):
    samples = []
    for _ in range(3):
        samples.append(imu.read_mag_raw())
        time.sleep(0.01)
    mag = [sorted(v[i] for v in samples)[1] for i in range(3)]
    return compass_heading(mag, calibration)


def main():
    args = sys.argv[1:]
    if len(args) % 2:
        sys.exit("missing value for %s (usage: python3 test_server.py [--id 4 --server IP --port 6800])" % args[-1])
    for flag, value in zip(args[::2], args[1::2]):
        key = flag.lstrip("-")
        if not flag.startswith("--") or key not in CONFIG:
            sys.exit("unknown option %s (usage: python3 test_server.py [--id 4 --server IP --port 6800])" % flag)
        CONFIG[key] = type(CONFIG[key])(value)
    calibration = load_calibration()
    load_drivers()
    import imu9_driver_v2
    import gps_driver_v2
    imu = imu9_driver_v2.Imu9IO()
    try:
        gps = gps_driver_v2.GpsIO(0)
        gps.send_mtk_cmd("PMTK386,0")
    except Exception as e:
        sys.exit("GPS not initialized: %r" % (e,))
    os.makedirs(LOG_DIR, exist_ok=True)
    n = 1
    while os.path.exists(os.path.join(LOG_DIR, "test_server_%03d.csv" % n)):
        n += 1
    path = os.path.join(LOG_DIR, "test_server_%03d.csv" % n)
    log = open(path, "w")
    log.write(",".join(COLUMNS) + "\n")
    print("Boat %d -> %s:%d, %.1f s period. No motor command. Ctrl+C ends." % (
        CONFIG["id"], CONFIG["server"], CONFIG["port"], CONFIG["period"]))
    t0 = time.time()
    last, fixes, raw = None, [], ""
    streak, seen = {}, set()
    sent = errors = 0
    next_tick = t0
    try:
        while True:
            for _ in range(5):
                ok, d = gps.read_rmc_non_blocking()
                if ok and not (d[0] == 0.0 and d[2] == 0.0):
                    lat, lon = ddmm(d[0], d[1]), ddmm(d[2], d[3])
                    if -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0:
                        last = (lat, lon)
                        fixes = (fixes + [last])[-10:]
                        raw = " ".join(str(c) for c in d)
            now = time.time()
            if now < next_tick:
                time.sleep(0.05)
                continue
            next_tick = next_tick + CONFIG["period"] if now - next_tick < CONFIG["period"] else now + CONFIG["period"]
            h = read_heading(imu, calibration)
            if last is None:
                print("[%d] waiting for GPS fix... hdg=%5.1f" % (CONFIG["id"], h))
                continue
            try:
                send_position(CONFIG["server"], CONFIG["port"], CONFIG["id"], last[0], last[1], h)
                sent += 1
            except Exception:
                errors += 1
            try:
                boats = get_boats()
            except Exception:
                boats = {}
                errors += 1
            spread = max(distance(f, fixes[-1]) for f in fixes)
            frozen = len(fixes) == 10 and spread < 0.01
            t = now - t0
            others = [i for i in sorted(boats) if i != CONFIG["id"]]
            print("[%d] %.7f,%.7f hdg=%5.1f | last %d fixes within %.2f m%s | sent=%d err=%d | others=%d" % (
                CONFIG["id"], last[0], last[1], h, len(fixes), spread, "  <-- FROZEN?" if frozen else "",
                sent, errors, len(others)))
            print("    RMC: %s" % raw)
            row = [t, last[0], last[1], h, spread]
            if not others:
                log.write(",".join("%.7f" % v if isinstance(v, float) else str(v) for v in row) + "\n")
            for i in others:
                b = boats[i]
                seen.add(i)
                dist = distance(last, (b["lat"], b["lon"]))
                err = wrap_deg(h - b["cap"] - 180.0)
                good = b["age"] <= CONFIG["max_age"] and abs(err) <= CONFIG["align_tol"]
                streak[i] = streak.get(i, 0) + 1 if good else 0
                print("    boat %d %.7f,%.7f cap=%5.1f age=%4.1f s | d=%6.1f m | err=%+6.1f deg | streak=%d%s" % (
                    i, b["lat"], b["lon"], b["cap"], b["age"], dist, err, streak[i],
                    "  <-- ALIGNED" if streak[i] >= CONFIG["align_reads"] else ""))
                full = row + [i, b["lat"], b["lon"], b["cap"], b["age"], dist, err, streak[i]]
                log.write(",".join("%.7f" % v if isinstance(v, float) else str(v) for v in full) + "\n")
            log.flush()
    except KeyboardInterrupt:
        pass
    finally:
        log.close()
        print("\nEnd: sent %d, errors %d, boats seen: %s. Log: %s" % (
            sent, errors, " ".join(str(i) for i in sorted(seen)) or "none", path))


if __name__ == "__main__":
    main()
