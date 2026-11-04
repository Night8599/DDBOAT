"""Regenerate the README figures from the Guerledan logs in results/logs/.

Usage (on a PC, from the repository root):
    python tools/make_figures.py

Needs matplotlib. Positions are projected into the local ENU frame used by the
mission scripts (origin = point B, equirectangular projection).
"""
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS = os.path.join(ROOT, "results", "logs")
OUT = os.path.join(ROOT, "docs", "figures")

LAT0, LON0 = 48.1990720, -3.0162580          # point B (origin of the ENU frame)
R_EARTH = 6371000.0

# Waypoints used in missions 3 to 6 (report, appendix A), ENU metres.
POINTS = {
    "A": (88.1, -23.3), "B": (0.0, 0.0), "C": (67.0, -105.5), "D": (-63.5, -90.5),
    "E": (-9.0, 61.9), "F": (81.6, 68.6), "G": (57.9, -14.2), "G'": (27.5, -5.0),
    "S": (73.0, -18.8),
}
OLD_EF = {"E": (-10.8, 70.7), "F": (83.2, 82.6)}   # mission 2 used the old E-F line

BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def enu(lat, lon):
    x = math.radians(lon - LON0) * R_EARTH * math.cos(math.radians(LAT0))
    y = math.radians(lat - LAT0) * R_EARTH
    return x, y


def read(name):
    with open(os.path.join(LOGS, name)) as f:
        rows = [r for r in csv.DictReader(l for l in f if not l.startswith("#"))]
    return rows


def style(ax, title):
    ax.set_title(title, loc="left", fontsize=11, color=INK)
    ax.set_aspect("equal")
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.tick_params(colors=INK2, labelsize=8)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_xlabel("East (m)", fontsize=8, color=INK2)
    ax.set_ylabel("North (m)", fontsize=8, color=INK2)


def waypoints(ax, names, pts=POINTS):
    for n in names:
        x, y = pts[n]
        ax.plot(x, y, "o", ms=5, color=INK, zorder=5)
        ax.annotate(n, (x, y), xytext=(4, 4), textcoords="offset points", fontsize=8, color=INK)


def other_boat(name, me=4):
    tracks = {}
    for r in read(name):
        if int(r["id"]) == me:
            continue
        tracks.setdefault(r["id"], []).append(enu(float(r["lat"]), float(r["lon"])))
    return tracks


def fig_kalman():
    rows = read("mission2_016.csv")
    raw = [enu(float(r["lat"]), float(r["lon"])) for r in rows if r["gps_new"] == "1"]
    kf = [(float(r["x_m"]), float(r["y_m"])) for r in rows]
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 5.2), gridspec_kw={"width_ratios": [1, 1]})
    for ax in (a, b):
        ax.plot(*zip(*raw), "o", ms=3, color=ORANGE, alpha=0.8, label="Raw GPS fixes (1 Hz)")
        ax.plot(*zip(*kf), "-", lw=2, color=BLUE, label="Kalman estimate (10 Hz)")
    style(a, "Mission 2: autonomous crossing and return (trial 016)")
    waypoints(a, ["A", "B"])
    (ex, ey), (fx, fy) = OLD_EF["E"], OLD_EF["F"]
    a.plot([ex, fx], [ey, fy], "-", color=INK2, lw=1)
    a.annotate("line E-F", (fx, fy), xytext=(-40, 6), textcoords="offset points", fontsize=8, color=INK2)
    a.legend(frameon=False, fontsize=8, loc="lower left")
    # zoom on the turn beyond the E-F line
    xs = [p[0] for p in kf]
    ys = [p[1] for p in kf]
    i = max(range(len(ys)), key=lambda k: ys[k])
    cx, cy = xs[i], ys[i]
    b.set_xlim(cx - 6, cx + 16)
    b.set_ylim(cy - 20, cy + 3)
    style(b, "Zoom on the U-turn")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "kalman_mission2.png"), dpi=150)
    plt.close(fig)


def fig_missions():
    fig, axes = plt.subplots(2, 2, figsize=(11, 10))
    (a3, a4), (a5, a6) = axes

    r3 = read("mission3_003.csv")
    route = [(float(r["x_m"]), float(r["y_m"])) for r in r3 if r["state"] != "RETURN"]
    ret = [(float(r["x_m"]), float(r["y_m"])) for r in r3 if r["state"] == "RETURN"]
    poly = [POINTS[n] for n in ("A", "C", "D", "B", "E", "F", "A")]
    a3.plot(*zip(*poly), "--", color=INK2, lw=1, label="Safety polygon")
    a3.plot(*zip(*route), "-", lw=2, color=BLUE, label="Waypoint route")
    if ret:
        a3.plot(*zip(*ret), "-", lw=2, color=ORANGE, label="Return-to-home (geofence)")
    waypoints(a3, ["A", "C", "D", "E", "F", "G", "G'"])
    style(a3, "Mission 3: waypoint route with geofence (trial 003)")
    a3.legend(frameon=False, fontsize=8, loc="upper left")

    r4 = read("mission4_006.csv")
    tr = [(float(r["x_m"]), float(r["y_m"])) for r in r4]
    sx, sy = POINTS["S"]
    t = [k * 2 * math.pi / 100 for k in range(101)]
    a4.plot([sx + 5 * math.cos(k) for k in t], [sy + 5 * math.sin(k) for k in t], "-", color=INK2, lw=1,
            label="5 m circle around S")
    a4.plot(*zip(*tr), "-", lw=2, color=BLUE, label="Boat 4")
    waypoints(a4, ["S", "A", "G"])
    style(a4, "Mission 4: station keeping, 60 s in a 5 m circle (trial 006)")
    a4.legend(frameon=False, fontsize=8, loc="lower left")

    r5 = read("mission5_004.csv")
    a5.plot(*zip(*[(float(r["x_m"]), float(r["y_m"])) for r in r5]), "-", lw=2, color=BLUE, label="Boat 4 (ours)")
    for _, pts in other_boat("mission5_004_boats.csv").items():
        a5.plot(*zip(*pts), "-", lw=2, color=AQUA, label="Boat 7 (other team)")
    waypoints(a5, ["G", "G'", "A"])
    style(a5, "Mission 5: two boats crossing on G-G' (trial 004)")
    a5.legend(frameon=False, fontsize=8, loc="lower left")

    r6 = read("mission6_002.csv")
    a6.plot(*zip(*[(float(r["x_m"]), float(r["y_m"])) for r in r6]), "-", lw=2, color=BLUE, label="Boat 4 (ours)")
    for _, pts in other_boat("mission6_002_boats.csv").items():
        a6.plot(*zip(*pts), "-", lw=2, color=AQUA, label="Boat 7 (other team)")
    gx, gy = POINTS["G"]
    a6.plot(gx, gy, "s", ms=7, color=ORANGE, label="Buoy (at G)", zorder=6)
    style(a6, "Mission 6: two-boat buoy capture with a rope (trial 002)")
    a6.legend(frameon=False, fontsize=8, loc="lower left")

    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "missions_3_to_6.png"), dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    fig_kalman()
    fig_missions()
    print("figures written to", OUT)
