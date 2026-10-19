# -*- coding: utf-8 -*-
"""Log -> KML (Google Earth) e GPX. Uso: python tools/log2kml.py data/logs/ARQ.csv"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ddboat.io.logger import read_csv  # noqa: E402


def main():
    if len(sys.argv) != 2:
        print("uso: python tools/log2kml.py LOG.csv")
        return 1
    path = sys.argv[1]
    cols, rows = read_csv(path)
    pts = [(r["lat"], r["lon"]) for r in rows if r.get("lat") not in (None, 0.0) and r.get("lon") is not None]
    if not pts:
        print("ERRO: log sem lat/lon")
        return 1
    base = os.path.splitext(path)[0]
    name = os.path.basename(base)
    coords = " ".join("%.8f,%.8f,0" % (lon, lat) for lat, lon in pts)
    with open(base + ".kml", "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2"><Document>'
                '<name>%s</name><Style id="l"><LineStyle><color>ff0000ff</color><width>3</width></LineStyle></Style>'
                '<Placemark><name>trajetoria</name><styleUrl>#l</styleUrl><LineString><tessellate>1</tessellate>'
                '<coordinates>%s</coordinates></LineString></Placemark></Document></kml>\n' % (name, coords))
    with open(base + ".gpx", "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n<gpx version="1.1" creator="ddboat" '
                'xmlns="http://www.topografix.com/GPX/1/1"><trk><name>%s</name><trkseg>\n' % name)
        for lat, lon in pts:
            f.write('<trkpt lat="%.8f" lon="%.8f"></trkpt>\n' % (lat, lon))
        f.write("</trkseg></trk></gpx>\n")
    print("%d pontos -> %s.kml e %s.gpx" % (len(pts), base, base))
    return 0


if __name__ == "__main__":
    sys.exit(main())
