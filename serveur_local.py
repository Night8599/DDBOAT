#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serveur_local.py  --  tourne sur le PC (Python 3, aucune dependance).

Remplace le serveur commun d'Antoine quand il est indisponible : MEME protocole,
donc rien a changer sur les bateaux sauf --serveur <IP de ce PC>.

  - ecoute en UDP (port 6800 par defaut) ;
  - recoit "id|lat|lon|cap"            (send_position du client d'Antoine)
       ou "id|$GPGLL,....|cap"         (send_gpgll) ;
  - garde la derniere position de chaque bateau ;
  - repond a CHAQUE paquet, a l'adresse qui l'a envoye :
       "ALL|id,lat,lon,cap,age|id,lat,lon,cap,age|..."   (age en s depuis sa derniere position)
  - affiche toutes les 2 s le tableau des bateaux, et enregistre tout dans
    serveur_local_<date>.csv (une ligne par paquet recu).

    sudo ufw allow 6800/udp           # une fois : ouvrir le port
    python3 serveur_local.py          # puis donner l'IP affichee aux bateaux :
    python3 test_serveur.py --id 7 --serveur 172.20.x.x      (sur chaque bateau)
Ctrl+C pour arreter.
"""
import argparse
import csv
import socket
import time


def mes_ip():
    """IP de ce PC sur le reseau (sans rien envoyer)."""
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("172.20.26.99", 6800))          # pas d'envoi : choisit juste l'interface
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            if not ip.startswith("127."):
                ips.add(ip)
    except OSError:
        pass
    return sorted(ips)


def nmea_vers_deg(v, h):
    v = float(v)
    d = int(v / 100) + (v % 100) / 60.0
    return -d if h.strip().upper() in ("S", "W") else d


def decoder(msg):
    """-> (id, lat, lon, cap) ou None si le message est illisible."""
    p = msg.strip().split("|")
    if len(p) == 4:                                   # id|lat|lon|cap
        return p[0].strip(), float(p[1]), float(p[2]), float(p[3]) % 360
    if len(p) == 3 and p[1].strip().startswith("$GPGLL"):   # id|$GPGLL,lat,N,lon,W,...|cap
        c = p[1].strip().split(",")
        if len(c) < 5 or not c[1] or not c[3]:
            return None
        return p[0].strip(), nmea_vers_deg(c[1], c[2]), nmea_vers_deg(c[3], c[4]), float(p[2]) % 360
    return None


def main():
    ap = argparse.ArgumentParser(description="Serveur UDP local, meme protocole que celui d'Antoine.")
    ap.add_argument("--port", type=int, default=6800)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(0.5)
    nom_log = "serveur_local_%s.csv" % time.strftime("%Y%m%d_%H%M%S")
    f_log = open(nom_log, "w", newline="")
    log = csv.writer(f_log)
    log.writerow(["t_pc", "adresse", "id", "lat", "lon", "cap", "message"])
    print("Serveur UDP en ecoute sur le port %d" % args.port)
    print("IP de ce PC a donner aux bateaux (--serveur) : %s" % (" ou ".join(mes_ip()) or "?  (taper hostname -I)"))
    print("  -> prendre celle en 172.20.x.x (reseau DARTAP). Log : %s\n" % nom_log)

    bateaux = {}          # id -> {"lat", "lon", "cap", "t", "adr", "n"}
    n_illisibles = 0
    t_aff = 0.0
    try:
        while True:
            try:
                data, adr = sock.recvfrom(4096)
            except socket.timeout:
                data = None
            t = time.time()
            if data:
                msg = data.decode("utf-8", "replace")
                try:
                    r = decoder(msg)
                except ValueError:
                    r = None
                if r is None:
                    n_illisibles += 1
                    print("message illisible de %s:%d : %r" % (adr[0], adr[1], msg[:80]))
                else:
                    ident, lat, lon, cap = r
                    b = bateaux.setdefault(ident, {"n": 0})
                    if b["n"] == 0:
                        print(">>> nouveau bateau : id %s depuis %s:%d" % (ident, adr[0], adr[1]))
                    b.update(lat=lat, lon=lon, cap=cap, t=t, adr=adr)
                    b["n"] += 1
                    log.writerow(["%.3f" % t, "%s:%d" % adr, ident, "%.8f" % lat, "%.8f" % lon, "%.1f" % cap, msg.strip()])
                    rep = "ALL" + "".join("|%s,%.8f,%.8f,%.1f,%.2f" % (i, v["lat"], v["lon"], v["cap"], t - v["t"])
                                          for i, v in sorted(bateaux.items()))
                    try:
                        sock.sendto(rep.encode("utf-8"), adr)
                    except OSError as ex:
                        print("reponse impossible vers %s:%d (%s)" % (adr[0], adr[1], ex))
            if t - t_aff >= 2.0:
                t_aff = t
                f_log.flush()
                if bateaux:
                    print(time.strftime("%H:%M:%S") + "  " + " | ".join(
                        "id %s : %.6f,%.6f cap %5.1f age %4.1f s (%d paquets, %s)" % (
                            i, v["lat"], v["lon"], v["cap"], t - v["t"], v["n"], v["adr"][0])
                        for i, v in sorted(bateaux.items())) +
                        ("  | %d illisible(s)" % n_illisibles if n_illisibles else ""))
                else:
                    print(time.strftime("%H:%M:%S") + "  en attente de paquets...")
    except KeyboardInterrupt:
        print("\nArret. %s" % ", ".join("id %s : %d paquets" % (i, v["n"]) for i, v in sorted(bateaux.items())))
    finally:
        f_log.close()


if __name__ == "__main__":
    main()
