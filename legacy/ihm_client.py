#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ihm_client.py  --  tourne SUR LE DD-BOAT (Raspberry Pi).

Lit le GPS, enregistre la trajectoire dans un CSV local et envoie chaque
position par wifi (TCP) a l'IHM qui tourne sur le PC (ihm_serveur.py).

Protocole : une ligne JSON par message, terminee par '\n', par exemple
    {"id": "Bottagisio", "t": 1792400000.12, "lat": 48.19901, "lon": -3.01386}
Champs optionnels : "cap" (degres, 0 = Nord, sens horaire), "mode", ou tout
autre champ ajoute par l'equipe (ils sont affiches tels quels par l'IHM).

Robustesse :
  - l'envoi se fait dans un thread separe : si le wifi tombe, la lecture GPS
    et le log CSV continuent sans jamais bloquer ;
  - les points non envoyes sont gardes en memoire et renvoyes a la reconnexion
    (pas de trou dans la trace affichee) ;
  - reconnexion automatique toutes les 2 s.

Utilisation (dans un screen !) :
    python3 ihm_client.py --serveur 172.20.25.XX --id Bottagisio
    python3 ihm_client.py --serveur 127.0.0.1 --simu      # sans GPS, pour tester

Utilisation comme module dans votre programme de controle :
    from ihm_client import ClientIHM
    ihm = ClientIHM("172.20.25.XX", ident="Bottagisio")
    ...
    ihm.envoyer(lat, lon, cap=cap_deg, mode="RETOUR")   # non bloquant
"""

import argparse
import collections
import csv
import json
import math
import os
import socket
import sys
import threading
import time

PORT_DEFAUT = 5005


# ---------------------------------------------------------------------------
# Envoi vers l'IHM
# ---------------------------------------------------------------------------
class ClientIHM:
    """Envoie des messages a l'IHM sans jamais bloquer l'appelant."""

    def __init__(self, serveur, port=PORT_DEFAUT, ident="ddboat",
                 taille_tampon=5000, verbose=True):
        self.serveur, self.port, self.ident = serveur, port, ident
        self.verbose = verbose
        # file des messages en attente (les plus vieux sont jetes si pleine)
        self.file = collections.deque(maxlen=taille_tampon)
        self.cond = threading.Condition()
        self.connecte = False
        self.actif = True
        self.thread = threading.Thread(target=self._boucle_envoi, daemon=True)
        self.thread.start()

    def envoyer(self, lat, lon, t=None, **extra):
        """Met une position dans la file d'envoi (retourne immediatement)."""
        msg = {"id": self.ident, "t": time.time() if t is None else t,
               "lat": round(lat, 7), "lon": round(lon, 7)}
        msg.update({k: v for k, v in extra.items() if v is not None})
        self._mettre_en_file(msg)

    def envoyer_etat(self, **infos):
        """Message sans position (ex. 'attente fix GPS'), affiche par l'IHM."""
        msg = {"id": self.ident, "t": time.time()}
        msg.update(infos)
        self._mettre_en_file(msg)

    def _mettre_en_file(self, msg):
        with self.cond:
            self.file.append(json.dumps(msg) + "\n")
            self.cond.notify()

    def fermer(self):
        self.actif = False
        with self.cond:
            self.cond.notify()

    def _log(self, txt):
        if self.verbose:
            # print peut bloquer si la connexion ssh est perdue : on protege
            try:
                print("[IHM] " + txt, flush=True)
            except Exception:
                pass

    def _boucle_envoi(self):
        sock = None
        while self.actif:
            if sock is None:
                try:
                    sock = socket.create_connection((self.serveur, self.port), timeout=2.0)
                    sock.settimeout(2.0)
                    self.connecte = True
                    self._log("connecte a %s:%d (%d points en attente)"
                              % (self.serveur, self.port, len(self.file)))
                except OSError:
                    self.connecte = False
                    time.sleep(2.0)
                    continue
            with self.cond:
                while self.actif and not self.file:
                    self.cond.wait(timeout=1.0)
                if not self.file:
                    continue
                # on envoie tout ce qui est en attente d'un coup
                lot = list(self.file)
                self.file.clear()
            try:
                sock.sendall("".join(lot).encode("utf-8"))
            except OSError:
                self._log("connexion perdue, on garde les points et on reessaie")
                with self.cond:
                    self.file.extendleft(reversed(lot))   # remet en tete, dans l'ordre
                try:
                    sock.close()
                except OSError:
                    pass
                sock, self.connecte = None, False
                time.sleep(1.0)
        if sock is not None:
            sock.close()


# ---------------------------------------------------------------------------
# Lecture du GPS (drivers-ddboat-v2)
# ---------------------------------------------------------------------------
def nmea_vers_degres(valeur, hemisphere):
    """DDMM.MMMM (ou DDDMM.MMMM) + 'N'/'S'/'E'/'W'  ->  degres decimaux signes."""
    v = float(valeur)
    deg = int(v // 100)
    minutes = v - 100 * deg
    d = deg + minutes / 60.0
    return -d if str(hemisphere).strip().upper() in ("S", "W") else d


class LecteurGPS:
    """Enveloppe autour de gps_driver_v2. Retourne (lat, lon) ou None."""

    def __init__(self, chemin_drivers=None):
        ici = os.path.dirname(os.path.abspath(__file__))
        candidats = [chemin_drivers] if chemin_drivers else [
            ici,                                  # drivers a cote du script (cas normal)
            os.getcwd(),                          # dossier depuis lequel on lance
            os.path.join(ici, "drivers-ddboat-v2"),
            os.path.join(ici, "..", "drivers-ddboat-v2"),
            os.path.expanduser("~/drivers-ddboat-v2"),
            os.path.expanduser("~/git/drivers-ddboat-v2")]
        for c in candidats:
            if os.path.isfile(os.path.join(c, "gps_driver_v2.py")):
                sys.path.append(c)
                break
        else:
            sys.exit("gps_driver_v2.py introuvable. Cherche ici : %s\n"
                     "-> preciser le dossier avec --drivers" % candidats)
        import gps_driver_v2 as gpsdrv          # noqa: E402  (fourni sur le bateau)
        self.gps = gpsdrv.GpsIO()
        try:
            self.gps.set_filter_speed("0")      # on garde aussi les points a l'arret
        except Exception:
            pass

        self.n_gll = 0           # trames GLL recues
        self.n_invalides = 0     # trames GLL sans position exploitable
        self.dernier_brut = None # derniere trame brute (pour le diagnostic)

    def lire(self):
        """Lit toutes les trames en attente (le driver en lit une par appel)
        et retourne la position la plus recente, ou None."""
        pos = None
        for _ in range(30):
            ok, data = self.gps.read_gll_non_blocking()
            if not ok or not data:
                continue
            self.n_gll += 1
            self.dernier_brut = data
            # data = [lat DDMM.MMMM, 'N', lon DDDMM.MMMM, 'W', heure, ...]
            try:
                lat = nmea_vers_degres(data[0], data[1])
                lon = nmea_vers_degres(data[2], data[3])
            except (ValueError, TypeError, IndexError):
                self.n_invalides += 1
                continue
            if lat == 0.0 and lon == 0.0:       # pas encore de fix
                self.n_invalides += 1
                continue
            pos = (lat, lon)
        return pos

    def diagnostic(self):
        return {"etat": "attente fix GPS", "trames_gll": self.n_gll,
                "trames_sans_fix": self.n_invalides,
                "derniere_trame": str(self.dernier_brut)[:60]}


class GPSSimule:
    """Faux GPS : le bateau tourne en rond sur le lac pres de la base."""

    def __init__(self, lat_c=48.197021, lon_c=-3.017909, rayon=150.0, vitesse=1.5):
        self.lat_c, self.lon_c, self.r, self.v = lat_c, lon_c, rayon, vitesse
        self.t0 = time.time()

    def lire(self):
        t = time.time() - self.t0
        a = self.v * t / self.r                  # angle parcouru
        x = self.r * math.cos(a) + 15 * math.sin(0.3 * t)   # m, vers l'Est
        y = 0.6 * self.r * math.sin(a)                       # m, vers le Nord
        lat = self.lat_c + math.degrees(y / 6371000.0)
        lon = self.lon_c + math.degrees(x / (6371000.0 * math.cos(math.radians(self.lat_c))))
        return lat, lon


# ---------------------------------------------------------------------------
# Programme principal
# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Envoie la position GPS du DD-Boat a l'IHM.")
    p.add_argument("--serveur", default=None,
                   help="IP du PC qui fait tourner ihm_serveur.py "
                        "(par defaut : le PC d'ou vient la connexion ssh)")
    p.add_argument("--port", type=int, default=PORT_DEFAUT)
    p.add_argument("--id", default="ddboat", help="nom du bateau / de l'equipe")
    p.add_argument("--drivers", default=None,
                   help="dossier contenant gps_driver_v2.py (cherche tout seul sinon)")
    p.add_argument("--periode", type=float, default=0.1, help="periode de lecture GPS (s)")
    p.add_argument("--simu", action="store_true", help="faux GPS pour tester sans bateau")
    p.add_argument("--duree", type=float, default=0, help="arret apres N s (0 = jamais)")
    args = p.parse_args()

    if args.serveur is None:
        # SSH_CLIENT = "IP_du_PC port_PC 22", defini dans toute session ssh (et dans screen)
        ssh = os.environ.get("SSH_CLIENT", "").split()
        if not ssh:
            sys.exit("Pas de --serveur et pas de session ssh : preciser --serveur <IP du PC>")
        args.serveur = ssh[0]
        print("IP du PC trouvee via ssh :", args.serveur, flush=True)

    gps =GPSSimule() if args.simu else LecteurGPS(args.drivers)
    ihm = ClientIHM(args.serveur, args.port, ident=args.id)

    nom_log = time.strftime("traj_%Y%m%d_%H%M%S.csv")
    f = open(nom_log, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "lat", "lon"])
    print("Log local :", nom_log, flush=True)

    t_debut = time.time()
    t_dernier_fix = t_debut
    t_dernier_diag = 0.0
    n = 0
    try:
        while args.duree <= 0 or time.time() - t_debut < args.duree:
            pos = gps.lire()
            maintenant = time.time()
            if pos is not None:
                if n == 0:
                    print("Premier fix GPS : %.6f %.6f" % pos, flush=True)
                t_dernier_fix = maintenant
                w.writerow(["%.2f" % maintenant, "%.7f" % pos[0], "%.7f" % pos[1]])
                n += 1
                if n % 10 == 0:
                    f.flush()
                ihm.envoyer(pos[0], pos[1], t=maintenant)
            elif maintenant - t_dernier_fix > 2 and maintenant - t_dernier_diag > 3:
                # pas de position depuis 2 s : on previent l'IHM et la console
                t_dernier_diag = maintenant
                diag = gps.diagnostic() if hasattr(gps, "diagnostic") else {"etat": "pas de position"}
                ihm.envoyer_etat(**diag)
                try:
                    print("[GPS] %s" % diag, flush=True)
                except Exception:
                    pass
            time.sleep(args.periode)
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        time.sleep(0.5)          # laisse partir les derniers points
        ihm.fermer()
        print("\n%d points enregistres dans %s" % (n, nom_log))


if __name__ == "__main__":
    main()
