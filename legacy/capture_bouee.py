#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capture_bouee.py  --  tourne SUR LE DD-BOAT (lac de Guerledan).  Python 3.5.

Mission "capture de bouee" a DEUX bateaux relies par un cable flottant de 20 m
(bateaux 7 et 4). La bouee est un 3e DD-Boat (id 9) en station keeping sur G.
Les DEUX bateaux font tourner CE MEME fichier ; seul --id change.

FICHIER AUTONOME : il n'importe que les drivers du bateau (gps_driver_v2,
imu9_driver_v2, arduino_driver_v2). Les fonctions de l'equipe 7 (suivi de
ligne, ddboat_mission.py, mission_abba.py, geofencing.py) sont RECOPIEES ici,
sans numpy ni scipy (memes formules), pour pouvoir donner le fichier a
l'equipe 4.

TRAJECTOIRE (identique sur les deux bateaux : calculee a partir de A, G et des
options, jamais a partir de mesures) -- variante "demi-tour en rotation" :
  route CENTRALE (milieu des deux bateaux), origine A, f = direction A -> G :
   1. ALLER  : de A, la formation se decale progressivement de 2R (24 m) sur le
               cote oppose au virage (raccord en S sur --l-trans m), puis va
               tout droit parallelement a A->G, en laissant la bouee a 13 m ;
   2. VIRAGE : demi-tour de rayon R (12 m) a --au-dela m au-dela de G ; les
               bateaux restent COTE A COTE (interieur 7 m, exterieur 17 m : le
               bateau exterieur va plus vite) -> le cable reste derriere eux ;
   3. RETOUR : ligne droite passant par G (la bouee passe entre les bateaux et
               entre dans le U du cable) jusqu'a A, en remorquant la bouee.
  Chaque bateau suit la route centrale decalee de +/- ecart/2 (5 m) :
   cote "droite" (defaut du 7) = a droite de la route au depart (point NORD,
   A + 5 m), il est a l'interieur du virage et finit au point SUD ; cote
   "gauche" (defaut du 4) = l'inverse.

SYNCHRONISATION (pas d'horloge commune necessaire) :
  s = abscisse curviligne sur la route CENTRALE (projection de la position) ;
  vitesse = V_centre(s) x rapport(s) + K_s x (s_autre - s_moi), bornee [0, 200]
  ((s_autre - s_moi) filtre sur --tau-ds = 2 s : le GPS bouge de ~1 m a chaque fix)
  - rapport(s) = longueur de MON trajet / longueur de la route centrale (1 en
    ligne droite, 17/12 a l'exterieur et 7/12 a l'interieur du virage) ;
  - s_autre : position de l'autre bateau lue sur le serveur, extrapolee (1-2 s
    de retard) puis projetee sur la route centrale.
  Guidage lateral : champ de ligne du suivi de l'equipe (meme formule que
  cap_potentiel, sens impose = sens de la route) sur MON trajet, evalue
  --avance m devant (anticipe les virages) ; regulateur du suivi de ligne
  (v = vitesse x max(0, cos e), u = KP x e, L = v + u, R = v - u).
  Bouee : pendant le RETOUR, la ligne est recentree sur la position REELLE du
  bateau 9 (serveur), au plus +/- 4 m, puis figee 10 m avant la bouee.

AUTOMATE :
  ATTENTE_FIX -> ATTENTE -> FORMATION (ALLER, VIRAGE, RETOUR) -> FIN
  ATTENTE : moteurs coupes. Depart (regle "pret" de mission_abba.py) quand
    - nous sommes a moins de --r-pret (8 m) de NOTRE point de depart (et plus
      pres du notre que de celui de l'autre : bateaux inverses = pas de depart), ET
    - l'autre a ete vu --n-pret (5) fix d'affilee a moins de --r-pret de SON
      point de depart, quasi immobile (< 0,6 m/s) ;
    - (secours) ou l'autre est deja parti (vu sur son point puis avance de
      plus de 4 m sur la route) : on part aussi et on le rattrape.
  FIN : arrivee au bout de la route (A), ou une alerte -> moteurs coupes. Le
    programme continue d'envoyer sa position et d'ecrire le log : Ctrl+C pour
    quitter.

SECURITES (moteurs coupes) :
  - pas de fix GPS depuis --perte-gps s (3 s) ;
  - position de l'autre perdue depuis 8 s (pause), FIN apres --perte-autre-max s ;
  - ecart > --ecart-max (18 m, le cable fait 20 m) : celui qui est devant attend ;
  - ecart < --ecart-min (3 m sur 2 mesures fraiches, une fois les bateaux ecartes) : arret jusqu'a 5 m ;
  - pas 1 m de progres en --timeout-progres s (45 s) -> FIN ;
  - sortie de la zone E-F-C-D (armee apres 3 fix dedans, coupee sur les 15
    derniers metres car A est 2,7 m hors zone) -> FIN ;
  - --duree-max (600 s) -> FIN ; Ctrl+C, kill, fermeture du terminal -> FIN ;
  - --a-sec : les moteurs ne sont JAMAIS commandes.
  Si le programme meurt sans passer par la fin (batterie, kill -9), l'Arduino
  garde la derniere commande -> kayak pres des bateaux.

CALIBRATION BOUSSOLE : celle de CHAQUE bateau (invA, xbar de 10__magcalib.py),
  dans CALIB ci-dessous, ou dans un fichier texte (--calib-fichier) contenant
  les 9 nombres de invA (ligne par ligne) puis les 3 de xbar (crochets,
  virgules et retours a la ligne acceptes).

LES DEUX BATEAUX DOIVENT AVOIR LES MEMES OPTIONS DE TRAJECTOIRE (--ecart,
--rayon, --au-dela, --l-trans, --virage, --a, --g) : comparer la ligne "PLAN"
affichee au demarrage sur les deux bateaux (meme longueur).

Fichiers dans le MEME dossier sur le bateau :
    capture_bouee.py  gps_driver_v2.py  imu9_driver_v2.py  arduino_driver_v2.py
    (+ i2creal.py, ou un dossier drivers/ avec i2creal.py et __init__.py)
    (ou --drivers DOSSIER si les drivers sont ailleurs)

Utilisation (TOUJOURS dans un screen) :
    python3 capture_bouee.py --id 7 --verifier     # affiche le plan et les controles, ne bouge pas
    python3 capture_bouee.py --id 7 --a-sec        # ponton : capteurs, serveur, regle de depart, sans moteurs
    python3 capture_bouee.py --id 7 --seul         # sur l'eau SANS cable ni partenaire : suit son trajet seul
    python3 capture_bouee.py --id 7                # mission (et --id 4 sur l'autre bateau)
Logs (dossier courant) : capture_b<id>_<date>.csv (chaque tour de boucle),
    capture_b<id>_<date>_console.txt (terminal), capture_b<id>_<date>_plan.csv
    (trajectoires prevues des deux bateaux).
Conventions : positions (est, nord) en m, origine A ; cap en rad, 0 = Nord,
sens horaire ; erreur > 0 -> L > R -> le bateau tourne a droite.
"""

import argparse
import csv
import math
import os
import re
import signal
import socket
import sys
import time
import traceback
import types


# ---------------------------------------------------------------------------
# PARAMETRES PAR DEFAUT (tous modifiables en option)
# ---------------------------------------------------------------------------
SERVEUR = "172.20.26.20"        # serveur commun (Antoine), UDP
PORT_SERVEUR = 6800
PERIODE_SERVEUR = 1.0           # s (1 Hz)
ID_BOUEE = "9"                  # bateau qui porte la bouee (station keeping sur G)
AUTRE_DEFAUT = {"7": "4", "4": "7"}
COTE_DEFAUT = {"7": "droite", "4": "gauche"}

A_DEG = (48.198863, -3.015070)  # depart et arrivee (milieu des deux bateaux)
G_DEG = (48.198944, -3.015478)  # bouee
# Zone de geofencing E-F-C-D (= geofence.txt / geofencing.py), dans l'ordre du contour
ZONE = [("E", 48.199707, -3.016404),
        ("F", 48.199814, -3.015137),
        ("C", 48.197408, -3.01506),
        ("D", 48.1970689, -3.017328)]

ECART = 10.0                    # m entre les deux bateaux (cable 20 m : U de 8,7 m)
RAYON = 12.0                    # m : rayon du demi-tour de la route centrale
AU_DELA = 28.0                  # m : debut du demi-tour, au-dela de G
L_TRANS = 40.0                  # m : longueur du raccord en S du decalage, a l'aller

V_BASE = 120                    # vitesse d'avance (0-255), V_BASE du suivi (~1 m/s)
V_MAX = 200                     # commande d'avance maximale (bateau exterieur au virage)
V_MIN = 0
KP = 57.0                       # KP du suivi (57 ~ 1 par degre)
K_N, K_V = 0.1, 1.0             # poids du champ de ligne du suivi
K_S = 10.0                      # synchro : commande par metre de retard sur l'autre
TAU_DS = 2.0                    # s : filtre (1er ordre) de s_autre - s_moi (bruit GPS de 1 m)
AVANCE = 2.0                    # m : le champ de ligne est evalue AVANCE m devant

R_PRET = 8.0                    # m : "sur son point de depart"
N_PRET = 5                      # fix d'affilee de l'autre sur son point
AGE_PRET = 3.0                  # s : position de l'autre assez recente pour compter
V_PRET = 0.6                    # m/s : l'autre compte comme pret s'il est quasi immobile
S_PARTI = 4.0                   # m : l'autre a avance de plus de ca -> il est parti

AGE_PERDU = 8.0                 # s : position de l'autre perdue -> pause moteurs
PERTE_AUTRE_MAX = 30.0          # s : ... puis FIN
HORIZON = 3.0                   # s : extrapolation max de la position de l'autre
V_MAX_AUTRE = 2.5               # m/s : vitesse estimee de l'autre bornee a ca
ECART_MAX = 18.0                # m : au-dela, celui qui est devant attend
ECART_MIN = 3.0                 # m : en dessous, arret (repart au-dela de ECART_MIN + 2)
RECENTRAGE_MAX = 4.0            # m : decalage maximal de la ligne de retour vers la bouee reelle
FIGE_AVANT = 10.0               # m : recentrage fige quand la formation est a ca de la bouee
RAMPE = 10.0                    # m : le recentrage s'applique progressivement au debut du retour
R_FIN = 1.5                     # m : FIN a moins de ca du bout de la route
FIN_SANS_ZONE = 15.0            # m : geofencing coupe sur les derniers metres (A hors zone)
MARGE_PLAN = 2.0                # m : le plan doit rester a ca du bord de la zone (hors 15 m des bouts)

R_TERRE = 6371000.0             # comme le suivi de ligne

# Calibration boussole (10__magcalib.py) de CHAQUE bateau : (invA 3x3, xbar 3)
CALIB = {
    "7": ([[-2.87564918e-04, -5.72701381e-05, 2.54545662e-05],
           [-3.51695171e-05, 2.92630051e-04, 5.44283319e-06],
           [-1.88775100e-05, 1.11759540e-05, -3.01125112e-04]],
          [-814.37049587, -241.77837075, 276.86354638]),
    "4": ([[3.07173218e-04, 2.08611770e-06, -1.07820372e-06],      # equipe 4, 22/10
           [2.08611770e-06, 3.18363454e-04, -2.28557633e-06],
           [-1.07820372e-06, -2.28557633e-06, 3.24015876e-04]],
          [3391.29, -4267.36, 5506.95]),
}


# ---------------------------------------------------------------------------
# Journal : tout ce qui est affiche est aussi ecrit dans le _console.txt
# (dire() de ddboat_mission.py)
# ---------------------------------------------------------------------------
JOURNAL = None
T_DEBUT = time.time()


def dire(txt):
    try:
        print(txt, flush=True)
    except Exception:
        pass                                      # terminal ferme : on continue
    if JOURNAL is not None:
        try:
            for ligne in str(txt).split("\n"):
                JOURNAL.write("[%7.1f s] %s\n" % (time.time() - T_DEBUT, ligne))
            JOURNAL.flush()
            os.fsync(JOURNAL.fileno())
        except Exception:
            pass


class ArretSignal(Exception):
    pass


def _sur_signal(signum, frame):
    raise ArretSignal(signum)


# ---------------------------------------------------------------------------
# Petits outils vectoriels (remplacent numpy) : vecteurs = tuples (est, nord)
# ---------------------------------------------------------------------------
def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def v_mul(a, k):
    return (a[0] * k, a[1] * k)


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def v_norm(a):
    return math.hypot(a[0], a[1])


def v_dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def unitaire(d):
    n = v_norm(d)
    return (d[0] / n, d[1] / n) if n > 1e-6 else (0.0, 1.0)


def gauche_de(u):
    """Vecteur unitaire a GAUCHE de la direction u (rotation +90 deg, sens trigo)."""
    return (-u[1], u[0])


def droite_de(u):
    """Vecteur unitaire a DROITE de la direction u (rotation -90 deg)."""
    return (u[1], -u[0])


def borne(x, a, b):
    return a if x < a else (b if x > b else x)


# ---------------------------------------------------------------------------
# Fonctions du SUIVI DE LIGNE de l'equipe (suivi_de_ligne.py), recopiees
# sans numpy/scipy (memes formules)
# ---------------------------------------------------------------------------
def nmea_vers_rad(v, h):
    """Format NMEA ddmm.mmmm -> radians (Sud et Ouest negatifs)."""
    deg = int(v / 100) + (v % 100) / 60.0
    deg = -deg if h in ("S", "W") else deg
    return math.radians(deg)


def vers_metres(lat, lon, lat0, lon0):
    """lat, lon, lat0, lon0 en radians -> (est, nord) en metres par rapport au point 0."""
    est = (lon - lon0) * R_TERRE * math.cos(lat0)
    nord = (lat - lat0) * R_TERRE
    return (est, nord)


def ecart_angle(a):
    """Dent de scie : ramene un angle (rad) dans [-pi, pi[.
    (= pi * sawtooth(a + pi) de scipy, ecrit sans scipy)"""
    return (a + math.pi) % (2 * math.pi) - math.pi


def cap_bateau(imu, invA, xbar):
    """Cap compas en radians, dans [0, 2pi[ (0 = Nord, pi/2 = Est).
    y = invA (x - xbar) ; cap = atan2(y1, y0)  (comme le suivi)."""
    x = imu.read_mag_raw()
    d = [float(x[i]) - xbar[i] for i in range(3)]
    y0 = sum(invA[0][j] * d[j] for j in range(3))
    y1 = sum(invA[1][j] * d[j] for j in range(3))
    return math.atan2(y1, y0) % (2 * math.pi)


def champ_ligne(p, c, u, k_n=K_N, k_v=K_V):
    """Champ de potentiel de cap_potentiel (suivi de ligne) pour la droite passant
    par c, de direction unitaire u :  F = K_N (-2 n n^T (p - c)) + K_V u.
    Seule difference : le sens le long de la ligne est toujours +u (le suivi
    choisit le sens qui ramene vers A). Retourne le cap voulu en radians."""
    n = gauche_de(u)                                # normale unitaire a la ligne
    k = -2.0 * v_dot(n, v_sub(p, c))                # -2 n n^T (p - c) = k n
    F = (k_n * k * n[0] + k_v * u[0], k_n * k * n[1] + k_v * u[1])
    return math.atan2(F[0], F[1]) % (2 * math.pi)  # F = (est, nord)


def regulateur(cap, cap_voulu, v_base, kp=KP):
    """Exactement le regulateur de la boucle du suivi de ligne
    (ddboat_mission.Guidage.regulateur), avec la vitesse en argument."""
    erreur = ecart_angle(cap_voulu - cap)
    v_av = v_base * max(0.0, math.cos(erreur))
    corr = kp * erreur
    L = int(max(-255, min(255, v_av + corr)))
    R = int(max(-255, min(255, v_av - corr)))
    return L, R, erreur


# ---------------------------------------------------------------------------
# Zone convexe, test des determinants (classe Zone de geofencing.py)
# ---------------------------------------------------------------------------
class Zone(object):
    def __init__(self, sommets_xy, noms):
        self.noms, self.P = noms, sommets_xy
        n = len(self.P)
        aire = sum(self.P[i][0] * self.P[(i + 1) % n][1] - self.P[(i + 1) % n][0] * self.P[i][1]
                   for i in range(n)) / 2
        self.signe = 1.0 if aire > 0 else -1.0       # pour que "dedans" <=> distances > 0
        self.aire = abs(aire)
        virages = []
        for i in range(n):
            u = v_sub(self.P[(i + 1) % n], self.P[i])
            v = v_sub(self.P[(i + 2) % n], self.P[(i + 1) % n])
            virages.append(u[0] * v[1] - u[1] * v[0])
        self.convexe = all(v > 0 for v in virages) or all(v < 0 for v in virages)

    def etat(self, p):
        """-> (dedans, marge en m (> 0 dedans), nom du cote le plus proche / franchi)."""
        n = len(self.P)
        d = []
        for i in range(n):
            a, b = self.P[i], self.P[(i + 1) % n]
            u = v_sub(b, a)
            det = u[0] * (p[1] - a[1]) - u[1] * (p[0] - a[0])
            d.append(self.signe * det / v_norm(u))
        i = min(range(n), key=lambda k: d[k])
        return d[i] > 0, d[i], "%s-%s" % (self.noms[i], self.noms[(i + 1) % n])


class Geofence(object):
    """(ddboat_mission.Geofence) Arme apres 'confirm' fix dedans ; sortie = 'confirm'
    fix d'affilee dehors ; une fois armee, plus de 'tolerance' m dehors = sortie."""
    def __init__(self, zone, confirm, tolerance):
        self.zone, self.confirm, self.tolerance = zone, confirm, tolerance
        self.arme, self.actif, self.deja_arme = False, True, False
        self.n_dedans = self.n_dehors = self.n_loin = 0
        self.dedans, self.marge, self.cote = None, float("nan"), ""

    def maj(self, p):
        """A appeler sur chaque fix neuf. Retourne True quand la zone vient de s'armer."""
        self.dedans, self.marge, self.cote = self.zone.etat(p)
        if self.dedans:
            self.n_dedans, self.n_dehors = self.n_dedans + 1, 0
        else:
            self.n_dedans, self.n_dehors = 0, self.n_dehors + 1
        self.n_loin = self.n_loin + 1 if self.marge < -self.tolerance else 0
        if self.actif and not self.arme and self.n_dedans >= self.confirm:
            self.arme = self.deja_arme = True
            return True
        return False

    def sortie(self):
        if not self.actif:
            return False
        if self.arme and self.n_dehors >= self.confirm:
            return True
        return self.deja_arme and self.n_loin >= self.confirm

    def texte(self):
        if self.dedans is None:
            return "zone ?"
        etat = "coupe" if not self.actif else ("arme" if self.arme else "non arme")
        return "%s %.1f m (%s) %s" % ("dedans" if self.dedans else "DEHORS", abs(self.marge), self.cote, etat)


# ---------------------------------------------------------------------------
# Drivers du bateau (seuls imports exterieurs)
# ---------------------------------------------------------------------------
gpsdrv = imudrv = arddrv = None


def charger_drivers(dossier):
    """Trouve les drivers (dossier donne, dossier du script, dossier courant,
    emplacements de l'equipe 4) et les importe. imu9_driver_v2 importe
    'drivers.i2creal' : si le paquet drivers/ n'existe pas, on le fabrique a
    partir de i2creal.py."""
    global gpsdrv, imudrv, arddrv
    ici = os.path.dirname(os.path.abspath(__file__))
    candidats = ([dossier] if dossier else []) + [
        ici, os.getcwd(), os.path.join(ici, "drivers"), os.path.join(ici, "src", "DriversDDBOAT_2026"),
        os.path.expanduser("~/GKKDossier/drivers-ddboat-v2")]
    for d in candidats:
        if os.path.isfile(os.path.join(d, "imu9_driver_v2.py")):
            if d not in sys.path:
                sys.path.insert(0, d)
            if ici not in sys.path:
                sys.path.insert(1, ici)
            dire("Drivers : %s" % d)
            break
    else:
        sys.exit("Drivers introuvables (imu9_driver_v2.py) : mettre les drivers dans le dossier du script "
                 "ou donner --drivers DOSSIER")
    try:
        __import__("drivers.i2creal")
    except ImportError:
        try:
            import i2creal
            paquet = types.ModuleType("drivers")
            paquet.__path__ = []
            paquet.i2creal = i2creal
            sys.modules["drivers"] = paquet
            sys.modules["drivers.i2creal"] = i2creal
        except ImportError:
            pass                                    # imu9_driver_v2 dira lui-meme ce qui manque
    import gps_driver_v2
    import imu9_driver_v2
    import arduino_driver_v2
    gpsdrv, imudrv, arddrv = gps_driver_v2, imu9_driver_v2, arduino_driver_v2


# ---------------------------------------------------------------------------
# Capteurs : cap (fonction du suivi) + GPS (trames RMC) -- ddboat_mission.Capteurs
# ---------------------------------------------------------------------------
class Capteurs(object):
    def __init__(self, invA, xbar, lat0, lon0, perte_gps, garder_filtre):
        self.invA, self.xbar = invA, xbar
        self.lat0, self.lon0 = lat0, lon0
        self.perte_gps = perte_gps
        self.imu = imudrv.Imu9IO()
        self.gps = gpsdrv.GpsIO()
        if not garder_filtre:
            try:
                self.gps.set_filter_speed("0")      # sinon la position reste figee a l'arret
                dire("GPS : filtre de vitesse du driver coupe (set_filter_speed('0'))")
            except Exception as ex:
                dire("GPS : set_filter_speed indisponible (%s)" % ex)
        self.cap = 0.0
        self.p = None
        self.lat = self.lon = None
        self.t_fix = -1e9
        self.fix_neuf = False
        self.rmc = ""
        self.n_fix = 0

    def lire(self, t):
        self.cap = cap_bateau(self.imu, self.invA, self.xbar)
        self.fix_neuf = False
        lat_r = lon_r = None
        for _ in range(5):
            ok, v = self.gps.read_rmc_non_blocking()
            if ok and v[0] != 0.0:
                lat_r, lon_r = nmea_vers_rad(v[0], v[1]), nmea_vers_rad(v[2], v[3])
                self.rmc = " ".join(str(c) for c in v)
                self.fix_neuf = True
        if self.fix_neuf:
            self.lat, self.lon = math.degrees(lat_r), math.degrees(lon_r)
            self.p = vers_metres(lat_r, lon_r, self.lat0, self.lon0)
            self.t_fix = t
            self.n_fix += 1

    def gps_ok(self, t):
        return self.p is not None and (t - self.t_fix) < self.perte_gps

    def dist(self, xy):
        return v_dist(self.p, xy) if self.p is not None else float("nan")


class Moteurs(object):
    """(ddboat_mission.Moteurs) UNE commande par tour de boucle."""
    def __init__(self, a_sec):
        self.a_sec = a_sec
        self.ard = arddrv.ArduinoIO()
        if getattr(self.ard, "arduino", 1) is None and not a_sec:
            sys.exit("Arduino non initialise (driver) : moteurs impossibles a commander")

    def commander(self, L, R):
        L, R = int(max(-255, min(255, L))), int(max(-255, min(255, R)))
        if not self.a_sec:
            self.ard.send_arduino_cmd_motor(L, R)

    def couper(self):
        for _ in range(3):
            try:
                self.ard.send_arduino_cmd_motor(0, 0)
            except Exception:
                pass
            time.sleep(0.05)


# ---------------------------------------------------------------------------
# Serveur commun (UDP, protocole d'Antoine) + positions des autres bateaux
# (ddboat_mission.ServeurUDP + mission_abba.ServeurBateaux)
# ---------------------------------------------------------------------------
class ServeurBateaux(object):
    """Envoi 'id|lat|lon|cap' (degres decimaux, cap en degres 0 = Nord horaire) ;
    le serveur renvoie 'ALL|id,lat,lon,cap,age|...' a l'adresse d'ou viennent
    les paquets. Socket NON BLOQUANTE : jamais d'attente, jamais d'exception."""
    def __init__(self, hote, port, ident, periode, autres):
        self.hote, self.port, self.ident, self.periode = hote, port, ident, periode
        self.autres = autres                  # {id: AutreBateau}
        self.sock = None
        self.t_env = -1e9
        self.n_ok = self.n_err = 0
        self.en_erreur = False
        self.t_reponse = None
        self.t_premiere = None
        self.ids = []
        self.notre_id_vu = False
        self.averti_silence = self.averti_id = False
        self._ouvrir()

    def _ouvrir(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)
        except Exception as ex:
            self.sock = None
            dire("UDP : impossible de creer la socket (%s), on reessaiera" % ex)

    def envoyer(self, t, lat, lon, cap_deg):
        if t - self.t_env < self.periode - 0.05:
            return
        self.t_env = t
        if self.sock is None:
            self._ouvrir()
            if self.sock is None:
                return
        msg = "%s|%.8f|%.8f|%.1f" % (self.ident, lat, lon, cap_deg % 360)
        try:
            self.sock.sendto(msg.encode("utf-8"), (self.hote, self.port))
            self.n_ok += 1
            if self.en_erreur:
                dire("UDP : envoi vers %s:%d repris" % (self.hote, self.port))
                self.en_erreur = False
        except Exception as ex:
            self.n_err += 1
            if not self.en_erreur:
                dire("UDP : envoi impossible vers %s:%d (%s) -> on continue d'essayer chaque seconde"
                     % (self.hote, self.port, ex))
                self.en_erreur = True

    def ecouter(self, t):
        if self.sock is None:
            return
        for _ in range(20):
            try:
                data, _ = self.sock.recvfrom(8192)
            except Exception:
                break                              # rien a lire : on passe
            try:
                parts = data.decode("utf-8", "replace").strip().split("|")
                if parts[0] != "ALL":
                    continue
                ids = []
                for q in parts[1:]:
                    champs = q.split(",")
                    if len(champs) != 5:
                        continue
                    ids.append(champs[0].strip())
                    a = self.autres.get(champs[0].strip())
                    if a is not None:
                        try:
                            a.recevoir(t, float(champs[1]), float(champs[2]), float(champs[3]), float(champs[4]))
                        except ValueError:
                            pass
                self.ids = ids
                self.t_reponse = t
                if self.t_premiere is None:
                    self.t_premiere = t
                    dire("UDP : 1re reponse du serveur, %d bateau(x) : %s" % (len(ids), " ".join(ids)))
                if str(self.ident) in ids and not self.notre_id_vu:
                    self.notre_id_vu = True
                    dire("UDP : le serveur voit bien notre bateau (id %s)" % self.ident)
            except Exception:
                pass
        if self.t_reponse is None and self.n_ok >= 10 and not self.averti_silence:
            self.averti_silence = True
            dire("UDP : ATTENTION aucune reponse du serveur %s:%d apres %d envois (serveur eteint ? mauvaise IP ?)"
                 % (self.hote, self.port, self.n_ok))
        if (self.t_premiere is not None and not self.notre_id_vu and t - self.t_premiere > 10
                and not self.averti_id):
            self.averti_id = True
            dire("UDP : ATTENTION le serveur ne renvoie pas notre id '%s' (ids : %s)"
                 % (self.ident, " ".join(self.ids) or "aucun"))

    def texte(self, t):
        if self.en_erreur:
            return "UDP ERREUR"
        if self.t_reponse is None:
            return "UDP envoi (pas de reponse)"
        return "UDP ok (rep. %.0f s)" % (t - self.t_reponse)


class AutreBateau(object):
    """(mission_abba.AutreBateau) Un autre bateau vu a travers le serveur."""
    def __init__(self, ident, local):
        self.ident, self.local = str(ident), local
        self.hist = []                    # [(t_pos, xy)] des derniers fix (8 max)
        self.lat = self.lon = self.cap = None
        self.xy = None
        self.t_pos = None                 # instant (horloge du bateau) de sa derniere position
        self.n_fix = 0
        self.fix_neuf = False

    def recevoir(self, t, lat, lon, cap, age):
        t_pos = t - max(0.0, age)
        xy = self.local((lat, lon))
        neuf = (self.t_pos is None or t_pos > self.t_pos + 0.5 or v_dist(xy, self.xy) > 0.01)
        self.lat, self.lon, self.cap = lat, lon, cap
        if neuf:
            self.xy, self.n_fix, self.fix_neuf = xy, self.n_fix + 1, True
            self.t_pos = t_pos if self.t_pos is None else max(t_pos, self.t_pos + 0.01)
            self.hist.append((self.t_pos, xy))
            self.hist = self.hist[-8:]
        elif t_pos > self.t_pos:
            self.t_pos = t_pos                    # meme position, age plus precis

    def prendre_fix(self):
        f, self.fix_neuf = self.fix_neuf, False
        return f

    def age(self, t):
        return float("inf") if self.t_pos is None else t - self.t_pos

    def vitesse(self):
        """Vitesse (m/s) estimee sur les fix des ~4 dernieres secondes (au moins 2,5 s
        d'ecart : sur 1 s, le bruit GPS donne de fausses vitesses, surtout apres une coupure)."""
        if len(self.hist) < 2:
            return (0.0, 0.0)
        t1, x1 = self.hist[-1]
        for t0, x0 in self.hist[:-1]:
            if 2.5 <= t1 - t0 <= 4.5:
                v = v_mul(v_sub(x1, x0), 1.0 / (t1 - t0))
                n = v_norm(v)
                return v_mul(v, V_MAX_AUTRE / n) if n > V_MAX_AUTRE else v
        return (0.0, 0.0)

    def predite(self, t):
        if self.xy is None:
            return None
        return v_add(self.xy, v_mul(self.vitesse(), min(HORIZON, max(0.0, self.age(t)))))


# ---------------------------------------------------------------------------
# PLAN : route centrale + trajets des deux bateaux (identiques sur les 2 bateaux)
# ---------------------------------------------------------------------------
class Plan(object):
    """Route centrale echantillonnee tous les 'pas' m d'abscisse curviligne s.
    origine A = (0, 0) ; G = point de la bouee ; sens = +1 virage a droite, -1 a gauche."""
    def __init__(self, G, ecart, rayon, au_dela, l_trans, sens, v_base, v_max, pas=0.25):
        self.G, self.ecart, self.rayon, self.sens, self.pas = G, ecart, rayon, sens, pas
        self.d_AG = v_norm(G)
        f = unitaire(G)
        self.f = f
        cote_decal = gauche_de(f) if sens > 0 else droite_de(f)   # decalage oppose au virage
        decal = 2.0 * rayon
        l1 = self.d_AG + au_dela
        if l_trans > l1 - 1.0:
            raise ValueError("--l-trans (%.0f m) doit etre plus court que A->debut du virage (%.0f m)"
                             % (l_trans, l1))
        self.l1, self.l_trans, self.decal = l1, l_trans, decal
        # 1) courbe dense (parametre quelconque)
        dense = []
        x = 0.0
        while x < l1:
            u = min(1.0, x / l_trans)
            k = decal * u * u * (3.0 - 2.0 * u)                    # raccord en S (smoothstep)
            dense.append(v_add(v_mul(f, x), v_mul(cote_decal, k)))
            x += 0.02
        centre = v_add(v_mul(f, l1), v_mul(cote_decal, decal - rayon))
        n_arc = int(math.pi * rayon / 0.02) + 1
        for i in range(n_arc + 1):
            phi = math.pi * i / n_arc
            dense.append(v_add(centre, v_mul(v_add(v_mul(cote_decal, math.cos(phi)), v_mul(f, math.sin(phi))), rayon)))
        y = 0.02
        while y <= l1 + 1e-9:
            dense.append(v_mul(f, l1 - y))
            y += 0.02
        if v_dist(dense[-1], (0.0, 0.0)) > 1e-6:
            dense.append((0.0, 0.0))
        # 2) reechantillonnage a pas constant d'abscisse curviligne
        cum = [0.0]
        for a, b in zip(dense, dense[1:]):
            cum.append(cum[-1] + v_dist(a, b))
        self.s_total = cum[-1]
        self.c = []
        j = 0
        s = 0.0
        while s <= self.s_total + 1e-9:
            while j < len(cum) - 2 and cum[j + 1] < s:
                j += 1
            l = cum[j + 1] - cum[j]
            w = (s - cum[j]) / l if l > 1e-12 else 0.0
            self.c.append(v_add(dense[j], v_mul(v_sub(dense[j + 1], dense[j]), borne(w, 0.0, 1.0))))
            s += pas
        n = len(self.c)
        self.t = []
        for i in range(n):
            a, b = self.c[max(0, i - 1)], self.c[min(n - 1, i + 1)]
            self.t.append(unitaire(v_sub(b, a)))
        self.n = [droite_de(u) for u in self.t]                    # normale a DROITE de la route
        # abscisses des phases
        self.s_virage = self._abscisse_de(v_add(v_mul(f, l1), v_mul(cote_decal, decal)), 0.0, self.s_total / 2 + 5)
        self.s_retour = self.s_virage + math.pi * rayon
        self.s_G = self.s_retour + (l1 - self.d_AG)
        # vitesse de la route centrale : reduite au virage si l'exterieur depasserait v_max
        r_ext = (rayon + ecart / 2.0) / rayon
        self.v_virage = min(float(v_base), v_max / r_ext)
        self.v_base = float(v_base)

    def _abscisse_de(self, p, s0, s1):
        i0, i1 = int(s0 / self.pas), min(len(self.c) - 1, int(s1 / self.pas) + 1)
        i = min(range(i0, i1 + 1), key=lambda k: v_dist(self.c[k], p))
        return i * self.pas

    def _i(self, s):
        return int(borne(round(s / self.pas), 0, len(self.c) - 1))

    def centre(self, s):
        return self.c[self._i(s)]

    def tangente(self, s):
        return self.t[self._i(s)]

    def normale(self, s):
        return self.n[self._i(s)]

    def point_bateau(self, s, cote, dec=0.0):
        """Point du trajet d'un bateau : cote = +1 (droite) ou -1 (gauche) ;
        dec = decalage lateral supplementaire (recentrage sur la bouee), vers la droite."""
        i = self._i(s)
        return v_add(self.c[i], v_mul(self.n[i], cote * self.ecart / 2.0 + dec))

    def rapport(self, s, cote):
        """Longueur de mon trajet / longueur de la route centrale autour de s."""
        a, b = self.point_bateau(s - 0.5, cote), self.point_bateau(s + 0.5, cote)
        da = v_dist(self.centre(s - 0.5), self.centre(s + 0.5))
        return v_dist(a, b) / da if da > 1e-6 else 1.0

    def v_centre(self, s):
        return self.v_virage if self.s_virage <= s <= self.s_retour else self.v_base

    def phase(self, s):
        if s < self.s_virage:
            return "ALLER"
        if s < self.s_retour:
            return "VIRAGE"
        return "RETOUR"

    def projeter(self, p, s_prec, arriere=3.0, avant=15.0):
        """Abscisse s du point de la route centrale le plus proche de p, cherche dans
        [s_prec - arriere, s_prec + avant] (la route repasse par A : pas de recherche globale)."""
        i0 = max(0, int((s_prec - arriere) / self.pas))
        i1 = min(len(self.c) - 1, int((s_prec + avant) / self.pas) + 1)
        if i1 <= i0:
            i0 = max(0, i1 - 1)
        best_i, best_d = i0, float("inf")
        for i in range(i0, i1 + 1):
            c = self.c[i]
            d = (c[0] - p[0]) ** 2 + (c[1] - p[1]) ** 2
            if d < best_d:
                best_i, best_d = i, d
        s = best_i * self.pas
        u = self.t[best_i]
        s += borne(v_dot(v_sub(p, self.c[best_i]), u), -self.pas, self.pas)   # affinage sur la tangente
        return borne(s, 0.0, self.s_total)


class Recentrage(object):
    """Decalage lateral de la ligne de RETOUR vers la position reelle de la bouee
    (bateau 9 en station keeping), moyenne des 5 dernieres positions, borne a
    +/- maxi, fige quand la formation arrive a fige_avant m de G."""
    def __init__(self, plan, maxi, fige_avant, rampe, actif):
        self.plan, self.maxi, self.fige_avant, self.rampe, self.actif = plan, maxi, fige_avant, rampe, actif
        self.n_ret = plan.normale(plan.s_retour + 1.0)
        self.hist = []
        self.dec = 0.0
        self.fige = False
        self.brut = float("nan")

    def bouee(self, xy):
        if self.fige or not self.actif:
            return
        self.brut = v_dot(v_sub(xy, self.plan.G), self.n_ret)
        self.hist = (self.hist + [self.brut])[-5:]
        self.dec = borne(sum(self.hist) / len(self.hist), -self.maxi, self.maxi)

    def maj(self, s_centre):
        """Retourne True quand on vient de figer."""
        if not self.fige and s_centre >= self.plan.s_G - self.fige_avant:
            self.fige = True
            return True
        return False

    def decalage(self, s):
        if s < self.plan.s_retour or not self.actif:
            return 0.0
        return self.dec * min(1.0, (s - self.plan.s_retour) / self.rampe)


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
def lire_calib(chemin):
    with open(chemin) as f:
        nombres = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", f.read())]
    if len(nombres) != 12:
        sys.exit("Calibration %s : il faut 12 nombres (9 de invA puis 3 de xbar), j'en lis %d"
                 % (chemin, len(nombres)))
    return [nombres[0:3], nombres[3:6], nombres[6:9]], nombres[9:12]


# ---------------------------------------------------------------------------
# Programme principal
# ---------------------------------------------------------------------------
def lire_latlon(texte):
    try:
        lat, lon = (float(v) for v in texte.replace(";", ",").replace(" ", ",").split(",") if v)
        return lat, lon
    except ValueError:
        sys.exit("Point illisible : %r (attendu lat,lon en degres decimaux)" % texte)


def main():
    global JOURNAL, T_DEBUT
    p_ = argparse.ArgumentParser(description="Capture de bouee a deux bateaux relies par un cable.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    g = p_.add_argument_group("bateaux")
    g.add_argument("--id", required=True, help="NOTRE id sur le serveur (7 ou 4)")
    g.add_argument("--id-autre", default=None, help="id de l'autre bateau tracteur (defaut : 4 pour 7, 7 pour 4)")
    g.add_argument("--id-bouee", default=ID_BOUEE, help="id du bateau-bouee (defaut %s)" % ID_BOUEE)
    g.add_argument("--cote", choices=["droite", "gauche"], default=None,
                   help="notre cote de la route (defaut : 7 droite, 4 gauche)")
    g.add_argument("--calib-fichier", default=None, help="calibration boussole : 9 nombres de invA puis 3 de xbar")
    g = p_.add_argument_group("trajectoire (IDENTIQUE sur les deux bateaux)")
    g.add_argument("--a", default=None, help="point de depart/arrivee lat,lon (defaut A %.6f,%.6f)" % A_DEG)
    g.add_argument("--g", default=None, help="bouee lat,lon (defaut G %.6f,%.6f)" % G_DEG)
    g.add_argument("--ecart", type=float, default=ECART, help="m entre les bateaux (defaut %.0f)" % ECART)
    g.add_argument("--rayon", type=float, default=RAYON, help="rayon du demi-tour de la route centrale (defaut %.0f m)" % RAYON)
    g.add_argument("--au-dela", type=float, default=AU_DELA, help="debut du demi-tour, m au-dela de G (defaut %.0f)" % AU_DELA)
    g.add_argument("--l-trans", type=float, default=L_TRANS, help="longueur du raccord en S de l'aller (defaut %.0f m)" % L_TRANS)
    g.add_argument("--virage", choices=["droite", "gauche"], default="droite", help="sens du demi-tour (defaut droite)")
    g = p_.add_argument_group("commande")
    g.add_argument("--v-base", type=float, default=V_BASE, help="vitesse d'avance en ligne droite (defaut %d)" % V_BASE)
    g.add_argument("--v-max", type=float, default=V_MAX, help="commande d'avance maximale (defaut %d)" % V_MAX)
    g.add_argument("--kp", type=float, default=KP, help="gain de cap (defaut %.0f, comme le suivi)" % KP)
    g.add_argument("--k-n", type=float, default=K_N, help="poids de la ligne attractive (defaut %.1f, comme le suivi)" % K_N)
    g.add_argument("--tau-ds", type=float, default=TAU_DS, help="filtre de l'ecart d'abscisse, s (defaut %.0f)" % TAU_DS)
    g.add_argument("--k-s", type=float, default=K_S, help="synchro : commande par m d'ecart avec l'autre (defaut %.0f)" % K_S)
    g.add_argument("--avance", type=float, default=AVANCE, help="champ de ligne evalue N m devant (defaut %.0f)" % AVANCE)
    g.add_argument("--sans-recentrage", action="store_true", help="ne recentre pas la ligne de retour sur la bouee reelle")
    g.add_argument("--recentrage-max", type=float, default=RECENTRAGE_MAX)
    g = p_.add_argument_group("depart")
    g.add_argument("--r-pret", type=float, default=R_PRET)
    g.add_argument("--n-pret", type=int, default=N_PRET)
    g.add_argument("--v-pret", type=float, default=V_PRET)
    g.add_argument("--depart-immediat", action="store_true", help="part sans attendre l'autre (test seul)")
    g.add_argument("--seul", action="store_true",
                   help="essai SANS partenaire ni cable : ignore l'autre (pas de synchro), part tout de suite")
    g.add_argument("--sans-depart", action="store_true", help="reste en ATTENTE (ponton : verifier la regle)")
    g = p_.add_argument_group("securites")
    g.add_argument("--a-sec", action="store_true", help="ne commande JAMAIS les moteurs")
    g.add_argument("--ecart-max", type=float, default=ECART_MAX)
    g.add_argument("--ecart-min", type=float, default=ECART_MIN)
    g.add_argument("--perte-autre-max", type=float, default=PERTE_AUTRE_MAX)
    g.add_argument("--perte-gps", type=float, default=3.0)
    g.add_argument("--timeout-progres", type=float, default=45.0)
    g.add_argument("--duree-max", type=float, default=600.0)
    g.add_argument("--confirm", type=int, default=3)
    g.add_argument("--tolerance-zone", type=float, default=5.0)
    g.add_argument("--hors-zone-ok", action="store_true", help="accepte un plan qui sort de la zone (test)")
    g.add_argument("--garder-filtre-gps", action="store_true")
    g.add_argument("--periode", type=float, default=0.1)
    g.add_argument("--verifier", action="store_true", help="affiche le plan et les controles puis s'arrete")
    g = p_.add_argument_group("transmission")
    g.add_argument("--serveur", default=SERVEUR, help="IP du serveur UDP (defaut %s, Antoine)" % SERVEUR)
    g.add_argument("--port-serveur", type=int, default=PORT_SERVEUR)
    g.add_argument("--drivers", default=None, help="dossier des drivers (defaut : dossier du script)")
    args = p_.parse_args()

    ident = str(args.id).strip()
    id_autre = args.id_autre or AUTRE_DEFAUT.get(ident)
    if id_autre is None:
        sys.exit("--id-autre obligatoire pour l'id %s" % ident)
    cote_txt = args.cote or COTE_DEFAUT.get(ident)
    if cote_txt is None:
        sys.exit("--cote droite|gauche obligatoire pour l'id %s" % ident)
    cote = 1 if cote_txt == "droite" else -1

    # ---------- journal ----------
    horodatage = time.strftime("%Y%m%d_%H%M%S")
    base = "capture_b%s_%s" % (ident, horodatage)
    nom_journal = base + "_console.txt"
    JOURNAL = open(nom_journal, "w")
    T_DEBUT = time.time()
    dire("Commande : python3 %s" % " ".join(sys.argv))
    dire("Journal du terminal : %s" % os.path.abspath(nom_journal))

    # ---------- calibration ----------
    if args.calib_fichier:
        invA, xbar = lire_calib(args.calib_fichier)
        src_calib = args.calib_fichier
    elif CALIB.get(ident):
        invA, xbar = CALIB[ident]
        src_calib = "CALIB du script (bateau %s)" % ident
    else:
        sys.exit("Pas de calibration boussole pour le bateau %s : remplir CALIB[\"%s\"] dans le script ou "
                 "donner --calib-fichier (9 nombres de invA puis 3 de xbar, sortie de 10__magcalib.py)"
                 % (ident, ident))
    dire("Calibration boussole : %s ; xbar = %s" % (src_calib, " ".join("%.2f" % v for v in xbar)))

    # ---------- repere, plan ----------
    a_ll = lire_latlon(args.a) if args.a else A_DEG
    g_ll = lire_latlon(args.g) if args.g else G_DEG
    lat0, lon0 = math.radians(a_ll[0]), math.radians(a_ll[1])

    def local(latlon):
        return vers_metres(math.radians(latlon[0]), math.radians(latlon[1]), lat0, lon0)

    G_xy = local(g_ll)
    try:
        plan = Plan(G_xy, args.ecart, args.rayon, args.au_dela, args.l_trans,
                    1 if args.virage == "droite" else -1, args.v_base, args.v_max)
    except ValueError as ex:
        sys.exit("Plan impossible : %s" % ex)
    zone = Zone([local((la, lo)) for _, la, lo in ZONE], [s[0] for s in ZONE])
    dep_moi, dep_autre = plan.point_bateau(0.0, cote), plan.point_bateau(0.0, -cote)
    fin_moi = plan.point_bateau(plan.s_total, cote)

    dire("PLAN : ecart %.1f m, rayon %.1f m, au-dela %.1f m, raccord %.1f m, virage a %s -> route %.1f m "
         "(A->G %.1f m, cap aller %.0f deg)" % (
             args.ecart, args.rayon, args.au_dela, args.l_trans, args.virage, plan.s_total, plan.d_AG,
             math.degrees(math.atan2(plan.f[0], plan.f[1])) % 360))
    dire("     -> comparer cette ligne PLAN sur les deux bateaux : elle doit etre IDENTIQUE")
    dire("     ALLER 0-%.1f m | VIRAGE %.1f-%.1f m | RETOUR %.1f-%.1f m, bouee a s=%.1f m" % (
        plan.s_virage, plan.s_virage, plan.s_retour, plan.s_retour, plan.s_total, plan.s_G))
    dire("Nous : bateau %s, cote %s ; autre tracteur : bateau %s ; bouee : bateau %s" % (
        ident, cote_txt.upper(), id_autre, args.id_bouee))
    for nom, p in (("notre depart", dep_moi), ("depart du bateau %s" % id_autre, dep_autre),
                   ("notre arrivee", fin_moi)):
        la = math.degrees(lat0 + p[1] / R_TERRE)
        lo = math.degrees(lon0 + p[0] / (R_TERRE * math.cos(lat0)))
        dire("  %-22s (%6.1f,%6.1f) m = %.7f,%.7f" % (nom, p[0], p[1], la, lo))
    # controles du plan
    erreurs = []
    marge_min = (float("inf"), None, None)
    d_bouee_aller = float("inf")
    r_ext = (args.rayon + args.ecart / 2.0) / args.rayon
    r_int = (args.rayon - args.ecart / 2.0) / args.rayon
    s = 0.0
    while s <= plan.s_total:
        for c in (1, -1):
            if FIN_SANS_ZONE <= s <= plan.s_total - FIN_SANS_ZONE:
                m = zone.etat(plan.point_bateau(s, c))[1]
                if m < marge_min[0]:
                    marge_min = (m, s, c)
        if s <= plan.s_retour:
            a, b = plan.point_bateau(s, 1), plan.point_bateau(s, -1)
            ab = v_sub(b, a)
            w = borne(v_dot(v_sub(G_xy, a), ab) / max(v_dot(ab, ab), 1e-9), 0.0, 1.0)
            d_bouee_aller = min(d_bouee_aller, v_dist(G_xy, v_add(a, v_mul(ab, w))))
        s += 0.5
    dire("Controles : bord de zone a %.1f m au plus pres (s=%.0f m, bateau %s) ; segment entre les bateaux a "
         "%.1f m de la bouee au plus pres a l'ALLER et au VIRAGE" % (
             marge_min[0], marge_min[1] or 0, "droite" if marge_min[2] == 1 else "gauche", d_bouee_aller))
    dire("           rayons du demi-tour : interieur %.1f m, exterieur %.1f m ; vitesse de la route au virage %.0f "
         "(exterieur %.0f, interieur %.0f)" % (args.rayon - args.ecart / 2, args.rayon + args.ecart / 2,
                                                plan.v_virage, plan.v_virage * r_ext, plan.v_virage * r_int))
    if marge_min[0] < MARGE_PLAN and not args.hors_zone_ok:
        erreurs.append("le plan passe a %.1f m du bord de la zone (< %.0f m)" % (marge_min[0], MARGE_PLAN))
    if d_bouee_aller < 8.0:
        erreurs.append("a l'aller, le cable passe a %.1f m de la bouee (< 8 m)" % d_bouee_aller)
    if args.rayon - args.ecart / 2.0 < 3.0:
        erreurs.append("rayon interieur du demi-tour trop petit (%.1f m < 3 m)" % (args.rayon - args.ecart / 2.0))
    if args.ecart >= 20.0:
        erreurs.append("ecart >= longueur du cable (20 m)")
    if erreurs:
        for e in erreurs:
            dire("ERREUR : " + e)
        sys.exit("Plan refuse (%d erreur(s))" % len(erreurs))
    dire("Plan verifie : OK")

    # fichier du plan (pour les traces)
    nom_plan = base + "_plan.csv"
    with open(nom_plan, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["s", "phase", "centre_x", "centre_y", "droite_x", "droite_y", "gauche_x", "gauche_y",
                    "a_lat", "a_lon", "g_x", "g_y"])
        s = 0.0
        while s <= plan.s_total + 1e-9:
            c, d, gg = plan.centre(s), plan.point_bateau(s, 1), plan.point_bateau(s, -1)
            w.writerow(["%.2f" % s, plan.phase(s), "%.2f" % c[0], "%.2f" % c[1], "%.2f" % d[0], "%.2f" % d[1],
                        "%.2f" % gg[0], "%.2f" % gg[1], "%.7f" % a_ll[0], "%.7f" % a_ll[1],
                        "%.2f" % G_xy[0], "%.2f" % G_xy[1]])
            s += 0.5
    dire("Plan ecrit dans %s" % os.path.abspath(nom_plan))
    if args.verifier:
        return

    a_sec = args.a_sec
    if a_sec:
        dire("*** MODE A SEC : les moteurs ne seront JAMAIS commandes (securite 'progres' coupee) ***")
        args.timeout_progres = float("inf")
    if args.seul:
        dire("*** MODE SEUL : pas de partenaire, pas de synchro, depart des que nous sommes sur notre point. "
             "NE PAS utiliser avec le cable ! ***")

    # ---------- materiel ----------
    charger_drivers(args.drivers)
    C = Capteurs(invA, xbar, lat0, lon0, args.perte_gps, args.garder_filtre_gps)
    M = Moteurs(a_sec)
    # le driver Arduino installe son propre Ctrl+C (sys.exit) : on remet le notre
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _sur_signal)
    G = Geofence(zone, args.confirm, args.tolerance_zone)
    autre = AutreBateau(id_autre, local)
    bouee = AutreBateau(args.id_bouee, local)
    udp = ServeurBateaux(args.serveur, args.port_serveur, ident, PERIODE_SERVEUR,
                         {str(id_autre): autre, str(args.id_bouee): bouee})
    reca = Recentrage(plan, args.recentrage_max, FIGE_AVANT, RAMPE, not args.sans_recentrage)
    dire("Serveur commun : UDP %s:%d, notre id %s ; on lit les bateaux %s (tracteur) et %s (bouee)"
         % (args.serveur, args.port_serveur, ident, id_autre, args.id_bouee))
    dire("Depart quand le bateau %s est vu %d fix d'affilee a moins de %.0f m de son point, a moins de %.1f m/s "
         "(et nous a moins de %.0f m du notre)" % (id_autre, args.n_pret, args.r_pret, args.v_pret, args.r_pret))

    nom_log = base + ".csv"
    f_log = open(nom_log, "w", newline="")
    log = csv.writer(f_log)
    log.writerow(["t", "t_mission", "etat", "phase", "lat", "lon", "x", "y", "fix_neuf", "cap_deg", "voulu_deg",
                  "err_deg", "L", "R", "v_cmd", "s_moi", "s_autre", "ds", "ds_filtre", "e_lat", "ecart",
                  "autre_x", "autre_y", "autre_age", "autre_pred_x", "autre_pred_y", "autre_fix_neuf",
                  "bouee_x", "bouee_y", "bouee_age", "dec_bouee", "dec_fige", "d_bouee",
                  "n_pret", "moi_pret", "pause", "attend", "trop_pres", "zone_dedans", "zone_marge", "geof_arme",
                  "geof_actif", "udp_envois", "udp_age_reponse", "rmc"])
    dire("Log CSV : %s" % os.path.abspath(nom_log))

    # ---------- etat ----------
    st = {"etat": "ATTENTE_FIX"}
    t0 = time.time()
    s_moi, s_autre = 0.0, 0.0
    regle = {"n": 0, "vu_pret": False}
    prog = {"best": -1e9, "t": t0}
    trop_pres = [False]
    n_pres = [0]                                      # mesures fraiches d'affilee sous ecart_min
    ecartes = [False]
    bil = {"t_depart": None, "t_fin": None, "fin": "", "ds_max": 0.0, "ecart_min": float("inf"),
           "ecart_max": 0.0, "elat": {"ALLER": 0.0, "VIRAGE": 0.0, "RETOUR": 0.0}, "pauses": 0,
           "attentes": 0, "d_bouee_aller": float("inf"), "passage": None, "t_phase": {}}
    pause = {"actif": False, "t": 0.0}
    ds_f = [None]                                     # s_autre - s_moi filtre
    attend_prec = [False]

    def changer(nouvel, raison):
        dire("\n>>> %s -> %s : %s  (t=%.1f s)\n" % (st["etat"], nouvel, raison, time.time() - t0))
        st["etat"] = nouvel

    bilan_fait = [False]

    def bilan(raison):
        if bilan_fait[0]:
            return
        bilan_fait[0] = True
        dire("\n===== BILAN CAPTURE (bateau %s, %.0f s) : %s =====" % (ident, time.time() - t0, raison))
        if bil["t_depart"] is None:
            dire("  pas de depart")
        else:
            dire("  depart a t=%.1f s ; %s" % (bil["t_depart"] - t0, " | ".join(
                "%s a t=%.1f s" % (k, v - t0) for k, v in sorted(bil["t_phase"].items(), key=lambda x: x[1]))))
            dire("  progression : s = %.1f / %.1f m" % (s_moi, plan.s_total))
            dire("  ecart lateral max a notre trajet : ALLER %.1f m, VIRAGE %.1f m, RETOUR %.1f m" % (
                bil["elat"]["ALLER"], bil["elat"]["VIRAGE"], bil["elat"]["RETOUR"]))
            if not args.seul:
                dire("  synchro : |s_autre - s_moi| max %.1f m ; ecart entre bateaux min %.1f m, max %.1f m"
                     % (bil["ds_max"], bil["ecart_min"], bil["ecart_max"]))
                dire("  pauses (bateau %s perdu) : %d ; attentes (ecart > %.0f m) : %d"
                     % (id_autre, bil["pauses"], args.ecart_max, bil["attentes"]))
            if bil["d_bouee_aller"] < float("inf"):
                dire("  bouee (bateau %s) : a %.1f m de nous au plus pres a l'aller/virage" % (
                    args.id_bouee, bil["d_bouee_aller"]))
            if bil["passage"]:
                dire("  PASSAGE DE LA BOUEE : %s" % bil["passage"])
            dire("  recentrage de la ligne de retour : %+.1f m (%s)" % (
                reca.dec, "fige" if reca.fige else "non fige"))
        dire("  serveur : %d envois, %d erreurs, %s ; bateau %s : %d fix recus ; bouee %s : %d fix recus"
             % (udp.n_ok, udp.n_err, "notre id vu" if udp.notre_id_vu else "notre id PAS vu",
                id_autre, autre.n_fix, args.id_bouee, bouee.n_fix))

    def fin(raison):
        if st["etat"] == "FIN":
            return
        bil["t_fin"] = time.time()
        changer("FIN", raison + " -> moteurs coupes (Ctrl+C pour quitter)")
        bilan(raison)

    t_affiche = 0.0
    t_prec = time.time()
    n_boucle = 0
    try:
        M.commander(0, 0)
        while True:
            t = time.time()
            dt, t_prec = t - t_prec, t
            C.lire(t)
            gps_ok = C.gps_ok(t)
            if C.fix_neuf and G.maj(C.p):
                dire(">>> Geofencing ARME : bateau dans la zone (%.1f m du bord)" % G.marge)

            # ----- serveur : envoyer notre position, lire l'autre et la bouee -----
            cap_deg = math.degrees(C.cap) % 360
            if C.lat is not None:
                udp.envoyer(t, C.lat, C.lon, cap_deg)
            udp.ecouter(t)
            autre_neuf = autre.prendre_fix()
            if bouee.prendre_fix() and st["etat"] in ("ATTENTE", "FORMATION"):
                reca.bouee(bouee.xy)
            age = autre.age(t)
            age_b = bouee.age(t)
            q = autre.predite(t) if age < AGE_PERDU else autre.xy
            if q is not None:
                s_autre = plan.projeter(q, s_autre, 5.0, 5.0 + V_MAX_AUTRE * min(60.0, age if age < 1e9 else 0.0))
            if C.p is not None and (C.fix_neuf or st["etat"] != "FORMATION"):
                s_moi = plan.projeter(C.p, s_moi)
            ecart = v_dist(C.p, q) if (C.p is not None and q is not None) else float("nan")
            d_bouee = v_dist(C.p, bouee.xy) if (C.p is not None and bouee.xy is not None) else float("nan")

            etat = st["etat"]
            phase = plan.phase(s_moi) if etat == "FORMATION" else ""
            L = R = 0
            v_cmd = 0.0
            voulu = err = float("nan")
            e_lat = float("nan")
            attend = False

            # ----- regle de depart (sur chaque fix neuf de l'autre) -----
            if autre_neuf and etat in ("ATTENTE_FIX", "ATTENTE"):
                da = v_dist(autre.xy, dep_autre)
                v_a = v_norm(autre.vitesse())
                if da < args.r_pret and age < AGE_PRET:
                    regle["vu_pret"] = True
                    if v_a < args.v_pret:
                        regle["s0"] = s_autre           # derniere position "a l'arret sur son point"
                if da < args.r_pret and age < AGE_PRET and v_a < args.v_pret:
                    regle["n"] += 1
                else:
                    regle["n"] = 0
            moi_pret = gps_ok and C.dist(dep_moi) < args.r_pret and C.dist(dep_moi) < C.dist(dep_autre)
            inverse = gps_ok and C.dist(dep_autre) < C.dist(dep_moi) and C.dist(dep_autre) < args.r_pret
            # deja parti : vu sur son point, puis a AVANCE de S_PARTI m sur la route depuis, et bouge encore
            parti = (regle["vu_pret"] and age < AGE_PRET and s_autre - regle.get("s0", 0.0) > S_PARTI
                     and v_norm(autre.vitesse()) > 0.3)

            # ----- automate -----
            if etat == "ATTENTE_FIX":
                if gps_ok:
                    dire("Premier fix GPS : (%.1f,%.1f) m, a %.1f m de notre depart ; %s" % (
                        C.p[0], C.p[1], C.dist(dep_moi), G.texte()))
                    changer("ATTENTE", "attente du bateau %s sur son point%s" % (
                        id_autre, " (--sans-depart : on ne partira pas)" if args.sans_depart else ""))
                    etat = st["etat"]

            if etat == "ATTENTE":
                if not args.sans_depart and moi_pret:
                    raison = None
                    if args.seul:
                        raison = "--seul"
                    elif args.depart_immediat:
                        raison = "--depart-immediat"
                    elif regle["n"] >= args.n_pret:
                        raison = "bateau %s vu %d fix d'affilee sur son point (%.1f m), immobile" % (
                            id_autre, regle["n"], v_dist(autre.xy, dep_autre))
                    elif parti:
                        raison = "bateau %s DEJA PARTI (s=%.1f m) : on part et on le rattrape" % (id_autre, s_autre)
                    if raison:
                        bil["t_depart"] = t
                        prog["best"], prog["t"] = s_moi, t
                        changer("FORMATION", "DEPART : " + raison)
                        etat = st["etat"]
                        phase = plan.phase(s_moi)

            if etat == "FORMATION":
                if phase not in bil["t_phase"]:
                    bil["t_phase"][phase] = t
                    if phase != "ALLER":
                        dire(">>> phase %s (s=%.1f m, ecart %.1f m)" % (phase, s_moi, ecart))
                if reca.maj(s_moi):
                    dire(">>> recentrage sur la bouee FIGE a %+.1f m (bouee vue a %.1f m de G%s)" % (
                        reca.dec, v_dist(bouee.xy, G_xy) if bouee.xy is not None else float("nan"),
                        "" if age_b < 5 else ", position ancienne de %.0f s" % age_b))
                if gps_ok:
                    s_ref = min(plan.s_total, s_moi + args.avance)
                    dec = reca.decalage(s_ref)
                    c_ref = plan.point_bateau(s_ref, cote, dec)
                    voulu = champ_ligne(C.p, c_ref, plan.tangente(s_ref), args.k_n)
                    e_lat = v_dot(v_sub(C.p, plan.point_bateau(s_moi, cote, reca.decalage(s_moi))),
                                  plan.normale(s_moi))
                    if args.seul or autre.xy is None:
                        ds = 0.0
                    else:
                        if ds_f[0] is None:
                            ds_f[0] = s_autre - s_moi
                        ds_f[0] += (s_autre - s_moi - ds_f[0]) * min(1.0, dt / max(args.tau_ds, 1e-3))
                        ds = ds_f[0]
                    v_cmd = borne(plan.v_centre(s_moi) * plan.rapport(s_moi, cote) + args.k_s * ds,
                                  V_MIN, args.v_max)
                    L, R, err = regulateur(C.cap, voulu, v_cmd, args.kp)
                    if phase in bil["elat"]:
                        bil["elat"][phase] = max(bil["elat"][phase], abs(e_lat))
                # --- securites liees a l'autre bateau ---
                if not args.seul:
                    if age >= AGE_PERDU:
                        if not pause["actif"]:
                            pause["actif"], pause["t"] = True, t
                            bil["pauses"] += 1
                            dire(">>> PAUSE : position du bateau %s perdue depuis %.0f s -> moteurs coupes"
                                 % (id_autre, age))
                        L = R = 0
                        if age > args.perte_autre_max:
                            fin("ALERTE : bateau %s muet depuis %.0f s" % (id_autre, age))
                    elif pause["actif"]:
                        pause["actif"] = False
                        dire(">>> bateau %s de nouveau visible (age %.1f s) : fin de la PAUSE" % (id_autre, age))
                    if not math.isnan(ecart):
                        if ecart > args.ecart_max and s_moi > s_autre:
                            attend = True
                            L = R = 0
                        if ecart > args.ecart_min + 3.0:
                            ecartes[0] = True
                        if C.fix_neuf or autre_neuf:              # une mesure fraiche de plus
                            n_pres[0] = n_pres[0] + 1 if (ecart < args.ecart_min and age < 2.5) else 0
                        if ecartes[0] and n_pres[0] >= 2 and not trop_pres[0]:
                            trop_pres[0] = True
                            dire(">>> TROP PRES du bateau %s (%.1f m) : moteurs coupes" % (id_autre, ecart))
                        elif trop_pres[0] and ecart > args.ecart_min + 2.0:
                            trop_pres[0] = False
                            dire(">>> bateau %s de nouveau a %.1f m : on repart" % (id_autre, ecart))
                        if trop_pres[0]:
                            L = R = 0
                        if age < 2.5:
                            bil["ecart_min"] = min(bil["ecart_min"], ecart)
                            bil["ecart_max"] = max(bil["ecart_max"], ecart)
                            bil["ds_max"] = max(bil["ds_max"], abs(s_autre - s_moi))
                if attend and not attend_prec[0]:
                    bil["attentes"] += 1
                    dire(">>> ecart %.1f m > %.0f m et nous sommes devant : on attend le bateau %s"
                         % (ecart, args.ecart_max, id_autre))
                attend_prec[0] = attend
                # --- bouee : distance a l'aller, passage au retour ---
                if not math.isnan(d_bouee) and age_b < 5 and s_moi < plan.s_retour:
                    bil["d_bouee_aller"] = min(bil["d_bouee_aller"], d_bouee)
                if (bil["passage"] is None and s_moi >= plan.s_G and C.p is not None and q is not None):
                    bxy = bouee.xy if (bouee.xy is not None and age_b < 5) else G_xy
                    ab = v_sub(q, C.p)
                    w = v_dot(v_sub(bxy, C.p), ab) / max(v_dot(ab, ab), 1e-9)
                    bil["passage"] = ("bouee (%s) %s les bateaux (position %.2f sur le segment nous->%s), "
                                      "ecart %.1f m, s_autre - s_moi = %+.1f m" % (
                                          "serveur" if bxy is not G_xy else "point G",
                                          "ENTRE" if 0.0 < w < 1.0 else "PAS entre", w, id_autre, ecart,
                                          s_autre - s_moi))
                    dire(">>> passage de la bouee : " + bil["passage"])
                # --- geofencing (coupe sur la fin : A est hors zone) ---
                if s_moi > plan.s_total - FIN_SANS_ZONE and G.actif:
                    G.actif = False
                    dire(">>> geofencing coupe pour l'arrivee sur A (hors zone)")
                if G.sortie():
                    L = R = 0
                    fin("ALERTE : SORTIE DE ZONE par le cote %s (%.1f m dehors)" % (G.cote, -G.marge))
                # --- progres ---
                elif gps_ok and not pause["actif"] and not attend and not trop_pres[0]:
                    if s_moi > prog["best"] + 1.0:
                        prog["best"], prog["t"] = s_moi, t
                    elif t - prog["t"] > args.timeout_progres:
                        L = R = 0
                        fin("ALERTE : pas de progres depuis %.0f s (s=%.1f m)" % (args.timeout_progres, s_moi))
                else:
                    prog["t"] = t                               # ce temps-la ne compte pas
                # --- arrivee ---
                if st["etat"] == "FORMATION" and C.fix_neuf and s_moi >= plan.s_total - R_FIN:
                    L = R = 0
                    fin("ARRIVEE : bout de la route atteint (s=%.1f m, %.1f m de notre point d'arrivee)" % (
                        s_moi, C.dist(fin_moi)))

            if st["etat"] in ("FIN", "ATTENTE", "ATTENTE_FIX"):
                L = R = 0
            elif t - t0 > args.duree_max:
                L = R = 0
                fin("ALERTE : duree max atteinte (%.0f s)" % args.duree_max)
            if not gps_ok:
                L = R = 0                                     # jamais de moteurs sans position fiable
            M.commander(L, R)

            # ----- log CSV -----
            f2 = lambda v: "" if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))) \
                else "%.2f" % v
            log.writerow([
                "%.3f" % t, "%.2f" % (t - t0), st["etat"], phase,
                "" if C.lat is None else "%.7f" % C.lat, "" if C.lon is None else "%.7f" % C.lon,
                "" if C.p is None else "%.2f" % C.p[0], "" if C.p is None else "%.2f" % C.p[1],
                int(C.fix_neuf), "%.1f" % cap_deg,
                "" if math.isnan(voulu) else "%.1f" % (math.degrees(voulu) % 360),
                "" if math.isnan(err) else "%.1f" % math.degrees(err), L, R, "%.0f" % v_cmd,
                "%.2f" % s_moi, "%.2f" % s_autre if autre.xy is not None else "",
                "%.2f" % (s_autre - s_moi) if autre.xy is not None else "", f2(ds_f[0]), f2(e_lat), f2(ecart),
                "" if autre.xy is None else "%.2f" % autre.xy[0], "" if autre.xy is None else "%.2f" % autre.xy[1],
                f2(age), "" if q is None else "%.2f" % q[0], "" if q is None else "%.2f" % q[1], int(autre_neuf),
                "" if bouee.xy is None else "%.2f" % bouee.xy[0], "" if bouee.xy is None else "%.2f" % bouee.xy[1],
                f2(age_b), "%.2f" % reca.dec, int(reca.fige), f2(d_bouee),
                regle["n"], int(moi_pret), int(pause["actif"]), int(attend), int(trop_pres[0]),
                "" if G.dedans is None else int(G.dedans), f2(G.marge), int(G.arme), int(G.actif),
                udp.n_ok, "%.1f" % (t - udp.t_reponse) if udp.t_reponse else "",
                C.rmc if C.fix_neuf else ""])
            n_boucle += 1
            if n_boucle % 10 == 0:
                f_log.flush()
                os.fsync(f_log.fileno())

            # ----- console (1 fois / s) -----
            if t - t_affiche >= 1.0:
                t_affiche = t
                txt = "[%-9s%-7s] " % (st["etat"], (" " + phase) if phase else "")
                txt += ("pos=(%.1f,%.1f) " % (C.p[0], C.p[1])) if C.p is not None else "Attente du fix GPS... "
                if st["etat"] == "ATTENTE":
                    txt += "| moi %s (%.1f m de mon point) | b%s %s %d/%d fix%s " % (
                        "PRET" if moi_pret else "pas pret", C.dist(dep_moi), id_autre,
                        "jamais vu" if autre.xy is None else "a %.1f m de son point" % v_dist(autre.xy, dep_autre),
                        regle["n"], args.n_pret, " (PARTI)" if parti else "")
                    if inverse:
                        txt += "| ATTENTION : nous sommes sur le point de depart du bateau %s (cotes inverses ?) " % id_autre
                elif st["etat"] in ("FORMATION", "FIN"):
                    txt += "s=%.1f/%.0f " % (s_moi, plan.s_total)
                    if not args.seul and autre.xy is not None:
                        txt += "s_%s=%.1f (ds %+.1f) ecart %.1f m " % (id_autre, s_autre, s_autre - s_moi, ecart)
                    if not math.isnan(e_lat):
                        txt += "lat %+.1f m " % e_lat
                txt += "cap=%.0f " % cap_deg
                if not math.isnan(voulu):
                    txt += "voulu=%.0f v=%.0f " % (math.degrees(voulu) % 360, v_cmd)
                txt += "L=%d R=%d" % (L, R)
                txt += " | b%s %s" % (id_autre, "jamais vu" if autre.xy is None else "age %.1f s" % age)
                txt += " | bouee %s" % ("jamais vue" if bouee.xy is None else "%.1f m de G, age %.0f s%s" % (
                    v_dist(bouee.xy, G_xy), age_b, ", dec %+.1f%s" % (reca.dec, " FIGE" if reca.fige else "")
                    if st["etat"] == "FORMATION" else ""))
                if pause["actif"]:
                    txt += " | PAUSE"
                if attend:
                    txt += " | ATTEND b%s" % id_autre
                if trop_pres[0]:
                    txt += " | TROP PRES"
                if C.p is not None and not gps_ok:
                    txt += " | GPS PERDU"
                txt += " | " + G.texte() + " | " + udp.texte(t)
                dire(txt)

            time.sleep(args.periode)

    except KeyboardInterrupt:
        dire("\nArret demande (Ctrl+C)")
    except ArretSignal as ex:
        dire("\nSignal %s recu (Ctrl+C / kill / terminal ferme) : arret" % ex.args[0])
    finally:
        M.couper()                                   # TOUJOURS couper les moteurs
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, signal.SIG_DFL)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        try:
            bilan("arret du programme (etat %s)" % st["etat"])
        except Exception:
            pass
        f_log.close()
        dire("Moteurs coupes. Logs : %s, %s et %s" % (nom_log, nom_journal, nom_plan))


if __name__ == "__main__":
    try:
        main()
    except SystemExit as e:
        if isinstance(e.code, str):
            dire("ARRET : " + e.code)
        raise
    except Exception:
        dire(traceback.format_exc().rstrip())
        raise
    finally:
        if JOURNAL is not None:
            JOURNAL.close()
