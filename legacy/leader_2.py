import time, math
import numpy as np
from scipy.signal import sawtooth
from arduino_driver_v2 import ArduinoIO
from gps_driver_v2 import GpsIO
from imu9_driver_v2 import Imu9IO

from ddboats_client import send_gpgll, send_position, get_boats


# ======================================================================
# 2) PARAMETRES
# ======================================================================
ESSAI_A_SEC = False          # True : aucune commande envoyee aux moteurs
DUREE_MAX = 20000            # securite (s)
R_TERRE = 6371000.0          # rayon de la Terre (m)
V_BASE = 80                  # vitesse d'avance (0-255)
KP = 57.0                    # commande par RADIAN d'erreur (57 ~ 1 par degre)
NB_TOURS = 1                  # nombre de tours à effectuer sur le parcours

SERVER_IP = "172.20.26.20"
SERVER_PORT = 6800
BATEAU_ID = 7

# ======================================================================
# 3) WAYPOINTS en degres (à rentrer)
# ======================================================================
A_DEG = (48.198823, -3.015067)   # home (lat, lon) en degres decimaux (saisie), origine du repère utilisé pour le calcul des positions relatives
B_DEG = (48.198839, -3.015685)   
C_DEG = (48.198874, -3.015795)   
D_DEG = (48.198942, -3.015862)   
E_DEG = (48.199022, -3.015901)       
F_DEG = (48.199101, -3.015843)  
G_DEG = (48.199150, -3.015718) 
H_DEG = (48.199157, -3.015605)
I_DEG = (48.199122, -3.015497)
J_DEG = (48.198926, -3.015046)

WAYPOINTS_DEG = [A_DEG, B_DEG, C_DEG, D_DEG, E_DEG, F_DEG, G_DEG, H_DEG, I_DEG, J_DEG]  # Liste des waypoints
NAME_WAYPOINTS = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J"]  # Noms des waypoints correspondants

# Conversion des waypoints en radians
WAYPOINTS = [tuple(math.radians(x) for x in w) for w in WAYPOINTS_DEG]

ORIGINE = WAYPOINTS[0]              # Définit l'origine comme le premier waypoint (point A)


# ======================================================================
# 4) CALIBRATION DU MAGNETOMETRE
# ======================================================================
#valeurs obtenues avec 10__magcalib.py#
invA = np.array([[-2.87564918e-04, -5.72701381e-05,  2.54545662e-05],
 [-3.51695171e-05,  2.92630051e-04,  5.44283319e-06],
 [-1.88775100e-05,  1.11759540e-05, -3.01125112e-04]])
xbar = np.array([[-814.37049587],
 [-241.77837075],
 [ 276.86354638]])


# ======================================================================
# 5) INITIALISATION DU MATERIEL ET DES VARIABLES D'ETAT
# ======================================================================
gps, imu = GpsIO(), Imu9IO()
ard = None if ESSAI_A_SEC else ArduinoIO() #permet de faire un premier essai sans envoyer de commande aux moteurs

lat = lon = None
t_debut = time.time()
arret = False


# ======================================================================
# 6) FONCTIONS : conversions et angles
# ======================================================================
def nmea_vers_rad(v, h):
    """Format NMEA ddmm.mmmm -> radians (Sud et Ouest negatifs)."""
    deg = int(v / 100) + (v % 100) / 60.0
    deg = -deg if h in ("S", "W") else deg
    return math.radians(deg)

def vers_metres(lat, lon, lat0, lon0):
    """lat, lon, lat0, lon0 en radians -> (est, nord) en metres par rapport au point 0."""
    est = (lon - lon0) * R_TERRE * math.cos(lat0)
    nord = (lat - lat0) * R_TERRE
    return np.array([est, nord])

def ecart_angle(a):
    """Dent de scie : ramene un angle (rad) dans [-pi, pi]."""
    return np.pi * sawtooth(a + np.pi)

def cap_vers(p, cible):
    """Cap compas (rad) du bateau p vers la cible."""
    d = cible - p
    return math.atan2(d[0], d[1]) % (2 * np.pi)


# ======================================================================
# 7) FONCTIONS : capteurs et moteurs
# ======================================================================
def cap_bateau(imu, invA, xbar):
    """Cap compas en radians, dans [0, 2pi[ (0 = Nord, pi/2 = Est)."""
    x = np.array(imu.read_mag_raw(), dtype=float).reshape(3, 1)
    y = np.real(invA @ (x - xbar))
    return math.atan2(y[1, 0], y[0, 0]) % (2 * np.pi)

def lire_gps(lat, lon):
    """Lit 5 messages GPS, garde la derniere position valide."""
    for _ in range(5):
        ok, data = gps.read_rmc_non_blocking()
        if ok and data[0] != 0.0:
            lat, lon = nmea_vers_rad(data[0], data[1]), nmea_vers_rad(data[2], data[3])
    return lat, lon

def moteurs(l, r):
    if ard is not None:
        ard.send_arduino_cmd_motor(l, r)


# ======================================================================
# 8) FONCTION : phase 0 (rejoindre un point)
# ======================================================================
def aller_vers(cible, lat, lon,RAYON_ORIGINE=5.0):
    """Pilote le bateau vers cible (en m, repère ORIGINE) jusqu'à être à RAYON_ORIGINE.
    Retourne (ok, lat, lon) ; ok=False si DUREE_MAX est dépassée."""
    k = 0
    while True:
        k += 1
        
        if time.time() - t_debut > DUREE_MAX:
            print("Duree max atteinte, arret.")
            return False, lat, lon

        lat, lon = lire_gps(lat, lon)
        p = vers_metres(lat, lon, ORIGINE[0], ORIGINE[1])
        dist = np.linalg.norm(cible - p)
        if dist < RAYON_ORIGINE:
            print("Début du parcours atteint")
            return True, lat, lon

        cap_voulu = cap_vers(p, cible)
        cap = cap_bateau(imu, invA, xbar)
        erreur = ecart_angle(cap_voulu - cap)

        if k % 10 == 0:
            k = 0
            send_position(SERVER_IP, SERVER_PORT, BATEAU_ID, lat=math.degrees(lat), lon=math.degrees(lon), cap=math.degrees(cap))

        v_av = V_BASE * max(0.0, math.cos(erreur))
        corr = KP * erreur
        cmdl = int(max(0, min(255, v_av + corr)))
        cmdr = int(max(0, min(255, v_av - corr)))
        moteurs(cmdl, cmdr)

        print("RETOUR ORIGINE dist=%.1f cap=%.0f voulu=%.0f L=%d R=%d"
              % (dist, math.degrees(cap), math.degrees(cap_voulu), cmdl, cmdr))
        time.sleep(0.1)


# ======================================================================
# 9) ATTENTE DU FIX GPS
# ======================================================================
while lat is None:                  # acquisition GPS initiale 
    lat, lon = lire_gps(lat, lon)
    if lat is None:
        print("Attente du fix GPS...")
        time.sleep(0.5)


# ======================================================================
# 10) BOUCLE PRINCIPALE
# ======================================================================
try:
    # ---------- Phase 0 : aller a l'origine A ----------
    print("Phase 0 : deplacement vers l'origine A")
    ok, lat, lon = aller_vers(np.array([0.0, 0.0]), lat, lon)
    arret = not ok
    if not arret:
        print("Origine atteinte, debut du parcours")
    
    # ---------- Parcours des waypoints ----------
    for tour in range(NB_TOURS):
        if arret:
            break
        for i in range(len(WAYPOINTS)-1):
            # --- definition de la ligne courante (source -> cible) ---
            point_source = WAYPOINTS[i]
            point_cible = WAYPOINTS[i+1]
            print("Navigation vers le waypoint {} depuis {}".format(NAME_WAYPOINTS[(i + 1) % len(WAYPOINTS)], NAME_WAYPOINTS[i]))

            a = vers_metres(point_source[0], point_source[1], ORIGINE[0], ORIGINE[1])
            b = vers_metres(point_cible[0], point_cible[1], ORIGINE[0], ORIGINE[1])

            u = b - a

            # --- position initiale pour le test de franchissement ---
            lat, lon = lire_gps(lat, lon)
            p = vers_metres(lat, lon, ORIGINE[0], ORIGINE[1])
            v = p - b

            k = 0
            # --- suivi jusqu'au franchissement du waypoint ---
            while (u@v) <= 0:  # tant que le bateau n'a pas dépassé le point cible
                k += 1
                
                if time.time() - t_debut > DUREE_MAX:
                    print("Duree max atteinte, arret.")
                    arret = True
                    break

                # 1) position GPS
                lat, lon = lire_gps(lat, lon)
                p = vers_metres(lat, lon, ORIGINE[0], ORIGINE[1])
                v = p - b  # vecteur du point cible vers le bateau

                # 2) cap voulu vers le waypoint, cap reel, erreur
                cap_voulu = cap_vers(p, b)
                cap = cap_bateau(imu, invA, xbar)
                erreur = ecart_angle(cap_voulu - cap)
                
                lat_deg = math.degrees(lat)
                lon_deg = math.degrees(lon)
                cap_deg = math.degrees(cap)
                
                if k % 10 == 0:
                    k = 0
                    send_position(SERVER_IP, SERVER_PORT, BATEAU_ID, lat=lat_deg, lon=lon_deg, cap=cap_deg)

                # 3) commandes moteurs (pas de marche arriere)
                v_av = V_BASE * max(0.0, math.cos(erreur))
                corr = KP * erreur
                cmdl = int(max(0, min(255, v_av + corr)))
                cmdr = int(max(0, min(255, v_av - corr)))
                moteurs(cmdl, cmdr)

                print("p=(%.1f,%.1f) cap=%.0f voulu=%.0f err=%.0f L=%d R=%d"
                      % (p[0], p[1], math.degrees(cap), math.degrees(cap_voulu),
                         math.degrees(erreur), cmdl, cmdr))
                for id_, b in get_boats().items():
                       print(id_, b["lat"], b["lon"], b["cap"], b["age"])
                time.sleep(0.1)

            # --- sortie du while : arret demande ou waypoint franchi ---
            if arret:
                break
            print("Waypoint {} franchi".format(NAME_WAYPOINTS[(i + 1) % len(WAYPOINTS)]))
        if arret:
            break

# ======================================================================
# 11) SECURITE : les moteurs sont toujours coupés a la sortie
# ======================================================================
finally:
    moteurs(0, 0)
    print("Fin du programme, moteurs arretés.")