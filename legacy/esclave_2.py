import time, math
import numpy as np
from arduino_driver_v2 import ArduinoIO
from gps_driver_v2 import GpsIO
from imu9_driver_v2 import Imu9IO

# Import du module de communication (vérifiez le nom exact de votre fichier)
from ddboats_client import send_position, get_boats

# ======================================================================
# 1) PARAMETRES DE MISSION ET RESEAU
# ======================================================================
ESSAI_A_SEC = False
R_TERRE = 6371000.0

SERVER_IP = "172.20.26.27"  # IP du serveur central
SERVER_PORT = 6800           
MY_ID = 4                    # Identifiant de CE bateau (Suiveur)
LEADER_ID = 7                # Identifiant du bateau à suivre (Leader)

DISTANCE_D = 5.0             # Distance d'espacement (mètres)

ORIGINE_DEG = (48.198864, -3.015074)
ORIGINE = (math.radians(ORIGINE_DEG[0]), math.radians(ORIGINE_DEG[1]))

# Paramètres du régulateur (à ajuster sur le lac)
V_MAX = 100        # Vitesse maximale autorisée pour rattraper
KP_DIST = 20.0     # Poussée par mètre de retard
KP_CAP = 57.0      # Force de rotation par radian d'erreur (57 ~ 1/deg)

# ======================================================================
# 2) CALIBRATION IMU (Reprendre les mêmes que d'habitude)
# ======================================================================
invA = np.array([[-2.87564918e-04, -5.72701381e-05,  2.54545662e-05],
                 [-3.51695171e-05,  2.92630051e-04,  5.44283319e-06],
                 [-1.88775100e-05,  1.11759540e-05, -3.01125112e-04]])
xbar = np.array([[-814.37049587], [-241.77837075], [ 276.86354638]])

# ======================================================================
# 3) INITIALISATION MATERIEL
# ======================================================================
gps, imu = GpsIO(), Imu9IO()
ard = None if ESSAI_A_SEC else ArduinoIO()
lat2 = lon2 = None

# ======================================================================
# 4) FONCTIONS MATHEMATIQUES ET CAPTEURS
# ======================================================================
def nmea_vers_rad(v, h):
    deg = int(v / 100) + (v % 100) / 60.0
    deg = -deg if h in ("S", "W") else deg
    return math.radians(deg)

def vers_metres(lat, lon, lat0, lon0):
    est = (lon - lon0) * R_TERRE * math.cos(lat0)
    nord = (lat - lat0) * R_TERRE
    return np.array([est, nord])

def ecart_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def cap_vers(p, cible):
    d = cible - p
    return math.atan2(d[0], d[1]) % (2 * np.pi)

def cap_bateau(imu, invA, xbar):
    x = np.array(imu.read_mag_raw(), dtype=float).reshape(3, 1)
    y = np.real(invA @ (x - xbar))
    return math.atan2(y[1, 0], y[0, 0]) % (2 * np.pi)

def lire_gps(lat_actuel, lon_actuel):
    for _ in range(5):
        ok, data = gps.read_rmc_non_blocking()
        if ok and data[0] != 0.0:
            return nmea_vers_rad(data[0], data[1]), nmea_vers_rad(data[2], data[3])
    return lat_actuel, lon_actuel

def moteurs(l, r):
    if ard is not None:
        ard.send_arduino_cmd_motor(l, r)

def calculer_cible_virtuelle(p1, cap1_rad, distance_d):
    """Décale le point de destination perpendiculairement au Leader"""
    angle = cap1_rad - (math.pi / 2)
    est = p1[0] + distance_d * math.sin(angle)
    nord = p1[1] + distance_d * math.cos(angle)
    return np.array([est, nord])

# ======================================================================
# 5) ATTENTE DU FIX GPS (Obligatoire pour commencer)
# ======================================================================
while lat2 is None:
    lat2, lon2 = lire_gps(lat2, lon2)
    if lat2 is None:
        print("Bateau 2 : Attente du fix GPS...")
        time.sleep(0.5)

print("Fix GPS obtenu ! Lancement du mode Suiveur.")

# ======================================================================
# 6) BOUCLE PRINCIPALE (Le cœur du Tracking)
# ======================================================================
try:
    k = 0
    signal_perdu = False
    while True:
        k += 1
        # 1. Mise à jour de sa propre position et cap (Bateau 2)
        lat2, lon2 = lire_gps(lat2, lon2)
        cap2 = cap_bateau(imu, invA, xbar)
        
        # Le bateau 2 DOIT envoyer sa position pour maintenir la connexion 
        # UDP active et recevoir en retour la liste des autres bateaux.
        if k % 10 == 0:
            k = 0
            send_position(SERVER_IP, SERVER_PORT, MY_ID, math.degrees(lat2), math.degrees(lon2), math.degrees(cap2))
        
        # Position locale Bateau 2
        p2 = vers_metres(lat2, lon2, ORIGINE[0], ORIGINE[1])

        # 2. Récupération des données du réseau
        bateaux_reseau = get_boats()
        
        if LEADER_ID in bateaux_reseau:
            donnees_leader = bateaux_reseau[LEADER_ID]
            
            # Sécurité : vérifier que le Leader émet toujours (données < 3 secondes)
            if donnees_leader["age"] < 5.0:
                # Récupération et conversion des données du Leader
                lat1 = math.radians(donnees_leader["lat"])
                lon1 = math.radians(donnees_leader["lon"])
                cap1 = math.radians(donnees_leader["cap"])
                
                # Position locale Leader
                p1 = vers_metres(lat1, lon1, ORIGINE[0], ORIGINE[1])
                
                # 3. Calcul de la cible virtuelle (Le "Fantôme")
                cible = calculer_cible_virtuelle(p1, cap1, DISTANCE_D)
                
                # 4. Calculs pour le pilotage (Distance et Cap Voulu)
                dist = np.linalg.norm(cible - p2)
                cap_voulu = cap_vers(p2, cible)
                erreur = ecart_angle(cap_voulu - cap2)
                
                # 5. Régulation proportionnelle des moteurs
                # La vitesse linéaire est proportionnelle à la distance. S'il est 
                # à 0m du fantôme, v_av vaudra 0 (il s'arrête net à côté du leader).
                v_av = min(V_MAX, KP_DIST * dist) * max(0.0, math.cos(erreur))
                corr = KP_CAP * erreur
                
                cmdl = int(max(0, min(V_MAX, v_av + corr)))
                cmdr = int(max(0, min(V_MAX, v_av - corr)))
                
                moteurs(cmdl, cmdr)
                
                # Affichage console pour débugger
                # print("Dist fantôme: {dist:.1f}m | Erreur Cap: {math.degrees(erreur):.0f}° | Mot: {cmdl}/{cmdr}")
                
            else:
                if k % 10 == 0:
                    print("Signal du Leader trop ancien. Arrêt par sécurité.")
                moteurs(0, 0)
        else:
            if k % 10 == 0:
                print("Leader introuvable sur le réseau. Arrêt par sécurité.")
            moteurs(0, 0)
            
        time.sleep(0.1) # Boucle à 10 Hz

finally:
    moteurs(0, 0)
    print("Fin de mission. Moteurs coupés.")