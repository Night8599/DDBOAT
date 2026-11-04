import time, math
import numpy as np
from arduino_driver_v2 import ArduinoIO
from gps_driver_v2 import GpsIO
from imu9_driver_v2 import Imu9IO
from ddboat_client import send_position, send_gpgll, get_boats


# ======================================================================
# 2) PARAMETRES
# ======================================================================
ESSAI_A_SEC = False          # True : aucune commande envoyee aux moteurs
DUREE_MAX = 2000            # securite (s)
R_TERRE = 6371000.0          # rayon de la Terre (m)
V_BASE = 140                  # vitesse d'avance (0-255)
KP = 57.0                   # commande par RADIAN d'erreur (57 ~ 1 par degre)
V_CENTRE = 0.15          # m/s, a regler (voir plus bas)          


SERVER_IP = "172.20.25.XX"     # à demander
SERVER_PORT = 6800
MON_ID = 7                     # à demander
ID_AUTRE = 6
t_envoi = 0.0
interaction_serveur = False


#Pour le test avec le point maitre#
OMEGA_MAITRE = 0.1                 
PHASE0_MAITRE = 0.0 

                   


# ======================================================================
# 3) WAYPOINTS en degres (à rentrer)
# ======================================================================
ORIGINE_DEG = (48.198863,-3.015070)   #origine du repère utilisé pour le calcul des positions relatives
H_DEG = (48.198944,-3.015478)   #point H (définit seulement le départ du DDB)
F_DEG = (48.199027,-3.015887)   #point F (définit seulement la direction du cercle)


ORIGINE = tuple(math.radians(x) for x in ORIGINE_DEG)              # Définit l'origine comme le premier waypoint (point A)
H = tuple(math.radians(x) for x in H_DEG)              # Définit
F = tuple(math.radians(x) for x in F_DEG)              # Cercle ou les bateaux se rejoignent
G=tuple(math.radians(x) for x in (48.199072,-3.016258))              # Départ du point de simulation



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
arret = False
t_0 = time.time()

import os
from ihm_client import ClientIHM

ssh = os.environ.get("SSH_CLIENT", "").split()
SERVEUR_IHM = ssh[0] if ssh else "172.20.25.XX"     # IP du PC qui fait tourner l'IHM
ihm = ClientIHM(SERVEUR_IHM, ident="MonEquipe") 


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

def sawtooth(x):
    return (x+math.pi)%2*math.pi-math.pi

def cap_vers(p, cible):
    """Cap compas (rad) du bateau p vers la cible."""
    d = cible - p
    return math.atan2(d[0], d[1]) % (2 * np.pi)

def metres_vers_deg(pt):
    """(est, nord) en m, repere ORIGINE -> (lat, lon) en degres."""
    lat_d = ORIGINE_DEG[0] + math.degrees(pt[1] / R_TERRE)
    lon_d = ORIGINE_DEG[1] + math.degrees(pt[0] / (R_TERRE * math.cos(ORIGINE[0])))
    return lat_d, lon_d


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
def deplacer_centre(centre, centre_cible, dt):
    ecart = centre_cible - centre
    dist = np.linalg.norm(ecart)
    pas = V_CENTRE * dt
    if dist <= pas:
        return centre_cible.copy()
    return centre + ecart / dist * pas




def aller_vers(cible, lat, lon,RAYON_ORIGINE=5.0):
    """Pilote le bateau vers cible (en m, repère ORIGINE) jusqu'à être à RAYON_ORIGINE.
    Retourne (ok, lat, lon) ; ok=False si DUREE_MAX est dépassée."""
    while True:

        if time.time() - t_0 > DUREE_MAX:
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
        ihm.envoyer(math.degrees(lat), math.degrees(lon), cap=math.degrees(cap),
            mode="HOMING", dist="%.1f m" % dist)
        erreur = ecart_angle(cap_voulu - cap)

        v_av = V_BASE * max(0.0, math.cos(erreur))
        corr = KP * erreur
        cmdl = int(max(0, min(255, v_av + corr)))
        cmdr = int(max(0, min(255, v_av - corr)))
        moteurs(cmdl, cmdr)

        print("RETOUR ORIGINE dist=%.1f cap=%.0f voulu=%.0f L=%d R=%d"
              % (dist, math.degrees(cap), math.degrees(cap_voulu), cmdl, cmdr))
        time.sleep(0.1)


def coeff_opposition_phase(p_suiveur, p_maitre, centre_suiveur, centre_maitre, tolerance=0.15):
    """Renvoie un coefficient de vitesse tangentielle en fonction de la phase du suiveur
    et de celle du maitre, toutes deux mesurees autour du meme centre."""
    r_maitre = p_maitre - centre_maitre
    phase_maitre = math.atan2(r_maitre[0], r_maitre[1]) % (2 * np.pi)
    r_suiveur = p_suiveur - centre_suiveur
    phase_suiveur = math.atan2(r_suiveur[0], r_suiveur[1]) % (2 * np.pi)

    phase_attendue_suiveur = (phase_maitre + np.pi) % (2 * np.pi)
    ecart_phase = ecart_angle(phase_attendue_suiveur - phase_suiveur)

    if abs(ecart_phase) < tolerance:           # zone morte de tolerance rad (~8.6 deg) autour de la position opposée
        return 1.0

    return max(0.5, 1.0 + ecart_phase / math.pi)

def champ_cercle(p, centre, rayon, sens=1, K_R=1, K_T=1.0, coef_t=1.0):
    """Vecteur (est, nord) du champ qui ramene le bateau sur le cercle
    et le fait tourner dessus.
    p, centre : np.array (est, nord) en m, meme repere ORIGINE
    sens : +1 = horaire (Nord -> Est), -1 = antihoraire
    coef_t : coefficient applique a la composante tangentielle"""
    r = p - centre                              # vecteur centre -> bateau
    d = max(np.linalg.norm(r),1e-3)
    e_r = r / d                                 # unitaire, vers l'exterieur
    e_t = sens * np.array([e_r[1], -e_r[0]])    # unitaire tangent
    return K_R * (rayon - d) * e_r + K_T * coef_t * e_t

def cap_cercle(p, centre, rayon, sens=1, coef_t=1.0):
    F = champ_cercle(p, centre, rayon, sens, coef_t=coef_t)
    return math.atan2(F[0], F[1]) % (2 * np.pi)




          
#pour le test avec le point maitre#

def point_maitre(t, centre, rayon, omega, phase0=0.0):
    """Maitre fictif sur un cercle, a l'instant t (s).
    Renvoie p = (est, nord) en m, repere ORIGINE.
    Phase 0 = Nord du centre, positive vers l'Est (sens horaire)."""
    phi = phase0 + omega * t
    return centre + rayon * np.array([math.sin(phi), math.cos(phi)])




#paramètres du cercle#

rayon=7.0 

centre_dd=vers_metres(H[0], H[1], ORIGINE[0], ORIGINE[1])  # centre du cercle (est, nord) en m, repère ORIGINE
centre_cible=vers_metres(F[0], F[1], ORIGINE[0], ORIGINE[1])  # centre du cercle (est, nord) en m, repère ORIGINE
centre_point_fictif=vers_metres(G[0], G[1], ORIGINE[0], ORIGINE[1])  # centre du cercle (est, nord) en m, repère ORIGINE

mode_point_fictif = False
mode_déplacement_centre = True

p_autre = None          # position de l'autre bateau (serveur ou point fictif)
coef_t = 1.0            # coefficient d'opposition de phase
t_maitre = None         # instant de depart du point fictif
cap = 0.0               # initialise pour ne jamais etre inconnu
ihm_point = None       # client IHM du point fictif (cree au passage de mode)
t_ihm_point = 0.0

#### Boucle principale ####

while lat is None: 
    ihm.envoyer_etat(etat="attente fix GPS")
    lat, lon = lire_gps(lat, lon)                 # acquisition GPS initiale , attente du fix    
    if lat is None:
        print("Attente du fix GPS...")
        time.sleep(0.5)


try: 
    # ---------- Phase 0 : aller au home ----------
    print("Phase 0 : deplacement vers le point de départ (home) du parcours")
    ok, lat, lon = aller_vers(vers_metres(H[0], H[1], ORIGINE[0], ORIGINE[1]), lat, lon)
    arret = not ok
    if not arret:
        print("Origine atteinte, debut du parcours")

    else : 
        print("Origine non atteinte, arret du programme")

    t_prec = time.time()
    t_maitre = time.time()
    ihm_point = ClientIHM(SERVEUR_IHM, ident="POINT_FICTIF")
    t_ihm_point = 0.0
    while True :
        
        if arret :
            break

        if time.time() - t_0 > DUREE_MAX:
            print("Duree max atteinte, arret.")
            arret = True
            break

        maintenant = time.time()
        dt = maintenant - t_prec
        t_prec = maintenant

        
        # 1) position GPS et cap
        lat, lon = lire_gps(lat, lon)
        p = vers_metres(lat, lon, ORIGINE[0], ORIGINE[1])
        cap = cap_bateau(imu, invA, xbar)

        
        if interaction_serveur : 

            if time.time() - t_envoi >= 1.0:
                send_position(SERVER_IP, SERVER_PORT, MON_ID,
                math.degrees(lat), math.degrees(lon), math.degrees(cap))
                t_envoi = time.time()

            autre = get_boats().get(ID_AUTRE)
            if autre is not None and autre["age"] < 5.0: #bateau connu et position relevée il y'a moins de 5s
                p_autre = vers_metres(math.radians(autre["lat"]), math.radians(autre["lon"]),
                            ORIGINE[0], ORIGINE[1])      # meme repere que p

        centre_dd = deplacer_centre(centre_dd, centre_cible, dt)
        cap_voulu = cap_cercle(p, centre_dd, rayon, sens=1)

        
        #Actualisation du point fictif#   
        centre_point_fictif = deplacer_centre(centre_point_fictif, centre_cible, dt)
        p_autre = point_maitre(time.time() - t_maitre, centre_point_fictif, rayon, OMEGA_MAITRE, PHASE0_MAITRE)
        
        if time.time() - t_ihm_point >= 1.0:
            lat_m, lon_m = metres_vers_deg(p_autre)
            ihm_point.envoyer(lat_m, lon_m, mode="POINT FICTIF")
            t_ihm_point = time.time()
        
        #actualisation du cap voulu du bateau#
        sur_cercle = abs(np.linalg.norm(p - centre_dd) - rayon) < 0.5 * rayon
        centres_proches = np.linalg.norm(centre_dd - centre_point_fictif) < 0.5 * rayon
        if sur_cercle and centres_proches:
            coef_t = coeff_opposition_phase(p, p_autre, centre_dd, centre_point_fictif)
        else:
            coef_t = 1.0
        cap_voulu = cap_cercle(p, centre_dd, rayon, sens=1, coef_t=coef_t)
            

        
        # 2) cap voulu vers le centre, cap reel, erreur
        ihm.envoyer(math.degrees(lat), math.degrees(lon), cap=math.degrees(cap),
            mode="CERCLE",
            voulu="%.0f" % math.degrees(cap_voulu),
            dist_centre="%.1f m" % np.linalg.norm(p - centre_dd))
        erreur = ecart_angle(cap_voulu - cap)
        
        # 4) commandes moteurs (pas de marche arriere)
        v_av = V_BASE * max(0.0, math.cos(erreur))
        corr = KP * erreur
        cmdl = int(max(0, min(255, v_av + corr)))
        cmdr = int(max(0, min(255, v_av - corr)))
        moteurs(cmdl, cmdr)
        
        print("p=(%.1f,%.1f) cap=%.0f voulu=%.0f err=%.0f L=%d R=%d"
                              % (p[0], p[1], math.degrees(cap), math.degrees(cap_voulu),
                                 math.degrees(erreur), cmdl, cmdr))
        time.sleep(0.1)



except KeyboardInterrupt:
    print("Arret demandé au clavier")


finally:
    moteurs(0, 0)
    ihm.fermer()
    if ihm_point is not None:
        ihm_point.fermer()