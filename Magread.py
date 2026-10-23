import sys
import os
import time
import csv
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import imu9_driver_v2 as imudrv
print(dir(imudrv))

imu = imudrv.Imu9IO()

print("Enregistrement des donnees magnetiques... (Ctrl+C pour arreter)")

with open("magcloud.csv", "w", newline="") as fichier:
    writer = csv.writer(fichier)
    for i in range(1000):
        xmag, ymag, zmag = imu.read_mag_raw()
        writer.writerow([xmag, ymag, zmag])
        fichier.flush()
        time.sleep(0.1)

print("Adquisition Terminée.")
print("Posez le bateau a plat vers le Nord.")
input("Appuyez sur Entree pour enregistrer...")
mesures_n = np.array([imu.read_mag_raw() for _ in range(20) if not time.sleep(0.1)])
xn, yn, zn = np.mean(mesures_n, axis=0)

print("Tournez le bateau vers l'Ouest.")
input("Appuyez sur Entree pour enregistrer...")
mesures_w = np.array([imu.read_mag_raw() for _ in range(20)])
xw, yw, zw = np.mean(mesures_w, axis=0)

with open("magNorthWest.csv", "w", newline="") as fichier2:
    writer2 = csv.writer(fichier2, delimiter=" ")
    writer2.writerow([xn, yn, zn, xw, yw, zw])