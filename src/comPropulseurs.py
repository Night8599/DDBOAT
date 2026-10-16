import sys
import time

import arduino_driver_v2 as arddrv

def main():
    # Vérifie que deux vitesses ont été fournies
    if len(sys.argv) != 3:
        print("Utilisation : python3 prog.py <vitesse_gauche> <vitesse_droite>")
        print("Exemple      : python3 prog.py 20 20")
        sys.exit(1)

    try:
        left_speed = int(sys.argv[1])
        right_speed = int(sys.argv[2])
    except ValueError:
        print("Erreur : les vitesses doivent etre des nombres entiers.")
        print("Exemple : python3 prog.py 20 20")
        sys.exit(1)

    # Creation de l'objet Arduino
    ard = arddrv.ArduinoIO()

    # Envoi des vitesses aux propulseurs
    ard.send_arduino_cmd_motor(left_speed, right_speed) 
    # Maintien de la commande pendant 2 secondes
    time.sleep(2)

    # Arret des propulseurs
    ard.send_arduino_cmd_motor(0, 0)
    print("Propulseurs arretes.")


if __name__ == "__main__":
    main()