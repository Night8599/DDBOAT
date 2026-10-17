import socket
import json
import time


# Cible de connexion
# UNITY_IP = "172.20.26.99" # 127.0.0.1 pour localhost
UNITY_IP = "127.0.0.1" # A CHANGER
COMMAND_PORT = 5005      # Port où envoyer les commandes
POSE_PORT = 5006    # Port où recevoir la télémétrie


# Créer une socket UDP pour envoyer les commandes
sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# Créer une socket UDP pour recevoir la télémétrie
sock_pose = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_pose.bind(("0.0.0.0", POSE_PORT))
sock_pose.settimeout(1.0)


# Configuration du robotsds
ROBOT_ID = 4
PASSWORD = " "


reset_msg = {"id": ROBOT_ID, "password": PASSWORD, "reset": True}
sock_cmd.sendto(json.dumps(reset_msg).encode("utf-8"), (UNITY_IP, COMMAND_PORT))
# Boucle de commande
while True:


   try:
       # Reception de la télémétrie
       pose_data, _ = sock_pose.recvfrom(1024)
       pose = json.loads(pose_data.decode("utf-8"))
      
       # Filtrer : récupérer seulement la télémétrie de notre robot
       if pose['id'] == ROBOT_ID:
           X = pose['X']
           Y = pose['Y']
           heading_deg = pose['heading_deg']
           print(f"pose: X={X:.2f}, Y={Y:.2f}, heading={heading_deg:.2f}")


   except socket.timeout:
       print("No pose received")


   # Message de commande
   cmd_msg = {
       "id": ROBOT_ID,
       "password": PASSWORD,
       "cmd_left": 5,
       "cmd_right": 5,
   }




   # Envoie de la commande
   cmd_data = json.dumps(cmd_msg).encode("utf-8")
   sock_cmd.sendto(cmd_data, (UNITY_IP, COMMAND_PORT))
  
