from ddboats_client import *

FREQUENCY = 1 

while True:
    send_position(SERVER_IP, 6800, 4, lat=48.199028, lon=-3.015556, cap=90)

    # Positions de TOUS les bateaux, telles que renvoyées par le serveur
    for id_, b in get_boats().items():
        print(id_, b["lat"], b["lon"], b["cap"], b["age"])
    time.sleep(FREQUENCY)