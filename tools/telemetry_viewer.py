# -*- coding: utf-8 -*-
"""Recebe a telemetria UDP do barco (config telemetry.*) e mostra no terminal.
Uso: python tools/telemetry_viewer.py [porta]"""
import json
import socket
import sys


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 6000
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", port))
    print("Escutando UDP %d (Ctrl+C para sair)" % port)
    try:
        while True:
            d, addr = s.recvfrom(4096)
            try:
                m = json.loads(d.decode())
            except ValueError:
                continue
            print("%s %s" % (addr[0], m))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
