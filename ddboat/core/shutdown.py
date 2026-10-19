# -*- coding: utf-8 -*-
"""Parada limpa: sinais e arquivo STOP.

O ArduinoIO da ENSTA instala o proprio handler de SIGINT (zera motores e chama
sys.exit). Chame install() DEPOIS de criar o backend para retomar o controle.
SIGHUP (queda do SSH sem nohup) e SIGTERM (kill) viram StopRequested, entao o
finally do programa roda e os motores sao zerados.
"""
import os
import signal


class StopRequested(Exception):
    pass


def _handler(signum, frame):
    raise StopRequested("sinal %d" % signum)


def install():
    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, _handler)


class FileFlags(object):
    """Arquivos de comando na pasta data/: STOP para tudo, RTH volta ao pontao.
    Uso pelo SSH:  touch data/STOP   (o arquivo e apagado ao iniciar o programa)."""

    def __init__(self, data_dir):
        self.stop_path = os.path.join(data_dir, "STOP")
        self.rth_path = os.path.join(data_dir, "RTH")
        for p in (self.stop_path, self.rth_path):
            if os.path.exists(p):
                os.remove(p)

    def stop(self):
        return os.path.exists(self.stop_path)

    def rth(self):
        return os.path.exists(self.rth_path)
