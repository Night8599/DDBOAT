# -*- coding: utf-8 -*-
"""Telemetria UDP barco -> PC, nao bloqueante.

Se a rede cair, os pacotes sao simplesmente perdidos: o laco de controle
nunca espera pela rede. No PC: python tools/telemetry_viewer.py
"""
import json
import socket


class Telemetry(object):
    def __init__(self, ip, port, period_s):
        self.addr = (ip, int(port))
        self.period = float(period_s)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.last = None
        self.errors = 0

    @classmethod
    def from_config(cls, cfg):
        if not cfg.require("telemetry.enabled"):
            return None
        return cls(cfg.require("telemetry.pc_ip"), cfg.require("telemetry.port"),
                   cfg.require("telemetry.period_s"))

    def maybe_send(self, t, payload):
        if self.last is not None and t - self.last < self.period:
            return
        self.last = t
        try:
            self.sock.sendto(json.dumps(payload).encode("utf-8"), self.addr)
        except (socket.error, OSError):
            self.errors += 1

    def close(self):
        self.sock.close()
