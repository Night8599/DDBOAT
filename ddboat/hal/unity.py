# -*- coding: utf-8 -*-
"""Backend do simulador Unity (protocolo do quadro da aula, ja testado).

Envia {"id", "password", "cmd_left", "cmd_right"} para ip:cmd_port (UDP)
Recebe {"id", "X", "Y", "heading_deg"} em pose_port (UDP); ignora outros ids
e mensagens mal formadas. Envia {"reset": true} no inicio.
"""
import json
import socket

from ddboat.core.clock import RealClock
from ddboat.hal.base import Backend, Sample, clamp_cmd


class UnitySim(Backend):
    name = "unity"
    has_pose = True

    def __init__(self, cfg):
        self.cfg = cfg
        self.clock = RealClock()
        self.ip = cfg.require("unity.ip")
        self.cmd_port = int(cfg.require("unity.cmd_port"))
        self.pose_port = int(cfg.require("unity.pose_port"))
        self.robot_id = cfg.require("unity.robot_id")
        self.password = cfg.require("unity.password")
        cmd_max = int(cfg.require("unity.cmd_max"))
        self.limits = (-cmd_max if cfg.require("unity.allow_reverse") else 0, cmd_max)
        self.sock_cmd = None
        self.sock_pose = None
        self.last_pose = None
        self.last_pose_t = None

    def start(self, reset=True):
        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_pose = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_pose.bind(("0.0.0.0", self.pose_port))
        self.sock_pose.setblocking(False)
        if reset:
            self._send({"id": self.robot_id, "password": self.password, "reset": True})

    def _send(self, msg):
        self.sock_cmd.sendto(json.dumps(msg).encode("utf-8"), (self.ip, self.cmd_port))

    def read(self):
        new = None
        while True:  # esvazia o socket e fica com a pose mais recente
            try:
                data, _ = self.sock_pose.recvfrom(4096)
            except (BlockingIOError, socket.error):
                break
            try:
                m = json.loads(data.decode("utf-8"))
                if m.get("id") == self.robot_id:
                    new = (float(m["X"]), float(m["Y"]), float(m["heading_deg"]))
            except (ValueError, KeyError, TypeError, AttributeError):
                continue
        t = self.clock.now()
        if new is not None:
            self.last_pose = new
            self.last_pose_t = t
        return Sample(t, pose=new)

    def set_motors(self, left, right):
        l = clamp_cmd(left, self.limits[0], self.limits[1])
        r = clamp_cmd(right, self.limits[0], self.limits[1])
        self._send({"id": self.robot_id, "password": self.password,
                    "cmd_left": l, "cmd_right": r})
        return l, r

    def stop(self):
        for _ in range(3):  # UDP pode perder pacote
            self.set_motors(0, 0)

    def close(self):
        try:
            if self.sock_cmd is not None:
                self.stop()
        finally:
            for s in (self.sock_cmd, self.sock_pose):
                if s is not None:
                    s.close()
