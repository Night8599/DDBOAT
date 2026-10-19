# -*- coding: utf-8 -*-
"""Mistura (empuxo, giro) -> (esquerdo, direito), dentro de [cmd_min, cmd_max].

Convencao: giro > 0 = virar para a ESQUERDA (anti-horario, theta aumenta),
ou seja, direito mais forte: esq = T - D, dir = T + D.
turn_sign permite inverter (Unity com convencao espelhada).

Prioridade para o giro: primeiro limita |D| a (cmd_max - cmd_min)/2, depois
encaixa T no espaco que sobra. Assim a saturacao nunca "come" a curva
(problema relatado pela turma anterior com os dois motores no maximo).

Depois da mistura (opcional, do config): ganho por motor (trim) e
compensacao de zona morta. Os dois ficam neutros (1.0 e 0) ate medirmos.
"""


class Mixer(object):
    def __init__(self, cmd_min, cmd_max, turn_sign=1, gain_left=1.0, gain_right=1.0,
                 deadzone=0):
        if cmd_max <= cmd_min:
            raise ValueError("cmd_max deve ser > cmd_min")
        self.cmd_min = cmd_min
        self.cmd_max = cmd_max
        self.turn_sign = turn_sign
        self.gl = gain_left
        self.gr = gain_right
        self.dz = deadzone

    def _dead(self, v):
        if self.dz <= 0 or v == 0:
            return v
        full = float(self.cmd_max)
        mag = self.dz + abs(v) * (full - self.dz) / full
        return mag if v > 0 else -mag

    def mix(self, thrust, turn):
        D = self.turn_sign * turn
        half = (self.cmd_max - self.cmd_min) / 2.0
        D = max(-half, min(half, D))
        lo = self.cmd_min + abs(D)
        hi = self.cmd_max - abs(D)
        T = max(lo, min(hi, thrust))
        left = self._dead(self.gl * (T - D))
        right = self._dead(self.gr * (T + D))
        left = max(self.cmd_min, min(self.cmd_max, left))
        right = max(self.cmd_min, min(self.cmd_max, right))
        return int(round(left)), int(round(right))
