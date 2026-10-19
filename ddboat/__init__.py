# -*- coding: utf-8 -*-
"""Biblioteca DDBoat (equipe barco 04).

Convencoes internas (valem para TODO o codigo):
  - Unidades SI: metros, segundos, radianos.
  - Referencial local ENU: x = leste, y = norte, origem definida no config.
  - Rumo interno "theta": angulo matematico em rad, anti-horario a partir
    do leste (0 = leste, pi/2 = norte). E a convencao do modelo do Jaulin.
  - Rumo de bussola (0 = norte, horario, graus) so aparece na tela e no log.
  - Todo texto impresso e ASCII (Python < 3.7 com locale C quebra com acento).
  - Compativel com Python 3.5 no lado do barco: sem f-string, sem anotacoes.
"""
__version__ = "0.1.0"
