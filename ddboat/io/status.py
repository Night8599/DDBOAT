# -*- coding: utf-8 -*-
"""Linha de status na tela, limitada em frequencia, que nunca derruba o programa.

Em terminal: uma linha que se reescreve (\\r). Sem terminal (nohup, arquivo):
uma linha a cada periodo. Erros de escrita (SSH caiu) sao ignorados.
"""
import sys


class Status(object):
    def __init__(self, period_s, quiet=False):
        self.period = period_s
        self.quiet = quiet
        self.last = None
        try:
            self.tty = sys.stdout.isatty()
        except Exception:
            self.tty = False

    def show(self, t, text, force=False):
        if self.quiet:
            return
        if not force and self.last is not None and t - self.last < self.period:
            return
        self.last = t
        try:
            if self.tty:
                sys.stdout.write("\r" + text + "   ")
            else:
                sys.stdout.write(text + "\n")
            sys.stdout.flush()
        except Exception:
            pass

    def line(self, text):
        if self.quiet:
            return
        try:
            sys.stdout.write(("\n" if self.tty else "") + text + "\n")
            sys.stdout.flush()
        except Exception:
            pass
