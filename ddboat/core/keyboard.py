# -*- coding: utf-8 -*-
"""Leitura de tecla sem Enter, em Linux (barco, Ubuntu) e Windows.

Linux: termios em modo cbreak + select (mesmo metodo do teleop.py testado no barco).
Windows: msvcrt (mesmo metodo do teleop_sim testado).
Ctrl+C continua gerando SIGINT nos dois casos.
"""
import os
import sys
import time


class KeyReader(object):
    def __init__(self):
        self.is_windows = (os.name == "nt")
        self._old = None

    def __enter__(self):
        if not self.is_windows:
            import termios
            import tty
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        self.restore()
        return False

    def restore(self):
        if not self.is_windows and self._old is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            self._old = None

    def resume(self):
        if not self.is_windows and self._old is None:
            import termios
            import tty
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)

    def read(self, timeout):
        """Retorna a tecla em minusculo, ou None se nada chegou em timeout s.
        Setas e teclas especiais sao descartadas."""
        if self.is_windows:
            import msvcrt
            end = time.time() + timeout
            while True:
                if msvcrt.kbhit():
                    k = msvcrt.getwch()
                    if k in ("\x00", "\xe0"):
                        msvcrt.getwch()
                        return None
                    if k == "\x03":
                        raise KeyboardInterrupt
                    return k.lower()
                if time.time() >= end:
                    return None
                time.sleep(0.005)
        import select
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1).lower()
        return None
