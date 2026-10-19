"""pyserial falso: /dev/ttyV0 = Arduino, /dev/ttyGPS* = GPS com frases programaveis.
Grava os comandos de motor em FAKE_ARDUINO_LOG (env) para testes com subprocesso."""
import os

ARDUINO_WRITES = []
GPS_LINES = []


def _nmea(body):
    x = 0
    for c in body:
        x ^= ord(c)
    return "$%s*%02X\r\n" % (body, x)


class Serial(object):
    def __init__(self, port, baudrate=9600, timeout=1.0, **kw):
        self.port = port
        self.timeout = timeout
        if port == "/dev/ttyV0":
            self.kind = "ard"
            self._init = True
        elif port.startswith("/dev/ttyGPS"):
            self.kind = "gps"
            self._pmtk = []
        else:
            raise OSError("porta inexistente %s" % port)

    def readline(self):
        if self.kind == "ard":
            if self._init:
                self._init = False
                return b"fake arduino ready\n"
            return b"S;fake\n"
        if self._pmtk:
            return self._pmtk.pop(0).encode()
        if GPS_LINES:
            return GPS_LINES.pop(0).encode()
        return b""

    def write(self, data):
        s = data.decode()
        if self.kind == "ard":
            ARDUINO_WRITES.append(s)
            p = os.environ.get("FAKE_ARDUINO_LOG")
            if p:
                with open(p, "a") as f:
                    f.write(s + "\n")
        else:
            if s.startswith("$PMTK"):
                self._pmtk.append(_nmea("PMTK001,386,3"))
        return len(data)

    def close(self):
        pass
