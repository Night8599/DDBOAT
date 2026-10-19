# -*- coding: utf-8 -*-
"""Calibracoes do acelerometro/giroscopio.

gyro_still: com o barco PARADO, bias do giro (cru) e vetor "para cima" no
  referencial da IMU (o acelerometro em repouso mede a forca especifica, que
  aponta para cima). Com isso a taxa de guinada anti-horaria (vista de cima) e
  r = (gyr - bias) . up_unit, sem precisar saber como a IMU foi montada.
  (Giro e acelerometro estao no mesmo chip LSM6DS33, mesmos eixos.)

axes_from_flat_bow: rotacao IMU -> barco (FLU) a partir de duas posturas:
  barco na horizontal e barco com a PROA levantada.
"""
import json
import math

from ddboat.core import linalg as la

GYRO_RAD_PER_LSB = math.radians(0.070)  # driver: FS_G = 2000 dps -> 70 mdps/LSB


class CalibError(Exception):
    pass


def _mean(vs):
    n = float(len(vs))
    return [sum(v[k] for v in vs) / n for k in range(3)]


def _std(vs, m):
    n = float(len(vs))
    return [math.sqrt(sum((v[k] - m[k]) ** 2 for v in vs) / n) for k in range(3)]


def gyro_still(gyr_samples, acc_samples, max_gyro_std_lsb):
    if len(gyr_samples) < 10:
        raise CalibError("amostras insuficientes")
    b = _mean(gyr_samples)
    s = _std(gyr_samples, b)
    if max(s) > max_gyro_std_lsb:
        raise CalibError("barco nao estava parado: desvio do giro %r LSB > %.1f"
                         % ([round(x, 1) for x in s], max_gyro_std_lsb))
    up = la.normalize(_mean(acc_samples))
    return {"gyro_bias_lsb": b, "gyro_std_lsb": s, "up_unit_imu": up}


def yaw_rate(gyr, cal):
    """Taxa de guinada anti-horaria vista de cima, em rad/s."""
    g = [a - b for a, b in zip(gyr, cal["gyro_bias_lsb"])]
    return la.dot(g, cal["up_unit_imu"]) * GYRO_RAD_PER_LSB


def axes_from_flat_bow(acc_flat, acc_bow, min_tilt_deg=10.0):
    """Retorna R (3x3) tal que v_barco = R v_imu, barco FLU."""
    z = la.normalize(acc_flat)
    b = la.normalize(acc_bow)
    tilt = math.degrees(math.acos(max(-1, min(1, la.dot(z, b)))))
    if tilt < min_tilt_deg:
        raise CalibError("proa levantada so %.1f graus (minimo %.0f)" % (tilt, min_tilt_deg))
    # com a proa para cima, "cima" no barco ganha componente +x (frente)
    xv = [bi - la.dot(b, z) * zi for bi, zi in zip(b, z)]
    x = la.normalize(xv)
    y = la.cross(z, x)
    return [x, y, z], tilt


def save(d, path):
    with open(path, "w") as f:
        json.dump(d, f, indent=2)


def load(path):
    with open(path) as f:
        return json.load(f)
