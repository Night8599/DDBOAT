import math
import time
import numpy as np

from src.DriversDDBOAT_2026 import imu9_driver_v2 as imu
from src.DriversDDBOAT_2026 import gps_driver_v2 as gps

from filter import kalman

# Unify all sensors in one class, to be used by the boat control code.

class Sensors:
    def __init__(self):
        self.imu = imu.Imu9IO()
        self.gps = gps.GpsIO()
        self.kalman = kalman.Kalman(q=0.1, r=5.0)
        self.gps.start()
        self.heading = None
        self.pos = []  # [x, y]
        self.dt_gps = 0.0

    def read_imu(self):
        mag = self.imu.read_mag_raw()
        acc = self.imu.read_accel_raw()
        gyro = self.imu.read_gyro_raw()
        return mag, acc, gyro
        
        # Calibrar primeiro para depois pegar a direção correta
        # self.heading = self.imu.heading_deg(mag[0], mag[1])

    def calibrate_imu(self, magx_min, magx_max, magy_min, magy_max):
        self.imu.fast_heading_calibration(magx_min, magx_max, magy_min, magy_max)

    # Preference for GLL data
    def read_gps(self):
        gll_ok,gll_data = self.gps.read_gll_non_blocking()
        if gll_ok:
            # Convert to metric coordinates (x, y) in meters
            lat, lon, time = gll_data[0], gll_data[2], gll_data[4]
            x = (lon - self.gps.lon0) * 111320 * math.cos(math.radians(lat))
            y = (lat - self.gps.lat0) * 110574
            self.raw_pos = [x, y]
            print("GPS: lat={}, lon={}, time={}\n".format(lat, lon, time))
            # Update Kalman filter
            now = time
            if hasattr(self, 'last_gps_time'):
                self.dt_gps = now - self.last_gps_time
                self.kalman.predict(self.dt_gps)
                self.kalman.update(np.array(self.raw_pos))
            self.last_gps_time = now

        else:
            self.raw_pos = None

    def read_all_sensors(self):
        try:
            self.read_imu()
        except Exception as e:
            print("Error reading IMU: ", e)
            # Maintain previous heading if IMU fails
        try:
            self.read_gps()
        except Exception as e:
            print("Error reading GPS: ", e)
            # Maintain previous position if GPS fails

while True:
    sensors = Sensors()
    sensors.read_gps()
    time.sleep(1)