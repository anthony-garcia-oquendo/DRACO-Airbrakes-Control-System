import time
import numpy as np
import struct

import adafruit_bmp3xx
import adafruit_icm20x
from adafruit_bus_device.i2c_device import I2CDevice


def _read_regs(dev: I2CDevice, start_reg: int, n: int) -> bytes:
    buf = bytearray(n)
    with dev as i2c:
        i2c.write(bytes([start_reg]))
        i2c.readinto(buf)
    return bytes(buf)


# =====================
# BMP390 (Altimeter)
# =====================

class BMP390:
    # Driver returns: temperature [°C], altitude [m], pressure [hPa]
    def __init__(
        self,
        i2c,
        simulation: bool = False,
        sim_addr: int = 0x28,
        sim_alt_reg: int = 0x04,
        sim_alt_scale: float = 1000.0,  # meters × 1000
    ) -> None:
        self.simulation = simulation

        if not simulation:
            self.altimeter = adafruit_bmp3xx.BMP3XX_I2C(i2c)
            self.altimeter.pressure_oversampling = 1
        else:
            self._sim = I2CDevice(i2c, sim_addr)
            self._sim_alt_reg = sim_alt_reg
            self._sim_alt_scale = sim_alt_scale

    def temperature(self):
        if self.simulation:
            return None
        return celsius_to_fahrenheit(self.altimeter.temperature)

    def altitude(self):
        """
        Always returns altitude in FEET.
        Internally:
          - real sensor: meters → feet
          - simulation: meters → feet
        """
        if not self.simulation:
            return meters_to_feet(self.altimeter.altitude)

        raw = _read_regs(self._sim, self._sim_alt_reg, 4)
        (alt_m_x1000,) = struct.unpack("<i", raw)
        alt_m = alt_m_x1000 / self._sim_alt_scale
        return meters_to_feet(alt_m)

    def pressure(self):
        if self.simulation:
            return None
        return hPa_to_psi(self.altimeter.pressure)

    def air_density(self):
        if self.simulation:
            raise RuntimeError("air_density() requires real temperature + pressure")

        R = 287.05  # J/kg-K
        T_kelvin = self.altimeter.temperature + 273.15
        P_pascals = self.altimeter.pressure * 100
        rho_kg_m3 = P_pascals / (R * T_kelvin)
        return kg_m3_to_lbm_ft3(rho_kg_m3)

    def zero(self, n=100, wait=0.01):
        if self.simulation:
            return
        pressure_sum = 0
        for _ in range(n):
            pressure_sum += self.altimeter.pressure
            time.sleep(wait)
        self.altimeter.sea_level_pressure = pressure_sum / n

    def reset(self):
        if self.simulation:
            return
        self.altimeter.reset()


# =====================
# ICM20948 (IMU)
# =====================

class ICM20948:
    # Driver returns: acceleration [m/s^2]
    def __init__(
        self,
        i2c,
        simulation: bool = False,
        sim_addr: int = 0x28,
        sim_accel_reg: int = 0x08,
        sim_accel_scale: float = 1000.0,  # ft/s^2 × 1000
    ) -> None:
        self.simulation = simulation

        if not simulation:
            self.imu = adafruit_icm20x.ICM20948(i2c)
            self.imu.gyro_range = adafruit_icm20x.GyroRange.RANGE_2000_DPS
            self.set_accel_high_g()
        else:
            self._sim = I2CDevice(i2c, sim_addr)
            self._sim_accel_reg = sim_accel_reg
            self._sim_accel_scale = sim_accel_scale

    def set_accel_high_g(self):
        if self.simulation:
            return
        self.imu.accelerometer_range = adafruit_icm20x.AccelRange.RANGE_16G

    def set_accel_post_burnout(self):
        if self.simulation:
            return
        self.imu.accelerometer_range = adafruit_icm20x.AccelRange.RANGE_8G

    def acceleration(self):
        """
        Always returns ft/s^2.
        """
        if not self.simulation:
            return tuple(map(meters_to_feet, self.imu.acceleration))

        raw = _read_regs(self._sim, self._sim_accel_reg, 6)
        ax, ay, az = struct.unpack("<hhh", raw)
        s = self._sim_accel_scale
        return (ax / s, ay / s, az / s)

    def gyro(self):
        if self.simulation:
            return None
        return tuple(map(rads_to_deg, self.imu.gyro))

    def magnetic(self):
        if self.simulation:
            return None
        return tuple(self.imu.magnetic)

    def reset(self):
        if self.simulation:
            return
        self.imu.reset()


# =====================
# Unit helpers
# =====================

def meters_to_feet(n):
    return n * 3.28084

def celsius_to_fahrenheit(n):
    return n * 1.8 + 32.0

def rads_to_deg(n):
    return n * (180.0 / np.pi)

def hPa_to_psi(n):
    return n * 0.0145038

def kg_m3_to_lbm_ft3(n):
    return n * 0.06242796
