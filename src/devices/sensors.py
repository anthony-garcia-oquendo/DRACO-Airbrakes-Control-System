import time
import numpy as np
import adafruit_bmp3xx
import adafruit_icm20x

class BMP390:
    # Driver returns: temperature [°C], altitude [m], pressure [hPa]
    def __init__(self, i2c) -> None:
        self.altimeter = adafruit_bmp3xx.BMP3XX_I2C(i2c)
        self.altimeter.pressure_oversampling = 1
    
    def temperature(self):
        return celsius_to_fahrenheit(self.altimeter.temperature)
    
    def altitude(self):
        return meters_to_feet(self.altimeter.altitude)
    
    def pressure(self):
        return hPa_to_psi(self.altimeter.pressure)
    
    def air_density(self):
        R = 287.05 # J/kg-K (dry air gas const)
        T_kelvin = self.altimeter.temperature + 273.15 # K
        P_pascals = self.altimeter.pressure * 100 # Pa
        rho_kg_m3 = P_pascals / (R * T_kelvin) # kg/m^3
        return kg_m3_to_lbm_ft3(rho_kg_m3) # lbm/ft^3

    def zero(self, n=100, wait=0.01):
        pressure_sum = 0
        for _ in range(n):
            pressure_sum += self.altimeter.pressure
            time.sleep(wait)
        self.altimeter.sea_level_pressure = pressure_sum / n

    def reset(self):
        self.altimeter.reset()


class ICM20948:
    # Driver returns: acceleration [m/s^2], gyro [rad/s], magnetic [uT]
    def __init__(self, i2c, ) -> None:
        self.imu = adafruit_icm20x.ICM20948(i2c)
        self.imu.gyro_range = adafruit_icm20x.GyroRange.RANGE_2000_DPS # avoid saturation during fast roll/spin
        self.set_accel_high_g() # start in high-g mode for Launch

    def set_accel_high_g(self):
        # Boost phase: prevent clipping from thrust/vibration/shock
        self.imu.accelerometer_range = adafruit_icm20x.AccelRange.RANGE_16G

    def set_accel_post_burnout(self):
        # Post-burnout/coast: lowering accel range improves resolution
        # (more LSB per g), which helps detect smaller acceleration changes
        # and generally gives cleaner data for filtering/estimation.
        self.imu.accelerometer_range = adafruit_icm20x.AccelRange.RANGE_8G

    def acceleration(self):
        return tuple(map(meters_to_feet, self.imu.acceleration))  # ft/s^2

    def gyro(self):
        return tuple(map(rads_to_deg, self.imu.gyro))  # deg/s 

    def magnetic(self):
        return tuple(self.imu.magnetic)  # uT
    
    def reset(self):
        self.imu.reset()

def meters_to_feet(n):
    return n * 3.28084

def celsius_to_fahrenheit(n):
    return n * 1.8 + 32.0

def newtons_to_pounds(n):
    return n * 0.224809

def rads_to_deg(n):
    return n * (180.0 / np.pi)

def hPa_to_psi(n):
    # hPa (hectopascals) to PSI
    # 1 hPa = 0.0145038 PSI
    return n * 0.0145038

def kg_m3_to_lbm_ft3(n):
    # 1 kg/m³ ≈ 0.06243 lbm/ft³
    return n * 0.06242796
