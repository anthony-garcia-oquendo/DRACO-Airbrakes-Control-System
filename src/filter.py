import time

from filterpy.kalman import KalmanFilter
import numpy as np


class DataFilter:
    def __init__(self):
        self._dt = 0
        self._t_prev = None
        self._initialize_filter()
        
        self.kalman_altitude = 0
        self.kalman_velocity = 0
        self.kalman_acceleration = 0

    def _initialize_filter(self):
        sensor_matrix = [
            [1, 0, 0],
            [0, 0, 1]
        ]

        self.filter = KalmanFilter(dim_x=3, dim_z=len(sensor_matrix))
        self.filter.H = np.array(sensor_matrix)

        self.filter.P *= 1
        self.filter.R *= 1
        self.filter.Q *= 1
        self.filter.x = np.array([[0.0],
                                  [0.0],
                                  [0.0]])
                               
    def _calculate_dt(self, in_time):
        if self._t_prev is None:
            self._dt = 0.1
        else:
            self._dt = in_time - self._t_prev
        self._t_prev = in_time

    def _generate_phi(self, dt = 0.01):
        if dt is None:
            self._calculate_dt(time.time())
        else:
            self._dt = dt

        dp = 1
        ds = 0
        di = (self._dt ** 2) / 2

        phi = np.array([
            [dp, self._dt, di],
            [ds, dp, self._dt],
            [ds, ds, dp]
        ])

        return phi

    def filter_data(self, altitude, acceleration, dt=0.5):
        measurements = [float(altitude), float(acceleration)]

        params = np.array(measurements)
        self.filter.F = self._generate_phi(dt=dt)
        self.filter.predict()
        self.filter.update(params)

        self.kalman_altitude     = float(self.filter.x[0, 0])
        self.kalman_velocity     = float(self.filter.x[1, 0])
        self.kalman_acceleration = float(self.filter.x[2, 0])
