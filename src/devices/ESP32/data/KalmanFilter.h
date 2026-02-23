#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
public:
    // Initialize with starting altitude (usually launchpad altitude) and velocity (0.0)
    KalmanFilter(double initial_alt = 0.0, double initial_vel = 0.0);

    // Step 1: Predict the new state based on IMU acceleration and time step
    void predict(double accel, double dt);

    // Step 2: Correct the prediction using the actual Barometer reading
    void update(double baro_alt);

    // Getters for your PID and Physics Engine
    double get_altitude() const { return x_alt; }
    double get_velocity() const { return x_vel; }

    // Tuning parameters (public so you can tweak them without recompiling everything)
    double Q_accel; // Process Noise (IMU variance)
    double R_baro;  // Measurement Noise (Barometer variance)

private:
    // State estimates
    double x_alt;
    double x_vel;

    // Estimation Covariance Matrix (P)
    // Tracks the "uncertainty" of our altitude and velocity
    double P_00, P_01;
    double P_10, P_11;
};

#endif // KALMAN_FILTER_H