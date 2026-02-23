#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double initial_alt, double initial_vel) {
    x_alt = initial_alt;
    x_vel = initial_vel;
    
    // Initial uncertainty is relatively high (1.0), but it will converge instantly
    P_00 = 1.0; P_01 = 0.0;
    P_10 = 0.0; P_11 = 1.0;

    // Default Tuning (You will tune these in SITL/HITL)
    Q_accel = 0.1;  // Adjust based on your motor vibration
    R_baro = 2.0;   // Adjust based on BMP388 noise profile
}

void KalmanFilter::predict(double accel, double dt) {
    // 1. Predict next state using basic kinematics:
    // Altitude = alt + (vel * dt) + (0.5 * accel * dt^2)
    // Velocity = vel + (accel * dt)
    double new_alt = x_alt + (x_vel * dt) + (0.5 * accel * dt * dt);
    double new_vel = x_vel + (accel * dt);

    x_alt = new_alt;
    x_vel = new_vel;

    // 2. Update the Covariance Matrix (Uncertainty grows when we predict)
    // P = F * P * F^T + Q
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    P_00 += dt * (P_10 + P_01 + dt * P_11) + (Q_accel * dt4 / 4.0);
    P_01 += dt * P_11 + (Q_accel * dt3 / 2.0);
    P_10 += dt * P_11 + (Q_accel * dt3 / 2.0);
    P_11 += (Q_accel * dt2);
}

void KalmanFilter::update(double baro_alt) {
    // 1. Calculate Innovation (Error between measurement and prediction)
    double y = baro_alt - x_alt;

    // 2. Innovation Covariance (How much uncertainty is in this measurement vs our model)
    double S = P_00 + R_baro;

    // 3. Calculate Kalman Gain (How much should we trust the sensor vs the prediction)
    double K_0 = P_00 / S;
    double K_1 = P_10 / S;

    // 4. Update the State Estimate using the Gain
    x_alt += K_0 * y;
    x_vel += K_1 * y;

    // 5. Update the Covariance Matrix (Uncertainty shrinks because we measured)
    // P = (I - K * H) * P
    double P00_temp = P_00;
    double P01_temp = P_01;

    P_00 -= K_0 * P00_temp;
    P_01 -= K_0 * P01_temp;
    P_10 -= K_1 * P00_temp;
    P_11 -= K_1 * P01_temp;
}