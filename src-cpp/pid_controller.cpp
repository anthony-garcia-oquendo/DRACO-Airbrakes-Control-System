#include "pid_controller.h"
#include "physics_engine.h"
#include <algorithm>
#include <cmath>

// Global PID Constants
// const double Kp = 0.356;
// const double Ki = 0.151;
// const double Kd = 0.11;

// For mini motor
const double Kp = 1.5;
const double Ki = 0.04;
const double Kd = 0.0;

double calculate_control_effort(double current_alt, double current_vel, double target_apogee, double dt, PIDState &state) {
    
    // 1. Get current prediction with flaps at 0
    double predicted = predict_apogee(0.0, current_alt, current_vel);
    
    // 2. Calculate Error
    double error = predicted - target_apogee;

    // 3. Proportional term
    double P_out = Kp * error;

    // 4. Integral term (with wind-up protection)
    state.integral += error * dt;
    state.integral = std::max(-50.0, std::min(50.0, state.integral)); 
    double I_out = Ki * state.integral;

    // 5. Derivative term
    double derivative = (error - state.prev_error) / dt;
    double D_out = Kd * derivative;

    state.prev_error = error;

    // 6. Total Output
    double output_angle = P_out + I_out + D_out;

    // 7. Physical Clamping
    if (output_angle < 0) output_angle = 0;
    if (output_angle > 45) output_angle = 45;

    return output_angle;
}

double slew_rate_limiter(double target_angle, double current_angle, double dt) {
    double max_change = MAX_FLAP_SPEED_DEG_PER_SEC * dt;
    double diff = target_angle - current_angle;
    
    // Limit the change
    double change = std::max(-max_change, std::min(max_change, diff));
    
    // Calculate new angle
    double new_angle = current_angle + change;
    
    // Hard limits check
    return std::max(0.0, std::min(45.0, new_angle));
}
