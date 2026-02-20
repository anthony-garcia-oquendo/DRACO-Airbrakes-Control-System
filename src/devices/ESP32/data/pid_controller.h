#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

struct PIDState {
    double integral = 0;
    double prev_error = 0;
    double last_time = 0;
};

// Slew rate limits
const double MAX_SERVO_DEG_PER_SEC = 285.71; // Servo max speed @ 7.4v = 285.71 deg/s


// Function Prototypes
double calculate_control_effort(double current_alt, double current_vel, double target_apogee, double dt, PIDState &state);
double slew_rate_limiter(double target_angle, double current_angle, double dt);

#endif
