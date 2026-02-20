// flight_logic.h
#ifndef FLIGHT_LOGIC_H
#define FLIGHT_LOGIC_H

struct PIDState {
    double integral = 0;
    double prev_error = 0;
};

// These functions will be used by both SITL and HITL
double predict_apogee(double flap_angle, double alt, double vel);
double calculate_control_effort(double alt, double vel, double target, double dt, PIDState &pid);
double calculate_drag(double flap_angle, double alt, double vel);

#endif