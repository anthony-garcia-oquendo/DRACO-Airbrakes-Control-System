#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <cmath>

// Environmental Constants
extern const double GRAVITY;
extern const double VEHICLE_MASS;
extern const double PREDICTION_FREQUENCY;

// Core Physics Functions
double calculate_drag(double flap_angle, double altitude, double velocity);
double predict_apogee(double flap_angle, double altitude, double velocity);

#endif
