#include "physics_engine.h"
#include <algorithm>
#include <vector>
#include <iostream>

// ENVIRONMENTAL CONSTANTS - CHANGE AT LAUNCH SITE
const double GROUND_TEMPERATURE_C = 15;   // °C
const double GROUND_PRESSURE_PA = 101325.0; // HPA (~1013 mb)
const double GRAVITY = 9.80665;             // m/s²
const double R_AIR = 287.058;               // J/(kg·K)
const double LAPSE_RATE = 0.0065;           // °C per meter
const double VEHICLE_MASS = 22.807; // kg at BURNOUT
// For mini motor
// const double VEHICLE_MASS = 22.664; // kg at BURNOUT
const double PREDICTION_FREQUENCY = 50.0; // Hz for RK4 integration

// Rows: Flap Angles (0, 7.5, 15, 22.5, 30, 37.5, 45)
// Cols: Mach Numbers (0.0, 0.18, 0.36, 0.54)
const double CD_TABLE[7][4] = {
    {0.0, 0.329, 0.311, 0.319}, // 0.0°  Flaps
    {0.0, 0.337, 0.318, 0.425}, // 7.5°  Flaps
    {0.0, 0.394, 0.372, 0.395}, // 15.0° Flaps
    {0.0, 0.477, 0.455, 0.491}, // 22.5° Flaps
    {0.0, 0.559, 0.569, 0.599}, // 30.0° Flaps
    {0.0, 0.535, 0.530, 0.572}, // 37.5° Flaps
    {0.0, 0.565, 0.562, 0.625}  // 45.0° Flaps
};

// Supporting axis arrays for interpolation logic
const double FLAP_AXIS[7] = {0.0, 7.5, 15.0, 22.5, 30.0, 37.5, 45.0};
const double MACH_AXIS[4] = {0.0, 0.18, 0.36, 0.54};

// 1D Array for Area (m^2) from 0 to 45 degrees
const double AREA_TABLE[46] = {
    0.02150, 0.02197, 0.02249, 0.02301, 0.02352, // 0-4
    0.02403, 0.02455, 0.02506, 0.02557, 0.02608, // 5-9
    0.02658, 0.02708, 0.02759, 0.02808, 0.02858, // 10-14
    0.02908, 0.02957, 0.03006, 0.03055, 0.03103, // 15-19
    0.03151, 0.03198, 0.03246, 0.03292, 0.03339, // 20-24
    0.03385, 0.03430, 0.03475, 0.03521, 0.03565, // 25-29
    0.03608, 0.03652, 0.03695, 0.03737, 0.03779, // 30-34
    0.03820, 0.03861, 0.03901, 0.03941, 0.03980, // 35-39
    0.04018, 0.04056, 0.04093, 0.04130, 0.04166, 0.04201  // 40-45
};

// Helper Functions (Internal linkage)
static double get_interpolated_cd(double flap, double mach) {
    flap = std::max(0.0, std::min(45.0, flap));
    mach = std::max(0.0, std::min(0.54, mach));

    int i = 0;
    while (i < 5 && flap > FLAP_AXIS[i + 1]) i++;
    
    int j = 0;
    while (j < 2 && mach > MACH_AXIS[j + 1]) j++;

    double x_weight = (flap - FLAP_AXIS[i]) / (FLAP_AXIS[i+1] - FLAP_AXIS[i]);
    double y_weight = (mach - MACH_AXIS[j]) / (MACH_AXIS[j+1] - MACH_AXIS[j]);

    double c00 = CD_TABLE[i][j];
    double c10 = CD_TABLE[i+1][j];
    double c01 = CD_TABLE[i][j+1];
    double c11 = CD_TABLE[i+1][j+1];

    return c00 * (1 - x_weight) * (1 - y_weight) +
           c10 * x_weight * (1 - y_weight) +
           c01 * (1 - x_weight) * y_weight +
           c11 * x_weight * y_weight;
}

static double get_interpolated_area(double flap) {
    flap = std::max(0.0, std::min(45.0, flap));
    int i = static_cast<int>(flap);
    if (i >= 45) return AREA_TABLE[45];
    
    double weight = flap - i;
    return AREA_TABLE[i] * (1 - weight) + AREA_TABLE[i + 1] * weight;
}

static double atmosphere_temperature_k(double altitude_m) {
    return (GROUND_TEMPERATURE_C + 273.15) - (LAPSE_RATE * altitude_m);
}

static double atmosphere_pressure_pa(double altitude_m) {
    double Tk = atmosphere_temperature_k(altitude_m);
    double T_ground_k = GROUND_TEMPERATURE_C + 273.15;
    return GROUND_PRESSURE_PA * std::pow(Tk / T_ground_k, GRAVITY / (R_AIR * LAPSE_RATE));
}

// Public Functions
double calculate_drag(double flap_angle, double altitude, double velocity) {
    double temp_k = atmosphere_temperature_k(altitude);
    double rho = atmosphere_pressure_pa(altitude) / (R_AIR * temp_k);
    
    double speed_of_sound = std::sqrt(1.4 * R_AIR * temp_k);
    double mach_number = std::abs(velocity) / speed_of_sound;

    double Cd = get_interpolated_cd(flap_angle, mach_number);
    double Area = get_interpolated_area(flap_angle);

    return 0.5 * rho * std::pow(velocity, 2) * Area * Cd;
}

double predict_apogee(double flap_angle, double altitude, double velocity){
    double dt = 1.0 / PREDICTION_FREQUENCY; // 50 Hz prediction loop
    double weight = 1.0 / 6.0;
    double apogee_prediction = altitude;
    double velocity_current = velocity;

    auto calculateAcceleration = [flap_angle](double altitude, double velocity) -> double {
        return -GRAVITY - calculate_drag(flap_angle, altitude, velocity) / VEHICLE_MASS;
    };

    while(velocity_current > 0){
        double kx1 = velocity_current;
        double kp1 = calculateAcceleration(apogee_prediction, velocity_current);

        double kx2 = velocity_current + 0.5 * kp1 * dt;
        double kp2 = calculateAcceleration(apogee_prediction + 0.5 * kx1 * dt, velocity_current + 0.5 * kp1 * dt);

        double kx3 = velocity_current + 0.5 * kp2 * dt;
        double kp3 = calculateAcceleration(apogee_prediction + 0.5 * kx2 * dt, velocity_current + 0.5 * kp2 * dt);

        double kx4 = velocity_current + kp3 * dt;
        double kp4 = calculateAcceleration(apogee_prediction + kx3 * dt, velocity_current + kp3 * dt);

        apogee_prediction += weight * (kx1 + 2 * kx2 + 2 * kx3 + kx4) * dt;
        velocity_current += weight * (kp1 + 2 * kp2 + 2 * kp3 + kp4) * dt;
    }
    return apogee_prediction;
}
