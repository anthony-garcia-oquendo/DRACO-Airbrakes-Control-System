#include <iostream>
#include <cmath>

// Constants for Huntsville, AL (late April)
const double GROUND_TEMPERATURE_C = 16.0;   // °C
const double GROUND_PRESSURE_HPA = 101500.0; // HPA (~1015 mb)
const double GRAVITY = 9.80665;             // m/s²
const double R_AIR = 287.058;               // J/(kg·K)
const double LAPSE_RATE = 0.0065;           // °C per meter
const double VEHICLE_MASS = 26.94; // kg

// Temperature at a given altitude (meters)
double atmosphere_temperature(double altitude_m) {
    return GROUND_TEMPERATURE_C - LAPSE_RATE * altitude_m;
}

// Pressure at a given altitude (Pa)
double atmosphere_pressure(double altitude_m) {
    double T0 = GROUND_TEMPERATURE_C ; // Celsius
    double P0 = GROUND_PRESSURE_HPA;             // HPa
    double T = atmosphere_temperature(altitude_m); // Celsius
    return P0 * pow(T / T0, GRAVITY / (R_AIR * LAPSE_RATE));
}

// Density at a given altitude (kg/m³)
double atmosphere_density(double altitude_m) {
    double T = atmosphere_temperature(altitude_m) ; // C
    double P = atmosphere_pressure(altitude_m);// HPa
    return P / (R_AIR * T);
}

double calculate_drag(double flap_angle, double altitude, double velocity) {
    // Convert outputs to SI inside this function
    double pressure_hPa = atmosphere_pressure(altitude);  // hPa
    double temp_C = atmosphere_temperature(altitude);    // °C

    double P_Pa = pressure_hPa * 100.0;                  // Pa
    double T_K = temp_C + 273.15;                        // K

    double density = P_Pa / (R_AIR * T_K);              // kg/m³

    // Calculate Mach number
    double mach_number = velocity / sqrt(1.4 * P_Pa / density);  // m/s / sqrt(P/rho) = dimensionless

    double drag = 0; // placeholder: replace with interpolation or formula

    // Drag proportional to air density (example scaling)
    drag *= density / 0.0025845; // adjust 0.0025845 if using SI

    if (drag <= 0) {
        return 0;
    }
    return drag; // drag in Newtons
}

double predict_apogee(double flap_angle, double altitude, double velocity){
    double dt = 0.5;

    double apogee_prediction = altitude;
    double velocity_current = velocity;

    // Assuming flap_angle, VEHICLE_MASS, and G are defined in scope

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

        apogee_prediction += (1 / 6) * (kx1 + 2 * kx2 + 2 * kx3 + kx4) * dt;
        velocity_current += (1 / 6) * (kp1 + 2 * kp2 + 2 * kp3 + kp4) * dt;
    }
    return apogee_prediction;



}


