#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <cstdlib>

// Constants for Huntsville, AL (late April)
const double GROUND_TEMPERATURE_C = 15;   // °C
const double GROUND_PRESSURE_PA = 101325.0; // HPA (~1013 mb)
const double GRAVITY = 9.80665;             // m/s²
const double R_AIR = 287.058;               // J/(kg·K)
const double LAPSE_RATE = 0.0065;           // °C per meter
const double VEHICLE_MASS = 21.912; // kg at BURNOUT

// Rows: Flap Angles (0, 7.5, 15, 22.5, 30, 37.5, 45)
// Cols: Mach Numbers (0.0, 0.18, 0.36, 0.54)
const double CD_TABLE[7][4] = {
    {0.0, 0.329, 0.311, 0.319}, // 0.0°  Flaps
    {0.0, 0.337, 0.318, 0.425}, // 7.5°  Flaps (Corrected 3.2 typo)
    {0.0, 0.394, 0.372, 0.395}, // 15.0° Flaps
    {0.0, 0.477, 0.455, 0.491}, // 22.5° Flaps
    {0.0, 0.559, 0.569, 0.599}, // 30.0° Flaps
    {0.0, 0.535, 0.530, 0.572}, // 37.5° Flaps
    {0.0, 0.565, 0.562, 0.625}  // 45.0° Flaps
};

// Supporting axis arrays for interpolation logic
const double FLAP_AXIS[7] = {0.0, 7.5, 15.0, 22.5, 30.0, 37.5, 45.0};
const double MACH_AXIS[4] = {0.0, 0.18, 0.36, 0.54};

//Interpolation function for drag coefficient
double get_interpolated_cd(double flap, double mach) {
    // 1. Clamp inputs to table bounds
    flap = std::max(0.0, std::min(45.0, flap));
    mach = std::max(0.0, std::min(0.54, mach));

    // 2. Find the bounding indices for Flap
    int i = 0;
    while (i < 5 && flap > FLAP_AXIS[i + 1]) i++;
    
    // 3. Find the bounding indices for Mach
    int j = 0;
    while (j < 2 && mach > MACH_AXIS[j + 1]) j++;

    // 4. Calculate local weights (0.0 to 1.0)
    double x_weight = (flap - FLAP_AXIS[i]) / (FLAP_AXIS[i+1] - FLAP_AXIS[i]);
    double y_weight = (mach - MACH_AXIS[j]) / (MACH_AXIS[j+1] - MACH_AXIS[j]);

    // 5. Bilinear interpolation formula
    double c00 = CD_TABLE[i][j];
    double c10 = CD_TABLE[i+1][j];
    double c01 = CD_TABLE[i][j+1];
    double c11 = CD_TABLE[i+1][j+1];

    double cd = c00 * (1 - x_weight) * (1 - y_weight) +
                c10 * x_weight * (1 - y_weight) +
                c01 * (1 - x_weight) * y_weight +
                c11 * x_weight * y_weight;

    return cd;
}

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

double get_interpolated_area(double flap) {
    flap = std::max(0.0, std::min(45.0, flap));
    int i = static_cast<int>(flap);
    if (i >= 45) return AREA_TABLE[45];
    
    // Linear interpolation between integer degrees
    double weight = flap - i;
    return AREA_TABLE[i] * (1 - weight) + AREA_TABLE[i + 1] * weight;
}


// Temperature at a given altitude (meters)
double atmosphere_temperature(double altitude_m) {
    return GROUND_TEMPERATURE_C - LAPSE_RATE * altitude_m;
}

// Update these to base SI units
const double GROUND_TEMPERATURE_K = GROUND_TEMPERATURE_C + 273.15; // 303.15 K

double atmosphere_temperature_k(double altitude_m) {
    return GROUND_TEMPERATURE_K - (LAPSE_RATE * altitude_m);
}

double atmosphere_pressure_pa(double altitude_m) {
    double Tk = atmosphere_temperature_k(altitude_m);
    // Barometric formula using Kelvin ratio
    return GROUND_PRESSURE_PA * std::pow(Tk / GROUND_TEMPERATURE_K, GRAVITY / (R_AIR * LAPSE_RATE));
}

double calculate_drag(double flap_angle, double altitude, double velocity) {
    double rho = atmosphere_pressure_pa(altitude) / (R_AIR * atmosphere_temperature_k(altitude));
    
    // Mach calculation (Speed of sound = sqrt(gamma * R * T))
    double speed_of_sound = std::sqrt(1.4 * R_AIR * atmosphere_temperature_k(altitude));
    double mach_number = std::abs(velocity) / speed_of_sound;

    double Cd = get_interpolated_cd(flap_angle, mach_number);
    double Area = get_interpolated_area(flap_angle);

    return 0.5 * rho * std::pow(velocity, 2) * Area * Cd;
}

double predict_apogee(double flap_angle, double altitude, double velocity){
    double dt = 0.01;
    double weight = 1.0 / 6.0;
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

        apogee_prediction += weight * (kx1 + 2 * kx2 + 2 * kx3 + kx4) * dt;
        velocity_current += weight * (kp1 + 2 * kp2 + 2 * kp3 + kp4) * dt;
    }
    return apogee_prediction;

}

struct PIDState {
    double integral = 0;
    double prev_error = 0;
    double last_time = 0;
};

// Global PID Constants - These will need "Tuning"
const double Kp = 0.408;  // Adjusts how aggressively we react to error
const double Ki = 0.127;  // Corrects steady-state offset over time
const double Kd = 0.11;  // Prevents jitter and over-correction

double calculate_control_effort(double current_alt, double current_vel, double target_apogee, double dt, PIDState &state) {
    
    // 1. Get current prediction with flaps at 0 (or current angle)
    double predicted = predict_apogee(0.0, current_alt, current_vel);
    
    // 2. Calculate Error
    double error = predicted - target_apogee;

    // 3. Proportional term
    double P_out = Kp * error;

    // 4. Integral term (with wind-up protection)
    state.integral += error * dt;
    // Clamp integral so it doesn't grow infinitely if brakes are maxed out
    state.integral = std::max(-50.0, std::min(50.0, state.integral)); 
    double I_out = Ki * state.integral;

    // 5. Derivative term (change in error)
    double derivative = (error - state.prev_error) / dt;
    double D_out = Kd * derivative;

    state.prev_error = error;

    // 6. Total Output (Flap Angle in degrees)
    double output_angle = P_out + I_out + D_out;

    // 7. Physical Clamping (Brakes can't go negative or past 45°)
    if (output_angle < 0) output_angle = 0;
    if (output_angle > 45) output_angle = 45;

    return output_angle;
}

// Mapping from Flap Degree (Index 0-45) to Servo Rotation Degree
const double CAM_SERVO_TABLE[46] = {
    0.00, 0.47, 0.94, 1.41, 1.88, 2.35, 2.83, 3.29, 3.77, 4.25,
    4.73, 5.21, 5.69, 6.17, 6.65, 7.13, 7.61, 8.10, 8.58, 9.06,
    9.54, 10.02, 10.50, 10.98, 11.46, 11.94, 12.41, 12.89, 13.36, 13.83,
    14.30, 14.77, 15.24, 15.70, 16.16, 16.62, 17.08, 17.53, 17.98, 18.43,
    18.88, 19.32, 19.75, 20.19, 20.62, 21.04
};

double get_servo_angle_from_cam(double flap_angle) {
    flap_angle = std::max(0.0, std::min(45.0, flap_angle));
    int i = static_cast<int>(flap_angle);
    if (i >= 45) return CAM_SERVO_TABLE[45];
    
    // Linear interpolation between the discrete degrees
    double weight = flap_angle - i;
    return CAM_SERVO_TABLE[i] * (1.0 - weight) + CAM_SERVO_TABLE[i+1] * weight;
}

int main() {
    // --- Initial Conditions from your OpenRocket Data ---
    double altitude = 275.877;     
    double velocity = 186.717;     
    double target_apogee = 1341.12; 
    double dt = 0.05;            // Loop frequency (20Hz)
    
    // Servo limits
    const double MAX_FLAP_SPEED_DEG_PER_SEC = 285.71; 
    
    PIDState airbrake_pid;
    
    // Declare BOTH variables before the loop so we can log them at t=0
    double actual_flap_angle = 0.0;   // Where the physical flaps are
    double desired_flap_angle = 0.0;  // What the PID is asking for

    // --- 1. Open the CSV File ---
    std::ofstream log_file("sitl_flight_log.csv");
    if (!log_file.is_open()) {
        std::cerr << "Error: Could not open sitl_flight_log.csv for writing!\n";
        return 1;
    }

    // --- 2. Write the CSV Headers ---
    log_file << "Time(s),Alt(m),Vel(m/s),Unbraked_Pred(m),PID_Wants(deg),Actual_Flap(deg),Servo_Cmd(deg)\n";
    
    log_file << std::fixed << std::setprecision(3);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Starting SITL simulation... Logging to 'sitl_flight_log.csv'\n";

    // --- 3. Simulation Loop ---
    for (double t = 0; t < 20.0; t += dt) {
        
        // 1. Prediction for LOGGING (What happens if we close the flaps right now?)
        double unbraked_pred = predict_apogee(0.0, altitude, velocity);

        // 2. Hardware Output calculation (for logging current physical state)
        double servo_cmd_angle = get_servo_angle_from_cam(actual_flap_angle);

        // 3. LOG DATA FIRST (so t=0 matches your actual starting variables!)
        log_file << t << "," 
                 << altitude << "," 
                 << velocity << "," 
                 << unbraked_pred << "," 
                 << desired_flap_angle << "," 
                 << actual_flap_angle << "," 
                 << servo_cmd_angle << "\n";

        // Print a simplified version to the console twice a second
        if (std::fmod(t, 0.5) < dt) { 
            std::cout << "T: " << t << "s | Alt: " << altitude << "m | Vel: " << velocity << "m/s | Pred: " << unbraked_pred << "m\n";
        }

        // Stop if we hit apogee
        if (velocity <= 0) break;

        // 4. Controller: What does the PID *want* to do for the NEXT step?
        desired_flap_angle = calculate_control_effort(altitude, velocity, target_apogee, dt, airbrake_pid);

        // 5. Hardware Actuation: Move physical flaps (Slew Rate)
        double max_change = MAX_FLAP_SPEED_DEG_PER_SEC * dt;
        double diff = desired_flap_angle - actual_flap_angle;
        actual_flap_angle += std::max(-max_change, std::min(max_change, diff));
        actual_flap_angle = std::max(0.0, std::min(45.0, actual_flap_angle));

        // 6. Physics: Air reacts to ACTUAL flap angle
        double drag_force = calculate_drag(actual_flap_angle, altitude, velocity);
        double acceleration = -GRAVITY - (drag_force / VEHICLE_MASS);

        // 7. Integration: Move the rocket for the NEXT loop
        velocity += acceleration * dt;
        altitude += velocity * dt;
    }

    log_file.close();
    
    double error = (target_apogee - altitude);
    double error_percentage = (error / target_apogee) * 100;

    std::string error_str = error < 0 ? "+" + std::to_string(error * -1) : "-" + std::to_string(error); // Overshot becomes positive error, undershot becomes negative error
    std::string error_percentage_str = error_percentage < 0 ? "+" + std::to_string(error_percentage * -1) : "-" + std::to_string(error_percentage); 


    std::cout << "--- FINAL APOGEE: " << altitude << " m ---\n";
    std::cout << "Target Apogee was: " << target_apogee << " m\n";
    std::cout << "Error: " << error_str << " m\n";
    std::cout << "Error Percentage: " << error_percentage_str << " %\n";
    std::cout << "Simulation complete. Check 'sitl_flight_log.csv' for full data.\n";
    
    // --- NEW: Automatically run the Python plotting script ---
    std::cout << "Generating graph...\n";
    
    // std::system executes a command exactly as if you typed it in the Linux terminal
    int result = std::system("source .venv/bin/activate && python3 plot_log.py");
    
    if (result != 0) {
        std::cerr << "Warning: Failed to run plotting script. Ensure Python 3, Pandas, and Matplotlib are installed.\n";
    }
    
    return 0;
}