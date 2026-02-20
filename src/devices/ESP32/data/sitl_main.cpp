#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <cstdlib>
#include <ctime>

#include "physics_engine.h"
#include "pid_controller.h"

// TARGET APOGEE (in meters) - This was the target discussed in the latest report, based on the best case from OpenRocket simulations. 
const double TARGET_APOGEE = 1341.12; // 4400 ft in meters

// TAKEN FROM LATEST OPENROCKET SIMULATION BEST CASE (2026-01-30)
const double BURNOUT_ALTITUDE = 275.877; // Altitude at motor burnout in meters
const double BURNOUT_VELOCITY = 186.717; // Velocity at motor burnout in m/s



// Mapping from Flap Degree (Index 0-45) to Servo Rotation Degree
const double CAM_SERVO_TABLE[46] = {
    0.00, 0.47, 0.94, 1.41, 1.88, 2.35, 2.83, 3.29, 3.77, 4.25,
    4.73, 5.21, 5.69, 6.17, 6.65, 7.13, 7.61, 8.10, 8.58, 9.06,
    9.54, 10.02, 10.50, 10.98, 11.46, 11.94, 12.41, 12.89, 13.36, 13.83,
    14.30, 14.77, 15.24, 15.70, 16.16, 16.62, 17.08, 17.53, 17.98, 18.43,
    18.88, 19.32, 19.75, 20.19, 20.62, 21.04
};

// Forward Mapping: Flap -> Servo
double get_servo_angle_from_cam(double flap_angle) {
    flap_angle = std::max(0.0, std::min(45.0, flap_angle));
    int i = static_cast<int>(flap_angle);
    if (i >= 45) return CAM_SERVO_TABLE[45];
    
    double weight = flap_angle - i;
    return CAM_SERVO_TABLE[i] * (1.0 - weight) + CAM_SERVO_TABLE[i+1] * weight;
}

// Inverse Mapping: Servo -> Flap (Linear interpolation search)
double get_flap_angle_from_servo(double servo_angle) {
    servo_angle = std::max(0.0, std::min(21.04, servo_angle));
    for (int i = 0; i < 45; ++i) {
        if (servo_angle >= CAM_SERVO_TABLE[i] && servo_angle <= CAM_SERVO_TABLE[i+1]) {
            double range = CAM_SERVO_TABLE[i+1] - CAM_SERVO_TABLE[i];
            double weight = (servo_angle - CAM_SERVO_TABLE[i]) / range;
            return static_cast<double>(i) + weight;
        }
    }
    return 45.0;
}

int main() {
    std::srand(std::time(0)); 

    // --- Initial Conditions (Burnout State) ---
    double altitude = BURNOUT_ALTITUDE;     
    double velocity = BURNOUT_VELOCITY;     
    double target_apogee = TARGET_APOGEE;
    double dt = 0.05; // 20Hz Loop
    
    PIDState airbrake_pid;
    
    // Physical State of Hardware
    double actual_servo_angle = 0.0;
    double actual_flap_angle = 0.0;

    // --- 1. Open CSV ---
    std::ofstream log_file("sitl_flight_log.csv");
    if (!log_file.is_open()) {
        std::cerr << "Error: Could not open sitl_flight_log.csv for writing!\n";
        return 1;
    }

    log_file << "Time(s),Alt(m),Vel(m/s),Unbraked_Pred(m),PID_Wants(deg),Actual_Flap(deg),Actual_Servo(deg)\n";
    
    log_file << std::fixed << std::setprecision(3);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Starting SITL simulation (Servo-Limited Logic)...\n";

    // --- 2. Simulation Loop ---
    for (double t = 0; t < 20.0; t += dt) {
        
        // A. Prediction (What we think will happen if we do nothing)
        double unbraked_pred = predict_apogee(0.0, altitude, velocity);

        // B. Controller Logic (Decision)
        double desired_flap_angle = calculate_control_effort(altitude, velocity, target_apogee, dt, airbrake_pid);

        // C. Hardware Mapping (Decision -> Motor Command)
        double desired_servo_angle = get_servo_angle_from_cam(desired_flap_angle);

        // D. Hardware Actuation (Slew Rate Limit applied to the MOTOR)
        // Servo speed is roughly 285 deg/s
        actual_servo_angle = slew_rate_limiter(desired_servo_angle, actual_servo_angle, dt);

        // E. Mechanical Feedback (Motor Pos -> Flap Pos)
        actual_flap_angle = get_flap_angle_from_servo(actual_servo_angle);

        // Log state BEFORE physics integration
        log_file << t << "," 
                 << altitude << "," 
                 << velocity << "," 
                 << unbraked_pred << "," 
                 << desired_flap_angle << "," 
                 << actual_flap_angle << "," 
                 << actual_servo_angle << "\n";

        if (std::fmod(t, 0.5) < dt) { 
            std::cout << "T: " << t << "s | Alt: " << altitude << "m | Pred: " << unbraked_pred 
                      << "m | Flap: " << actual_flap_angle << " deg\n";
        }

        if (velocity <= 0) break; // Apogee reached

        // F. Physics Integration
        double drag_force = calculate_drag(actual_flap_angle, altitude, velocity);

        //Noise Stress Test to simulate real-world variability (e.g., wind gusts, sensor noise)
        double noise_intensity = 0.02; // Set to 0 for no noise, increase for more variability 
        double noise_factor = 1.0 + ((static_cast<double>(std::rand()) / RAND_MAX) * (noise_intensity * 2.0) - noise_intensity);
        drag_force *= noise_factor;

        // Introduce bias to test robustness (e.g., miscalibrated drag coefficient). Default is 1.0 (no bias).
        drag_force *= 1.00;

        // Update physics
        double acceleration = -GRAVITY - (drag_force / VEHICLE_MASS);
        velocity += acceleration * dt;
        altitude += velocity * dt;
    }

    log_file.close();
    
    double error = (altitude - target_apogee);
    std::cout << "\n--- FINAL APOGEE: " << altitude << " m ---\n";
    std::cout << "Target: " << target_apogee << " m | Error: " << error << " m\n";
    std::cout << "Percentage Error: " << (error / target_apogee) * 100 << " %\n";

    // --- 3. Run Plot Script ---
    std::cout << "Running plot_log.py...\n";
    system("python3 plot_log.py");

    return 0;
}