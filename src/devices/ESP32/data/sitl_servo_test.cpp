#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <chrono>

#include "physics_engine.h"
#include "pid_controller.h"
#include "ServoControllerI2C.h" // Hardware driver

// TARGET APOGEE (in meters)
const double TARGET_APOGEE = 1341.12; 

// TAKEN FROM LATEST OPENROCKET SIMULATION BEST CASE (2026-01-30)
const double BURNOUT_ALTITUDE = 275.877; 
const double BURNOUT_VELOCITY = 186.717; 

// Forward Mapping: Flap -> Servo
double get_servo_angle_from_cam(double flap_angle) {
    flap_angle = std::max(0.0, std::min(45.0, flap_angle));
    int i = static_cast<int>(flap_angle);
    if (i >= 45) return CAM_SERVO_TABLE[45];
    
    double weight = flap_angle - i;
    return CAM_SERVO_TABLE[i] * (1.0 - weight) + CAM_SERVO_TABLE[i+1] * weight;
}

// Inverse Mapping: Servo -> Flap
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

    // --- Hardware Setup ---
    ServoControllerI2C servo(1, 0x40);
    const int AIRBRAKE_CHANNEL = 0;
    servo.rotate_cam(AIRBRAKE_CHANNEL, 0.0); // Start stowed

    // --- Initial Conditions ---
    double altitude = BURNOUT_ALTITUDE;     
    double velocity = BURNOUT_VELOCITY;     
    double target_apogee = TARGET_APOGEE;
    double dt = 0.05; 
    
    PIDState airbrake_pid = {0.0, 0.0, 0.0};

    double actual_servo_angle = 0.0;
    double actual_flap_angle = 0.0;

    // Ramp Deployment Variables
    double RAMP_DURATION = 1.0; // Time to ramp from 0 to full control effort

    // --- Open CSV ---
    std::ofstream log_file("sitl_flight_log.csv");
    log_file << "Time(s),Alt(m),Vel(m/s),Unbraked_Pred(m),PID_Wants(deg),Actual_Flap(deg),Actual_Servo(deg)\n";
    log_file << std::fixed << std::setprecision(3);
    
    std::cout << "Starting SITL + REAL SERVO simulation in 3 seconds...\n";
    std::cout << "Watch the hardware!\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Real-time clock enforcer
    auto next_loop_time = std::chrono::steady_clock::now();

    // --- Simulation Loop ---
    for (double t = 0; t < 20.0; t += dt) {
        
        // A. Prediction
        double unbraked_pred = predict_apogee(0.0, altitude, velocity);

        // B. Controller Logic
        double desired_flap_angle = calculate_control_effort(altitude, velocity, target_apogee, dt, airbrake_pid);

        // Ramp factor for gradual deployment (prevents shock to the system at t=0)
        double ramp_factor = std::min(1.0, t / RAMP_DURATION);
        double max_control_effort = 45.0 * ramp_factor; // Linearly ramp from 0 to 45 degrees over RAMP_DURATION seconds
        desired_flap_angle = std::max(0.0, std::min(max_control_effort, desired_flap_angle));

        // C. Hardware Mapping
        double desired_servo_angle = get_servo_angle_from_cam(desired_flap_angle);

        // D. Hardware Actuation (Slew Rate Limit)
        actual_servo_angle = slew_rate_limiter(desired_servo_angle, actual_servo_angle, dt);

        // *** E. FIRE REAL PHYSICAL SERVO ***
        servo.rotate_cam(AIRBRAKE_CHANNEL, actual_servo_angle);

        // F. Mechanical Feedback
        actual_flap_angle = get_flap_angle_from_servo(actual_servo_angle);

        // G. Logging
        log_file << t << "," << altitude << "," << velocity << "," 
                 << unbraked_pred << "," << desired_flap_angle << "," 
                 << actual_flap_angle << "," << actual_servo_angle << "\n";

        // Print Telemetry (overwrite same line for clean terminal)
        std::cout << "\rT: " << std::fixed << std::setprecision(2) << t 
                  << "s | Alt: " << altitude 
                  << "m | Flap: " << actual_flap_angle 
                  << "deg | Servo: " << actual_servo_angle << "deg   " << std::flush;

        if (velocity <= 0) break; 

        // H. Physics Integration
        double drag_force = calculate_drag(actual_flap_angle, altitude, velocity);

        double noise_intensity = 0.02; 
        double noise_factor = 1.0 + ((static_cast<double>(std::rand()) / RAND_MAX) * (noise_intensity * 2.0) - noise_intensity);
        drag_force *= noise_factor;
        drag_force *= 1.00;

        double acceleration = -GRAVITY - (drag_force / VEHICLE_MASS);
        velocity += acceleration * dt;
        altitude += velocity * dt;

        // *** I. REAL-TIME ENFORCEMENT ***
        next_loop_time += std::chrono::milliseconds(50);
        std::this_thread::sleep_until(next_loop_time);
    }

    log_file.close();
    servo.rotate_cam(AIRBRAKE_CHANNEL, 0.0); // Stow flaps at apogee
    
    double error = (altitude - target_apogee);
    std::cout << "\n\n--- FINAL APOGEE: " << altitude << " m ---\n";
    std::cout << "Target: " << target_apogee << " m | Error: " << error << " m\n";

    std::cout << "Running plot_log.py...\n";
    system("python3 plot_log.py");

    return 0;
}