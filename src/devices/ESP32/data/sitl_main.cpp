#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <algorithm>

#include "physics_engine.h"
#include "pid_controller.h"

// Mapping from Flap Degree (Index 0-45) to Servo Rotation Degree
// Note: This table is here for simulation visualization/logging purposes
// In HITL, this would drive the actual servo.
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
    
    double weight = flap_angle - i;
    return CAM_SERVO_TABLE[i] * (1.0 - weight) + CAM_SERVO_TABLE[i+1] * weight;
}

int main() {
    // --- Initial Conditions ---
    double altitude = 275.877;     
    double velocity = 186.717;     
    double target_apogee = 1341.12; 
    double dt = 0.05; // 20Hz
    
    PIDState airbrake_pid;
    
    // State variables
    double actual_flap_angle = 0.0;
    double desired_flap_angle = 0.0;

    // --- 1. Open CSV ---
    std::ofstream log_file("sitl_flight_log.csv");
    if (!log_file.is_open()) {
        std::cerr << "Error: Could not open sitl_flight_log.csv for writing!\n";
        return 1;
    }

    log_file << "Time(s),Alt(m),Vel(m/s),Unbraked_Pred(m),PID_Wants(deg),Actual_Flap(deg),Servo_Cmd(deg)\n";
    
    log_file << std::fixed << std::setprecision(3);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Starting SITL simulation...\n";

    // --- 2. Simulation Loop ---
    for (double t = 0; t < 20.0; t += dt) {
        
        // Log Data (Hardware Output calculation for log)
        double unbraked_pred = predict_apogee(0.0, altitude, velocity);
        double servo_cmd_angle = get_servo_angle_from_cam(actual_flap_angle);

        log_file << t << "," 
                 << altitude << "," 
                 << velocity << "," 
                 << unbraked_pred << "," 
                 << desired_flap_angle << "," 
                 << actual_flap_angle << "," 
                 << servo_cmd_angle << "\n";

        if (std::fmod(t, 0.5) < dt) { 
            std::cout << "T: " << t << "s | Alt: " << altitude << "m | Vel: " << velocity << "m/s | Pred: " << unbraked_pred << "m\n";
        }

        if (velocity <= 0) break;

        // Controller
        desired_flap_angle = calculate_control_effort(altitude, velocity, target_apogee, dt, airbrake_pid);

        // Hardware Actuation (Slew Rate Limiting)
        actual_flap_angle = slew_rate_limiter(desired_flap_angle, actual_flap_angle, dt);

        // Physics Integration
        double drag_force = calculate_drag(actual_flap_angle, altitude, velocity);
        double acceleration = -GRAVITY - (drag_force / VEHICLE_MASS);

        velocity += acceleration * dt;
        altitude += velocity * dt;
    }

    log_file.close();
    
    double error = (target_apogee - altitude);
    std::cout << "--- FINAL APOGEE: " << altitude << " m ---\n";
    std::cout << "Target: " << target_apogee << " m | Error: " << error << " m\n" << "Percentage Error: " << (error / target_apogee) * 100 << " %\n";

    // --- 3. Run Plot Script ---
    std::cout << "Running plot_log.py...\n";
    int result = system("python3 plot_log.py");
    if (result != 0) {
         // Fallback if python3 is not found or fails
         system("source .venv/bin/activate && python plot_log.py");
    }

    return 0;
}
