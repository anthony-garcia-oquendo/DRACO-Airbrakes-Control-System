#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>

// Custom Hardware Drivers
#include "BMP390.h"
#include "ICM20948.h"
#include "ServoControllerI2C.h"

// Math & Logic
#include "KalmanFilter.h"
#include "physics_engine.h"
#include "pid_controller.h"

// --- Flight States ---
enum FlightState {
    ON_PAD = 0,
    BOOST = 1,
    COAST = 2,
    DESCENT = 3
};

// Forward Mapping: Flap -> Servo
double get_servo_angle_from_cam(double flap_angle) {
    flap_angle = std::max(0.0, std::min(45.0, flap_angle));
    int i = static_cast<int>(flap_angle);
    if (i >= 45) return CAM_SERVO_TABLE[45];
    
    double weight = flap_angle - i;
    return CAM_SERVO_TABLE[i] * (1.0 - weight) + CAM_SERVO_TABLE[i+1] * weight;
}

// --- Helper for Zeroing the Barometer ---
double calculate_launchpad_zero(BMP390& baro, int samples = 100) {
    std::cout << "[SYSTEM] Zeroing Barometer. Do not touch...\n";
    double sum = 0.0;
    for (int i = 0; i < samples; ++i) {
        sum += baro.get_altitude();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    double baseline = sum / samples;
    std::cout << "[SYSTEM] Launchpad Baseline Locked: " << baseline << " m MSL\n";
    return baseline;
}

int main() {
    std::cout << "====================================\n";
    std::cout << "   ACTIVE DRAG FLIGHT COMPUTER V1   \n";
    std::cout << "====================================\n\n";

    // 1. Initialize Hardware
    BMP390 baro(1, 0x77);
    ICM20948 imu(1, 0x69);
    ServoControllerI2C servo(1, 0x40);
    const int AIRBRAKE_CHANNEL = 0;

    if (!baro.initialize() || !imu.initialize()) {
        std::cerr << "[FATAL] Sensor initialization failed. Aborting.\n";
        return 1;
    }

    // 2. Open the "Black Box" Log File
    std::ofstream log_file("hitl_flight_log.csv");
    if (!log_file.is_open()) {
        std::cerr << "[FATAL] Could not create flight log on SD card!\n";
        return 1;
    }
    
    // Write CSV Headers
    log_file << "Time(s),State,Raw_AGL(m),Accel_Z(m/s2),KF_Alt(m),KF_Vel(m/s),Cmd_Flap(deg),Cmd_Servo(deg)\n";
    log_file << std::fixed << std::setprecision(3); // Log data with 3 decimal places

    // 3. Initialize Math & States
    double launchpad_msl = calculate_launchpad_zero(baro);
    KalmanFilter kf(0.0, 0.0);
    PIDState pid = {0.0, 0.0, 0.0};
    
    FlightState current_state = ON_PAD;
    const double TARGET_APOGEE = 1341.12; 
    const double LOOP_DT = 0.05; 

    // Actuator setup variables
    double flap_cmd = 0.0;
    double servo_angle = 0.0;

    servo.rotate(AIRBRAKE_CHANNEL, 0.0);
    std::cout << "[SYSTEM] Flaps Stowed. Ready for Launch.\n";

    // Start a master clock to track actual flight time
    auto flight_start_time = std::chrono::steady_clock::now();
    auto next_loop_time = flight_start_time;

    // 4. Main Flight Loop
    while (true) {
        // Calculate exact time elapsed since boot
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - flight_start_time).count();

        // --- A. READ SENSORS ---
        double current_agl = baro.get_altitude() - launchpad_msl;
        double accel_z = imu.get_accel_z(); 

        // --- B. UPDATE KALMAN FILTER ---
        kf.predict(accel_z, LOOP_DT);
        kf.update(current_agl);

        double kf_alt = kf.get_altitude();
        double kf_vel = kf.get_velocity();

        // Reset control values to 0 each loop unless actively commanded in COAST
        flap_cmd = 0.0;
        servo_angle = 0.0;

        // --- C. STATE MACHINE LOGIC ---
        switch (current_state) {
            case ON_PAD:
                if (accel_z > 20.0) {
                    current_state = BOOST;
                    std::cout << "\n[FLIGHT] LIFTOFF DETECTED! Transition to BOOST.\n";
                }
                break;

            case BOOST:
                if (accel_z < 0.0 && kf_vel > 50.0) {
                    current_state = COAST;
                    std::cout << "\n[FLIGHT] BURNOUT DETECTED! ACTIVE CONTROL ENGAGED.\n";
                }
                break;

            case COAST:
                if (kf_vel < 0.0) {
                    current_state = DESCENT;
                    servo.rotate(AIRBRAKE_CHANNEL, 0.0);
                    std::cout << "\n[FLIGHT] APOGEE DETECTED! Apogee: " << kf_alt << " m\n";
                } else {
                    flap_cmd = calculate_control_effort(kf_alt, kf_vel, TARGET_APOGEE, LOOP_DT, pid);
                    servo_angle = get_servo_angle_from_cam(flap_cmd);
                    servo.rotate(AIRBRAKE_CHANNEL, servo_angle);

                    std::cout << "\rAlt: " << std::fixed << std::setprecision(1) << kf_alt << "m | "
                              << "Vel: " << kf_vel << "m/s | "
                              << "Flap: " << flap_cmd << "deg   " << std::flush;
                }
                break;

            case DESCENT:
                if (kf_alt < 10.0 && kf_vel > -2.0) {
                    std::cout << "\n[SYSTEM] Touchdown detected. Closing log and shutting down.\n";
                    log_file.close(); // Safely close the file
                    return 0; 
                }
                break;
        }

        // --- D. LOGGING (The Black Box) ---
        // We log everything, regardless of the flight state
        log_file << t << "," 
                 << current_state << "," 
                 << current_agl << "," 
                 << accel_z << "," 
                 << kf_alt << "," 
                 << kf_vel << "," 
                 << flap_cmd << "," 
                 << servo_angle << "\n";
        
        // CRITICAL: Force write to SD card immediately
        log_file << std::flush; 

        // --- E. TIMING ENFORCEMENT (20Hz) ---
        next_loop_time += std::chrono::milliseconds(50);
        std::this_thread::sleep_until(next_loop_time);
    }

    return 0;
}