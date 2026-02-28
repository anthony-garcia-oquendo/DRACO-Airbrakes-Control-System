#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include <random>   // Added for noise generation
#include <algorithm>
#include <cstdlib>  // Added for system() call

#include "KalmanFilter.h"

// --- TEST CONFIGURATION ---
enum TestMode {
    CLEAN_DATA = 0,       // Pure CSV data (Ideal conditions)
    JITTER_ONLY = 1,      // Constant high-frequency noise (Sensor vibration)
    JUMPS_AND_JITTER = 2  // Noise + random massive spikes (Hardware glitches)
};

struct DataPoint {
    double time;
    double alt;
    double vel;
    double accel;
};

int main() {
    // =========================================================
    // CONFIGURATION: Change this flag to switch test scenarios
    // =========================================================
    TestMode current_mode = JUMPS_AND_JITTER; 
    // =========================================================

    std::cout << "=========================================================\n";
    std::cout << "            KALMAN FILTER SIL DATA REPLAY                \n";
    std::cout << "            MODE: " << (current_mode == 0 ? "CLEAN" : (current_mode == 1 ? "JITTER" : "JUMPS+JITTER")) << "\n";
    std::cout << "=========================================================\n\n";

    std::ifstream file("latest_rocket.csv");
    if (!file.is_open()) {
        std::cerr << "[FATAL] Could not open latest_rocket.csv\n";
        return 1;
    }

    std::string line;
    std::vector<DataPoint> truth_data;

    // Parse the CSV file
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string token;
        DataPoint dp;

        try {
            std::getline(ss, token, ','); dp.time = std::stod(token);
            std::getline(ss, token, ','); dp.alt = std::stod(token);
            std::getline(ss, token, ','); dp.vel = std::stod(token);
            std::getline(ss, token, ','); dp.accel = std::stod(token);
            truth_data.push_back(dp);
        } catch (...) {
            continue; 
        }
    }
    file.close();

    if (truth_data.empty()) {
        std::cerr << "[FATAL] No valid data found in CSV!\n";
        return 1;
    }

    // --- SETUP LOGGING ---
    std::ofstream log_file("kalman_test_log.csv");
    if (!log_file.is_open()) {
        std::cerr << "[FATAL] Could not create kalman_test_log.csv\n";
        return 1;
    }
    log_file << "Time,True_Alt,Noisy_Alt,KF_Alt,True_Vel,KF_Vel,True_Accel,Status\n";
    log_file << std::fixed << std::setprecision(4);

    // Setup Randomness Generators
    std::random_device rd;
    std::mt19937 gen(rd());
    // Distribution for jitter (Gaussian noise with mean 0 and std dev 1.5)
    std::normal_distribution<double> jitter_dist(0.0, 40.0);
    // Distribution for massive jumps (±25 meters)
    std::uniform_real_distribution<double> jump_dist(-300.0, 300.0);
    // Dice roll for glitch probability (5% chance)
    std::uniform_real_distribution<double> glitch_chance(0.0, 1.0);

    std::cout << "[SYSTEM] Loaded " << truth_data.size() << " data points.\n";
    std::cout << "TIME(s) | TRUE ALT(m) | KF ALT(m) | ALT ERR | TRUE VEL(m/s) | KF VEL(m/s) | VEL ERR | STATUS\n";
    std::cout << "--------------------------------------------------------------------------------------------\n";

    KalmanFilter kf(truth_data[0].alt, truth_data[0].vel);
    double prev_time = truth_data[0].time;
    
    double alt_error_sum = 0.0;
    double alt_true_sum = 0.0;
    double vel_error_sum = 0.0;
    double vel_true_sum = 0.0;

    std::cout << std::fixed << std::setprecision(2);

    for (size_t i = 1; i < truth_data.size(); ++i) {
        double dt = truth_data[i].time - prev_time;
        if (dt <= 0) continue; 
        
        // 1. Prepare the sensor input (Start with truth)
        double noisy_sensor_alt = truth_data[i].alt;
        std::string status = "OK";

        // 2. Inject Noise based on Mode
        if (current_mode >= JITTER_ONLY) {
            noisy_sensor_alt += jitter_dist(gen);
        }

        if (current_mode == JUMPS_AND_JITTER) {
            if (glitch_chance(gen) < 0.05) { // 5% chance of a jump
                noisy_sensor_alt += jump_dist(gen);
                status = "!!! JUMP !!!";
            }
        }

        // 3. Run Kalman Filter Prediction & Update
        kf.predict(truth_data[i].accel, dt);
        kf.update(noisy_sensor_alt);
        
        double kf_alt = kf.get_altitude();
        double kf_vel = kf.get_velocity();
        
        // Error is calculated against the original TRUTH from CSV, not the noisy input
        double alt_err = std::abs(truth_data[i].alt - kf_alt);
        double vel_err = std::abs(truth_data[i].vel - kf_vel);

        // 4. Console Logging
        std::cout << std::setw(7)  << truth_data[i].time << " | "
                  << std::setw(11) << truth_data[i].alt << " | "
                  << std::setw(9)  << kf_alt << " | "
                  << std::setw(7)  << alt_err << " | "
                  << std::setw(13) << truth_data[i].vel << " | "
                  << std::setw(11) << kf_vel << " | "
                  << std::setw(7)  << vel_err << " | "
                  << status << "\n";

        // 5. CSV Logging
        log_file << truth_data[i].time << ","
                 << truth_data[i].alt << ","
                 << noisy_sensor_alt << ","
                 << kf_alt << ","
                 << truth_data[i].vel << ","
                 << kf_vel << ","
                 << truth_data[i].accel << ","
                 << status << "\n";

        alt_error_sum += alt_err;
        alt_true_sum += std::abs(truth_data[i].alt);
        vel_error_sum += vel_err;
        vel_true_sum += std::abs(truth_data[i].vel);
        
        prev_time = truth_data[i].time;
    }
    
    log_file.close();

    double alt_error_pct = (alt_true_sum > 0) ? (alt_error_sum / alt_true_sum) * 100.0 : 0.0;
    double vel_error_pct = (vel_true_sum > 0) ? (vel_error_sum / vel_true_sum) * 100.0 : 0.0;

    std::cout << "\n====================================\n";
    std::cout << "         FINAL TEST RESULTS         \n";
    std::cout << "====================================\n";
    std::cout << "Altitude Mean Error: " << alt_error_pct << " %\n";
    std::cout << "Velocity Mean Error: " << vel_error_pct << " %\n\n";

    // --- RUN PYTHON SCRIPT ---
    std::cout << "[SYSTEM] Running plot_kalman.py to generate graph...\n";
    system("python3 plot_kalman.py");

    return 0;
}