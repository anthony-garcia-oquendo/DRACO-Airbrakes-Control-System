#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

#include "KalmanFilter.h"

// Structure to hold one row of CSV data
struct DataPoint {
    double time;
    double alt;
    double vel;
    double accel;
};

int main() {
    std::cout << "====================================\n";
    std::cout << "    KALMAN FILTER SIL DATA REPLAY   \n";
    std::cout << "====================================\n\n";

    // 1. Open the OpenRocket CSV
    std::ifstream file("latest_rocket.csv");
    if (!file.is_open()) {
        std::cerr << "[FATAL] Could not open latest_rocket.csv\n";
        return 1;
    }

    std::string line;
    std::vector<DataPoint> truth_data;

    // 2. Parse the CSV file
    while (std::getline(file, line)) {
        // Skip empty lines and OpenRocket comment lines starting with '#'
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string token;
        DataPoint dp;

        // Parse Time, Altitude, Velocity, Acceleration (Assuming standard OpenRocket export format)
        try {
            std::getline(ss, token, ','); dp.time = std::stod(token);
            std::getline(ss, token, ','); dp.alt = std::stod(token);
            std::getline(ss, token, ','); dp.vel = std::stod(token);
            std::getline(ss, token, ','); dp.accel = std::stod(token);
            truth_data.push_back(dp);
        } catch (...) {
            // Ignore any lines that fail to parse (like trailing headers)
            continue; 
        }
    }
    file.close();

    if (truth_data.empty()) {
        std::cerr << "[FATAL] No valid data found in CSV!\n";
        return 1;
    }

    std::cout << "[SYSTEM] Loaded " << truth_data.size() << " data points from OpenRocket.\n";

    // 3. Initialize Kalman Filter 
    // We start it exactly where the simulation starts
    KalmanFilter kf(truth_data[0].alt, truth_data[0].vel);
    double prev_time = truth_data[0].time;
    
    // Trackers for our Error Math
    double alt_error_sum = 0.0;
    double alt_true_sum = 0.0;
    double vel_error_sum = 0.0;
    double vel_true_sum = 0.0;

    // 4. Run the Data Replay Loop
    for (size_t i = 1; i < truth_data.size(); ++i) {
        double dt = truth_data[i].time - prev_time;
        if (dt <= 0) continue; // Prevent division-by-zero on duplicate timestamps
        
        // --- A. FEED SENSORS TO FILTER ---
        kf.predict(truth_data[i].accel, dt);
        kf.update(truth_data[i].alt);
        
        // --- B. GET FILTER'S ESTIMATE ---
        double kf_alt = kf.get_altitude();
        double kf_vel = kf.get_velocity();
        
        // --- C. CALCULATE ERROR ---
        alt_error_sum += std::abs(truth_data[i].alt - kf_alt);
        alt_true_sum += std::abs(truth_data[i].alt);
        
        vel_error_sum += std::abs(truth_data[i].vel - kf_vel);
        vel_true_sum += std::abs(truth_data[i].vel);
        
        prev_time = truth_data[i].time;
    }
    
    // 5. Final Percentage Error Math
    // (Sum of absolute errors / Sum of true values) * 100
    double alt_error_pct = (alt_true_sum > 0) ? (alt_error_sum / alt_true_sum) * 100.0 : 0.0;
    double vel_error_pct = (vel_true_sum > 0) ? (vel_error_sum / vel_true_sum) * 100.0 : 0.0;

    std::cout << "\n--- TEST RESULTS ---\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Altitude Overall Error: " << alt_error_pct << " %\n";
    std::cout << "Velocity Overall Error: " << vel_error_pct << " %\n\n";

    if (alt_error_pct < 2.0 && vel_error_pct < 5.0) {
        std::cout << "[SUCCESS] Filter is tracking perfectly!\n";
    } else {
        std::cout << "[WARNING] Filter error is high. You may need to tune the noise matrices in KalmanFilter.cpp.\n";
    }

    return 0;
}