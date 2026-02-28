#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

#include "KalmanFilter.h"

struct DataPoint {
    double time;
    double alt;
    double vel;
    double accel;
};

int main() {
    std::cout << "=========================================================\n";
    std::cout << "            KALMAN FILTER SIL DATA REPLAY                \n";
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

    std::cout << "[SYSTEM] Loaded " << truth_data.size() << " data points.\n";
    std::cout << "TIME(s) | TRUE ALT(m) | KF ALT(m) | ALT ERR | TRUE VEL(m/s) | KF VEL(m/s) | VEL ERR\n";
    std::cout << "-----------------------------------------------------------------------------------\n";

    KalmanFilter kf(truth_data[0].alt, truth_data[0].vel);
    double prev_time = truth_data[0].time;
    
    double alt_error_sum = 0.0;
    double alt_true_sum = 0.0;
    double vel_error_sum = 0.0;
    double vel_true_sum = 0.0;

    // Setup console formatting
    std::cout << std::fixed << std::setprecision(2);

    for (size_t i = 1; i < truth_data.size(); ++i) {
        double dt = truth_data[i].time - prev_time;
        if (dt <= 0) continue; 
        
        kf.predict(truth_data[i].accel, dt);
        kf.update(truth_data[i].alt);
        
        double kf_alt = kf.get_altitude();
        double kf_vel = kf.get_velocity();
        
        double alt_err = std::abs(truth_data[i].alt - kf_alt);
        double vel_err = std::abs(truth_data[i].vel - kf_vel);

        // --- CONSOLE LOGGING EVERY DATA POINT ---
        std::cout << std::setw(7) << truth_data[i].time << " | "
                  << std::setw(11) << truth_data[i].alt << " | "
                  << std::setw(9) << kf_alt << " | "
                  << std::setw(7) << alt_err << " | "
                  << std::setw(13) << truth_data[i].vel << " | "
                  << std::setw(11) << kf_vel << " | "
                  << std::setw(7) << vel_err << "\n";

        alt_error_sum += alt_err;
        alt_true_sum += std::abs(truth_data[i].alt);
        
        vel_error_sum += vel_err;
        vel_true_sum += std::abs(truth_data[i].vel);
        
        prev_time = truth_data[i].time;
    }
    
    double alt_error_pct = (alt_true_sum > 0) ? (alt_error_sum / alt_true_sum) * 100.0 : 0.0;
    double vel_error_pct = (vel_true_sum > 0) ? (vel_error_sum / vel_true_sum) * 100.0 : 0.0;

    std::cout << "\n====================================\n";
    std::cout << "          FINAL TEST RESULTS        \n";
    std::cout << "====================================\n";
    std::cout << "Altitude Mean Error: " << alt_error_pct << " %\n";
    std::cout << "Velocity Mean Error: " << vel_error_pct << " %\n\n";

    return 0;
}