#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include "ICM20948.h"

int main() {
    std::cout << "Starting Adafruit ICM-20948 Hardware Test...\n";

    // Initialize the IMU on I2C Bus 1. 
    // Note: Adafruit's default I2C address for this board is usually 0x69.
    ICM20948 imu(1, 0x69); 

    if (!imu.initialize()) {
        std::cerr << "Failed to initialize IMU! Check your wiring and I2C address.\n";
        return 1;
    }

    std::cout << "IMU Online! Reading Z-Axis Acceleration...\n";
    std::cout << "Move the board up and down. Press Ctrl+C to stop.\n\n";

    while (true) {
        double accel_z = imu.get_accel_z();

        // Print to the same line repeatedly for a clean terminal output
        std::cout << "\rZ-Axis Acceleration: " 
                  << std::fixed << std::setprecision(2) << accel_z 
                  << " m/s^2    " << std::flush;

        // Sleep for 50ms to simulate your 20Hz flight loop
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}