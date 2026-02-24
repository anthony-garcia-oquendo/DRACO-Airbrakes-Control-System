#include <iostream>
#include <csignal>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <iomanip>
#include "ServoControllerI2C.h"

const double CAM_SERVO_TABLE[46] = {
    0.00, 0.47, 0.94, 1.41, 1.88, 2.35, 2.83, 3.29, 3.77, 4.25,
    4.73, 5.21, 5.69, 6.17, 6.65, 7.13, 7.61, 8.10, 8.58, 9.06,
    9.54, 10.02, 10.50, 10.98, 11.46, 11.94, 12.41, 12.89, 13.36, 13.83,
    14.30, 14.77, 15.24, 15.70, 16.16, 16.62, 17.08, 17.53, 17.98, 18.43,
    18.88, 19.32, 19.75, 20.19, 20.62, 21.04
};

// Global pointer so our emergency shutdown can access the servo
ServoControllerI2C* global_servo = nullptr;

// This function runs automatically when you press Ctrl+C
void emergency_shutdown(int signum) {
    std::cout << "\n\n[EMERGENCY] Ctrl+C detected! Shutting down hardware...\n";
    if (global_servo != nullptr) {
        std::cout << "Stowing flaps to 0 degrees...\n";
        global_servo->rotate(0, 0.0); // Channel 0, 0 degrees
        
        // Deleting the object triggers the destructor, putting PCA9685 to sleep
        delete global_servo; 
    }
    std::cout << "Hardware safe. Exiting.\n";
    exit(signum);
}

int main() {
    // 1. Register the emergency shutdown interceptor
    signal(SIGINT, emergency_shutdown);

    std::cout << "Starting PCA9685 Cam Profile Test...\n";

    // 2. Initialize the servo dynamically
    global_servo = new ServoControllerI2C(1, 0x40);
    const int AIRBRAKE_CHANNEL = 0;

    std::cout << "Sweeping mapped flap angles 0 -> 45 -> 0. Press Ctrl+C to stop.\n\n";

    // 3. Custom Sweep Loop using the Cam Table
    while (true) {
        
        // --- DEPLOY FLAPS (Sweep Up) ---
        for (int flap_deg = 0; flap_deg <= 45; ++flap_deg) {
            double servo_deg = CAM_SERVO_TABLE[flap_deg];
            
            // Console log using \r to overwrite the same line cleanly
            std::cout << "\r[DEPLOYING] Flap: " << std::setw(2) << flap_deg 
                      << " deg -> Servo: " << std::fixed << std::setprecision(2) << servo_deg 
                      << " deg   " << std::flush;
            
            global_servo->rotate(AIRBRAKE_CHANNEL, servo_deg);
            
            // Wait 50ms between steps so you can actually watch the mechanism move smoothly
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        // Small pause at full deployment
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // --- STOW FLAPS (Sweep Down) ---
        for (int flap_deg = 45; flap_deg >= 0; --flap_deg) {
            double servo_deg = CAM_SERVO_TABLE[flap_deg];
            
            std::cout << "\r[ STOWING ] Flap: " << std::setw(2) << flap_deg 
                      << " deg -> Servo: " << std::fixed << std::setprecision(2) << servo_deg 
                      << " deg   " << std::flush;
            
            global_servo->rotate(AIRBRAKE_CHANNEL, servo_deg);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        // Small pause at fully stowed
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}