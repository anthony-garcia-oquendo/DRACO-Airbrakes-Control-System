#include <iostream>
#include <csignal>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <iomanip>
#include "ServoControllerI2C.h"

// Global pointer so our emergency shutdown can access the servo
ServoControllerI2C* global_servo = nullptr;

// This function runs automatically when you press Ctrl+C
void emergency_shutdown(int signum) {
    std::cout << "\n\n[EMERGENCY] Ctrl+C detected! Shutting down hardware...\n";
    if (global_servo != nullptr) {
    std::cout << "Stowing flaps to 0 degrees (Servo 21.04)...\n";
    global_servo->rotate(0, 0.0); // Stowed position
        
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
            // THE FIX: Invert the servo angle mathematically!
            double servo_deg = CAM_SERVO_TABLE[flap_deg]; 
            
            std::cout << "\r[DEPLOYING] Flap: " << std::setw(2) << flap_deg 
                      << " deg -> Servo: " << std::fixed << std::setprecision(2) << servo_deg 
                      << " deg   " << std::flush;
            
            global_servo->rotate_cam(AIRBRAKE_CHANNEL, servo_deg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // --- STOW FLAPS (Sweep Down) ---
        for (int flap_deg = 45; flap_deg >= 0; --flap_deg) {
            // THE FIX: Invert the servo angle mathematically!
            double servo_deg = CAM_SERVO_TABLE[flap_deg];
            
            std::cout << "\r[ STOWING ] Flap: " << std::setw(2) << flap_deg 
                      << " deg -> Servo: " << std::fixed << std::setprecision(2) << servo_deg 
                      << " deg   " << std::flush;
            
            global_servo->rotate_cam(AIRBRAKE_CHANNEL, servo_deg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}