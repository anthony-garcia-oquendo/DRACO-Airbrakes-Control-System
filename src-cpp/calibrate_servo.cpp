#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include "ServoControllerI2C.h"

double get_servo_angle_from_cam(double flap_angle) {
    flap_angle = std::max(0.0, std::min(45.0, flap_angle));
    int i = static_cast<int>(flap_angle);
    if (i >= 45) return CAM_SERVO_TABLE[45];

    double weight = flap_angle - i;
    float servo_angle = CAM_SERVO_TABLE[i] * (1.0 - weight) + CAM_SERVO_TABLE[i + 1] * weight;
    return servo_angle;
}

int main() {
    // Initialize the servo driver
    ServoControllerI2C servo(1, 0x40);
    const int CHANNEL = 0;

    std::cout << "================================================\n";
    std::cout << "       AIRBRAKE SERVO CALIBRATION TOOL          \n";
    std::cout << "================================================\n";
    std::cout << "Instructions:\n";
    std::cout << "1. Enter a Target FLAP Angle (0.0 to 45.0).\n";
    std::cout << "2. Tool maps flap angle to servo angle via CAM_SERVO_TABLE.\n";
    std::cout << "3. Measure/verify the physical flap angle with a protractor.\n";
    std::cout << "4. Type 'exit' to quit.\n\n";

    std::string input;
    double target_flap_angle = 0.0;

    while (true) {
        std::cout << "Enter Target Flap Angle (deg): ";
        std::cin >> input;

        if (input == "exit" || input == "quit") break;

        try {
            target_flap_angle = std::stod(input);
            if (target_flap_angle < 0.0 || target_flap_angle > 45.0) {
                std::cout << "[ERROR] Please stay within 0-45 degrees for safety.\n";
                continue;
            }

            const double servo_angle = get_servo_angle_from_cam(target_flap_angle);

            std::cout << "[MOVING] Flap target: " << std::fixed << std::setprecision(2)
                      << target_flap_angle << " deg -> servo table angle: "
                      << servo_angle << " deg\n";

            servo.rotate_cam(CHANNEL, servo_angle);

        } catch (...) {
            std::cout << "[ERROR] Invalid input. Enter a number or 'exit'.\n";
        }
    }

    // Safety: stow before exiting
    std::cout << "\nCleaning up. Moving to 0.0 and shutting down.\n";
    servo.rotate_cam(CHANNEL, 0.0);
    
    return 0;
}