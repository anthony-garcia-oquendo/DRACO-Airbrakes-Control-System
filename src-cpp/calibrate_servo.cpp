#include <iostream>
#include <iomanip>
#include <string>
#include "ServoControllerI2C.h"

int main() {
    // Initialize the servo driver
    ServoControllerI2C servo(1, 0x40);
    const int CHANNEL = 0;

    std::cout << "================================================\n";
    std::cout << "       AIRBRAKE SERVO CALIBRATION TOOL          \n";
    std::cout << "================================================\n";
    std::cout << "Instructions:\n";
    std::cout << "1. Enter a Servo Angle (0.0 to 45.0).\n";
    std::cout << "2. Measure the physical FLAP angle with a protractor.\n";
    std::cout << "3. Record the values for your CAM_SERVO_TABLE.\n";
    std::cout << "4. Type 'exit' to quit.\n\n";

    std::string input;
    double target_angle = 0.0;

    while (true) {
        std::cout << "Enter Target Servo Angle (deg): ";
        std::cin >> input;

        if (input == "exit" || input == "quit") break;

        try {
            target_angle = std::stod(input);
            
            // if (target_angle < 0.0 || target_angle > 45.0) {
            //     std::cout << "[ERROR] Please stay within 0-45 degrees for safety.\n";
            //     continue;
            // }

            std::cout << "[MOVING] Setting Servo to: " << target_angle << " degrees...\n";
            
            // Use raw rotate to bypass the cam inversion logic during testing
            servo.rotate(CHANNEL, target_angle);

        } catch (...) {
            std::cout << "[ERROR] Invalid input. Enter a number or 'exit'.\n";
        }
    }

    // Safety: stow before exiting
    std::cout << "\nCleaning up. Moving to 0.0 and shutting down.\n";
    servo.rotate(CHANNEL, 0.0);
    
    return 0;
}