#include <iostream>
#include <thread>
#include <chrono>
#include "ServoControllerI2C.h"

int main() {
    std::cout << "Starting PCA9685 Hardware Test...\n";

    // Initialize the I2C driver on Bus 1, Address 0x40
    ServoControllerI2C servo_driver(1, 0x40);

    int channel = 0; // The channel your servo is plugged into

    std::cout << "Moving to 0 degrees (Stowed)...\n";
    servo_driver.rotate(channel, 0.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Moving to 45 degrees (Fully Deployed)...\n";
    servo_driver.rotate(channel, 45.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Starting Sweep Test (Ctrl+C to stop)...\n";
    // This will slowly sweep back and forth to test your cam linkage
    servo_driver.test_rotation(channel, 0.5, 20); // Move 0.5 deg every 20ms

    return 0;
}