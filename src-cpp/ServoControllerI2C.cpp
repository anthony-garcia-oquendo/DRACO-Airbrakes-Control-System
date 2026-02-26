#include "ServoControllerI2C.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <thread>
#include <chrono>

ServoControllerI2C::ServoControllerI2C(int bus, int address) {
    std::string bus_path = "/dev/i2c-" + std::to_string(bus);
    
    // Open the I2C bus
    if ((i2c_fd = open(bus_path.c_str(), O_RDWR)) < 0) {
        std::cerr << "[I2C ERROR] Failed to open the i2c bus.\n";
        return;
    }

    // Connect to the device address (Default 0x40)
    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        std::cerr << "[I2C ERROR] Failed to acquire bus access/talk to slave.\n";
        return;
    }

    // Initialize PCA9685
    write_register(MODE1, 0x00); // Normal mode
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    set_pwm_freq(50.0); // Set to 50Hz for standard servos
    std::cout << "[SERVO I2C] Initialized PCA9685 on bus " << bus << " at address 0x" << std::hex << address << std::dec << "\n";
}

ServoControllerI2C::~ServoControllerI2C() {
    if (i2c_fd >= 0) {
        // Put chip to sleep to disable outputs safely
        write_register(MODE1, 0x10); 
        close(i2c_fd);
    }
}

void ServoControllerI2C::set_pwm_freq(double freq) {
    freq *= 0.9;  // Hardware overshoot correction factor for PCA9685
    double prescaleval = 25000000.0; // 25MHz internal clock
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    int prescale = floor(prescaleval + 0.5);

    int oldmode = read_register(MODE1);
    int newmode = (oldmode & 0x7F) | 0x10; // sleep mode
    write_register(MODE1, newmode);        // go to sleep
    write_register(PRESCALE, prescale);    // set the prescaler
    write_register(MODE1, oldmode);        // wake up
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    write_register(MODE1, oldmode | 0x80); // turn on auto-increment
}

void ServoControllerI2C::rotate(int channel, double angle) {
    // 1. Safety Clamping
    if (angle < 0.0) {
        std::cerr << "[SERVO ERROR] Angle too low. Clamping to 0.\n";
        angle = 0.0;
    } else if (angle > 45.0) {
        std::cerr << "[SERVO ERROR] Angle too high. Clamping to 45.\n";
        angle = 45.0;
    }

    // 2. Map angle to 12-bit tick (102 to 512)
    double fraction = angle / MAX_ANGLE;
    int tick = TICK_MIN + static_cast<int>(fraction * (TICK_MAX - TICK_MIN));

    // 3. Send to specific channel
    set_pwm(channel, 0, tick);
}

void ServoControllerI2C::rotate_cam(int channel, double servo_angle) {
    // 1. Safety clamp specifically for your cam mechanism
    if (servo_angle < 0.0) {
        servo_angle = 0.0;
    } else if (servo_angle > 21.04) {
        servo_angle = 21.04;
    }

    // 2. Safely invert the angle so it moves clockwise!
    // An input of 0.0 (stowed) becomes physical 21.04
    // An input of 21.04 (deployed) becomes physical 0.0
    double safe_physical_angle = 21.04 - servo_angle;

    // 3. Send the calculated angle to the raw driver
    rotate(channel, safe_physical_angle);
}

void ServoControllerI2C::set_pwm(int channel, int on_tick, int off_tick) {
    int reg_base = LED0_ON_L + 4 * channel;
    write_register(reg_base, on_tick & 0xFF);
    write_register(reg_base + 1, on_tick >> 8);
    write_register(reg_base + 2, off_tick & 0xFF);
    write_register(reg_base + 3, off_tick >> 8);
}

void ServoControllerI2C::write_register(int reg, int value) {
    unsigned char buf[2];
    buf[0] = reg;
    buf[1] = value;
    write(i2c_fd, buf, 2);
}

int ServoControllerI2C::read_register(int reg) {
    unsigned char buf[1];
    buf[0] = reg;
    write(i2c_fd, buf, 1);
    read(i2c_fd, buf, 1);
    return buf[0];
}



void ServoControllerI2C::test_rotation(int channel, double delta, int wait_ms) {
    std::cout << "[SERVO] Starting test rotation on channel " << channel << "...\n";
    double current = 0.0;
    
    // Force the initial movement to be positive (upward)
    delta = std::abs(delta); 

    while (true) {
        current += delta;

        // Bounce off the ceiling
        if (current >= 45.0) {
            current = 45.0;
            delta = -std::abs(delta); // Force downward
        } 
        // Bounce off the floor
        else if (current <= 0.0) {
            current = 0.0;
            delta = std::abs(delta);  // Force upward
        }

        rotate(channel, current);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }
}