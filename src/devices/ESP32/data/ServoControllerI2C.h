#ifndef SERVO_CONTROLLER_I2C_H
#define SERVO_CONTROLLER_I2C_H

#include <string>

const double CAM_SERVO_TABLE[46] = {
    0.00, 0.47, 0.94, 1.41, 1.88, 2.35, 2.83, 3.29, 3.77, 4.25,
    4.73, 5.21, 5.69, 6.17, 6.65, 7.13, 7.61, 8.10, 8.58, 9.06,
    9.54, 10.02, 10.50, 10.98, 11.46, 11.94, 12.41, 12.89, 13.36, 13.83,
    14.30, 14.77, 15.24, 15.70, 16.16, 16.62, 17.08, 17.53, 17.98, 18.43,
    18.88, 19.32, 19.75, 20.19, 20.62, 21.04
};

class ServoControllerI2C {
public:
    // Constructor: Takes I2C bus (usually 1) and device address (usually 0x40)
    ServoControllerI2C(int bus = 1, int address = 0x40);
    ~ServoControllerI2C();

    // Actuate the specific servo channel (0-15) to a given angle
    void rotate(int channel, double angle);

    // Automatically inverts the angle for clockwise deployment
    void rotate_cam(int channel, double servo_angle);

    // Test sequence
    void test_rotation(int channel, double delta = 1.0, int wait_ms = 1000);

private:
    int i2c_fd; // File descriptor for the I2C bus

    // Standard Servo Constants
    const double MIN_ANGLE = 0.0;
    const double MAX_ANGLE = 180.0;
    
    // PCA9685 12-bit (0-4095) Tick Constants for 50Hz (20ms period)
    // 20ms / 4096 = 4.88 microseconds per tick
    // 0.5ms pulse (0 deg) = 500us / 4.88us = ~102 ticks
    // 2.5ms pulse (180 deg) = 2500us / 4.88us = ~512 ticks
    const int TICK_MIN = 102; 
    const int TICK_MAX = 512; 

    // PCA9685 Registers
    const int MODE1 = 0x00;
    const int PRESCALE = 0xFE;
    const int LED0_ON_L = 0x06;

    // Helper functions for I2C communication
    void write_register(int reg, int value);
    int read_register(int reg);
    void set_pwm_freq(double freq);
    void set_pwm(int channel, int on_tick, int off_tick);
};

#endif // SERVO_CONTROLLER_I2C_H