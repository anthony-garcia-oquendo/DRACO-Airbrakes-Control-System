#ifndef SERVO_CONTROLLER_I2C_H
#define SERVO_CONTROLLER_I2C_H

#include <string>

class ServoControllerI2C {
public:
    // Constructor: Takes I2C bus (usually 1) and device address (usually 0x40)
    ServoControllerI2C(int bus = 1, int address = 0x40);
    ~ServoControllerI2C();

    // Actuate the specific servo channel (0-15) to a given angle
    void rotate(int channel, double angle);

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