#ifndef ICM20948_H
#define ICM20948_H

#include <string>

class ICM20948 {
public:
    // Adafruit's default I2C address for this chip is 0x69
    ICM20948(int bus = 1, int address = 0x69);
    ~ICM20948();

    // Wakes the sensor up from sleep mode
    bool initialize();

    // Reads the Z-axis acceleration and converts it to m/s^2
    double get_accel_z();

private:
    int i2c_fd;

    // ICM-20948 Registers (Bank 0)
    const int REG_BANK_SEL = 0x7F;
    const int PWR_MGMT_1 = 0x06;
    const int ACCEL_ZOUT_H = 0x31;
    const int ACCEL_ZOUT_L = 0x32;

    // Conversion constants
    // Default power-up range is +/- 2g. 
    // 16-bit resolution means 1g = 16384 LSB.
    const double ACCEL_SCALE = 16384.0;
    const double GRAVITY = 9.80665; 

    // I2C Helpers
    void write_register(int reg, int value);
    int read_register(int reg);
    void select_bank(int bank);
};

#endif // ICM20948_H