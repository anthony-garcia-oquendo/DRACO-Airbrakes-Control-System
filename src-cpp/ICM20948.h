#ifndef ICM20948_H
#define ICM20948_H

#include <string>

struct IMUVector3
{
    double x;
    double y;
    double z;
};

class ICM20948 {
public:
    // Adafruit's default I2C address for this chip is 0x69
    ICM20948(int bus = 1, int address = 0x69);
    ~ICM20948();

    // Wakes the sensor up from sleep mode
    bool initialize();

    // Reads the full accelerometer vector in m/s^2
    IMUVector3 get_accel();

    // Reads the Z-axis acceleration and converts it to m/s^2
    double get_accel_z();

    // Reads the full gyro vector in rad/s
    IMUVector3 get_gyro();

private:
    int i2c_fd;

    // ICM-20948 Registers (Bank 0)
    const int REG_BANK_SEL = 0x7F;
    const int PWR_MGMT_1 = 0x06;
    const int ACCEL_XOUT_H = 0x2D;
    const int ACCEL_XOUT_L = 0x2E;
    const int ACCEL_YOUT_H = 0x2F;
    const int ACCEL_YOUT_L = 0x30;
    const int ACCEL_ZOUT_H = 0x31;
    const int ACCEL_ZOUT_L = 0x32;
    const int GYRO_XOUT_H = 0x33;
    const int GYRO_XOUT_L = 0x34;
    const int GYRO_YOUT_H = 0x35;
    const int GYRO_YOUT_L = 0x36;
    const int GYRO_ZOUT_H = 0x37;
    const int GYRO_ZOUT_L = 0x38;

    // Conversion constants
    // Default power-up range is +/- 2g.
    // 16-bit resolution means 1g = 16384 LSB.
    const double ACCEL_SCALE = 16384.0;
    // Default power-up gyro range is +/- 250 deg/s.
    const double GYRO_SCALE = 131.0;
    const double GRAVITY = 9.80665;
    const double DEG_TO_RAD = 0.017453292519943295;

    // I2C Helpers
    void write_register(int reg, int value);
    int read_register(int reg);
    int16_t read_word(int high_reg, int low_reg);
    void select_bank(int bank);
};

#endif // ICM20948_H
