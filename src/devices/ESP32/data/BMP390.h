#ifndef BMP390_H
#define BMP390_H

#include <string>
#include <stdint.h>

class BMP390 {
public:
    // Adafruit's default I2C address for the BMP390 is usually 0x77
    BMP390(int bus = 1, int address = 0x77);
    ~BMP390();

    // Wakes the sensor, sets oversampling/IIR filters, and reads factory calibration
    bool initialize();

    // Returns the current Altitude in meters
    // Requires a baseline sea-level pressure (defaults to standard 1013.25 hPa)
    double get_altitude(double sea_level_hPa = 1013.25);

    // Returns raw compensated pressure in Pascals
    double get_pressure();

private:
    int i2c_fd;

    // Factory Calibration Data Structure
    struct CalibData {
        double t1, t2, t3;
        double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
    } calib;

    // BMP390 Registers
    const int REG_CHIP_ID = 0x00;
    const int REG_DATA_START = 0x04;
    const int REG_PWR_CTRL = 0x1B;
    const int REG_OSR = 0x1C;
    const int REG_ODR = 0x1D;
    const int REG_CONFIG = 0x1F;
    const int REG_CALIB_DATA = 0x31;

    // Helper Functions
    void read_calibration_data();
    void write_register(int reg, int value);
    int read_register(int reg);
    void read_bytes(int reg, uint8_t* buffer, int length);
};

#endif // BMP390_H