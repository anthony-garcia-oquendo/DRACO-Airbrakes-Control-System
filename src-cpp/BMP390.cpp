#include "BMP390.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>

BMP390::BMP390(int bus, int address) {
    std::string bus_path = "/dev/i2c-" + std::to_string(bus);
    if ((i2c_fd = open(bus_path.c_str(), O_RDWR)) < 0) {
        std::cerr << "[BARO ERROR] Failed to open I2C bus.\n";
        return;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        std::cerr << "[BARO ERROR] Failed to connect to BMP390.\n";
        return;
    }
}

BMP390::~BMP390() {
    if (i2c_fd >= 0) {
        // Put the sensor back to sleep
        write_register(REG_PWR_CTRL, 0x00); 
        close(i2c_fd);
    }
}

bool BMP390::initialize() {
    if (i2c_fd < 0) return false;

    // Check Chip ID (BMP390 is usually 0x60 or 0x50)
    int chip_id = read_register(REG_CHIP_ID);
    if (chip_id != 0x60 && chip_id != 0x50) {
        std::cerr << "[BARO ERROR] Invalid Chip ID: 0x" << std::hex << chip_id << std::dec << "\n";
        return false;
    }

    read_calibration_data();

    // 1. Set Oversampling (Pressure x8, Temperature x1)
    // This gives very low noise but is fast enough for 20Hz
    write_register(REG_OSR, 0x03);

    // 2. Set Output Data Rate to 50Hz
    write_register(REG_ODR, 0x02);

    // 3. Set IIR Filter (Coefficient 3) - Smooths out wind gusts
    write_register(REG_CONFIG, 0x04);

    // 4. Power Control: Enable Pressure & Temp, set to Normal Mode
    write_register(REG_PWR_CTRL, 0x33);

    // Give it time to take its first few readings
    usleep(50000); 

    std::cout << "[BARO] BMP390 Initialized successfully.\n";
    return true;
}

double BMP390::get_pressure() {
    uint8_t data[6];
    read_bytes(REG_DATA_START, data, 6);

    // Combine raw bytes
    uint32_t uncomp_press = (data[2] << 16) | (data[1] << 8) | data[0];
    uint32_t uncomp_temp  = (data[5] << 16) | (data[4] << 8) | data[3];

    // --- Bosch Temperature Compensation ---
    double pd1 = (double)uncomp_temp - calib.t1;
    double pd2 = pd1 * calib.t2;
    double temperature = pd2 + (pd1 * pd1) * calib.t3;

    // --- Bosch Pressure Compensation ---
    double pr1 = calib.p6 * temperature;
    double pr2 = calib.p7 * (temperature * temperature);
    double pr3 = calib.p8 * (temperature * temperature * temperature);
    double out1 = calib.p5 + pr1 + pr2 + pr3;

    pr1 = calib.p2 * temperature;
    pr2 = calib.p3 * (temperature * temperature);
    pr3 = calib.p4 * (temperature * temperature * temperature);
    double out2 = (double)uncomp_press * (calib.p1 + pr1 + pr2 + pr3);

    pr1 = (double)uncomp_press * (double)uncomp_press;
    pr2 = calib.p9 + calib.p10 * temperature;
    pr3 = pr1 * pr2;
    double out3 = pr3 + ((double)uncomp_press * (double)uncomp_press * (double)uncomp_press) * calib.p11;

    double pressure = out1 + out2 + out3;
    return pressure;
}

double BMP390::get_altitude(double sea_level_hPa) {
    double pressure_Pa = get_pressure();
    double pressure_hPa = pressure_Pa / 100.0;

    // The Standard Barometric Formula
    return 44330.0 * (1.0 - std::pow((pressure_hPa / sea_level_hPa), (1.0 / 5.255)));
}

void BMP390::read_calibration_data() {
    uint8_t coeff[21];
    read_bytes(REG_CALIB_DATA, coeff, 21);

    calib.t1 = (double)((uint16_t)(coeff[1] << 8 | coeff[0])) / 0.00390625;
    calib.t2 = (double)((uint16_t)(coeff[3] << 8 | coeff[2])) / 1073741824.0;
    calib.t3 = (double)((int8_t)coeff[4]) / 281474976710656.0;
    calib.p1 = ((double)((int16_t)(coeff[6] << 8 | coeff[5])) - 16384.0) / 1048576.0;
    calib.p2 = ((double)((int16_t)(coeff[8] << 8 | coeff[7])) - 16384.0) / 536870912.0;
    calib.p3 = (double)((int8_t)coeff[9]) / 4294967296.0;
    calib.p4 = (double)((int8_t)coeff[10]) / 137438953472.0;
    calib.p5 = (double)((uint16_t)(coeff[12] << 8 | coeff[11])) / 0.125;
    calib.p6 = (double)((uint16_t)(coeff[14] << 8 | coeff[13])) / 64.0;
    calib.p7 = (double)((int8_t)coeff[15]) / 256.0;
    calib.p8 = (double)((int8_t)coeff[16]) / 32768.0;
    calib.p9 = (double)((int16_t)(coeff[18] << 8 | coeff[17])) / 281474976710656.0;
    calib.p10 = (double)((int8_t)coeff[19]) / 281474976710656.0;
    calib.p11 = (double)((int8_t)coeff[20]) / 36893488147419103232.0;
}

void BMP390::write_register(int reg, int value) {
    unsigned char buf[2] = {(unsigned char)reg, (unsigned char)value};
    write(i2c_fd, buf, 2);
}

int BMP390::read_register(int reg) {
    unsigned char buf[1] = {(unsigned char)reg};
    write(i2c_fd, buf, 1);
    read(i2c_fd, buf, 1);
    return buf[0];
}

void BMP390::read_bytes(int reg, uint8_t* buffer, int length) {
    buffer[0] = reg;
    write(i2c_fd, buffer, 1);
    read(i2c_fd, buffer, length);
}