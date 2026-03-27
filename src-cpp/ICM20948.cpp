#include "ICM20948.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

ICM20948::ICM20948(int bus, int address) {
    std::string bus_path = "/dev/i2c-" + std::to_string(bus);
    
    if ((i2c_fd = open(bus_path.c_str(), O_RDWR)) < 0) {
        std::cerr << "[IMU ERROR] Failed to open I2C bus.\n";
        return;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        std::cerr << "[IMU ERROR] Failed to connect to ICM20948 at 0x" << std::hex << address << std::dec << ".\n";
        return;
    }
}

ICM20948::~ICM20948() {
    if (i2c_fd >= 0) {
        // Put the sensor back to sleep safely
        select_bank(0);
        write_register(PWR_MGMT_1, 0x40); 
        close(i2c_fd);
    }
}

bool ICM20948::initialize() {
    if (i2c_fd < 0) return false;

    // 1. Make sure we are in User Bank 0
    select_bank(0);

    // 2. Wake the chip up and set the clock source to Auto (0x01)
    // By default, the chip powers on in sleep mode.
    write_register(PWR_MGMT_1, 0x01);
    
    // Give it a moment to stabilize
    usleep(50000); 

    std::cout << "[IMU] ICM-20948 Initialized successfully.\n";
    return true;
}

IMUVector3 ICM20948::get_accel() {
    const int16_t raw_x = read_word(ACCEL_XOUT_H, ACCEL_XOUT_L);
    const int16_t raw_y = read_word(ACCEL_YOUT_H, ACCEL_YOUT_L);
    const int16_t raw_z = read_word(ACCEL_ZOUT_H, ACCEL_ZOUT_L);

    return {
        (static_cast<double>(raw_x) / ACCEL_SCALE) * GRAVITY,
        (static_cast<double>(raw_y) / ACCEL_SCALE) * GRAVITY,
        (static_cast<double>(raw_z) / ACCEL_SCALE) * GRAVITY
    };
}

double ICM20948::get_accel_z() {
    return get_accel().z;
}

IMUVector3 ICM20948::get_gyro() {
    const int16_t raw_x = read_word(GYRO_XOUT_H, GYRO_XOUT_L);
    const int16_t raw_y = read_word(GYRO_YOUT_H, GYRO_YOUT_L);
    const int16_t raw_z = read_word(GYRO_ZOUT_H, GYRO_ZOUT_L);

    return {
        (static_cast<double>(raw_x) / GYRO_SCALE) * DEG_TO_RAD,
        (static_cast<double>(raw_y) / GYRO_SCALE) * DEG_TO_RAD,
        (static_cast<double>(raw_z) / GYRO_SCALE) * DEG_TO_RAD
    };
}

// --- I2C Helper Functions ---

void ICM20948::select_bank(int bank) {
    // Bank selection is shifted by 4 bits in the register
    write_register(REG_BANK_SEL, (bank << 4));
}

void ICM20948::write_register(int reg, int value) {
    unsigned char buf[2];
    buf[0] = reg;
    buf[1] = value;
    const ssize_t bytes_written = write(i2c_fd, buf, 2);
    if (bytes_written != 2) {
        std::cerr << "[IMU ERROR] Failed to write register 0x"
                  << std::hex << reg << std::dec << ".\n";
    }
}

int ICM20948::read_register(int reg) {
    unsigned char buf[1];
    buf[0] = reg;
    // Tell the chip which register we want to read
    const ssize_t addr_bytes_written = write(i2c_fd, buf, 1);
    if (addr_bytes_written != 1) {
        std::cerr << "[IMU ERROR] Failed to select register 0x"
                  << std::hex << reg << std::dec << " for read.\n";
        return 0;
    }
    // Read the response
    const ssize_t bytes_read = read(i2c_fd, buf, 1);
    if (bytes_read != 1) {
        std::cerr << "[IMU ERROR] Failed to read register 0x"
                  << std::hex << reg << std::dec << ".\n";
        return 0;
    }
    return buf[0];
}

int16_t ICM20948::read_word(int high_reg, int low_reg) {
    const int high = read_register(high_reg);
    const int low = read_register(low_reg);
    return static_cast<int16_t>((high << 8) | low);
}
