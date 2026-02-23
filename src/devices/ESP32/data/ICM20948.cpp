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

double ICM20948::get_accel_z() {
    // Read the High and Low bytes for the Z axis
    int high = read_register(ACCEL_ZOUT_H);
    int low  = read_register(ACCEL_ZOUT_L);

    // Combine them into a 16-bit signed integer
    int16_t raw_z = (high << 8) | low;

    // Convert raw LSB to G-forces, then to m/s^2
    double g_force = static_cast<double>(raw_z) / ACCEL_SCALE;
    
    return g_force * GRAVITY;
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
    write(i2c_fd, buf, 2);
}

int ICM20948::read_register(int reg) {
    unsigned char buf[1];
    buf[0] = reg;
    // Tell the chip which register we want to read
    write(i2c_fd, buf, 1);
    // Read the response
    read(i2c_fd, buf, 1);
    return buf[0];
}