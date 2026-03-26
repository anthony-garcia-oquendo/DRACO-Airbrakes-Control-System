#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include "BMP390.h"

int main() {
    BMP390 baro(1, 0x77); // Try 0x76 if 0x77 fails

    if (!baro.initialize()) {
        std::cerr << "Failed to initialize BMP390!\n";
        return 1;
    }

    // Get an initial baseline (simplified "Zeroing")
    double launchpad_alt = baro.get_altitude();
    std::cout << "Calibrated Launchpad Altitude: " << launchpad_alt << " m\n";

    while (true) {
        double current_alt = baro.get_altitude();
        double relative_alt = current_alt - launchpad_alt;
        double pressure_pa = baro.get_pressure();
        double pressure_hpa = pressure_pa / 100.0;
        double pressure_bar = pressure_pa / 100000.0;
        std::cout << "\rAltitude: " << std::fixed << std::setprecision(2) 
                  << relative_alt << " m    " << std::flush;
        
        std::cout << "\rPressure: "
              << std::fixed << std::setprecision(2)
              << pressure_pa << " Pa | "
              << pressure_hpa << " hPa | "
              << pressure_bar << " bar | "
              << "Altitude: " << relative_alt << " m     "
              << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}   