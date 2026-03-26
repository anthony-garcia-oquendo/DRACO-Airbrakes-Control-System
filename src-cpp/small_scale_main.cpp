#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>

// Custom Hardware Drivers
#include "BMP390.h"
#include "ICM20948.h"
#include "Buzzer.h"

// Math & Logic
#include "KalmanFilter.h"

// --- Flight States ---
enum FlightState
{
    ON_PAD = 0,
    BOOST = 1,
    COAST = 2,
    DESCENT = 3
};

// --- Helper for Zeroing the Barometer ---
double calculate_launchpad_zero(BMP390 &baro, int samples = 100)
{
    std::cout << "[SYSTEM] Zeroing Barometer. Do not touch...\n";
    double sum = 0.0;
    for (int i = 0; i < samples; ++i)
    {
        sum += baro.get_altitude();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    double baseline = sum / samples;
    std::cout << "[SYSTEM] Launchpad Baseline Locked: " << baseline << " m MSL\n";
    return baseline;
}

/* In this small scale test, we will only test the imu, barometer, and Kalman filter and state machine logic.
   The flaps will never be actuated. We will log all data and state transitions to a CSV file for post-flight analysis.
*/
int main()
{
    std::cout << "====================================\n";
    std::cout << "   SMALL SCALE FLIGHT COMPUTER      \n";
    std::cout << "====================================\n\n";

    BMP390 baro(1, 0x77);
    ICM20948 imu(1, 0x69);
    PassiveBuzzerPWM buzzer(0, 2);

    if (!imu.initialize())
    {
        std::cerr << "[FATAL] IMU initialization failed. Aborting.\n";
        return -1;
    }

    std::ofstream log_file("small_scale_flight_log.csv");
    if (!log_file.is_open())
    {
        std::cerr << "[FATAL] Could not create flight log on SD card!\n";
        return 1;
    }

    log_file << "Time(s),State,Raw_AGL(m),Accel_Z(m/s2),KF_Alt(m),KF_Vel(m/s)\n";
    log_file << std::fixed << std::setprecision(3);

    // Init math and start states
    KalmanFilter kf(0.0, 0.0);
    double launchpad_msl = calculate_launchpad_zero(baro);
    const double LOOP_DT = 0.01;

    FlightState current_state = ON_PAD;
    std::cout << "[SYSTEM] Zeroing complete. Starting main loop. Waiting for liftoff...\n";

    auto start_time = std::chrono::steady_clock::now();
    auto next_loop_time = start_time;

    // Play a little tune to signal the start of the test to signal correct initialization
    buzzer.playMelody(Salgo_Pa_La_Calle, 20, 0.75);

    // Main Loop
    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start_time).count();

        double current_agl = baro.get_altitude() - launchpad_msl;
        double accel_z = imu.get_accel_z();

        kf.predict(accel_z, LOOP_DT);
        kf.update(current_agl);

        double kf_alt = kf.get_altitude();
        double kf_vel = kf.get_velocity();

        log_file << t << "," << current_state << "," << current_agl << "," << accel_z << "," << kf_alt << "," << kf_vel << "\n";

        switch (current_state)
        {
        case ON_PAD:
            if (accel_z > 20.0)
            {
                current_state = BOOST;
                std::cout << "\n[FLIGHT] LIFTOFF DETECTED! Transition to BOOST.\n";
                log_file << "\n[FLIGHT] LIFTOFF DETECTED! Transition to BOOST.\n";
            }
            break;

        case BOOST:
            if (accel_z < 0.0 && kf_vel > 50.0)
            {
                current_state = COAST;
                std::cout << "\n[FLIGHT] BURNOUT DETECTED! Transition to COAST.\n";
                log_file << "\n[FLIGHT] BURNOUT DETECTED! Transition to COAST.\n";
            }
            break;

        case COAST:
            if (kf_vel < 0.0)
            {
                current_state = DESCENT;
                std::cout << "\n[FLIGHT] APOGEE DETECTED! Apogee: " << kf_alt << " m\n";
                log_file << "\n[FLIGHT] APOGEE DETECTED! Apogee: " << kf_alt << " m\n";
            }
            break;

        case DESCENT:
            if (kf_alt < 10.0 && kf_vel > -2.0)
            {
                std::cout << "\n[SYSTEM] Touchdown detected. Closing log and shutting down.\n";
                log_file << "\n[SYSTEM] Touchdown detected. Closing log and shutting down.\n";
                log_file.close();
                return 0;
            }
            break;
        }
        log_file << std::flush;

        // Enforce loop timing
        next_loop_time += std::chrono::milliseconds(static_cast<int>(LOOP_DT * 1000));
        std::this_thread::sleep_until(next_loop_time);
    }
}