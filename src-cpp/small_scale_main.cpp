#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <cmath>

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

struct IMUCalibration
{
    IMUVector3 launchpad_up_body;
    IMUVector3 gyro_bias;
};

IMUVector3 add(const IMUVector3 &a, const IMUVector3 &b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

IMUVector3 subtract(const IMUVector3 &a, const IMUVector3 &b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

IMUVector3 scale(const IMUVector3 &v, double scalar)
{
    return {v.x * scalar, v.y * scalar, v.z * scalar};
}

double dot(const IMUVector3 &a, const IMUVector3 &b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

IMUVector3 cross(const IMUVector3 &a, const IMUVector3 &b)
{
    return {
        (a.y * b.z) - (a.z * b.y),
        (a.z * b.x) - (a.x * b.z),
        (a.x * b.y) - (a.y * b.x)};
}

double magnitude(const IMUVector3 &v)
{
    return std::sqrt(dot(v, v));
}

IMUVector3 normalize(const IMUVector3 &v)
{
    const double mag = magnitude(v);
    if (mag < 1e-6)
    {
        return {0.0, 0.0, -1.0};
    }
    return scale(v, 1.0 / mag);
}

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

IMUCalibration calibrate_launchpad_imu(ICM20948 &imu, int samples = 200)
{
    std::cout << "[SYSTEM] Calibrating IMU attitude reference. Do not touch...\n";
    IMUVector3 accel_sum{0.0, 0.0, 0.0};
    IMUVector3 gyro_sum{0.0, 0.0, 0.0};
    for (int i = 0; i < samples; ++i)
    {
        accel_sum = add(accel_sum, imu.get_accel());
        gyro_sum = add(gyro_sum, imu.get_gyro());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const IMUVector3 accel_avg = scale(accel_sum, 1.0 / samples);
    const IMUVector3 gyro_avg = scale(gyro_sum, 1.0 / samples);
    const IMUVector3 launchpad_up_body = normalize(accel_avg);

    std::cout << "[SYSTEM] Launchpad Up Vector Locked: ("
              << launchpad_up_body.x << ", "
              << launchpad_up_body.y << ", "
              << launchpad_up_body.z << ")\n";
    std::cout << "[SYSTEM] Launchpad Gyro Bias Locked: ("
              << gyro_avg.x << ", "
              << gyro_avg.y << ", "
              << gyro_avg.z << ") rad/s\n";

    return {launchpad_up_body, gyro_avg};
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

    if (!baro.initialize())
    {
        std::cerr << "[FATAL] Barometer initialization failed. Aborting.\n";
        return -1;
    }

    std::ofstream log_file("small_scale_flight_log.csv");
    if (!log_file.is_open())
    {
        std::cerr << "[FATAL] Could not create flight log on SD card!\n";
        return 1;
    }

    log_file << "Time(s),State,Raw_AGL(m),Raw_Accel_Z(m/s2),Vert_Accel(m/s2),KF_Alt(m),KF_Vel(m/s)\n";
    log_file << std::fixed << std::setprecision(3);

    // Init math and start states
    KalmanFilter kf(0.0, 0.0);
    double launchpad_msl = calculate_launchpad_zero(baro);
    IMUCalibration imu_calibration = calibrate_launchpad_imu(imu);
    IMUVector3 up_body_estimate = imu_calibration.launchpad_up_body;
    const double LOOP_DT = 0.01;
    const double GRAVITY = 9.80665;
    const double ACCEL_TRUST_BAND = 1.5;
    const double UP_BLEND_ALPHA = 0.02;
    const auto LOOP_PERIOD = std::chrono::milliseconds(static_cast<int>(LOOP_DT * 1000));
    const auto LOG_PERIOD = std::chrono::milliseconds(100); // 10 Hz logging, 100 Hz control loop
    const int LIFTOFF_CONFIRM_SAMPLES = 5;                  // 50 ms
    const int BURNOUT_CONFIRM_SAMPLES = 3;                  // 30 ms
    const int APOGEE_CONFIRM_SAMPLES = 3;                   // 30 ms
    const int TOUCHDOWN_CONFIRM_SAMPLES = 10;               // 100 ms

    FlightState current_state = ON_PAD;
    int liftoff_counter = 0;
    int burnout_counter = 0;
    int apogee_counter = 0;
    int touchdown_counter = 0;
    std::cout << "[SYSTEM] Zeroing complete. Starting main loop. Waiting for liftoff...\n";

    auto start_time = std::chrono::steady_clock::now();
    auto next_loop_time = start_time;
    auto next_log_time = start_time;

    // Play a little tune to signal the start of the test to signal correct initialization
    buzzer.playMelody(Salgo_Pa_La_Calle, 20, 0.75);

    // Main Loop
    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start_time).count();

        double current_agl = baro.get_altitude() - launchpad_msl;
        IMUVector3 accel_body = imu.get_accel();
        IMUVector3 gyro_body = subtract(imu.get_gyro(), imu_calibration.gyro_bias);
        double raw_accel_z = accel_body.z;

        // Keep tracking the launchpad up-axis in body coordinates so tilt does not
        // look like vertical acceleration.
        up_body_estimate = normalize(subtract(up_body_estimate, scale(cross(gyro_body, up_body_estimate), LOOP_DT)));

        const double accel_magnitude = magnitude(accel_body);
        if (std::abs(accel_magnitude - GRAVITY) < ACCEL_TRUST_BAND)
        {
            const IMUVector3 measured_up_body = normalize(accel_body);
            up_body_estimate = normalize(add(scale(up_body_estimate, 1.0 - UP_BLEND_ALPHA),
                                             scale(measured_up_body, UP_BLEND_ALPHA)));
        }

        const double vertical_specific_force = dot(accel_body, up_body_estimate);
        const double vertical_accel = vertical_specific_force - GRAVITY;

        kf.predict(vertical_accel, LOOP_DT);
        kf.update(current_agl);

        double kf_alt = kf.get_altitude();
        double kf_vel = kf.get_velocity();

        if (now >= next_log_time)
        {
            log_file << t << "," << current_state << "," << current_agl << "," << raw_accel_z << "," << vertical_accel << "," << kf_alt << "," << kf_vel << "\n";
            log_file << std::flush;

            do
            {
                next_log_time += LOG_PERIOD;
            } while (next_log_time <= now);
        }

        switch (current_state)
        {
        case ON_PAD:
            if (vertical_accel > 20.0 && (kf_vel > 5.0 || current_agl > 0.10))
            {
                liftoff_counter++;
            }
            else
            {
                liftoff_counter = 0;
            }

            if (liftoff_counter >= LIFTOFF_CONFIRM_SAMPLES)
            {
                current_state = BOOST;
                liftoff_counter = 0;
                std::cout << "\n[FLIGHT] LIFTOFF DETECTED! Transition to BOOST.\n";
                log_file << "\n[FLIGHT] LIFTOFF DETECTED! Transition to BOOST.\n"
                         << std::flush;
            }
            break;

        case BOOST:
            if (vertical_accel < 0.0 && kf_vel > 30.0)
            {
                burnout_counter++;
            }
            else
            {
                burnout_counter = 0;
            }

            if (burnout_counter >= BURNOUT_CONFIRM_SAMPLES)
            {
                current_state = COAST;
                burnout_counter = 0;
                std::cout << "\n[FLIGHT] BURNOUT DETECTED! Transition to COAST.\n";
                log_file << "\n[FLIGHT] BURNOUT DETECTED! Transition to COAST.\n"
                         << std::flush;
            }
            break;

        case COAST:
            if (kf_vel < -1.0)
            {
                apogee_counter++;
            }
            else
            {
                apogee_counter = 0;
            }

            if (apogee_counter >= APOGEE_CONFIRM_SAMPLES)
            {
                current_state = DESCENT;
                apogee_counter = 0;
                std::cout << "\n[FLIGHT] APOGEE DETECTED! Apogee: " << kf_alt << " m\n";
                log_file << "\n[FLIGHT] APOGEE DETECTED! Apogee: " << kf_alt << " m\n"
                         << std::flush;
            }
            break;

        case DESCENT:
            if (current_agl < 2.0 && std::abs(kf_vel) < 6.0)
            {
                touchdown_counter++;
            }
            else
            {
                touchdown_counter = 0;
            }

            if (touchdown_counter >= TOUCHDOWN_CONFIRM_SAMPLES)
            {
                std::cout << "\n[SYSTEM] Touchdown detected. Closing log and shutting down.\n";
                log_file << "\n[SYSTEM] Touchdown detected. Closing log and shutting down.\n"
                         << std::flush;
                log_file.close();
                return 0;
            }
            break;
        }

        // Enforce loop timing
        next_loop_time += LOOP_PERIOD;
        std::this_thread::sleep_until(next_loop_time);
    }
}
