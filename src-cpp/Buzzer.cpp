#include "Buzzer.h"

#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <filesystem>

namespace {
std::string buildIOError(const std::string& action, const std::string& path) {
    int err = errno;
    std::ostringstream oss;
    oss << "Failed to " << action << ": " << path;
    if (err != 0) {
        oss << " (" << std::strerror(err) << ")";
    }
    return oss.str();
}
}

void PassiveBuzzerPWM::writeFile(const std::string& path, const std::string& value) {
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error(buildIOError("open", path));
    }

    file << value;

    if (!file) {
        throw std::runtime_error(buildIOError("write", path));
    }
}

bool PassiveBuzzerPWM::fileExists(const std::string& path) {
    std::ifstream file(path);
    return file.good();
}

std::string PassiveBuzzerPWM::listAvailablePwmChips() {
    const std::filesystem::path pwmRoot("/sys/class/pwm");
    if (!std::filesystem::exists(pwmRoot)) {
        return "none (/sys/class/pwm is missing)";
    }

    std::ostringstream oss;
    bool found = false;
    for (const auto& entry : std::filesystem::directory_iterator(pwmRoot)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("pwmchip", 0) == 0) {
            if (found) {
                oss << ", ";
            }
            oss << name;
            found = true;
        }
    }

    if (!found) {
        return "none";
    }

    return oss.str();
}

int PassiveBuzzerPWM::readIntFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error(buildIOError("open", path));
    }

    int value = -1;
    file >> value;
    if (!file) {
        throw std::runtime_error("Failed to parse integer from: " + path);
    }

    return value;
}

PassiveBuzzerPWM::PassiveBuzzerPWM(int chip, int channel) : enabled(false) {
    if (chip < 0) {
        throw std::invalid_argument("PWM chip index must be >= 0");
    }
    if (channel < 0) {
        throw std::invalid_argument("PWM channel index must be >= 0");
    }

    std::string chipPath = "/sys/class/pwm/pwmchip" + std::to_string(chip);
    pwmPath = chipPath + "/pwm" + std::to_string(channel);

    if (!fileExists(chipPath)) {
        throw std::runtime_error(
            "PWM chip path not found: " + chipPath +
            ". Available chips: " + listAvailablePwmChips() +
            ". Enable PWM in your kernel/device-tree and retry."
        );
    }

    const int numChannels = readIntFile(chipPath + "/npwm");
    if (channel >= numChannels) {
        throw std::runtime_error(
            "Requested pwm channel " + std::to_string(channel) +
            " but " + chipPath + " exposes channels [0.." + std::to_string(numChannels - 1) + "]"
        );
    }

    if (!fileExists(pwmPath)) {
        try {
            writeFile(chipPath + "/export", std::to_string(channel));
        } catch (const std::exception& e) {
            throw std::runtime_error(
                std::string(e.what()) +
                ". You may need root or a udev rule for /sys/class/pwm access."
            );
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!fileExists(pwmPath)) {
        throw std::runtime_error("PWM channel did not appear at " + pwmPath);
    }
}

PassiveBuzzerPWM::~PassiveBuzzerPWM() {
    try {
        stop();
    } catch (...) {
    }
}

void PassiveBuzzerPWM::setFrequency(int frequencyHz, double dutyCycle) {
    if (frequencyHz <= 0) {
        throw std::invalid_argument("Frequency must be greater than 0");
    }

    if (dutyCycle <= 0.0 || dutyCycle >= 1.0) {
        throw std::invalid_argument("Duty cycle must be between 0 and 1");
    }

    long long periodNs = 1000000000LL / frequencyHz;
    long long dutyNs = static_cast<long long>(periodNs * dutyCycle);

    if (enabled) {
        writeFile(pwmPath + "/enable", "0");
        enabled = false;
    }

    writeFile(pwmPath + "/period", std::to_string(periodNs));
    writeFile(pwmPath + "/duty_cycle", std::to_string(dutyNs));
}

void PassiveBuzzerPWM::start() {
    if (!enabled) {
        writeFile(pwmPath + "/enable", "1");
        enabled = true;
    }
}

void PassiveBuzzerPWM::stop() {
    if (enabled) {
        writeFile(pwmPath + "/enable", "0");
        enabled = false;
    }
}

void PassiveBuzzerPWM::playTone(int frequencyHz, int durationMs, double dutyCycle) {
    if (frequencyHz <= 0) {
        stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));
        return;
    }

    setFrequency(frequencyHz, dutyCycle);
    start();
    std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));
    stop();
}

void PassiveBuzzerPWM::playMelody(const std::vector<Note>& melody, int gapMs, double dutyCycle) {
    for (const auto& note : melody) {
        playTone(note.frequency, note.durationMs, dutyCycle);
        std::this_thread::sleep_for(std::chrono::milliseconds(gapMs));
    }
}