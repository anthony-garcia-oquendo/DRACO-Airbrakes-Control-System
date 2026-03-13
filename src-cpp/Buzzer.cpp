#include "PassiveBuzzerPWM.hpp"

#include <fstream>
#include <thread>
#include <chrono>
#include <stdexcept>

void PassiveBuzzerPWM::writeFile(const std::string& path, const std::string& value) {
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open: " + path);
    }

    file << value;

    if (!file) {
        throw std::runtime_error("Failed to write to: " + path);
    }
}

bool PassiveBuzzerPWM::fileExists(const std::string& path) {
    std::ifstream file(path);
    return file.good();
}

PassiveBuzzerPWM::PassiveBuzzerPWM(int chip, int channel) : enabled(false) {
    std::string chipPath = "/sys/class/pwm/pwmchip" + std::to_string(chip);
    pwmPath = chipPath + "/pwm" + std::to_string(channel);

    if (!fileExists(pwmPath)) {
        writeFile(chipPath + "/export", std::to_string(channel));
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