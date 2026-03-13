#include "Buzzer.h"
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[]) {
    try {
        // Explicitly use /sys/class/pwm/pwmchip0/pwm2.
        PassiveBuzzerPWM buzzer(0, 2);

        std::cout << "Using sysfs PWM: /sys/class/pwm/pwmchip0/pwm2" << std::endl;

        if (argc > 1 && std::string(argv[1]) == "--sweep") {
            const std::vector<double> dutyCandidates = {
                0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50
            };
            const int testFrequencyHz = 2000;
            const int testDurationMs = 700;

            std::cout << "Duty sweep mode (rate loudness 1-10 for each test)" << std::endl;
            std::cout << "Frequency: " << testFrequencyHz << " Hz" << std::endl;

            double bestDuty = dutyCandidates.front();
            int bestScore = -1;

            for (double duty : dutyCandidates) {
                std::cout << "\nTesting duty=" << (duty * 100.0) << "%" << std::endl;
                buzzer.playTone(testFrequencyHz, testDurationMs, duty);

                std::cout << "Loudness score (1-10): ";
                int score = 0;
                std::cin >> score;
                if (!std::cin.good()) {
                    std::cerr << "Invalid input. Exiting sweep." << std::endl;
                    return 1;
                }

                if (score > bestScore) {
                    bestScore = score;
                    bestDuty = duty;
                }
            }

            std::cout << "\nBest duty cycle: " << (bestDuty * 100.0)
                      << "% (score " << bestScore << ")" << std::endl;
            return 0;
        }

        const double dutyCycle = 0.75;

        std::vector<Note> FurEliseMelody = {
            {659, 160}, {622, 160}, {659, 160}, {622, 160}, {659, 160},
            {494, 160}, {587, 160}, {523, 160}, {440, 280}, {0, 120},

            {262, 160}, {330, 160}, {440, 160}, {494, 280}, {0, 120},
            {330, 160}, {415, 160}, {494, 160}, {523, 280}, {0, 120},

            {330, 160}, {659, 160}, {622, 160}, {659, 160}, {622, 160}, {659, 160},
            {494, 160}, {587, 160}, {523, 160}, {440, 320}
        };
        
        std::vector<Note> melody = {
            {932, 83},
            {1109, 83},
            {932, 83},
            {698, 83},
            {554, 83},
            {466, 83},
            {554, 83},
            {466, 83},
            {349, 83},
            {277, 83},
            {233, 83},
            {277, 83},
            {233, 83},
            {175, 83},
            {139, 83},
            {0, 83},
            {831, 83},
            {1047, 83},
            {831, 83},
            {622, 83},
            {523, 83},
            {415, 83},
            {523, 83},
            {415, 83},
            {311, 83},
            {262, 83},
            {208, 83},
            {262, 83},
            {208, 83},
            {156, 83},
            {131, 83},
            {0, 83},
            {622, 83},
            {622, 83},
            {932, 83},
            {740, 83},
            {622, 83},
            {466, 83},
            {370, 83},
            {466, 83},
            {370, 83},
            {311, 83},
            {466, 83},
            {370, 83},
            {311, 83},
            {466, 83},
            {370, 83},
            {311, 83},
            {233, 83},
            {622, 83},
            {932, 83},
            {740, 83},
            {622, 83},
            {466, 83},
            {370, 83},
            {466, 83},
            {370, 83},
            {311, 83},
            {466, 83},
            {0, 500},
            {233, 83},
            {233, 83},
            {277, 83},
            {349, 83},
            {277, 83},
            {349, 83},
            {466, 83},
            {554, 83},
            {466, 83},
            {466, 83},
            {554, 83},
            {698, 83},
            {932, 83},
            {1109, 83},
            {932, 83},
            {0, 250},
            {208, 83},
            {262, 83},
            {311, 83},
            {262, 83},
            {311, 83},
            {415, 83},
            {523, 83},
            {415, 83},
            {523, 83},
            {415, 83},
            {523, 83},
            {622, 83},
            {523, 83},
            {0, 250},
            {185, 167},
            {156, 167},
            {233, 167},
            {185, 167},
            {117, 167},
            {92, 167},
            {78, 167},
            {117, 167},
            {92, 167},
            {46, 167},
            {39, 167},
            {0, 167},
            {92, 167},
            {117, 167},
            {92, 167},
            {78, 167}
        };

        buzzer.playMelody(FurEliseMelody, 20, dutyCycle);
        buzzer.stop();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}