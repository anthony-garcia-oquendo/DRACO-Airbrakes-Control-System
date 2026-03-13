#include <iostream>
#include <vector>
#include "PassiveBuzzerPWM.hpp"

constexpr int C4 = 262;
constexpr int D4 = 294;
constexpr int E4 = 330;
constexpr int F4 = 349;
constexpr int G4 = 392;
constexpr int A4 = 440;
constexpr int B4 = 494;
constexpr int C5 = 523;

int main() {
    try {
        PassiveBuzzerPWM buzzer(0, 2);

        std::vector<Note> melody = {
            {E4, 250}, {E4, 250}, {F4, 250}, {G4, 250},
            {G4, 250}, {F4, 250}, {E4, 250}, {D4, 250},
            {C4, 250}, {C4, 250}, {D4, 250}, {E4, 250},
            {E4, 375}, {D4, 125}, {D4, 500}
        };

        buzzer.playMelody(melody, 30);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}