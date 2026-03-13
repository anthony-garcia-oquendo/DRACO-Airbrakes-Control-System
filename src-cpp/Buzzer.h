#ifndef PASSIVE_BUZZER_PWM_HPP
#define PASSIVE_BUZZER_PWM_HPP

#include <string>
#include <vector>

struct Note {
    int frequency;
    int durationMs;
};

class PassiveBuzzerPWM {
private:
    std::string pwmPath;
    bool enabled;

    static void writeFile(const std::string& path, const std::string& value);
    static bool fileExists(const std::string& path);
    static std::string listAvailablePwmChips();
    static int readIntFile(const std::string& path);

public:
    PassiveBuzzerPWM(int chip, int channel);
    ~PassiveBuzzerPWM();

    void setFrequency(int frequencyHz, double dutyCycle = 0.5);
    void start();
    void stop();
    void playTone(int frequencyHz, int durationMs, double dutyCycle = 0.5);
    void playMelody(const std::vector<Note>& melody, int gapMs = 20, double dutyCycle = 0.5);
};

#endif