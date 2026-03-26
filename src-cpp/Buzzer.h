#ifndef PASSIVE_BUZZER_PWM_HPP
#define PASSIVE_BUZZER_PWM_HPP

#include <string>
#include <vector>

struct Note
{
    int frequency;
    int durationMs;
};

std::vector<Note> OurMelody = {
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
    {78, 167}};

class PassiveBuzzerPWM
{
private:
    std::string pwmPath;
    bool enabled;

    static void writeFile(const std::string &path, const std::string &value);
    static bool fileExists(const std::string &path);
    static std::string listAvailablePwmChips();
    static int readIntFile(const std::string &path);

public:
    PassiveBuzzerPWM(int chip, int channel);
    ~PassiveBuzzerPWM();

    void setFrequency(int frequencyHz, double dutyCycle = 0.5);
    void start();
    void stop();
    void playTone(int frequencyHz, int durationMs, double dutyCycle = 0.5);
    void playMelody(const std::vector<Note> &melody, int gapMs = 20, double dutyCycle = 0.5);
};

#endif