#include "indi_waveshare_28byj48.h"
#include <fstream>
#include <thread>
#include <chrono>
#include <sstream>

Waveshare28BYJ::Waveshare28BYJ() {}
Waveshare28BYJ::~Waveshare28BYJ() { disconnectHook(); }

bool Waveshare28BYJ::InitProperties()
{
    // Initialize properties
    IDSetSwitch(&dir_prop, "DIR", "Direction", "Forward", "Backward", 0, IP_RW, 0);
    IDSetSwitch(&abort_prop, "ABORT", "Abort", "Abort", nullptr, 0, IP_RW, 0);
    IDSetNumber(&steps_prop, "STEPS", "Steps", 0, 0, 10000, 0);

    return true;
}

bool Waveshare28BYJ::UpdateProperties()
{
    // Add properties to INDI
    if (isConnected())
    {
        defineSwitch(&dir_prop);
        defineSwitch(&abort_prop);
        defineNumber(&steps_prop);
    }
    else
    {
        deleteProperty(dir_prop);
        deleteProperty(abort_prop);
        deleteProperty(steps_prop);
    }
    return true;
}

// GPIO helpers
void write_gpio(int pin, int value)
{
    std::ofstream gpio("/sys/class/gpio/gpio" + std::to_string(pin) + "/value");
    if (gpio.is_open())
        gpio << value;
}

void export_gpio(int pin)
{
    std::ofstream export_file("/sys/class/gpio/export");
    if (export_file.is_open())
        export_file << pin;

    std::ofstream dir_file("/sys/class/gpio/gpio" + std::to_string(pin) + "/direction");
    if (dir_file.is_open())
        dir_file << "out";
}

bool Waveshare28BYJ::connectHook()
{
    export_gpio(step_pin);
    export_gpio(dir_pin);
    write_status("Connected to DRV8825 stepper.");
    return true;
}

bool Waveshare28BYJ::disconnectHook()
{
    write_status("Disconnected.");
    return true;
}

void Waveshare28BYJ::move_steps(long long steps)
{
    bool direction = steps >= 0;
    write_gpio(dir_pin, direction ? 1 : 0);
    steps = std::abs(steps);

    for (long long i = 0; i < steps; i++)
    {
        write_gpio(step_pin, 1);
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        write_gpio(step_pin, 0);
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    current_position += direction ? steps : -steps;
}

void Waveshare28BYJ::write_status(const string &s)
{
    IDSetText(&IUTextVectorProperty, "STATUS", s.c_str());
}

void Waveshare28BYJ::ISNewSwitch(const char *name, const ISwitchVectorProperty *svp)
{
    // Handle switch events (direction / abort)
}

void Waveshare28BYJ::ISNewNumber(const char *name, const INumberVectorProperty *nvp)
{
    if (!strcmp(name, "STEPS"))
    {
        move_steps(static_cast<long long>(nvp->np[0].value));
    }
}
