#include "indi_waveshare_28byj48.h"
#include <unistd.h>
#include <chrono>
#include <thread>

Waveshare28BYJ::Waveshare28BYJ() : gpio_handle(-1) {}

Waveshare28BYJ::~Waveshare28BYJ() { disconnectHook(); }

bool Waveshare28BYJ::initProperties()
{
    // Position property
    defineNumber(&position_prop, "POSITION", "Position", current_position, 0, 10000);

    // Direction property
    defineSwitch(&direction_prop, "DIRECTION", "Direction", {"Forward", "Backward"}, 1);

    // Abort property
    defineSwitch(&abort_prop, "ABORT", "Abort", {"Abort"}, 0);

    // Microstep property
    defineSwitch(&microstep_prop, "MICROSTEP", "Microstep", {"1", "2", "4", "8"}, 1);

    return true;
}

bool Waveshare28BYJ::updateProperties()
{
    if (isConnected())
    {
        defineNumber(position_prop);
        defineSwitch(direction_prop);
        defineSwitch(abort_prop);
        defineSwitch(microstep_prop);
    }
    else
    {
        deleteProperty(position_prop);
        deleteProperty(direction_prop);
        deleteProperty(abort_prop);
        deleteProperty(microstep_prop);
    }
    return true;
}

bool Waveshare28BYJ::connectHook()
{
    gpio_handle = lgGpiochipOpen(0);
    if (gpio_handle < 0)
    {
        write_status("Failed to open GPIO chip");
        return false;
    }
    write_status("Connected to Waveshare 28BYJ");
    return true;
}

bool Waveshare28BYJ::disconnectHook()
{
    if (gpio_handle >= 0)
    {
        lgGpiochipClose(gpio_handle);
        gpio_handle = -1;
    }
    write_status("Disconnected");
    return true;
}

void Waveshare28BYJ::ISNewSwitch(const char *dev, const char *name, const IUSwitchVectorProperty *svp)
{
    if (IUFindSwitch(svp, "DIRECTION"))
    {
        // read direction
    }
    else if (IUFindSwitch(svp, "ABORT"))
    {
        // handle abort
    }
    else if (IUFindSwitch(svp, "MICROSTEP"))
    {
        // set microstepping
    }
    IDSetSwitch
