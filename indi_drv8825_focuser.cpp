#include "indi_drv8825_focuser.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>

DRV8825Focuser::DRV8825Focuser()
    : stepPin(17), dirPin(27), enablePin(22), motorEnabled(false),
      chip(nullptr), stepLine(nullptr), dirLine(nullptr), enableLine(nullptr)
{
}

DRV8825Focuser::~DRV8825Focuser()
{
    disconnectHook();
}

bool DRV8825Focuser::InitProperties()
{
    IUFillSwitch(&dir_prop, "DIR", "Direction", "Forward", "Backward", 0, IP_RW, 0);
    IUFillSwitch(&abort_prop, "ABORT", "Abort", "Abort", nullptr, 0, IP_RW, 0);
    IUFillNumber(&steps_prop, "STEPS", "Steps", 0, 0, 10000, 0);

    defineSwitch(&dir_prop);
    defineSwitch(&abort_prop);
    defineNumber(&steps_prop);

    IUFillText(&status_prop, "STATUS", "Status", "");
    defineText(&status_prop);

    return true;
}

bool DRV8825Focuser::UpdateProperties()
{
    if (isConnected())
    {
        defineSwitch(&dir_prop);
        defineSwitch(&abort_prop);
        defineNumber(&steps_prop);
        defineText(&status_prop);
    }
    else
    {
        deleteProperty(&dir_prop);
        deleteProperty(&abort_prop);
        deleteProperty(&steps_prop);
        deleteProperty(&status_prop);
    }
    return true;
}

void DRV8825Focuser::ISNewSwitch(const char *name, const ISwitchVectorProperty *svp)
{
    if (!strcmp(name, "DIR"))
    {
        if (svp->s[0].s == ISS_ON)
            gpiod_line_set_value(dirLine, 1);
        else
            gpiod_line_set_value(dirLine, 0);
    }

    if (!strcmp(name, "ABORT") && svp->s[0].s == ISS_ON)
    {
        writeStatus("Abort pressed");
    }
}

void DRV8825Focuser::ISNewNumber(const char *name, const INumberVectorProperty *nvp)
{
    if (!strcmp(name, "STEPS"))
    {
        moveSteps(static_cast<long long>(nvp->n[0].value));
    }
}

bool DRV8825Focuser::connectHook()
{
    chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip)
    {
        writeStatus("Failed to open GPIO chip");
        return false;
    }

    stepLine = gpiod_chip_get_line(chip, stepPin);
    dirLine  = gpiod_chip_get_line(chip, dirPin);
    enableLine = gpiod_chip_get_line(chip, enablePin);

    if (!stepLine || !dirLine || !enableLine)
    {
        writeStatus("Failed to get GPIO lines");
        return false;
    }

    gpiod_line_request_output(stepLine, "drv8825_step", 0);
    gpiod_line_request_output(dirLine, "drv8825_dir", 0);
    gpiod_line_request_output(enableLine, "drv8825_enable", 0);

    gpiod_line_set_value(enableLine, 0); // enable motor
    motorEnabled = true;
    writeStatus("DRV8825 connected");

    return true;
}

bool DRV8825Focuser::disconnectHook()
{
    if (motorEnabled)
    {
        gpiod_line_set_value(enableLine, 1); // disable motor
        motorEnabled = false;
    }

    if (stepLine) gpiod_line_release(stepLine);
    if (dirLine) gpiod_line_release(dirLine);
    if (enableLine) gpiod_line_release(enableLine);
    if (chip) gpiod_chip_close(chip);

    writeStatus("DRV8825 disconnected");
    return true;
}

void DRV8825Focuser::moveSteps(long long steps)
{
    if (!motorEnabled)
        return;

    writeStatus("Moving steps...");

    for (long long i = 0; i < steps; ++i)
    {
        gpiod_line_set_value(stepLine, 1);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        gpiod_line_set_value(stepLine, 0);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    writeStatus("Move complete");
}

void DRV8825Focuser::writeStatus(const std::string &s)
{
    IDSetText(&status_prop, "STATUS", s.c_str());
}
