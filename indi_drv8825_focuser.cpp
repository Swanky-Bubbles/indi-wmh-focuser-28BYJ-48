#include "indi_drv8825_focuser.h"
#include <sstream>
#include <chrono>
#include <thread>

DRV8825Focuser::DRV8825Focuser() {}

DRV8825Focuser::~DRV8825Focuser() {
    disconnectHook();
}

bool DRV8825Focuser::InitProperties()
{
    IUFillSwitch(&dir_prop, "DIR", "Direction", "Forward", "Backward", 0, IP_RW, 0);
    IUFillSwitch(&abort_prop, "ABORT", "Abort", "Abort", nullptr, 0, IP_RW, 0);
    IUFillNumber(&steps_prop, "STEPS", "Steps", 0, 0, 10000, 0);
    IUFillText(&status_prop, "STATUS", "Status", "");

    defineSwitch(&dir_prop);
    defineSwitch(&abort_prop);
    defineNumber(&steps_prop);
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

bool DRV8825Focuser::connectHook()
{
    chip = gpiod_chip_open_by_name("gpiochip0"); // change if needed
    if (!chip) return false;

    step_line = gpiod_chip_get_line(chip, 21); // Step GPIO
    dir_line  = gpiod_chip_get_line(chip, 20); // Dir GPIO
    if (!step_line || !dir_line) return false;

    if (gpiod_line_request_output(step_line, "drv8825", 0) < 0) return false;
    if (gpiod_line_request_output(dir_line,  "drv8825", 0) < 0) return false;

    writeStatus("Connected");
    return true;
}

bool DRV8825Focuser::disconnectHook()
{
    running = false;
    abort_flag = true;
    if (stepper_thread.joinable())
        stepper_thread.join();

    if (step_line) gpiod_line_release(step_line);
    if (dir_line) gpiod_line_release(dir_line);
    if (chip) gpiod_chip_close(chip);

    writeStatus("Disconnected");
    return true;
}

void DRV8825Focuser::ISNewSwitch(const ISwitchVectorProperty *svp)
{
    if (!svp) return;

    if (!strcmp(svp->name, "DIR"))
    {
        direction = (svp->s[0].s == ISS_ON) ? 1 : -1;
    }
    else if (!strcmp(svp->name, "ABORT") && svp->s[0].s == ISS_ON)
    {
        abort_flag = true;
        writeStatus("Abort triggered");
    }
}

void DRV8825Focuser::ISNewNumber(const INumberVectorProperty *nvp)
{
    if (!nvp) return;

    if (!strcmp(nvp->name, "STEPS"))
    {
        move_steps_count = static_cast<long long>(nvp->n[0].value);
        abort_flag = false;

        if (stepper_thread.joinable())
            stepper_thread.join();

        stepper_thread = std::thread([this](){ moveSteps(move_steps_count); });
    }
}

void DRV8825Focuser::moveSteps(long long steps)
{
    running = true;
    int dir_value = (direction > 0) ? 1 : 0;
    gpiod_line_set_value(dir_line, dir_value);

    for (long long i = 0; i < steps && !abort_flag; i++)
    {
        stepMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(2)); // adjust step speed
    }

    running = false;
    writeStatus(abort_flag ? "Aborted" : "Done");
}

void DRV8825Focuser::stepMotor()
{
    gpiod_line_set_value(step_line, 1);
    std::this_threa_
}