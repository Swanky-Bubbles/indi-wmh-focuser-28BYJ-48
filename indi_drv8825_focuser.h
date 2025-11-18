#pragma once

#include <indidevapi.h>
#include <cstring>
#include <string>
#include <gpiod.h>
#include <thread>
#include <atomic>

class DRV8825Focuser : public INDI::DefaultDevice
{
public:
    DRV8825Focuser();
    ~DRV8825Focuser() override;

    bool InitProperties() override;
    bool UpdateProperties() override;

    void ISNewSwitch(const ISwitchVectorProperty *svp) override;
    void ISNewNumber(const INumberVectorProperty *nvp) override;

    bool connectHook() override;
    bool disconnectHook() override;

private:
    ISwitchVectorProperty *dir_prop = nullptr;
    ISwitchVectorProperty *abort_prop = nullptr;
    INumberVectorProperty *steps_prop = nullptr;
    ITextVectorProperty *status_prop = nullptr;

    struct gpiod_chip *chip = nullptr;
    struct gpiod_line *step_line = nullptr;
    struct gpiod_line *dir_line = nullptr;

    std::thread stepper_thread;
    std::atomic<bool> running {false};
    std::atomic<bool> abort_flag {false};
    long long move_steps_count = 0;
    int direction = 1;

    void moveSteps(long long steps);
    void writeStatus(const std::string &s);
    void stepMotor();
};
