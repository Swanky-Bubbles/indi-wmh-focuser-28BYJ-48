#pragma once

#include <indi/indibase.h>
#include <indi/indidevapi.h>
#include <indi/base/indicom.h>
#include <string>

using namespace std;

class Waveshare28BYJ : public INDI::DefaultDevice
{
public:
    Waveshare28BYJ();
    ~Waveshare28BYJ();

    bool InitProperties() override;
    bool UpdateProperties() override;

    void ISNewSwitch(const char *name, const ISwitchVectorProperty *svp) override;
    void ISNewNumber(const char *name, const INumberVectorProperty *nvp) override;

    bool connectHook() override;
    bool disconnectHook() override;

private:
    // GPIO pins
    int step_pin = 17; // change to your wiring
    int dir_pin = 27;

    long current_position = 0;

    // Stepper motor control
    void move_steps(long long steps);
    void write_status(const string &s);

    // INDI properties
    ISwitchVectorProperty *dir_prop = nullptr;
    ISwitchVectorProperty *abort_prop = nullptr;
    INumberVectorProperty *steps_prop = nullptr;
};
