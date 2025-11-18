#ifndef INDI_DRV8825_FOCUSER_H
#define INDI_DRV8825_FOCUSER_H

#include <indidevapi.h>
#include <indibase.h>
#include <indicom.h>
#include <cstring>
#include <string>

#include <gpiod.h>

class DRV8825Focuser : public INDI::DefaultDevice
{
public:
    DRV8825Focuser();
    ~DRV8825Focuser() override;

    bool InitProperties() override;
    bool UpdateProperties() override;

    void ISNewSwitch(const char *name, const ISwitchVectorProperty *svp) override;
    void ISNewNumber(const char *name, const INumberVectorProperty *nvp) override;

    bool connectHook() override;
    bool disconnectHook() override;

private:
    // Stepper properties
    ISwitchVectorProperty dir_prop{};   // Direction
    ISwitchVectorProperty abort_prop{}; // Abort
    INumberVectorProperty steps_prop{}; // Step count

    ITextVectorProperty status_prop{};  // Status messages

    // GPIO for DRV8825
    struct gpiod_chip *chip;
    struct gpiod_line *stepLine;
    struct gpiod_line *dirLine;
    struct gpiod_line *enableLine;

    int stepPin;
    int dirPin;
    int enablePin;

    bool motorEnabled;

    void moveSteps(long long steps);
    void writeStatus(const std::string &s);
};

#endif
