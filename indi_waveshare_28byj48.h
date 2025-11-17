#ifndef INDIDEV_WAVESHARE_28BYJ48_H
#define INDIDEV_WAVESHARE_28BYJ48_H

#include <indifocuser.h>
#include <lgpio.h>
#include <string>

using namespace std;

class Waveshare28BYJ : public INDI::Focuser
{
public:
    Waveshare28BYJ();
    ~Waveshare28BYJ();

    bool initProperties() override;
    bool updateProperties() override;

    const char *getDefaultName() override { return "Waveshare_28BYJ"; }

    void ISNewSwitch(const char *dev, const char *name, const IUSwitchVectorProperty *svp) override;
    void ISNewNumber(const char *dev, const char *name, const IUNumberVectorProperty *nvp) override;

    bool connectHook() override;
    bool disconnectHook() override;

private:
    int gpio_handle;
    long long current_position = 0;
    long long target_position = 0;

    IUSwitchVectorProperty *direction_prop = nullptr;
    IUSwitchVectorProperty *abort_prop = nullptr;
    IUSwitchVectorProperty *microstep_prop = nullptr;
    IUNumberVectorProperty *position_prop = nullptr;

    void move_steps(long long steps);
    void write_status(const string &s);
};

#endif
