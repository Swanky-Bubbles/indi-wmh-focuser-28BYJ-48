#pragma once

#include <indidevapi.h>
#include <indicom.h>
#include <indifocuser.h>
#include <indibase.h>
#include <lgpio.h>
#include <string>

using namespace std;

class Waveshare28BYJ : public INDI::DefaultDevice
{
public:
    Waveshare28BYJ();
    ~Waveshare28BYJ();

    bool InitProperties() override;
    bool UpdateProperties() override;
    const char *getDefaultName() override { return "Waveshare_28BYJ"; }

    void ISNewSwitch(const char *name, const INDI::SwitchVector *svp) override;
    void ISNewNumber(const char *name, const INDI::NumberVector *nvp) override;

    bool connectHook() override;
    bool disconnectHook() override;

private:
    ISwitchVectorProperty *dir_prop = nullptr;
    ISwitchVectorProperty *abort_prop = nullptr;
    ISwitchVectorProperty *ms_prop = nullptr;

    void move_steps(long long steps);
    void write_status(const string &s);
};
