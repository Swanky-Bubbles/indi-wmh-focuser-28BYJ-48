#include <indifocuser.h>
#include <indilogger.h>
#include "hatb_motor.h"

using namespace INDI;

static const char *DEVICE_NAME = "Waveshare Motor HAT(B) Focuser";

class HatBFocuser : public Focuser
{
public:
    HatBFocuser();
    virtual ~HatBFocuser() = default;

    const char *getDefaultName() override { return DEVICE_NAME; }

    bool initProperties() override;
    bool updateProperties() override;

    IPState MoveAbsFocuser(uint32_t target) override;
    IPState MoveRelFocuser(FocusDirection dir, uint32_t amount) override;
    bool AbortFocuser() override;

protected:
    bool saveConfigItems(FILE *fp) override;

private:
    std::unique_ptr<HatBMotor> motor;

    // Legacy INDI property API
    INumber FocusAbsPosN[1];
    INumberVectorProperty FocusAbsPosNP;

    INumber DelayN[1];
    INumberVectorProperty DelayNP;

    INumber MaxPosN[1];
    INumberVectorProperty MaxPosNP;

    int delayUs = 1500;
    int maxPos  = 50000;

    void doMove(int steps);
};

HatBFocuser::HatBFocuser()
{
    setSupportedConnections(CONNECTION_NONE);
}

bool HatBFocuser::initProperties()
{
    Focuser::initProperties();

    uint32_t cap = FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT;
    SetCapability(cap);

    // Absolute position
    IUFillNumber(&FocusAbsPosN[0], "POSITION", "Position", "%0.f", 0, maxPos, 1, 0);
    IUFillNumberVector(&FocusAbsPosNP, FocusAbsPosN, 1,
                       getDeviceName(), "FOCUS_ABSOLUTE_POSITION", "Absolute Position",
                       MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    // Delay
    IUFillNumber(&DelayN[0], "DELAY_US", "Delay (µs)", "%0.f", 200, 20000, 50, delayUs);
    IUFillNumberVector(&DelayNP, DelayN, 1,
                       getDeviceName(), "DELAY_PER_STEP", "Delay per step",
                       OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    // Max Position
    IUFillNumber(&MaxPosN[0], "MAX_POS", "Max Position", "%0.f", 1000, 200000, 100, maxPos);
    IUFillNumberVector(&MaxPosNP, MaxPosN, 1,
                       getDeviceName(), "MAX_POSITION", "Max Position",
                       OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    return true;
}

bool HatBFocuser::updateProperties()
{
    Focuser::updateProperties();

    if (isConnected())
    {
        defineNumber(&FocusAbsPosNP);
        defineNumber(&DelayNP);
        defineNumber(&MaxPosNP);

        if (!motor)
        {
            HatBMotor::Pins p;
            p.chip = 0;
            p.in1 = 12;
            p.in2 = 13;
            p.in3 = 19;
            p.in4 = 16;

            try {
                motor = std::make_unique<HatBMotor>(p, delayUs);
            }
            catch (const std::exception &e) {
                LOGF_ERROR("Motor init failed: %s", e.what());
                return false;
            }
        }
    }
    else
    {
        deleteProperty(FocusAbsPosNP.name);
        deleteProperty(DelayNP.name);
        deleteProperty(MaxPosNP.name);
    }

    return true;
}

bool HatBFocuser::saveConfigItems(FILE *fp)
{
    Focuser::saveConfigItems(fp);

    IUSaveConfigNumber(fp, &DelayNP);
    IUSaveConfigNumber(fp, &MaxPosNP);

    return true;
}

void HatBFocuser::doMove(int steps)
{
    if (!motor)
        return;

    delayUs = DelayN[0].value;
    motor->setDelayUs(delayUs);

    motor->moveSteps(steps);
}

IPState HatBFocuser::MoveAbsFocuser(uint32_t target)
{
    int32_t current = FocusAbsPosN[0].value;
    int32_t diff = target - current;

    doMove(diff);

    FocusAbsPosN[0].value = target;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    return IPS_OK;
}

IPState HatBFocuser::MoveRelFocuser(FocusDirection dir, uint32_t amount)
{
    int sign = (dir == FOCUS_INWARD) ? 1 : -1;
    int steps = sign * amount;

    doMove(steps);

    int32_t pos = FocusAbsPosN[0].value + steps;

    if (pos < 0) pos = 0;
    if (pos > MaxPosN[0].value) pos = MaxPosN[0].value;

    FocusAbsPosN[0].value = pos;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    return IPS_OK;
}

bool HatBFocuser::AbortFocuser()
{
    LOG_INFO("Abort requested");
    return true;
}

// C wrapper
static HatBFocuser hatBFocuser;

extern "C" {
void ISGetProperties(const char *dev) { hatBFocuser.ISGetProperties(dev); }
void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{ hatBFocuser.ISNewNumber(dev, name, values, names, n); }
void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{ hatBFocuser.ISNewSwitch(dev, name, states, names, n); }
void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{ hatBFocuser.ISNewText(dev, name, texts, names, n); }
void ISSnoopDevice(XMLEle *root)
{ hatBFocuser.ISSnoopDevice(root); }
}
