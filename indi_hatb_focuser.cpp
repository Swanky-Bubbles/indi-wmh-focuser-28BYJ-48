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

    // Manual connect logic for INDI 2.1.6
    bool Connect() override;
    bool Disconnect() override;

protected:
    bool saveConfigItems(FILE *fp) override;

private:
    std::unique_ptr<HatBMotor> motor;

    // Legacy INDI Numbers
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
    // INDI 2.1.6 does NOT support CONNECTION_LOCAL
    // so we stay with NONE and implement Connect()/Disconnect()
    setSupportedConnections(CONNECTION_NONE);
}

bool HatBFocuser::Connect()
{
    LOG_INFO("Connecting to Waveshare Motor HAT(B) focuser.");

    // Mark device as connected
    setConnected(true);

    return true;
}

bool HatBFocuser::Disconnect()
{
    LOG_INFO("Disconnecting focuser.");

    motor.reset();
    setConnected(false);

    return true;
}

bool HatBFocuser::initProperties()
{
    Focuser::initProperties();

    uint32_t cap = FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT;
    SetCapability(cap);

    // Absolute Position
    IUFillNumber(&FocusAbsPosN[0], "POSITION", "Position", "%.0f",
                 0, maxPos, 1, 0);
    IUFillNumberVector(&FocusAbsPosNP, FocusAbsPosN, 1,
                       getDeviceName(), "FOCUS_ABSOLUTE_POSITION",
                       "Absolute Position",
                       MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    // Delay (µs)
    IUFillNumber(&DelayN[0], "DELAY_US", "Delay (µs)", "%.0f",
                 200, 20000, 50, delayUs);
    IUFillNumberVector(&DelayNP, DelayN, 1,
                       getDeviceName(), "DELAY_PER_STEP",
                       "Delay per step",
                       OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    // Maximum position
    IUFillNumber(&MaxPosN[0], "MAX_POS", "Max Position", "%.0f",
                 1000, 200000, 100, maxPos);
    IUFillNumberVector(&MaxPosNP, MaxPosN, 1,
                       getDeviceName(), "MAX_POSITION",
                       "Max Position",
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
                LOG_INFO("Motor initialized.");
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

        motor.reset();
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
    int32_t diff    = target - current;

    doMove(diff);

    FocusAbsPosN[0].value = target;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    return IPS_OK;
}

IPState HatBFocuser::MoveRelFocuser(FocusDirection dir, uint32_t amount)
{
    int sign  = (dir == FOCUS_INWARD) ? 1 : -1;
    int steps = sign * static_cast<int>(amount);

    doMove(steps);

    int32_t pos = FocusAbsPosN[0].value + steps;

    if (pos < 0)
        pos = 0;
    if (pos > MaxPosN[0].value)
        pos = MaxPosN[0].value;

    FocusAbsPosN[0].value = pos;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    return IPS_OK;
}

bool HatBFocuser::AbortFocuser()
{
    LOG_INFO("Abort requested.");
    return true;
}

// --- INDI C wrapper ---
static HatBFocuser hatBFocuser;

extern "C" {
void ISGetProperties(const char *dev) { hatBFocuser.ISGetProperties(dev); }
void ISNewNumber(const char *dev, const char *name, double vals[], char *names[], int n)
{ hatBFocuser.ISNewNumber(dev, name, vals, names, n); }
void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{ hatBFocuser.ISNewSwitch(dev, name, states, names, n); }
void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{ hatBFocuser.ISNewText(dev, name, texts, names, n); }
void ISSnoopDevice(XMLEle *root)
{ hatBFocuser.ISSnoopDevice(root); }
}
