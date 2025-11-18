#include <indifocuser.h>
#include <indilogger.h>

#include "hatb_motor.h"

#include <mutex>
#include <memory>

using namespace INDI;

static const char *DEVICE_NAME = "Waveshare Stepper HAT(B) Focuser";

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
    std::mutex motorMutex;

    // New API INDI properties
    PropertyNumber AbsPosNP;
    PropertyNumber DelayNP;
    PropertyNumber MaxPosNP;

    int delayUs = 1500;
    int maxPosition = 50000;

    void doMove(int steps);
};

HatBFocuser::HatBFocuser()
{
    setSupportedConnections(CONNECTION_NONE);
}

bool HatBFocuser::initProperties()
{
    Focuser::initProperties();

    // Capabilities
    uint32_t cap = FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT;
    SetCapability(cap);

    //-------------------------------------------------------------
    // ABSOLUTE POSITION (new property API)
    //-------------------------------------------------------------
    AbsPosNP = PropertyNumber()
        .add("POSITION", 0)
        .setLabel("Absolute Position")
        .setMinMaxStep(0, maxPosition, 1);

    defineProperty(AbsPosNP);

    //-------------------------------------------------------------
    // Delay (µs)
    //-------------------------------------------------------------
    DelayNP = PropertyNumber()
        .add("DELAY_US", delayUs)
        .setLabel("Delay per step (µs)")
        .setMinMaxStep(200, 20000, 50);

    defineProperty(DelayNP);

    //-------------------------------------------------------------
    // Max Position
    //-------------------------------------------------------------
    MaxPosNP = PropertyNumber()
        .add("MAX_POS", maxPosition)
        .setLabel("Max Position")
        .setMinMaxStep(1000, 200000, 100);

    defineProperty(MaxPosNP);

    return true;
}

bool HatBFocuser::updateProperties()
{
    Focuser::updateProperties();

    if (isConnected())
    {
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

    return true;
}

bool HatBFocuser::saveConfigItems(FILE *fp)
{
    Focuser::saveConfigItems(fp);

    IUSaveConfigNumber(fp, DelayNP.getNumber());
    IUSaveConfigNumber(fp, MaxPosNP.getNumber());

    return true;
}

void HatBFocuser::doMove(int steps)
{
    if (!motor)
        return;

    std::lock_guard<std::mutex> lk(motorMutex);

    delayUs = DelayNP[0].getValue();
    motor->setDelayUs(delayUs);

    motor->moveSteps(steps);
}

IPState HatBFocuser::MoveAbsFocuser(uint32_t target)
{
    int32_t current = AbsPosNP[0].getValue();
    int32_t diff = static_cast<int32_t>(target) - current;

    doMove(diff);

    AbsPosNP[0].setValue(target);
    applyChanges(AbsPosNP);

    return IPS_OK;
}

IPState HatBFocuser::MoveRelFocuser(FocusDirection dir, uint32_t amount)
{
    int s = (dir == FOCUS_INWARD) ? 1 : -1;
    int steps = s * static_cast<int>(amount);

    doMove(steps);

    int32_t pos = AbsPosNP[0].getValue() + steps;

    if (pos < 0) pos = 0;
    if (pos > MaxPosNP[0].getValue()) pos = MaxPosNP[0].getValue();

    AbsPosNP[0].setValue(pos);
    applyChanges(AbsPosNP);

    return IPS_OK;
}

bool HatBFocuser::AbortFocuser()
{
    LOG_INFO("Abort requested (instant-stop driver)");
    return true;
}

//-------------------------------------------------------------
// C wrapper
//-------------------------------------------------------------
static HatBFocuser wsHatBFocuser;

extern "C" {
void ISGetProperties(const char *dev) { wsHatBFocuser.ISGetProperties(dev); }
void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{ wsHatBFocuser.ISNewNumber(dev, name, values, names, n); }
void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{ wsHatBFocuser.ISNewSwitch(dev, name, states, names, n); }
void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{ wsHatBFocuser.ISNewText(dev, name, texts, names, n); }
void ISSnoopDevice(XMLEle *root)
{ wsHatBFocuser.ISSnoopDevice(root); }
}
