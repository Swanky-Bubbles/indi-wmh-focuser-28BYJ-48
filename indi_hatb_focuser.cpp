#include <indifocuser.h>
#include <indilogger.h>

#include "hatb_motor.h"

#include <memory>
#include <mutex>

using namespace INDI;

static const char *DEVICE_NAME = "Waveshare Stepper HAT(B) Focuser";

class HatBFocuser : public Focuser
{
public:
    HatBFocuser();
    virtual ~HatBFocuser() override = default;

    const char *getDefaultName() override { return DEVICE_NAME; }

    bool initProperties() override;
    bool updateProperties() override;

    IPState MoveAbsFocuser(uint32_t target) override;
    IPState MoveRelFocuser(FocusDirection dir, uint32_t amount) override;
    bool AbortFocuser() override;

    bool Handshake() override { return true; }

protected:
    bool saveConfigItems(FILE *fp) override;

private:
    std::unique_ptr<HatBMotor> motor;
    std::mutex motorMutex;

    // default parameters
    int delayUs = 1500;   // 1.5 ms per half-step
    int maxPosition = 50000;

    // config properties
    INumber DelayPerStepN[1];
    INumberVectorProperty DelayPerStepNP;

    INumber MaxPositionN[1];
    INumberVectorProperty MaxPositionNP;

    void doMove(int steps);
};

HatBFocuser::HatBFocuser()
{
    setSupportedConnections(Focuser::CONNECTION_NONE);
}

bool HatBFocuser::initProperties()
{
    Focuser::initProperties();

    // Capability: abs + rel + abort
    uint32_t cap = 0;
    cap |= FOCUSER_CAN_ABS_MOVE;
    cap |= FOCUSER_CAN_REL_MOVE;
    cap |= FOCUSER_CAN_ABORT;
    SetCapability(cap);

    // Absolute position property defaults
    FocusAbsPosN[0].min   = 0;
    FocusAbsPosN[0].max   = maxPosition;
    FocusAbsPosN[0].value = 0;

    // Delay per step (μs)
    IUFillNumber(&DelayPerStepN[0], "DELAY_US", "Delay per step (us)",
                 "%.0f", 500, 20000, 100, delayUs);
    IUFillNumberVector(&DelayPerStepNP, DelayPerStepN, 1, getDeviceName(),
                       "HAT_DELAY", "Motor", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    // Max travel setting
    IUFillNumber(&MaxPositionN[0], "MAX_POSITION", "Max position (steps)",
                 "%.0f", 1000, 200000, 100, maxPosition);
    IUFillNumberVector(&MaxPositionNP, MaxPositionN, 1, getDeviceName(),
                       "HAT_MAXPOS", "Motor", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    return true;
}

bool HatBFocuser::updateProperties()
{
    Focuser::updateProperties();

    if (isConnected())
    {
        defineNumber(&DelayPerStepNP);
        defineNumber(&MaxPositionNP);

        if (!motor)
        {
            HatBMotor::Pins pins;
            pins.chip = 0;      // /dev/gpiochip0
            pins.in1  = 12;     // Motor 1
            pins.in2  = 13;
            pins.in3  = 19;
            pins.in4  = 16;

            try
            {
                motor = std::make_unique<HatBMotor>(pins, delayUs);
            }
            catch (const std::exception &e)
            {
                LOGF_ERROR("Motor init failed: %s", e.what());
                return false;
            }
        }
    }
    else
    {
        deleteProperty(DelayPerStepNP.name);
        deleteProperty(MaxPositionNP.name);
    }

    return true;
}

bool HatBFocuser::saveConfigItems(FILE *fp)
{
    Focuser::saveConfigItems(fp);
    IUSaveConfigNumber(fp, &DelayPerStepNP);
    IUSaveConfigNumber(fp, &MaxPositionNP);
    return true;
}

void HatBFocuser::doMove(int steps)
{
    if (!motor)
        return;

    std::lock_guard<std::mutex> lk(motorMutex);

    motor->setDelayUs(delayUs);
    motor->moveSteps(steps);
}

IPState HatBFocuser::MoveAbsFocuser(uint32_t target)
{
    int32_t current = FocusAbsPosN[0].value;
    int32_t dest    = target;

    int32_t delta = dest - current;
    if (delta == 0)
        return IPS_OK;

    doMove(delta);

    FocusAbsPosN[0].value = dest;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    return IPS_OK;
}

IPState HatBFocuser::MoveRelFocuser(FocusDirection dir, uint32_t amount)
{
    int32_t sign = (dir == FOCUS_INWARD) ? 1 : -1;
    int32_t steps = static_cast<int32_t>(amount) * sign;

    doMove(steps);

    FocusAbsPosN[0].value += steps;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    return IPS_OK;
}

bool HatBFocuser::AbortFocuser()
{
    LOG_INFO("Abort requested (blocking driver — completes moves before return).");
    return true;
}

// --- global instance + C linkage ---
static HatBFocuser HatBInstance;

extern "C" {

void ISGetProperties(const char *dev) { HatBInstance.ISGetProperties(dev); }
void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
    { HatBInstance.ISNewNumber(dev, name, values, names, n); }
void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
    { HatBInstance.ISNewSwitch(dev, name, states, names, n); }
void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
    { HatBInstance.ISNewText(dev, name, texts, names, n); }
void ISSnoopDevice(XMLEle *root) { HatBInstance.ISSnoopDevice(root); }

}
