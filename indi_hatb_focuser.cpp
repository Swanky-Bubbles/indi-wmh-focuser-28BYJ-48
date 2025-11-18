#include <indifocuser.h>
#include <indilogger.h>
#include "hatb_motor.h"

#include <memory>
#include <mutex>

using namespace INDI;

static const char *DEVICE_NAME = "Waveshare Stepper HAT(B) Focuser";

// One global instance of the device
class HatBFocuser : public Focuser
{
public:
    HatBFocuser();
    virtual ~HatBFocuser() override = default;

    const char *getDefaultName() override { return DEVICE_NAME; }

    bool initProperties() override;
    bool updateProperties() override;

    // Focuser interface overrides:
    IPState MoveAbsFocuser(uint32_t targetTicks) override;
    IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
    bool AbortFocuser() override;

    bool Handshake() override { return true; }

protected:
    bool saveConfigItems(FILE *fp) override;

private:
    // Motor + movement bookkeeping
    std::unique_ptr<HatBMotor> motor;
    std::mutex motorMutex;

    // Parameters
    uint32_t stepsPerRev     = 2048;   // 28BYJ-48 with HR8825 full-step
    double   micronsPerStep  = 1.0;    // user-tunable scale
    int      delayUsPerStep  = 800;

    // Properties
    INumber  StepsPerRevN[1];
    INumberVectorProperty StepsPerRevNP;

    INumber  MicronsPerStepN[1];
    INumberVectorProperty MicronsPerStepNP;

    INumber  DelayPerStepN[1];
    INumberVectorProperty DelayPerStepNP;

    // Internal helpers
    void doMove(int32_t steps);
};

HatBFocuser::HatBFocuser()
{
    // This focuser is purely GPIO; no serial/tcp
    setSupportedConnections(Focuser::CONNECTION_NONE);
}

bool HatBFocuser::initProperties()
{
    Focuser::initProperties();

    // Declare we support abs + rel move + abort
    uint32_t cap = 0;
    cap |= FOCUSER_CAN_ABS_MOVE;
    cap |= FOCUSER_CAN_REL_MOVE;
    cap |= FOCUSER_CAN_ABORT;
    SetCapability(cap);

    // Default position range: 0..50000 steps
    FocusAbsPosN[0].min = 0;
    FocusAbsPosN[0].max = 50000;
    FocusAbsPosN[0].value = 0;

    // StepsPerRev
    IUFillNumber(&StepsPerRevN[0], "STEPS_PER_REV", "Steps per rev", "%.0f", 100, 100000, 1, stepsPerRev);
    IUFillNumberVector(&StepsPerRevNP, StepsPerRevN, 1, getDeviceName(),
                       "HAT_STEPS_PER_REV", "Stepper", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    // MicronsPerStep
    IUFillNumber(&MicronsPerStepN[0], "MICRONS_PER_STEP", "µm per step", "%.3f",
                 0.001, 100.0, 0.001, micronsPerStep);
    IUFillNumberVector(&MicronsPerStepNP, MicronsPerStepN, 1, getDeviceName(),
                       "HAT_MICRONS_PER_STEP", "Stepper", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    // Delay per step
    IUFillNumber(&DelayPerStepN[0], "DELAY_US_PER_STEP", "Delay µs/half-cycle", "%.0f",
                 100, 50000, 10, delayUsPerStep);
    IUFillNumberVector(&DelayPerStepNP, DelayPerStepN, 1, getDeviceName(),
                       "HAT_DELAY_PER_STEP", "Stepper", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    return true;
}

bool HatBFocuser::updateProperties()
{
    Focuser::updateProperties();

    if (isConnected())
    {
        defineNumber(&StepsPerRevNP);
        defineNumber(&MicronsPerStepNP);
        defineNumber(&DelayPerStepNP);

        // Lazy-init motor on first connection
        if (!motor)
        {
            try
            {
                HatBMotor::Pins pins;
                // pins defaults are for channel 2, /dev/gpiochip0
                motor = std::make_unique<HatBMotor>(pins, delayUsPerStep);
            }
            catch (const std::exception &e)
            {
                LOGF_ERROR("Failed to init HAT(B) motor: %s", e.what());
                return false;
            }
        }
    }
    else
    {
        deleteProperty(StepsPerRevNP.name);
        deleteProperty(MicronsPerStepNP.name);
        deleteProperty(DelayPerStepNP.name);
    }

    return true;
}

bool HatBFocuser::saveConfigItems(FILE *fp)
{
    Focuser::saveConfigItems(fp);
    IUSaveConfigNumber(fp, &StepsPerRevNP);
    IUSaveConfigNumber(fp, &MicronsPerStepNP);
    IUSaveConfigNumber(fp, &DelayPerStepNP);
    return true;
}

void HatBFocuser::doMove(int32_t steps)
{
    if (!motor)
        return;

    std::lock_guard<std::mutex> lk(motorMutex);

    motor->setDelayUsPerStep(delayUsPerStep);
    motor->moveSteps(steps);
}

IPState HatBFocuser::MoveAbsFocuser(uint32_t targetTicks)
{
    // Current absolute position
    int32_t current = static_cast<int32_t>(FocusAbsPosN[0].value);
    int32_t target  = static_cast<int32_t>(targetTicks);

    int32_t delta = target - current;
    if (delta == 0)
        return IPS_OK;

    doMove(delta);

    FocusAbsPosN[0].value = target;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    return IPS_OK;
}

IPState HatBFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    int32_t sign = (dir == FOCUS_INWARD) ? 1 : -1;
    int32_t steps = sign * static_cast<int32_t>(ticks);

    doMove(steps);

    FocusAbsPosN[0].value += steps;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    return IPS_OK;
}

bool HatBFocuser::AbortFocuser()
{
    // This implementation is synchronous: moves finish before we return.
    // If you later move to a threaded implementation, you’ll set a flag here.
    LOG_INFO("Abort requested (no active motion).");
    return true;
}
// Global device instance
static HatBFocuser wsHatBFocuser;

extern "C" {

void ISGetProperties(const char *dev)
{
    wsHatBFocuser.ISGetProperties(dev);
}

void ISNewNumber(const char *dev, const char *name,
                 double values[], char *names[], int n)
{
    wsHatBFocuser.ISNewNumber(dev, name, values, names, n);
}

void ISNewSwitch(const char *dev, const char *name,
                 ISState *states, char *names[], int n)
{
    wsHatBFocuser.ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name,
               char *texts[], char *names[], int n)
{
    wsHatBFocuser.ISNewText(dev, name, texts, names, n);
}

void ISSnoopDevice(XMLEle *root)
{
    wsHatBFocuser.ISSnoopDevice(root);
}

}
