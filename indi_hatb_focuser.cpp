#include <indifocuser.h>
#include <indilogger.h>
#include "hatb_motor.h"

#include <memory>
#include <mutex>
#include <cstdint>

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

    // Connection-less device
    bool Connect() override;
    bool Disconnect() override;

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

    // Internal absolute position (in "ticks"/steps)
    int32_t  positionTicks   = 0;

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
    // Declare this device’s interface
    setDriverInterface(FOCUSER_INTERFACE);

    // Pure GPIO; no serial/tcp connection plugins
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

    // NOTE: We no longer touch FocusAbsPosN here; newer libindi hides it.

    // StepsPerRev
    IUFillNumber(&StepsPerRevN[0], "STEPS_PER_REV", "Steps per rev", "%.0f",
                 100, 100000, 1, stepsPerRev);
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

bool HatBFocuser::Connect()
{
    // For a GPIO-only driver, there is nothing to "open" here.
    LOG_INFO("Connecting to Waveshare Stepper HAT(B) Focuser (GPIO).");
    // Returning true tells INDI the connection succeeded; it will
    // mark the device connected and call updateProperties().
    return true;
}

bool HatBFocuser::Disconnect()
{
    LOG_INFO("Disconnecting Waveshare Stepper HAT(B) Focuser.");
    // If desired, you could reset or release the motor here.
    // motor.reset();
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
    // Use our internal absolute position counter instead of FocusAbsPosN
    int32_t target  = static_cast<int32_t>(targetTicks);
    int32_t current = positionTicks;

    int32_t delta = target - current;
    if (delta == 0)
        return IPS_OK;

    doMove(delta);

    // Update internal position
    positionTicks = target;

    // We don't touch FocusAbsPosNP directly (new libindi API hides it).
    return IPS_OK;
}

IPState HatBFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    int32_t sign  = (dir == FOCUS_INWARD) ? 1 : -1;
    int32_t steps = sign * static_cast<int32_t>(ticks);

    doMove(steps);

    // Update internal absolute position
    positionTicks += steps;

    // Again, we don't touch FocusAbsPosNP directly.
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
