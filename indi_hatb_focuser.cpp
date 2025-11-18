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

protected:
    bool saveConfigItems(FILE *fp) override;

private:
    std::unique_ptr<HatBMotor> motor;
    std::mutex motorMutex;

    //-----------------------------
    // Config values
    //-----------------------------
    int delayUs = 1500;
    int maxPosition = 50000;

    //-----------------------------
    // INDI Properties (old API)
    //-----------------------------
    INumber DelayN[1];
    INumberVectorProperty DelayNP;

    INumber MaxPosN[1];
    INumberVectorProperty MaxPosNP;

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

    //-----------------------------
    // Focuser absolute position
    //-----------------------------
    IUFillNumber(&FocusAbsPosN[0], "FOCUS_ABSOLUTE_POSITION",
                 "Position", "%.0f",
                 0, maxPosition, 1, 0);

    IUFillNumberVector(&FocusAbsPosNP, FocusAbsPosN, 1,
                       getDeviceName(),
                       "ABS_FOCUS_POSITION",
                       "Focuser", MAIN_CONTROL_TAB,
                       IP_RW, 0, IPS_IDLE);

    //-----------------------------
    // Delay per step
    //-----------------------------
    IUFillNumber(&DelayN[0], "DELAY_US", "Delay (us)", "%.0f",
                 200, 20000, 100, delayUs);

    IUFillNumberVector(&DelayNP, DelayN, 1,
                       getDeviceName(),
                       "HAT_DELAY", "Motor",
                       OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    //-----------------------------
    // Max position
    //-----------------------------
    IUFillNumber(&MaxPosN[0], "MAX_POSITION", "Max position", "%.0f",
                 1000, 200000, 100, maxPosition);

    IUFillNumberVector(&MaxPosNP, MaxPosN, 1,
                       getDeviceName(),
                       "HAT_MAXPOS", "Motor",
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
            HatBMotor::Pins pins;
            pins.chip = 0;
            pins.in1 = 12;
            pins.in2 = 13;
            pins.in3 = 19;
            pins.in4 = 16;

            try {
                motor = std::make_unique<HatBMotor>(pins, delayUs);
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

    std::lock_guard<std::mutex> lk(motorMutex);

    delayUs = DelayN[0].value;
    motor->setDelayUs(delayUs);
    motor->moveSteps(steps);
}

IPState HatBFocuser::MoveAbsFocuser(uint32_t target)
{
    int32_t current = FocusAbsPosN[0].value;
    int32_t diff = static_cast<int32_t>(target) - current;

    doMove(diff);

    FocusAbsPosN[0].value = target;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    return IPS_OK;
}

IPState HatBFocuser::MoveRelFocuser(FocusDirection dir, uint32_t amount)
{
    int32_t sign = (dir == FOCUS_INWARD) ? 1 : -1;
    int32_t diff = static_cast<int32_t>(amount) * sign;

    doMove(diff);

    FocusAbsPosN[0].value += diff;

    if (FocusAbsPosN[0].value < 0)
        FocusAbsPosN[0].value = 0;
    if (FocusAbsPosN[0].value > MaxPosN[0].value)
        FocusAbsPosN[0].value = MaxPosN[0].value;

    IDSetNumber(&FocusAbsPosNP, nullptr);

    return IPS_OK;
}

bool HatBFocuser::AbortFocuser()
{
    LOG_INFO("Abort requested — instantaneous driver.");
    return true;
}

// Global instance
static HatBFocuser wsHatBFocuser;

extern "C" {
void ISGetProperties(const char *dev) { wsHatBFocuser.ISGetProperties(dev); }
void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{ wsHatBFocuser.ISNewNumber(dev, name, values, names, n); }
void ISNewSwitch(const char *dev, const char *name, ISState *s, char *names[], int n)
{ wsHatBFocuser.ISNewSwitch(dev, name, s, names, n); }
void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{ wsHatBFocuser.ISNewText(dev, name, texts, names, n); }
void ISSnoopDevice(XMLEle *root)
{ wsHatBFocuser.ISSnoopDevice(root); }
}
