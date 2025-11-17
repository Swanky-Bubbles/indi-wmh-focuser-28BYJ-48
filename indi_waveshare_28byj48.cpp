/*
 * INDI driver for Waveshare Motor HAT controlling 28BYJ-48 stepper motors
 * Target: INDI 2.1.6+ (API compatible)
 * STEP/DIR pulse output version
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

#include <indi/indidevapi.h>
#include <indi/base/indicom.h>
#include <lgpio.h>

using namespace std;

static const int DEFAULT_STEP_DELAY_US = 2000; // microseconds

struct MotorPins {
    int dir, step, enable;
    int mode0, mode1, mode2; // microstep pins
};

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
    // INDI properties
    IUSwitchVectorProperty *enable_prop = nullptr;
    IUSwitchVectorProperty *dir_prop = nullptr;
    INumberVectorProperty *steps_prop = nullptr;
    INumberVectorProperty *pos_prop = nullptr;
    INumberVectorProperty *speed_prop = nullptr;
    IUSwitchVectorProperty *abort_prop = nullptr;
    IUSwitchVectorProperty *ms_prop = nullptr;
    ITextVectorProperty *status_prop = nullptr;

    MotorPins pins;
    int lgpio_handle = 0;

    atomic<bool> connected{false};
    atomic<bool> stop_requested{false};
    atomic<long long> position{0};
    mutex hw_mutex;
    thread worker_thread;

    void move_steps(long long steps);
    void write_status(const string &s);
};

Waveshare28BYJ::Waveshare28BYJ() {}
Waveshare28BYJ::~Waveshare28BYJ() { disconnectHook(); }

bool Waveshare28BYJ::InitProperties()
{
    // Motor enable
    enable_prop = IUFillSwitch(&enable_prop, "MOTOR_ENABLE", "Enable Motor", "WAVESHARE_STEP", 2);
    IUSwitch(&enable_prop->sp[0], "ON", "On", ISS_OFF);
    IUSwitch(&enable_prop->sp[1], "OFF", "Off", ISS_ON);
    defineProperty(enable_prop);

    // Direction
    dir_prop = IUFillSwitch(&dir_prop, "MOTOR_DIR", "Direction", "WAVESHARE_STEP", 2);
    IUSwitch(&dir_prop->sp[0], "FORWARD", "Forward", ISS_ON);
    IUSwitch(&dir_prop->sp[1], "REVERSE", "Reverse", ISS_OFF);
    defineProperty(dir_prop);

    // Steps
    steps_prop = IUFillNumber(&steps_prop, "STEPS", "Steps to Move", "WAVESHARE_STEP", 1);
    IUFillNumber(&steps_prop->np[0], "STEPS_TO_MOVE", "Steps", 0, -1000000, 1000000, 1);
    defineProperty(steps_prop);

    // Position
    pos_prop = IUFillNumber(&pos_prop, "POSITION", "Position (steps)", "WAVESHARE_STEP", 1);
    IUFillNumber(&pos_prop->np[0], "ABS_POSITION", "Position", 0, -10000000, 10000000, 1);
    defineProperty(pos_prop);

    // Speed (step delay)
    speed_prop = IUFillNumber(&speed_prop, "SPEED", "Step Delay (us)", "WAVESHARE_STEP", 1);
    IUFillNumber(&speed_prop->np[0], "DELAY_US", "Delay (µs)", DEFAULT_STEP_DELAY_US, 50, 1000000, 1);
    defineProperty(speed_prop);

    // Abort
    abort_prop = IUFillSwitch(&abort_prop, "ABORT", "Abort", "WAVESHARE_STEP", 1);
    IUSwitch(&abort_prop->sp[0], "ABORT_NOW", "Abort", ISS_OFF);
    defineProperty(abort_prop);

    // Microstep override
    ms_prop = IUFillSwitch(&ms_prop, "MICROSTEP_MODE", "Microstep Mode", "WAVESHARE_STEP", 7);
    IUSwitch(&ms_prop->sp[0], "DIP", "Use DIP", ISS_ON);
    IUSwitch(&ms_prop->sp[1], "FULL", "Full", ISS_OFF);
    IUSwitch(&ms_prop->sp[2], "HALF", "1/2", ISS_OFF);
    IUSwitch(&ms_prop->sp[3], "QUARTER", "1/4", ISS_OFF);
    IUSwitch(&ms_prop->sp[4], "EIGHTH", "1/8", ISS_OFF);
    IUSwitch(&ms_prop->sp[5], "SIXTEENTH", "1/16", ISS_OFF);
    IUSwitch(&ms_prop->sp[6], "THIRTYSECOND", "1/32", ISS_OFF);
    defineProperty(ms_prop);

    // Status
    status_prop = IUFillText(&status_prop, "STATUS", "Status", "WAVESHARE_STEP", 1);
    IUFillText(&status_prop->tp[0], "MESSAGE", "Idle");
    defineProperty(status_prop);

    return true;
}

bool Waveshare28BYJ::UpdateProperties()
{
    if (isConnected()) {
        defineProperty(enable_prop);
        defineProperty(dir_prop);
        defineProperty(steps_prop);
        defineProperty(pos_prop);
        defineProperty(speed_prop);
        defineProperty(abort_prop);
        defineProperty(ms_prop);
        defineProperty(status_prop);
    } else {
        deleteProperty(enable_prop);
        deleteProperty(dir_prop);
        deleteProperty(steps_prop);
        deleteProperty(pos_prop);
        deleteProperty(speed_prop);
        deleteProperty(abort_prop);
        deleteProperty(ms_prop);
        deleteProperty(status_prop);
    }
    return true;
}

void Waveshare28BYJ::ISNewSwitch(const char *name, const INDI::SwitchVector *svp)
{
    lock_guard<mutex> lk(hw_mutex);
    if (!strcmp(name, "MOTOR_ENABLE")) {
        if (svp->sp[0].s == ISS_ON) {
            connected = true;
            lgWrite(lgpio_handle, pins.enable, 0); // enable motor (LOW)
            write_status("Motor enabled");
        } else {
            connected = false;
            stop_requested = true;
            lgWrite(lgpio_handle, pins.enable, 1); // disable motor (HIGH)
            write_status("Motor disabled");
        }
    } else if (!strcmp(name, "ABORT")) {
        if (svp->sp[0].s == ISS_ON) {
            stop_requested = true;
            write_status("Abort requested");
            IDSetSwitch(&abort_prop->sp[0], "ABORT_NOW", ISS_OFF);
            sendNewSwitch(abort_prop);
        }
    }
}

void Waveshare28BYJ::ISNewNumber(const char *name, const INDI::NumberVector *nvp)
{
    lock_guard<mutex> lk(hw_mutex);
    if (!strcmp(name, "STEPS")) {
        long long steps_to_move = (long long)nvp->np[0].value;
        if (steps_to_move == 0 || !connected) return;

        if (worker_thread.joinable())
            worker_thread.join();

        stop_requested = false;
        worker_thread = thread(&Waveshare28BYJ::move_steps, this, steps_to_move);
    }
}

bool Waveshare28BYJ::connectHook()
{
    lock_guard<mutex> lk(hw_mutex);
    lgpio_handle = lgGpiochipOpen(0);
    if (lgpio_handle < 0) {
        write_status("Failed to open lgpio");
        return false;
    }

    // BCM pins mapping
    pins.dir = 13;
    pins.step = 19;
    pins.enable = 12;
    pins.mode0 = 16;
    pins.mode1 = 17;
    pins.mode2 = 20;

    lgSetDirection(lgpio_handle, pins.dir, LG_OUTPUT);
    lgSetDirection(lgpio_handle, pins.step, LG_OUTPUT);
    lgSetDirection(lgpio_handle, pins.enable, LG_OUTPUT);
    lgSetDirection(lgpio_handle, pins.mode0, LG_OUTPUT);
    lgSetDirection(lgpio_handle, pins.mode1, LG_OUTPUT);
    lgSetDirection(lgpio_handle, pins.mode2, LG_OUTPUT);

    // Disable motor initially
    lgWrite(lgpio_handle, pins.enable, 1);
    connected = true;
    write_status("Connected");

    return true;
}

bool Waveshare28BYJ::disconnectHook()
{
    lock_guard<mutex> lk(hw_mutex);
    stop_requested = true;
    if (worker_thread.joinable())
        worker_thread.join();

    // Disable motor
    if (lgpio_handle)
        lgWrite(lgpio_handle, pins.enable, 1);

    if (lgpio_handle)
        lgGpiochipClose(lgpio_handle);
    lgpio_handle = 0;
    connected = false;
    write_status("Disconnected");
    return true;
}

void Waveshare28BYJ::move_steps(long long steps)
{
    int dir = (steps >= 0) ? 1 : 0;
    lgWrite(lgpio_handle, pins.dir, dir);
    steps = llabs(steps);

    int delay_us = (int)speed_prop->np[0].value;

    for (long long i = 0; i < steps && !stop_requested; i++) {
        lgWrite(lgpio_handle, pins.step, 1);
        this_thread::sleep_for(chrono::microseconds(delay_us / 2));
        lgWrite(lgpio_handle, pins.step, 0);
        this_thread::sleep_for(chrono::microseconds(delay_us / 2));
        position += (dir ? 1 : -1);
        pos_prop->np[0].value = (double)position.load();
        sendNewNumber(pos_prop);
    }

    if (stop_requested)
        write_status("Move aborted");
    else
        write_status("Move complete");
}

void Waveshare28BYJ::write_status(const string &s)
{
    if (!status_prop) return;
    lock_guard<mutex> lk(hw_mutex);
    strncpy(status_prop->tp[0].text, s.c_str(), INDI_TEXT_SIZE - 1);
    status_prop->tp[0].text[INDI_TEXT_SIZE - 1] = '\0';
    sendNewText(status_prop);
}

// ---------------- Plugin entry points ----------------
extern "C" {
const char *ipGetDefaultDeviceName(int n) { return n==0?"Waveshare_28BYJ":nullptr; }
}
