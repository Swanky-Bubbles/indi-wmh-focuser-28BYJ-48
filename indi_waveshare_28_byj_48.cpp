/*
 * INDI driver for Waveshare Motor HAT controlling 28BYJ-48 stepper motors
 * Target: INDI 2.1.6+ (API compatible)
 * Uses lgpio for GPIO control (Raspberry Pi)
 * Single-file driver module ready for Git repo
 * 
 * This version uses true STEP/DIR pulse output for the HAT (B)
 * Microstepping handled via MODE0/MODE1/MODE2 pins
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

#include <indidevapi.h>
#include <indicom.h>
#include <lgpio.h>

using namespace std;

static const int DEFAULT_STEPPER_DELAY_US = 2000; // microseconds between steps

struct MotorPins {
    int in1, in2, in3, in4; // DIR, STEP, ENABLE, legacy
    int mode0, mode1, mode2; // Microstep pins
};

class Waveshare28BYJ : public INDI::DefaultDevice {
public:
    Waveshare28BYJ();
    ~Waveshare28BYJ();

    bool InitProperties() override;
    bool UpdateProperties() override;
    const char *getDefaultName() override { return "Waveshare_28BYJ"; }

    void ISNewSwitch(const char *name, const INDI::SwitchVector *svp) override;
    void ISNewNumber(const char *name, const INDI::NumberVector *nvp) override;
    void ISNewText(const char *name, const INDI::TextVector *tvp) override;

    bool connectHook() override;
    bool disconnectHook() override;

private:
    INDI::Property *enable_prop = nullptr;
    INDI::Property *direction_prop = nullptr;
    INumberVectorProperty *steps_prop = nullptr;
    INumberVectorProperty *position_prop = nullptr;
    INumberVectorProperty *speed_prop = nullptr;
    INDI::SwitchVectorProperty *step_mode_prop = nullptr;
    INDI::TextVectorProperty *status_prop = nullptr;
    INDI::SwitchVectorProperty *abort_prop = nullptr;

    MotorPins pins;
    int lgpio_handle = 0;

    atomic<bool> connected{false};
    atomic<bool> stop_requested{false};
    atomic<long long> position{0};
    mutex hw_mutex;
    thread worker_thread;

    void write_status(const string &s);
    void do_step(int) {};
    void set_coils(const int[4]) {};
};

Waveshare28BYJ::Waveshare28BYJ() {}
Waveshare28BYJ::~Waveshare28BYJ() { disconnectHook(); }

bool Waveshare28BYJ::InitProperties() {
    // Motor Enable
    IUSwitchVectorProperty *mp = IUFillSwitch(&enable_prop, "MOTOR_ENABLE", "Motor Enable", "WAVESHARE_STEP", 2);
    IUSwitch(&mp->sp[0], "ON", "On", ISS_OFF);
    IUSwitch(&mp->sp[1], "OFF", "Off", ISS_ON);
    IUSetSwitch(mp, "OFF", true);
    defineProperty(mp);

    // Direction
    IUSwitchVectorProperty *dirp = IUFillSwitch(&direction_prop, "MOTOR_DIR", "Direction", "WAVESHARE_STEP", 2);
    IUSwitch(&dirp->sp[0], "FORWARD", "Forward", ISS_ON);
    IUSwitch(&dirp->sp[1], "REVERSE", "Reverse", ISS_OFF);
    defineProperty(dirp);

    // Steps to move
    INumberVectorProperty *steps = IUFillNumber(&steps_prop, "STEPS", "Steps to Move", "WAVESHARE_STEP", 1);
    IUFillNumber(&steps->np[0], "STEPS_TO_MOVE", "Steps", 0, -9e9, 9e9, 1);
    defineProperty(steps);

    // Position
    INumberVectorProperty *pos = IUFillNumber(&position_prop, "POSITION", "Position (steps)", "WAVESHARE_STEP", 1);
    IUFillNumber(&pos->np[0], "ABS_POSITION", "Position", 0, -9e9, 9e9, 1);
    defineProperty(pos);

    // Speed
    INumberVectorProperty *spd = IUFillNumber(&speed_prop, "SPEED", "Step Delay (us)", "WAVESHARE_STEP", 1);
    IUFillNumber(&spd->np[0], "DELAY_US", "Delay (us)", DEFAULT_STEPPER_DELAY_US, 100, 1000000, 1);
    defineProperty(spd);

    // Abort
    ISwitchVectorProperty *ab = IUFillSwitch(&abort_prop, "ABORT", "Abort", "WAVESHARE_STEP", 1);
    IUSwitch(&ab->sp[0], "ABORT_NOW", "Abort", ISS_OFF);
    defineProperty(ab);

    // Status
    ITextVectorProperty *st = IUFillText(&status_prop, "STATUS", "Status", "WAVESHARE_STEP", 1);
    IUFillText(&st->tp[0], "MESSAGE", "Idle");
    defineProperty(st);

    // Microstep override
    IUSwitchVectorProperty *ms = IUFillSwitch(&step_mode_prop, "MICROSTEP_MODE", "Microstep Mode", "WAVESHARE_STEP", 7);
    IUSwitch(&ms->sp[0], "DIP", "Use DIP", ISS_ON);
    IUSwitch(&ms->sp[1], "FULL", "Full", ISS_OFF);
    IUSwitch(&ms->sp[2], "HALF", "1/2", ISS_OFF);
    IUSwitch(&ms->sp[3], "QUARTER", "1/4", ISS_OFF);
    IUSwitch(&ms->sp[4], "EIGHTH", "1/8", ISS_OFF);
    IUSwitch(&ms->sp[5], "SIXTEENTH", "1/16", ISS_OFF);
    IUSwitch(&ms->sp[6], "THIRTYSECOND", "1/32", ISS_OFF);
    defineProperty(ms);

    return true;
}

bool Waveshare28BYJ::UpdateProperties() {
    if (isConnected()) {
        defineProperty(enable_prop);
        defineProperty(direction_prop);
        defineProperty(steps_prop);
        defineProperty(position_prop);
        defineProperty(speed_prop);
        defineProperty(abort_prop);
        defineProperty(status_prop);
    } else {
        deleteProperty(enable_prop);
        deleteProperty(direction_prop);
        deleteProperty(steps_prop);
        deleteProperty(position_prop);
        deleteProperty(speed_prop);
        deleteProperty(abort_prop);
        deleteProperty(status_prop);
    }
    return true;
}

void Waveshare28BYJ::ISNewSwitch(const char *name, const INDI::SwitchVector *svp) {
    lock_guard<mutex> lk(hw_mutex);
    if (!strcmp(name, "MOTOR_ENABLE")) {
        connected = (svp->sp[0].s == ISS_ON);
        write_status(connected ? "Motor enabled" : "Motor disabled");
        if (!connected) stop_requested = true;
    } else if (!strcmp(name, "ABORT")) {
        if (svp->sp[0].s == ISS_ON) { stop_requested = true; write_status("Abort requested"); }
    }
}

void Waveshare28BYJ::ISNewNumber(const char *name, const INDI::NumberVector *nvp) {
    lock_guard<mutex> lk(hw_mutex);
    if (!strcmp(name, "STEPS")) {
        long long steps_to_move = (long long)nvp->np[0].value;
        if (steps_to_move == 0) return;

        bool forward = !(direction_prop && direction_prop->sp[1].s == ISS_ON);
        stop_requested = false;
        if (worker_thread.joinable()) worker_thread.join();

        worker_thread = thread([this, steps_to_move, forward]() {
            long long steps_left = llabs(steps_to_move);
            int dir_val = forward ? 1 : 0;
            lgWrite(lgpio_handle, pins.in1, dir_val);
            lgWrite(lgpio_handle, pins.in3, 1);

            int step_delay_us = (int)speed_prop->np[0].value;
            const int pulse_width_us = 5;

            while (steps_left > 0 && !stop_requested) {
                lgWrite(lgpio_handle, pins.in2, 1);
                this_thread::sleep_for(chrono::microseconds(pulse_width_us));
                lgWrite(lgpio_handle, pins.in2, 0);

                steps_left--;
                position += forward ? 1 : -1;
                position_prop->np[0].value = (double)position.load();
                sendNewNumber(position_prop);

                if (step_delay_us > pulse_width_us)
                    this_thread::sleep_for(chrono::microseconds(step_delay_us - pulse_width_us));
            }

            lgWrite(lgpio_handle, pins.in3, 0);
            write_status(stop_requested ? "Move aborted" : "Move complete");
        });
    }
}

void Waveshare28BYJ::ISNewText(const char *, const INDI::TextVector *) {}

bool Waveshare28BYJ::connectHook() {
    lock_guard<mutex> lk(hw_mutex);
    lgpio_handle = lgGpiochipOpen(0);
    if (lgpio_handle < 0) { write_status("Failed to open lgpio"); return false; }

    int dip_pins[6] = {5,6,23,24,25,26};
    pins.in1=13; pins.in2=19; pins.in3=12;
    pins.mode0=16; pins.mode1=17; pins.mode2=20;

    lgSetDirection(lgpio_handle, pins.in1, LG_OUTPUT);
    lgSetDirection(lgpio_handle, pins.in2, LG_OUTPUT);
    lgSetDirection(lgpio_handle, pins.in3, LG_OUTPUT);
    lgSetDirection(lgpio_handle, pins.mode0, LG_OUTPUT);
    lgSetDirection(lgpio_handle, pins.mode1, LG_OUTPUT);
    lgSetDirection(lgpio_handle, pins.mode2, LG_OUTPUT);

    lgWrite(lgpio_handle, pins.in1, 0);
    lgWrite(lgpio_handle, pins.in2, 0);
    lgWrite(lgpio_handle, pins.in3, 0);
    lgWrite(lgpio_handle, pins.mode0, 0);
    lgWrite(lgpio_handle, pins.mode1, 0);
    lgWrite(lgpio_handle, pins.mode2, 0);

    // Read DIP and apply microstep mode
    int dip_vals[6];
    for(int i=0;i<6;i++){ lgSetDirection(lgpio_handle,dip_pins[i],LG_INPUT); dip_vals[i]=lgRead(lgpio_handle,dip_pins[i])?1:0; }

    bool use_dip = step_mode_prop && step_mode_prop->sp[0].s == ISS_ON;
    int m0=0,m1=0,m2=0;
    if(use_dip){ m0=dip_vals[0]; m1=dip_vals[1]; m2=dip_vals[2]; }
    lgWrite(lgpio_handle,pins.mode0,m0); lgWrite(lgpio_handle,pins.mode1,m1); lgWrite(lgpio_handle,pins.mode2,m2);
    write_status("Connected");
    connected=true;
    return true;
}

bool Waveshare28BYJ::disconnectHook() {
    lock_guard<mutex> lk(hw_mutex);
    stop_requested=true;
    if(worker_thread.joinable()) worker_thread.join();
    lgWrite(lgpio_handle,pins.in3,0);
    if(lgpio_handle) lgGpiochipClose(lgpio_handle);
    connected=false;
    write_status("Disconnected");
    return true;
}

void Waveshare28BYJ::write_status(const string &s){
    if(!status_prop) return;
    lock_guard<mutex> lk(hw_mutex);
    strncpy(status_prop->tp[0].text,s.c_str(),INDI_TEXT_SIZE-1);
    status_prop->tp[0].text[INDI_TEXT_SIZE-1]='\0';
    sendNewText(status_prop);
}

// Plugin entry points
extern "C" {
    const char *ipGetDefaultDeviceName(int n){ return n==0?"Waveshare_28BYJ":nullptr; }
}