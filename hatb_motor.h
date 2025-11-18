#pragma once
#include <lgpio.h>
#include <stdexcept>
#include <unistd.h>
#include <mutex>
#include <vector>

class HatBMotor
{
public:
    struct Pins
    {
        int chip = 0;      // /dev/gpiochip0
        int in1  = 12;     // Motor 1 A1
        int in2  = 13;     // Motor 1 A2
        int in3  = 19;     // Motor 1 B1
        int in4  = 16;     // Motor 1 B2
    };

    HatBMotor(const Pins &pins, int delayUs)
        : p(pins), delayUs(delayUs)
    {
        handle = lgGpiochipOpen(p.chip);
        if (handle < 0)
            throw std::runtime_error("Failed to open gpiochip");

        auto claim = [&](int pin) {
            if (lgGpioClaimOutput(handle, 0, pin, 0) < 0)
                throw std::runtime_error("Failed to claim pin");
        };

        claim(p.in1);
        claim(p.in2);
        claim(p.in3);
        claim(p.in4);

        // Smoothest half-step table for 28BYJ-48
        halfStepSeq = {
            {1,0,0,0},
            {1,1,0,0},
            {0,1,0,0},
            {0,1,1,0},
            {0,0,1,0},
            {0,0,1,1},
            {0,0,0,1},
            {1,0,0,1}
        };
    }

    ~HatBMotor()
    {
        if (handle >= 0)
        {
            // De-energize
            lgGpioWrite(handle, p.in1, 0);
            lgGpioWrite(handle, p.in2, 0);
            lgGpioWrite(handle, p.in3, 0);
            lgGpioWrite(handle, p.in4, 0);
            lgGpiochipClose(handle);
        }
    }

    void setDelayUs(int d) { delayUs = d; }

    void setBacklashSteps(int steps) { backlash = steps; }

    void moveSteps(int steps)
    {
        std::lock_guard<std::mutex> lk(mtx);

        if (steps == 0 || handle < 0)
            return;

        int dir        = (steps > 0) ? 1 : -1;
        int totalSteps = std::abs(steps);
        int seqCount   = halfStepSeq.size();

        // Pre-energize current step pattern
        const auto &pre = halfStepSeq[stepIndex];
        lgGpioWrite(handle, p.in1, pre[0]);
        lgGpioWrite(handle, p.in2, pre[1]);
        lgGpioWrite(handle, p.in3, pre[2]);
        lgGpioWrite(handle, p.in4, pre[3]);
        usleep(300);

        for (int i = 0; i < totalSteps; i++)
        {
            stepIndex = (stepIndex + dir + seqCount) % seqCount;
            const auto &s = halfStepSeq[stepIndex];

            // Fast contiguous writes
            lgGpioWrite(handle, p.in1, s[0]);
            lgGpioWrite(handle, p.in2, s[1]);
            lgGpioWrite(handle, p.in3, s[2]);
            lgGpioWrite(handle, p.in4, s[3]);

            usleep(delayUs);
        }

        // Optional: Leave coils energized for stability
        // Comment out if you prefer de-energizing:
        // lgGpioWrite(handle, p.in1, 0);
        // lgGpioWrite(handle, p.in2, 0);
        // lgGpioWrite(handle, p.in3, 0);
        // lgGpioWrite(handle, p.in4, 0);
    }

    // backlash compensation move
    void moveWithBacklash(int steps)
    {
        if (backlash == 0)
            return moveSteps(steps);

        int dir = (steps > 0) ? 1 : -1;

        // If direction changes, apply backlash correction
        if (dir != lastDir)
        {
            moveSteps(dir * backlash);
            lastDir = dir;
        }

        moveSteps(steps);
    }

private:
    Pins p;
    int handle = -1;
    int delayUs;
    int backlash = 0;
    int lastDir = 0;

    int stepIndex = 0;   // persistent index — essential for smooth motion

    std::mutex mtx;
    std::vector<std::vector<int>> halfStepSeq;
};
