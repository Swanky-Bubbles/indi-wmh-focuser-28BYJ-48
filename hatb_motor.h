#pragma once
#include <lgpio.h>
#include <stdexcept>
#include <unistd.h>
#include <mutex>
#include <vector>
#include <cmath>

class HatBMotor
{
public:
    struct Pins
    {
        int chip = 0;      // /dev/gpiochip0
        int in1  = 12;     // Motor 1 coil A1
        int in2  = 13;     // Motor 1 coil A2
        int in3  = 19;     // Motor 1 coil B1
        int in4  = 16;     // Motor 1 coil B2
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

        // Half-step sequence for 28BYJ-48
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
            lgGpioWrite(handle, p.in1, 0);
            lgGpioWrite(handle, p.in2, 0);
            lgGpioWrite(handle, p.in3, 0);
            lgGpioWrite(handle, p.in4, 0);
            lgGpiochipClose(handle);
        }
    }

    void setDelayUs(int d) { delayUs = d; }

    void moveSteps(int steps)
    {
        std::lock_guard<std::mutex> lk(mtx);

        int seqCount = static_cast<int>(halfStepSeq.size());
        int idx = 0;

        int total = std::abs(steps);
        int dir = (steps >= 0) ? 1 : -1;

        for (int i = 0; i < total; ++i)
        {
            if (dir > 0)
                idx = (idx + 1) % seqCount;
            else
                idx = (idx - 1 + seqCount) % seqCount;

            auto &s = halfStepSeq[idx];
            lgGpioWrite(handle, p.in1, s[0]);
            lgGpioWrite(handle, p.in2, s[1]);
            lgGpioWrite(handle, p.in3, s[2]);
            lgGpioWrite(handle, p.in4, s[3]);

            usleep(delayUs);
        }

        // optional: de-energise after movement
        lgGpioWrite(handle, p.in1, 0);
        lgGpioWrite(handle, p.in2, 0);
        lgGpioWrite(handle, p.in3, 0);
        lgGpioWrite(handle, p.in4, 0);
    }

private:
    Pins p;
    int handle = -1;
    int delayUs;
    std::mutex mtx;
    std::vector<std::vector<int>> halfStepSeq;
};
