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
        int chip = 0;      // /dev/gpiochip0 owns GPIO12/13/19/16 on Pi 4
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
            // de-energize
            lgGpioWrite(handle, p.in1, 0);
            lgGpioWrite(handle, p.in2, 0);
            lgGpioWrite(handle, p.in3, 0);
            lgGpioWrite(handle, p.in4, 0);
            lgGpiochipClose(handle);
        }
    }

    void setDelayUs(int d) { delayUs = d; }

    // Steps may be positive or negative
    void moveSteps(int steps)
    {
        std::lock_guard<std::mutex> lk(mtx);

        int seqCount = halfStepSeq.size();
        int idx = 0;

        for (int i = 0; i < abs(steps); i++)
        {
            if (steps > 0)
                idx = (idx + 1) % seqCount;      // forward
            else
                idx = (idx - 1 + seqCount) % seqCount;  // backward

            auto &s = halfStepSeq[idx];
            lgGpioWrite(handle, p.in1, s[0]);
            lgGpioWrite(handle, p.in2, s[1]);
            lgGpioWrite(handle, p.in3, s[2]);
            lgGpioWrite(handle, p.in4, s[3]);

            usleep(delayUs);
        }
    }

private:
    Pins p;
    int handle = -1;
    int delayUs;
    std::mutex mtx;

    // 8-step half-step sequence
    std::vector<std::vector<int>> halfStepSeq;
};
