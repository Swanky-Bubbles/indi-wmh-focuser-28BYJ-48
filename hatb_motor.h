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
        int in1  = 12;     // A+
        int in2  = 13;     // B+
        int in3  = 19;     // A-
        int in4  = 16;     // B-
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

        // Correct coil sequence for Waveshare HAT(B)
        halfStepSeq = {
            {1,0,0,0}, // A+
            {1,1,0,0}, // A+ B+
            {0,1,0,0}, // B+
            {0,1,1,0}, // B+ A-
            {0,0,1,0}, // A-
            {0,0,1,1}, // A- B-
            {0,0,0,1}, // B-
            {1,0,0,1}  // B- A+
        };
    }

    ~HatBMotor()
    {
        if (handle >= 0)
        {
            stop();
            lgGpiochipClose(handle);
        }
    }

    void stop()
    {
        lgGpioWrite(handle, p.in1, 0);
        lgGpioWrite(handle, p.in2, 0);
        lgGpioWrite(handle, p.in3, 0);
        lgGpioWrite(handle, p.in4, 0);
    }

    void setDelayUs(int d) { delayUs = d; }

    // Smooth movement with persistent index
    void moveSteps(int steps)
    {
        std::lock_guard<std::mutex> lk(mtx);

        const int seqCount = halfStepSeq.size();
        int dir = (steps >= 0) ? 1 : -1;
        int total = std::abs(steps);

        for (int i = 0; i < total; i++)
        {
            indexPos = (indexPos + dir + seqCount) % seqCount;

            const auto &s = halfStepSeq[indexPos];
            lgGpioWrite(handle, p.in1, s[0]);
            lgGpioWrite(handle, p.in2, s[1]);
            lgGpioWrite(handle, p.in3, s[2]);
            lgGpioWrite(handle, p.in4, s[3]);

            usleep(delayUs);
        }

        stop();
    }

private:
    Pins p;
    int handle = -1;
    int delayUs;
    int indexPos = 0;      // Persistent coil index
    std::mutex mtx;

    std::vector<std::vector<int>> halfStepSeq;
};
