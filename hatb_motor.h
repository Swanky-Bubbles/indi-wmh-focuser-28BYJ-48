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
        int in1  = 12;     // Motor 1 IN1 (HAT B)
        int in2  = 13;     // Motor 1 IN2
        int in3  = 19;     // Motor 1 IN3
        int in4  = 16;     // Motor 1 IN4
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

        // Half-step sequence tuned for 28BYJ-48 on Waveshare HAT(B)
        // Order: IN1, IN2, IN3, IN4
        // This sequence is a rotated version of the classic table,
        // which often matches boards that wire coils in a different order.
        // Corrected half-step sequence for Waveshare Motor HAT(B)
        halfStepSeq = {
            {1,1,0,0},  // Step 0
            {0,1,1,0},  // Step 1
            {0,0,1,1},  // Step 2
            {1,0,0,1},  // Step 3
            {1,0,0,0},  // Step 4
            {0,1,0,0},  // Step 5
            {0,0,1,0},  // Step 6
            {0,0,0,1}   // Step 7
        };
    }

    ~HatBMotor()
    {
        if (handle >= 0)
        {
            // De-energise coils
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

        if (handle < 0 || steps == 0)
            return;

        int seqCount = static_cast<int>(halfStepSeq.size());
        int total    = std::abs(steps);
        int dir      = (steps >= 0) ? 1 : -1;

        for (int i = 0; i < total; ++i)
        {
            // advance index, keeping phase continuous across moves
            currentIndex = (currentIndex + dir + seqCount) % seqCount;

            const auto &s = halfStepSeq[currentIndex];
            lgGpioWrite(handle, p.in1, s[0]);
            lgGpioWrite(handle, p.in2, s[1]);
            lgGpioWrite(handle, p.in3, s[2]);
            lgGpioWrite(handle, p.in4, s[3]);

            usleep(delayUs);
        }

        // optional: de-energise after movement so the motor doesn't get hot
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

    // keep track of current phase between moves
    int currentIndex = 0;
};
