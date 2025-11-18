#pragma once

#include <cstdint>
#include <string>
#include <stdexcept>
#include <lgpio.h>
#include <unistd.h>

class HatBMotor
{
public:
    struct Pins
    {
        int chip   = 0;   // usually 0 for /dev/gpiochip0
        int dir    = 24;  // BCM24
        int step   = 18;  // BCM18
        int enable = 4;   // BCM4 (active LOW)
        int mode0  = 21;  // BCM21
        int mode1  = 22;  // BCM22
        int mode2  = 27;  // BCM27
    };

    HatBMotor(const Pins &pins, int delayUsPerStep = 800)
        : pins_(pins),
          delayUsPerStep_(delayUsPerStep)
    {
        hChip_ = lgGpiochipOpen(pins_.chip);
        if (hChip_ < 0)
            throw std::runtime_error("Failed to open GPIO chip");

        claimOutput(pins_.dir);
        claimOutput(pins_.step);
        claimOutput(pins_.enable);
        claimOutput(pins_.mode0);
        claimOutput(pins_.mode1);
        claimOutput(pins_.mode2);

        // Full-step: MODE pins all low
        setPin(pins_.mode0, 0);
        setPin(pins_.mode1, 0);
        setPin(pins_.mode2, 0);

        // Disable driver by default (active low)
        setPin(pins_.enable, 1);
    }

    ~HatBMotor()
    {
        if (hChip_ >= 0)
        {
            lgGpiochipClose(hChip_);
        }
    }

    void setDelayUsPerStep(int delayUs)
    {
        delayUsPerStep_ = delayUs;
    }

    /// Move 'steps' ticks. Positive = one direction, negative = opposite.
    void moveSteps(int32_t steps)
    {
        if (steps == 0)
            return;

        const bool forward = steps > 0;
        int32_t remaining = steps > 0 ? steps : -steps;

        // Set direction
        setPin(pins_.dir, forward ? 1 : 0);

        // Enable driver (active low)
        setPin(pins_.enable, 0);
        usleep(1000);

        while (remaining-- > 0)
        {
            // Rising edge on STEP
            setPin(pins_.step, 1);
            usleep(delayUsPerStep_);
            setPin(pins_.step, 0);
            usleep(delayUsPerStep_);
        }

        // Disable driver to keep things cool
        setPin(pins_.enable, 1);
    }

private:
    void claimOutput(int line)
    {
        int rc = lgGpioClaimOutput(hChip_, 0, line, 0);
        if (rc < 0)
            throw std::runtime_error("Failed to claim GPIO line " + std::to_string(line));
    }

    void setPin(int line, int value)
    {
        lgGpioWrite(hChip_, line, value);
    }

    Pins pins_;
    int  hChip_           = -1;
    int  delayUsPerStep_  = 800; // tweak for torque vs speed
};
