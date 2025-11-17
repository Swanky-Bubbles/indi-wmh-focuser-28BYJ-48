# INDI Driver for Waveshare 28BYJ-48 Stepper Motor HAT (B)

This is an INDI 2.1.6+ compatible driver for the Waveshare Motor HAT (B) controlling 28BYJ-48 stepper motors.

## Features

- STEP/DIR pulse control
- Supports microstepping via DIP switches or INDI override
- Enable/disable, direction, speed, absolute/relative move, abort
- Position tracking
- XML description included for KStars/Ekos integration

## Build & Install

```bash
git clone <repo_url>
cd indi-waveshare-28byj48
mkdir build
cd build
cmake ..
make -j4
sudo make install
