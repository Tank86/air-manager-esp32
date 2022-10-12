#include "fanControl.hpp"

void FanControl::init()
{
    initDigitalPot();
    pinMode(PINS_REPLAY_1, OUTPUT);
    pinMode(PINS_REPLAY_2, OUTPUT);
}

void FanControl::loop() {}

void FanControl::initDigitalPot()
{
    pot.setup();      // Gpio init
    pot.begin(10000); // MCP4011 10Kohm installed
    pot.zeroWiper();

    off();
}

void FanControl::setSpeed(uint8_t targetSpeedPercent)
{
    if (targetSpeedPercent == 0)
    {
        targetSpeed = 0;
        pot.zeroWiper();

        // Set Motor relay off
        off();
    }
    else
    {
        // Saturation
        if (targetSpeedPercent > 100) targetSpeed = 100;
        else targetSpeed = targetSpeedPercent;

        // Assign pot value according to speed
        uint16_t potPos = ((targetSpeed * 64) / 100);
        pot.setTap(potPos);

        // Set Motor relay on
        on();
    }
}

void FanControl::on()
{
    digitalWrite(PINS_REPLAY_1, HIGH);
    digitalWrite(PINS_REPLAY_2, HIGH);

    currentSpeed = targetSpeed;
}

void FanControl::off()
{
    digitalWrite(PINS_REPLAY_1, LOW);
    digitalWrite(PINS_REPLAY_2, LOW);

    currentSpeed = 0;
}

uint8_t FanControl::getCurrentSpeed()
{
    return currentSpeed;
}

uint8_t FanControl::getTargetSpeed()
{
    return targetSpeed;
}
