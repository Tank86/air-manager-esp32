#ifndef INCLUDE_FAN_CONTROL_HPP
#define INCLUDE_FAN_CONTROL_HPP

#include <MCP40xx.h>

class FanControl
{
  public:
    FanControl() = default;

    void init();

    void loop();

    void    on();
    void    off();
    void    setSpeed(uint8_t targetSpeedPercent);
    uint8_t getCurrentSpeed(); // 0 means off
    uint8_t getTargetSpeed();

  private:
    void    initDigitalPot();

    MCP40xx pot{PINS_POT_CS, PINS_POT_UD};
    uint8_t targetSpeed{0};
    uint8_t currentSpeed{0};
};

#endif
