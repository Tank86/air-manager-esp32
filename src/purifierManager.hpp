#ifndef INCLUDE_PURIFIER_MANAGER_HPP
#define INCLUDE_PURIFIER_MANAGER_HPP

#include <Arduino.h>
#include <bsec2.h>

class PurifierManager
{
  public:
    PurifierManager() = default;

    void process(float dust);
    void process(const bsecData sensor);

    bool isAutoModeActive() const { return autoModeActive; }
    void setAutoMode(bool active);

    typedef void (*changeMotorSpeedCBK)(uint8_t motorPercent);
    typedef void (*changeLedColorCBK)(uint8_t red, uint8_t green, uint8_t blue);

    void registerMotorSpeedcallBack(changeMotorSpeedCBK c) { setMotorSpeed = c; }
    void registerLedColorcallBack(changeLedColorCBK c) { setLedColor = c; }

  private:
    changeMotorSpeedCBK setMotorSpeed{nullptr};
    changeLedColorCBK setLedColor{nullptr};

    bool autoModeActive{true};

    float sensor_dust{nanf("")};
    float sensor_gasvoc{nanf("")};
    float sensor_iaq{nanf("")};

    void process();
    float interpolate(float val, float x0, float x1, float y0, float y1, bool saturate = true);
};

#endif
