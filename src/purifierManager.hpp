#ifndef INCLUDE_PURIFIER_MANAGER_HPP
#define INCLUDE_PURIFIER_MANAGER_HPP

#include <Arduino.h>
#include <bsec2.h>

class PurifierManager
{
  public:
    PurifierManager() = default;

    enum Modes
    {
        Off       = 0,
        Manual    = 1,
        Automatic = 2,
        Night     = 3,
        AutoLed   = 4,
    };
    bool        isModeValid(Modes newMode) { return ((newMode >= Modes::Off) && (newMode <= Modes::AutoLed)); }
    bool        isModeValid(int8_t newModeIdx) { return isModeValid((Modes)(newModeIdx)); }
    const char* getModeListStr() const { return "Off;Manual;Automatic;Night;AutoLed"; }
    const char* getModeStr() const
    {
        if (currentMode == Modes::Off) return "Off";
        else if (currentMode == Modes::Manual) return "Manual";
        else if (currentMode == Modes::Automatic) return "Automatic";
        else if (currentMode == Modes::Night) return "Night";
        else return "unknown"; // should never happen
    }
    int8_t getModeIndex() const { return (int8_t)(currentMode); }

    bool isAutoModeActive() const { return (currentMode != Modes::Manual); }
    bool setMode(Modes newMode);
    bool setMode(int8_t newModeIdx) { return setMode((Modes)newModeIdx); }
    Modes getCurrentMode() const { return currentMode; }

    // Process Methods on data reception
    void process(float dust);
    void process(const bsecData sensor);

    typedef void (*changeMotorSpeedCBK)(uint8_t motorPercent);
    typedef void (*changeLedColorCBK)(uint8_t red, uint8_t green, uint8_t blue);

    void registerMotorSpeedcallBack(changeMotorSpeedCBK c) { setMotorSpeed = c; }
    void registerLedColorcallBack(changeLedColorCBK c) { setLedColor = c; }

  private:
    changeMotorSpeedCBK setMotorSpeed{nullptr};
    changeLedColorCBK   setLedColor{nullptr};

    Modes currentMode{Automatic};

    float sensor_dust{nanf("")};
    float sensor_gasvoc{nanf("")};
    float sensor_iaq{nanf("")};

    void  process();
    void  processAuto(bool fullAuto);
    void  processOff();
    void  processNight();
    float interpolate(float val, float x0, float x1, float y0, float y1, bool saturate = true);
};

#endif
