#ifndef INCLUDE_AIR_SENSOR_HPP
#define INCLUDE_AIR_SENSOR_HPP

#include <Arduino.h>
#include <bsec2.h>

class AirSensor
{
public:
    AirSensor() = default;

    void init(uint16_t EEPROM_BaseAddress);
    void loop();

    void attachCallback(bsecCallback c);

private:
    // Helper functions declarations
    void checkBsecStatus(Bsec2 bsec);
    void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);
    void updateBsecState(Bsec2 bsec);
    void errLeds(void);
    bool loadState(Bsec2 bsec);
    bool saveState(Bsec2 bsec);

    Bsec2 bmeSensor;
    uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE]{0};
    bool initOK{false};
    uint16_t EEPROM_BaseAddress{0};
};

#endif
