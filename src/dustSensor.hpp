#ifndef INCLUDE_DUST_SENSOR_HPP
#define INCLUDE_DUST_SENSOR_HPP

#include <Arduino.h>
#include <driver/adc.h>

///////////Sharp DustSensor GP2Y1010AU0F | GP2Y1014AU0F////////
class DustSensor
{
  public:
    DustSensor(const uint8_t ledPin, const adc1_channel_t adcChannel) :
        iled(ledPin),
        adcChannel(adcChannel)
    {
    }

    void init();
    void loop();

    typedef void (*dustAcquiredCallback)(float dustValue);
    void registercallBack(dustAcquiredCallback c);

  private:
    uint32_t readADC_Cal(uint16_t ADC_Raw);
    uint16_t Filter(uint16_t m);

    void printDustAirQuality() const;

    float                dust_density{nanf("")};
    dustAcquiredCallback acquiredCallback{nullptr};

    // pins
    const uint8_t        iled;       // drive the led of sensor
    const adc1_channel_t adcChannel; // analog input
};

#endif
