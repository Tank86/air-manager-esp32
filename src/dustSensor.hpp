#ifndef INCLUDE_DUST_SENSOR_HPP
#define INCLUDE_DUST_SENSOR_HPP

#include <Arduino.h>
#include <esp_adc/adc_oneshot.h>

///////////Sharp DustSensor GP2Y1010AU0F | GP2Y1014AU0F////////
class DustSensor
{
  public:
    DustSensor(const uint8_t ledPin, const adc_channel_t adcChannel) :
        iled(ledPin),
        adcChannel(adcChannel)
    {
    }

    void init();
    void loop();

    typedef void (*dustAcquiredCallback)(float dustValue);
    void registercallBack(dustAcquiredCallback c);

  private:
    uint32_t readADC_Cal(int ADC_Raw);
    uint16_t filter(uint16_t m);
    uint16_t movingAverage(uint16_t m);

    void printDustAirQuality() const;

    float                dust_density{nanf("")};
    dustAcquiredCallback acquiredCallback{nullptr};

    // pins
    const uint8_t        iled;       // drive the led of sensor
    const adc_channel_t adcChannel; // analog input

    adc_oneshot_unit_handle_t handle{nullptr};
    adc_cali_handle_t cali_handle{nullptr};
};

#endif
