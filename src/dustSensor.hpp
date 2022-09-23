#ifndef INCLUDE_DUST_SENSOR_HPP
#define INCLUDE_DUST_SENSOR_HPP

#include <Arduino.h>

///////////Sharp DustSensor GP2Y1010AU0F | GP2Y1014AU0F////////
class DustSensor
{
  public:
    DustSensor(const uint8_t ledPin, const uint8_t AnalogOutPin) :
        iled(ledPin),
        vout(AnalogOutPin)
    { }

    void init();
    void loop();

    typedef void (*dustAcquiredCallback)(float dustValue);
    void registercallBack(dustAcquiredCallback c);

  private:
    uint16_t Filter(uint16_t m);

    float dust_density{nanf("")};
    dustAcquiredCallback acquiredCallback{nullptr};

    //pins 
    const uint8_t iled;  //drive the led of sensor
    const uint8_t vout;  //analog input
};

#endif
