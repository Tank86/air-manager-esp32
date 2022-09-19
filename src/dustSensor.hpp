#ifndef INCLUDE_DUST_SENSOR_HPP
#define INCLUDE_DUST_SENSOR_HPP

#include <Arduino.h>


class DustSensor
{
public:
    DustSensor() = default;

    void init();
    void loop();

    typedef void (*dustAcquiredCallback)(float dustValue);
    void registercallBack(dustAcquiredCallback c);


private:
    uint16_t Filter(uint16_t m);

    float dust_density{nanf("")};
    dustAcquiredCallback acquiredCallback{nullptr};

    /* I/O define */
    const uint8_t iled = 3; //A2;  //drive the led of sensor
    const uint8_t vout = 5; //A4;  //analog input
};

#endif
