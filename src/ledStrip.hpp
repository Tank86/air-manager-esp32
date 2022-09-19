#ifndef INCLUDE_LED_STRIP_HPP
#define INCLUDE_LED_STRIP_HPP

#include <Arduino.h>
#include <FastLED.h>

class LedStrip
{
public:
    LedStrip() = default;

    void init();

    void demo();
    void loop();

    void setBrightness(uint8_t brightness);
    void setColor(uint8_t red, uint8_t green, uint8_t blue);
    void set(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue);

    void setState(bool active);

private:
    /* I/O define */
    #define CHIPSET  WS2812
    static constexpr uint8_t LED_PIN = 9;
    static constexpr uint8_t NUM_LEDS = 26;
    static constexpr EOrder COLOR_ORDER = EOrder::GRB;
    CRGB leds[NUM_LEDS];

    uint8_t brightness = 0;
    bool ledStripAcive = false;
};

#endif
