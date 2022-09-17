#include <Arduino.h>
#include <FastLED.h>


class LedStrip
{
public:
    LedStrip() = default;

    void init();
    void loop();


private:
    /* I/O define */
    #define CHIPSET  WS2812
    static constexpr uint8_t LED_PIN = 9;
    static constexpr uint8_t NUM_LEDS = 30;
    static constexpr EOrder COLOR_ORDER = EOrder::GRB;
    CRGB leds[NUM_LEDS];
};
