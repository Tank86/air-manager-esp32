#ifndef INCLUDE_LED_STRIP_HPP
#define INCLUDE_LED_STRIP_HPP

#include <Arduino.h>
#include <FastLED.h>

class LedStrip
{
  public:
    enum Mode
    {
        Off = 0,
        PowerOn,
        Kitt,
        Normal,
        Breath,
    };

    LedStrip() = default;

    void init();

    void setBrightness(uint8_t brightness);
    void setColor(uint8_t red, uint8_t green, uint8_t blue);
    void set(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue);
    void setState(bool active);

    void setMode(LedStrip::Mode m);

  private:
/* I/O define */
#define CHIPSET WS2812
#if !defined(LEDSTRIP_LEDS_COUNT)
#define LEDSTRIP_LEDS_COUNT 26
#endif
    static constexpr uint8_t NUM_LEDS    = LEDSTRIP_LEDS_COUNT;
    static constexpr uint8_t LED_PIN     = PINS_LEDSTRIP_PIN;
    static constexpr EOrder  COLOR_ORDER = EOrder::GRB;

    CRGB leds[NUM_LEDS];
    struct
    {
        uint8_t brightness;
        CRGB    color;
    } target{0};
    Mode currentMode{Off};

    static void startTask(void*);
    void        task();

    // Utils
    int8_t distance(uint8_t current, uint8_t next, int8_t step);
    CRGB   distance(CRGB& current, CRGB& next, int8_t step);

    // Effects
    void runningLights(uint8_t red, uint8_t green, uint8_t blue, uint32_t WaveDelay);
    void colorWipe(uint8_t red, uint8_t green, uint8_t blue, uint32_t SpeedDelay);
};

#endif
