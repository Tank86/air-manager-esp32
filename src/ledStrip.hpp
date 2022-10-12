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
        Idle,
        PowerOn,
        Kitt,
        Normal,
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
    static constexpr uint8_t LED_PIN     = PINS_LEDSTRIP_PIN;
    static constexpr uint8_t NUM_LEDS    = 50;
    static constexpr EOrder  COLOR_ORDER = EOrder::GRB;

    CRGB    leds[NUM_LEDS];
    uint8_t brightness{0};
    bool    ledStripAcive{false};
    Mode    currentMode{Off};

    static void startTask(void*);
    void        task();

    // Effects
    void runningLights(uint8_t red, uint8_t green, uint8_t blue, uint32_t WaveDelay);
    void colorWipe(uint8_t red, uint8_t green, uint8_t blue, uint32_t SpeedDelay);
};

#endif
