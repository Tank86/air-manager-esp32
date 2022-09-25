#include "ledStrip.hpp"
#include <SPI.h>

// Information about the LED strip itself

void LedStrip::init()
{
    // It's important to set the color correction for your LED strip here,
    // so that colors can be more accurately rendered through the 'temperature' profiles
    FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);

    // by default leds are off
    FastLED.setBrightness(0);
    FastLED.show(0);
}

void LedStrip::demo()
{
    // draw a generic, no-name rainbow
    static uint8_t starthue = 0;
    fill_rainbow(leds, NUM_LEDS, --starthue, 20);

    FastLED.setBrightness(64);           // 1/4 brightness
    FastLED.setTemperature(Tungsten40W); // first temperature

    FastLED.show();
    FastLED.delay(8);
}

void LedStrip::loop()
{
    if (ledStripAcive && (brightness > 0)) FastLED.show();
}

void LedStrip::setBrightness(uint8_t brightness)
{
    this->brightness = brightness;
    FastLED.setBrightness(brightness);
    FastLED.show();
}

void LedStrip::set(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue)
{
    this->brightness = brightness;
    FastLED.setBrightness(brightness);
    fill_solid(leds, NUM_LEDS, CRGB(red, green, blue));
    FastLED.show();
}

void LedStrip::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
    fill_solid(leds, NUM_LEDS, CRGB(red, green, blue));
    FastLED.show();
}

void LedStrip::setState(bool active)
{
    ledStripAcive = active;

    if (active) FastLED.setBrightness(brightness);
    else FastLED.setBrightness(0);

    FastLED.show();
}