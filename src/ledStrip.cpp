#include "ledStrip.hpp"
#include <SPI.h>

void LedStrip::init()
{
    // It's important to set the color correction for your LED strip here,
    // so that colors can be more accurately rendered through the 'temperature' profiles
    FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);

    // by default leds are off
    FastLED.setBrightness(0);
    FastLED.show(0);

    xTaskCreate(this->startTask, /* Task function. */
                "LedStripTask",  /* String with name of task. */
                1024,            /* Stack size in bytes. */
                this,            /* Parameter passed as input of the task */
                5,               /* Priority of the task. */
                NULL);           /* Task handle. */
}

void LedStrip::startTask(void* _this)
{
    static_cast<LedStrip*>(_this)->task();
}

void LedStrip::task()
{
    while (1)
    {
        // get current mode
        auto activeMode = currentMode;

        // process current mode
        switch (activeMode)
        {
            case PowerOn:
            {
                FastLED.setBrightness(128);
                runningLights(0, 225, 0, 50);
            }
            break;
            case Kitt:
            {
                FastLED.setBrightness(128);
                colorWipe(0x00, 0x00, 0xff, 50);
                colorWipe(0x00, 0x00, 0x00, 50);
            }
            break;
            default:
            {
                if (ledStripAcive && (brightness > 0)) FastLED.show();

                // 1 second delay
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            break;
        }

        // blank if mode changed
        if (currentMode != activeMode)
        {
            FastLED.clear(true);
            FastLED.show();
            FastLED.delay(100);
        }
    }
}

void LedStrip::setMode(LedStrip::Mode m)
{
    currentMode = m;
}

void LedStrip::colorWipe(uint8_t red, uint8_t green, uint8_t blue, uint32_t SpeedDelay)
{
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        leds[i].setRGB(red, green, blue);
        FastLED.show();
        FastLED.delay(SpeedDelay);
    }
}

void LedStrip::runningLights(uint8_t red, uint8_t green, uint8_t blue, uint32_t WaveDelay)
{
    uint8_t Position = 0;

    for (uint8_t j = 0; j < NUM_LEDS * 2; j++)
    {
        Position++; // = 0; //Position + Rate;
        for (uint8_t i = 0; i < NUM_LEDS; i++)
        {
            // sine wave, 3 offset waves make a rainbow!
            // float level = sin(i+Position) * 127 + 128;
            // setPixel(i,level,0,0);
            // float level = sin(i+Position) * 127 + 128;
            leds[i].setRGB(((sin(i + Position) * 127 + 128) / 255) * red, ((sin(i + Position) * 127 + 128) / 255) * green,
                           ((sin(i + Position) * 127 + 128) / 255) * blue);
        }

        FastLED.show();
        FastLED.delay(WaveDelay);
    }
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
