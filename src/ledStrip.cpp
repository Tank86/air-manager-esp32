#include "ledStrip.hpp"
#include <Arduino.h>
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
                1,               /* Lower task priority (1) */
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
            case Breath:
            {
                static uint8_t breathStep          = 0;
                static uint8_t breathBrightnessMin = 32;
                static uint8_t breathBrightnessMax = 128;
                static uint8_t breathTimeRef       = millis();

                // Fade color
                fill_solid(leds, NUM_LEDS, distance(leds[0], target.color, 6));

                switch (breathStep)
                {
                    case 0:

                        // Fade to 100% in 1s
                        FastLED.setBrightness(distance(FastLED.getBrightness(), breathBrightnessMax, 4));
                        if (FastLED.getBrightness() == breathBrightnessMax)
                        {
                            breathTimeRef = millis();
                            breathStep++;
                        }
                        break;
                    case 1:
                        // Stay 100 during 500ms
                        if((millis() - breathTimeRef) > 500) breathStep++;
                        break;
                    case 2:
                        // Fade to 30% in 1500ms
                        FastLED.setBrightness(distance(FastLED.getBrightness(), breathBrightnessMin, 2));
                        if (FastLED.getBrightness() == breathBrightnessMin)
                        {
                            breathTimeRef = millis();
                            breathStep++;
                        }
                        break;
                    case 3:
                        // Stay 100 during 1500ms
                        if((millis() - breathTimeRef) > 1500) breathStep++;
                        break;

                    default: breathStep = 0; break;
                }
                // Show
                FastLED.show();
                FastLED.delay(50);
            }
            break;
            case Normal:
            {
                // Fade color
                // for (uint16_t i = 0; i < NUM_LEDS; i++)
                //    nblend(leds[i], target.color, 8);
                fill_solid(leds, NUM_LEDS, distance(leds[0], target.color, 6));

                // Fade brightness
                FastLED.setBrightness(distance(FastLED.getBrightness(), target.brightness, 6));

                // Show
                FastLED.show();
                FastLED.delay(30);
            }
            break;
            default:
            {
                // Nothing to do
                delay(1000); // let some time to other tasks
            }
            break;
        }

        delay(5); // let some time to other tasks
    }
}

void LedStrip::setMode(LedStrip::Mode m)
{
    currentMode = m;
}

void LedStrip::set(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue)
{
    setBrightness(brightness);
    setColor(red, green, blue);
}

void LedStrip::setBrightness(uint8_t brightness)
{
    target.brightness = brightness;
    // FastLED.setBrightness(brightness);
    // FastLED.show();
}

void LedStrip::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
    target.color.red   = red;
    target.color.green = green;
    target.color.blue  = blue;
    // fill_solid(leds, NUM_LEDS, CRGB(red, green, blue));
    // FastLED.show();
}

void LedStrip::setState(bool active)
{
    if (active && (FastLED.getBrightness() == 0)) FastLED.setBrightness(target.brightness);
    else if (!active && (FastLED.getBrightness() != 0)) FastLED.setBrightness(0);
    else
    {
        /* Nothing changes */
    }

    // FastLED.show();
}

/////////// UTILS ////////////
int8_t LedStrip::distance(uint8_t current, uint8_t next, int8_t step)
{
    int16_t distance = next - current;
    if (distance > step) return (current + step);
    else if ((-distance) > step) return (current - step);
    else return next;
}

CRGB LedStrip::distance(CRGB& current, CRGB& next, int8_t step)
{
    CRGB out;
    out.r = distance(current.r, next.r, step);
    out.g = distance(current.g, next.g, step);
    out.b = distance(current.b, next.b, step);
    return out;
}

/////////// EFFECTS ////////////
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
