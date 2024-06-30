#include "purifierManager.hpp"

void PurifierManager::process(float dust)
{
    sensor_dust = dust;
    process();
}

void PurifierManager::process(const bsecData sensor)
{
    switch (sensor.sensor_id)
    {
        case BSEC_OUTPUT_IAQ:
            if (sensor.accuracy != 0)
            {
                sensor_iaq = sensor.signal;
                process();
            }
            break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            if (sensor.accuracy != 0)
            {
                sensor_gasvoc = sensor.signal;
                process();
            }
            break;
        case BSEC_OUTPUT_CO2_EQUIVALENT:
        case BSEC_OUTPUT_RAW_GAS:
        case BSEC_OUTPUT_GAS_PERCENTAGE:
        case BSEC_OUTPUT_GAS_ESTIMATE_1:
        case BSEC_OUTPUT_GAS_ESTIMATE_2:
        case BSEC_OUTPUT_GAS_ESTIMATE_3:
        case BSEC_OUTPUT_GAS_ESTIMATE_4:
        default:
            // automode is not interested on this sensor
            break;
    }
}

bool PurifierManager::setMode(Modes newMode)
{
    if (!isModeValid(newMode)) return false;
    else
    {
        // Alwsays set private mode
        currentMode = newMode;

        // Process data
        process();

        // Mode is valid, process has occured
        return true;
    }
}

// General process function Process modes according to currentMode
void PurifierManager::process()
{
    // Serial.println("Manager Processing");

    // safety
    if ((setMotorSpeed == nullptr) || (setLedColor == nullptr)) return;

    switch (currentMode)
    {
        case Modes::Off: processOff(); break;
        case Modes::Automatic: processAuto(true); break;
        case Modes::AutoLed: processAuto(false); break;
        case Modes::Night: processNight(); break;
        default:
            // ManualMode or unsupported, do nothing
            break;
    }
}

void PurifierManager::processAuto(bool fullAuto)
{
    // TODO : For the moment only very basic function (a linearity algorithm must be computed)

    //  PM2.5 density  |  Air quality   |  Air quality   | Air quality
    //     0-35        |      0-50      |        I       |  Excellent
    //     35-75       |      51-100    |       II       |  Average
    //     75-115      |      101-150   |      III       |  Light pollution
    //     115-150     |      151-200   |       IV       |  Moderate pollution
    //     150-250     |      201-300   |        V       |  Heavy pollution
    //     250-500     |      ≥300 	    |       VI       |  Serious pullution

    // linear interpolaton of sensors, then take the worst case
    uint32_t dust_level        = isnan(sensor_dust) ? 0 : interpolate(sensor_dust, 35, 300, 0, 255);
    uint32_t iaq_level         = isnan(sensor_iaq) ? 0 : interpolate(sensor_iaq, 50, 300, 0, 255);
    uint32_t max_level         = max(dust_level, iaq_level);
    uint32_t max_level_percent = (max_level * 100) / 255;

    // Set motor speed & led accordingly
    //  - off/green if air quality is ok
    //  - linearily interporlated speed/color for each intermediate values
    //  - full/red  if quality is very bad
    setLedColor(max_level, (255 - max_level), 0);

    if (fullAuto)
    {
        setMotorSpeed(max_level_percent);
    }
}

void PurifierManager::processNight()
{
    // linear interpolaton of sensors, then take the worst case
    uint32_t dust_level        = isnan(sensor_dust) ? 0 : interpolate(sensor_dust, 35, 300, 0, 255);
    uint32_t iaq_level         = isnan(sensor_iaq) ? 0 : interpolate(sensor_iaq, 50, 300, 0, 255);
    uint32_t max_level         = max(dust_level, iaq_level);
    uint32_t max_level_percent = (max_level * 100) / 255;

    // Night mode
    // - color is off
    // - fan speed is max 20%
    if (max_level_percent > 20) max_level_percent = 20;

    setMotorSpeed(max_level_percent);
    setLedColor(0, 0, 0);
}

void PurifierManager::processOff()
{
    setMotorSpeed(0);
    setLedColor(0, 0, 0);
}

float PurifierManager::interpolate(float val, float x0, float x1, float y0, float y1, bool saturate)
{
    if (saturate)
    {
        if (val < x0) return y0;
        else if (val > x1) return y1;
    }

    float numerator   = y0 * (x1 - val) + y1 * (val - x0);
    float denominator = x1 - x0;
    return numerator / denominator;
}
