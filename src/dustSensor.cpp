#include "dustSensor.hpp"
#include <esp_adc/adc_cali.h>

uint16_t DustSensor::filter(uint16_t m)
{
    const size_t    _buff_max  = 10;
    static uint16_t _buff[10]  = {0};
    static int      flag_first = 0, sum;
    uint16_t        i;

    if (flag_first == 0)
    {
        flag_first = 1;
        for (i = 0, sum = 0; i < _buff_max; i++)
        {
            _buff[i] = m;
            sum += _buff[i];
        }
        return m;
    }
    else
    {
        sum -= _buff[0];
        for (i = 0; i < (_buff_max - 1); i++)
        {
            _buff[i] = _buff[i + 1];
        }
        _buff[9] = m;
        sum += _buff[9];

        i = sum / 10.0;
        return i;
    }
}

uint16_t DustSensor::movingAverage(uint16_t m) 
{
    #define WINDOW_SIZE 60
    static uint32_t values[WINDOW_SIZE];
    static uint32_t index = 0;
    static uint32_t sum = 0;

    // Remove the oldest entry from the sum
    sum -= values[index];
    // Add the newest reading to the window
    values[index] = m;
    // Add the newest reading to the sum
    sum += m;
    index = (index + 1) % WINDOW_SIZE;

    // Divide the sum of the window by the window size for the result
    uint16_t averaged = sum / WINDOW_SIZE;      

    return averaged;
}

uint32_t DustSensor::readADC_Cal(int ADC_Raw)
{
    int output_mv = 0;
    adc_cali_raw_to_voltage(cali_handle, ADC_Raw, &output_mv);
    return static_cast<uint32_t>(output_mv);
}

void DustSensor::registercallBack(dustAcquiredCallback c)
{
    acquiredCallback = c;
}

void DustSensor::init()
{
    // iled default closed
    pinMode(iled, OUTPUT);
    digitalWrite(iled, LOW);

    // Configure adc
    // We want fastest possible measurement //TODO: check if this is correct for all esp32

    const adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
#ifdef CONFIG_IDF_TARGET_ESP32C3
        .clk_src = ADC_DIGI_CLK_SRC_APB,
#else
        .clk_src = ADC_RTC_CLK_SRC_RC_FAST,
#endif
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &handle));

    const adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_13,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, ADC_CHANNEL_0, &config));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    const adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    const adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_13,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));
#endif
    Serial.println("Dust sensor initialised");
}

void DustSensor::loop()
{
    static uint32_t lastAcqTimer = millis();

    //every second
    if ((millis() - lastAcqTimer) > (1000))
    {
        lastAcqTimer = millis();
        int adcValueRaw;

        //adc_power_acquire();
        // ACtivate the pulse, do the adc measurment, then stop the pulse
        digitalWrite(iled, HIGH);
        delayMicroseconds(280);
        // Use esp idf measurement as it is WAYYYYYYY faster than arduino, so the value is correctly read at a precise time
        adc_oneshot_read(handle, adcChannel, &adcValueRaw);

        // adcValueRaw = analogRead(vout); //Start the adc reading here, as it takes a long time to read the value
        digitalWrite(iled, LOW);
        //adc_power_release();

        // convert adc to voltage using internal efuse calibration
        uint16_t adcvalue = readADC_Cal(adcValueRaw);
        // Serial.println("Dust Volatage: Raw=" + String(adcValueRaw) + "  Count   " + String(adcvalue) + "mV");

// We have a 50% resistor divider so multiply by 2
#if defined(WAVESHARE_DUST_SENSOR_MODE)
        float voltage = (adcvalue * 11.0);
#else
        float voltage = (adcvalue * 2.0);
#endif

        // Average filter (60 samples)
        voltage = movingAverage(voltage);

        // voltage to density
#if false
        static const float    COV_RATIO       = 0.2;  // ug/mmm / mv
        static const uint16_t NO_DUST_VOLTAGE = 400;  // mv

        if (voltage >= NO_DUST_VOLTAGE)
        {
            voltage -= NO_DUST_VOLTAGE;
            dust_density = voltage * COV_RATIO;
        }
        else dust_density = 0;
#else
        // See http://www.howmuchsnow.com/arduino/airquality/
        dust_density = ((0.172 * (voltage / 1000.0)) - 0.0999) * 1000.0;
        //dust_density = ((0.172 * (voltage / 1000.0)) - 0.085) * 1000.0;
        if (dust_density < 0) dust_density = 0;
#endif

        printDustAirQuality();

        // Send dust density
        if ((acquiredCallback != nullptr) && (!isnan(dust_density))) acquiredCallback(dust_density);
    }
}

void DustSensor::printDustAirQuality() const
{
#if true
    static uint32_t lastSent = millis();

    //  PM2.5 density  |  Air quality   |  Air quality   | Air quality
    // value(μg/m3)    |  index  (AQi)  |     level      |  evaluation
    //     0-35        |      0-50      |        I       |  Excellent
    //     35-75       |      51-100    |       II       |  Average
    //     75-115      |      101-150   |      III       |  Light pollution
    //     115-150     |      151-200   |       IV       |  Moderate pollution
    //     150-250     |      201-300   |        V       |  Heavy pollution
    //     250-500     |      ≥300 	    |       VI       |  Serious pullution

    // print dust status only every 10 seconds
    uint32_t now = millis();
    if ((now - lastSent) > (10 * 1000))
    {
        lastSent = now;
        Serial.print("The current dust concentration is: ");
        Serial.print(dust_density);
        Serial.print(" ug/m3 ");
        if (dust_density <= 50) Serial.print("- Air quality: Excelent");
        else if (dust_density <= 100) Serial.print("- Air quality: Average");
        else if (dust_density <= 150) Serial.print("- Air quality: Light pollution");
        else if (dust_density <= 200) Serial.print("- Air quality: Moderatre pollution");
        else if (dust_density <= 300) Serial.print("- Air quality: Heavy pollution");
        else if (dust_density > 300)  Serial.print("- Air quality: Serious pollution");
        Serial.println("");
    }
#endif
}
