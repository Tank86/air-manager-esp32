#include "dustSensor.hpp"


///////////Sharp DustSensor GP2Y1010AU0F | GP2Y1014AU0F////////
#define COV_RATIO        0.17//0.2    //ug/mmm / mv
#define NO_DUST_VOLTAGE  600//400    //mv
#define SYS_VOLTAGE      3300   

uint16_t DustSensor::Filter(uint16_t m)
{
    const size_t _buff_max = 10;
    static uint16_t _buff[10];
    static int flag_first = 0, sum;
    uint16_t i;

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


void DustSensor::registercallBack(dustAcquiredCallback c)
{
    acquiredCallback = c;
}

void DustSensor::init()
{
    // iled default closed
    pinMode(iled, OUTPUT);
    digitalWrite(iled, LOW);

    Serial.println("Dust sensor initialised");
}

void DustSensor::loop()
{
    static uint32_t lastAcqTimer = millis();
    static uint32_t lastSendTimer = millis();

    if ((millis() - lastAcqTimer) > (1000))
    {
        lastAcqTimer = millis();

        //  get adcvalue
        digitalWrite(iled, HIGH);
        delayMicroseconds(280);
        analogReadResolution(12); //12 bits
        analogSetAttenuation(ADC_11db);
        // ADC_0db provides no attenuation so IN/OUT = 1 / 1 an input of 3 volts remains at 3 volts before ADC measurement
        // ADC_2_5db provides an attenuation so that IN/OUT = 1 / 1.34 an input of 3 volts is reduced to 2.238 volts before ADC measurement
        // ADC_6db provides an attenuation so that IN/OUT = 1 / 2 an input of 3 volts is reduced to 1.500 volts before ADC measurement
        // ADC_11db provides an attenuation so that IN/OUT = 1 / 3.6 an input of 3 volts is reduced to 0.833 volts before ADC measurement

        //uint16_t adcvalue = analogRead(vout);
        uint16_t adcvalue = analogReadMilliVolts(vout);
        digitalWrite(iled, LOW);

        //  convert voltage (mv)
        // voltage = (SYS_VOLTAGE / 1024.0) * adcvalue * 11;
        // 4.56 => 2.275 (Ratio = 0,5)
        // 3.3V 12bits => 3.3/4096 = 0.8mV/step
        float voltage = (adcvalue * (1.0 / 0.5));
        //Serial.print("Adc(mV): "); Serial.print(adcvalue); Serial.print(" Volt(mV) : "); Serial.println(voltage); 

        // Average filter
        voltage = Filter(voltage);

        // voltage to density
        /*
        if(voltage >= NO_DUST_VOLTAGE) {
          voltage -= NO_DUST_VOLTAGE;
          density = voltage * COV_RATIO;
        } else
          density = 0;
        */
        //See http://www.howmuchsnow.com/arduino/airquality/
        dust_density = ((0.17 * (voltage / 1000.0)) - 0.1) * 1000.0;
        if (dust_density < 0) dust_density = 0;
        Serial.print("The current dust concentration is: ");
        Serial.print(dust_density);
        Serial.print(" ug/m3 ");
        if (dust_density <= 50)        Serial.print("- Air quality: Excelent");
        else if (dust_density <= 100)  Serial.print("- Air quality: Average");
        else if (dust_density <= 150)  Serial.print("- Air quality: Light pollution");
        else if (dust_density <= 200)  Serial.print("- Air quality: Moderatre pollution");
        else if (dust_density <= 300)  Serial.print("- Air quality: Heavy pollution");
        else if (dust_density > 300)   Serial.print("- Air quality: Serious pollution");
        Serial.println("");

        //  PM2.5 density  |  Air quality   |  Air quality   | Air quality 
        // value(μg/m3)    |  index  (AQi)  |     level      |  evaluation
        //     0-35        |      0-50      |        I       |  Excellent
        //     35-75       |      51-100    |       II       |  Average
        //     75-115      |      101-150   |      III       |  Light pollution
        //     115-150     |      151-200   |       IV       |  Moderate pollution
        //     150-250     |      201-300   |        V       |  Heavy pollution
        //     250-500     |      ≥300 	    |       VI       |  Serious pullution 

        if ((millis() - lastSendTimer) > (5 * 60 * 1000))
        {
            lastSendTimer = millis();

            //Send dust density
            if ((acquiredCallback != nullptr) && (!isnan(dust_density)))
                acquiredCallback(dust_density);
        }
    }
}
