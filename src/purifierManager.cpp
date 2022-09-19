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
    case BSEC_OUTPUT_COMPENSATED_GAS:
    case BSEC_OUTPUT_GAS_PERCENTAGE:
    case BSEC_OUTPUT_GAS_ESTIMATE_1:
    case BSEC_OUTPUT_GAS_ESTIMATE_2:
    case BSEC_OUTPUT_GAS_ESTIMATE_3:
    case BSEC_OUTPUT_GAS_ESTIMATE_4:
    default:
        //automode is not interested on this sensor
        break;
    }
}

void PurifierManager::setAutoMode(bool active) 
{
    //Process data only if automode was not active befores
    if(active && !autoModeActive)
    {
        process();
    }

    //Alwsays re-set private mode
    autoModeActive = active;
}

void PurifierManager::process()
{
    //General process function (according to sensors values)
    //TODO : For the moment only very basic function (a linearity algorithm must be computed)

    //  PM2.5 density  |  Air quality   |  Air quality   | Air quality
    //     0-35        |      0-50      |        I       |  Excellent
    //     35-75       |      51-100    |       II       |  Average
    //     75-115      |      101-150   |      III       |  Light pollution
    //     115-150     |      151-200   |       IV       |  Moderate pollution
    //     150-250     |      201-300   |        V       |  Heavy pollution
    //     250-500     |      ≥300 	    |       VI       |  Serious pullution

    //Automode disabled => exit
    if(!autoModeActive)
        return;
    else
        Serial.println("Manager Processing");

    //safety
    if((setMotorSpeed == nullptr) || (setLedColor == nullptr))
        return;

    //Mode off
    if((sensor_dust < 30.0) || (sensor_iaq < 50.0))
    {
        //Set motor off
        setMotorSpeed(0);
        //Set led green
        setLedColor(0, 255, 0);
    }
    //Mode slow
    else if ((sensor_dust < 100.0) || (sensor_iaq < 100.0))
    {
        //Set motor slow speed
        setMotorSpeed(20);
        //Set led yellow
        setLedColor(255, 255, 0);
    }
    //Mode Medium
    else if ((sensor_dust < 200.0) || (sensor_iaq < 200.0))
    {
        //Set motor medium speed
        setMotorSpeed(50);
        //Set led orange
        setLedColor(255, 128, 0);
    }
    //Mode Fast
    else if ((sensor_dust < 300.0) || (sensor_iaq < 300.0))
    {
        //Set motor fast speed
        setMotorSpeed(75);
        //Set led red
        setLedColor(255, 0, 0);
    }
    //Full speed
    else
    {
        //Set motor full speed
        setMotorSpeed(100);
        //Set led purple
        setLedColor(128, 0, 128);
    }
}
