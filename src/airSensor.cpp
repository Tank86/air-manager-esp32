#include "airSensor.hpp"
#include <EEPROM.h>
#include <Wire.h>

/**
 * For more information visit : 
 * https://www.bosch-sensortec.com/software-tools/software/bme688-software/
 * 
 * For quick integration test, example code can be used with configuration file under folder
 * Bosch_BSEC2_Library/src/config/FieldAir_HandSanitizer (Configuration file added as simple
 * code example for integration but not optimized on classification performance)
 * Config string for H2S and NonH2S target classes is also kept for reference (Suitable for
 * lab-based characterization of the sensor)
 */
//#include "config/Default_H2S_NonH2S/Default_H2S_NonH2S.h"

/* Configuration for two class classification used here
 * For four class classification please use configuration under config/FieldAir_HandSanitizer_Onion_Cinnamon
 */
//#include "config/FieldAir_HandSanitizer/FieldAir_HandSanitizer.h"
/* Gas estimate names will be according to the configuration classes used */
// const String gasName[] = { "Field Air", "Hand sanitizer", "Undefined 3", "Undefined 4"};

#define STATE_SAVE_PERIOD UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day

void AirSensor::attachCallback(bsecCallback c)
{
    /* Whenever new data is available call the function */
    bmeSensor.attachCallback(c);
}

void AirSensor::init(uint16_t EE_Address)
{
    String output;

    Wire.begin();
    EEPROM_BaseAddress = EE_Address;

    /* Desired subscription list of BSEC2 outputs */
    bsecSensor sensorList[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,        /*!< co2 equivalent estimate [ppm] */
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT, /*!< breath VOC concentration estimate [ppm] */
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_STABILIZATION_STATUS, // ongoing (0) or stabilization is finished (1)
        BSEC_OUTPUT_RUN_IN_STATUS,        // ongoing (0) or stabilization is finished (1)
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        /*
        BSEC_OUTPUT_COMPENSATED_GAS,
        BSEC_OUTPUT_GAS_PERCENTAGE,
        BSEC_OUTPUT_GAS_ESTIMATE_1,                        // Gas estimate output channel 1
        BSEC_OUTPUT_GAS_ESTIMATE_2,                        // Gas estimate output channel 2
        BSEC_OUTPUT_GAS_ESTIMATE_3,                        // Gas estimate output channel 3
        BSEC_OUTPUT_GAS_ESTIMATE_4                         // Gas estimate output channel 4
        */
    };

    /* Initialize the library and interfaces */
    if (!bmeSensor.begin(BME68X_I2C_ADDR_HIGH, Wire))
    {
        checkBsecStatus(bmeSensor);
        initOK = false;
    }
    /* Load the configuration string that stores information on how to classify the detected gas */
    else if (0) // (!bmeSensor.setConfig(FieldAir_HandSanitizer_config))
    {
        checkBsecStatus(bmeSensor);
        initOK = false;
    }
    /* Copy state from the EEPROM to the algorithm */
    else if (!loadState(bmeSensor))
    {
        checkBsecStatus(bmeSensor);
        initOK = false;
    }
    /* Subsribe to the desired BSEC2 outputs */
    else if (!bmeSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_ULP)) // BSEC_SAMPLE_RATE_HIGH_PERFORMANCE))
    {
        checkBsecStatus(bmeSensor);
        initOK = false;
    }
    else
    {
        initOK = true;
    }

    Serial.println("BSEC library version " + String(bmeSensor.version.major) + "." + String(bmeSensor.version.minor) + "." +
                   String(bmeSensor.version.major_bugfix) + "." + String(bmeSensor.version.minor_bugfix));
}

void AirSensor::loop(void)
{
    // Check if init was done ok, else, the sensor.run() will reboot the board...
    if (!initOK)
    {
        errLeds();
    }
    // Call the run function often so that the library can check if it is time to read new data from the sensor and process it.
    else if (!bmeSensor.run())
    {
        checkBsecStatus(bmeSensor);
    }
    else
    {
        // everithing is fine, check if eeprom data has to be saved
        updateBsecState(bmeSensor);
    }
}

// Helper function definitions
void AirSensor::checkBsecStatus(Bsec2 bsec)
{
    if (bsec.status < BSEC_OK)
    {
        Serial.println("BSEC error code : " + String(bsec.status));
        errLeds(); /* Halt in case of failure */
    }
    else if (bsec.status > BSEC_OK)
    {
        Serial.println("BSEC warning code : " + String(bsec.status));
    }

    if (bsec.sensor.status < BME68X_OK)
    {
        Serial.println("BME68X error code : " + String(bsec.sensor.status));
        errLeds(); /* Halt in case of failure */
    }
    else if (bsec.sensor.status > BME68X_OK)
    {
        Serial.println("BME68X warning code : " + String(bsec.sensor.status));
    }
}

bool AirSensor::loadState(Bsec2 bsec)
{
    if (EEPROM.read(EEPROM_BaseAddress + 0) == BSEC_MAX_STATE_BLOB_SIZE)
    {
        /* Existing state in EEPROM */
        Serial.println("Reading state from EEPROM");
        Serial.print("State file: ");
        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
        {
            bsecState[i] = EEPROM.read(EEPROM_BaseAddress + i + 1);
            Serial.print(String(bsecState[i], HEX) + ", ");
        }
        Serial.println();

        if (!bsec.setState(bsecState)) return false;
    }
    else
    {
        /* Erase the EEPROM with zeroes */
        Serial.println("Erasing EEPROM");

        for (uint8_t i = 0; i <= BSEC_MAX_STATE_BLOB_SIZE; i++)
            EEPROM.write(EEPROM_BaseAddress + i, 0);

        EEPROM.commit();
    }

    return true;
}

bool AirSensor::saveState(Bsec2 bsec)
{
    if (!bsec.getState(bsecState)) return false;

    Serial.println("Writing state to EEPROM");
    Serial.print("State file: ");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
        EEPROM.write(EEPROM_BaseAddress + i + 1, bsecState[i]);
        Serial.print(String(bsecState[i], HEX) + ", ");
    }
    Serial.println();

    EEPROM.write(EEPROM_BaseAddress + 0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();

    return true;
}

void AirSensor::errLeds(void)
{
#if defined(CONFIG_IDF_TARGET_ESP32C3)
    // Lolin ESP32_C3 has builtin RGB led
    // TODO, manage RGB LED
#else
    pinMode(PINS_STATUS_LED, OUTPUT);
    digitalWrite(PINS_STATUS_LED, HIGH);
    delay(100);
    digitalWrite(PINS_STATUS_LED, LOW);
    delay(100);
#endif
}

void AirSensor::updateBsecState(Bsec2 bsec)
{
    static uint16_t stateUpdateCounter = 0;

    if (!stateUpdateCounter || (stateUpdateCounter * STATE_SAVE_PERIOD) < millis())
    {
        /* Update every STATE_SAVE_PERIOD minutes */
        stateUpdateCounter++;

        if (!saveState(bsec)) checkBsecStatus(bsec);
    }
}
