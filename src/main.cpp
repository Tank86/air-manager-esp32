#include "CredentialsManager.hpp"
#include "airSensor.hpp"
#include "dustSensor.hpp"
#include "fanControl.hpp"
#include "ledStrip.hpp"
#include "purifierManager.hpp"
#include <Arduino.h>
#include <ArduinoHA.h>
#include <AsyncElegantOTA.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>

AsyncWebServer    server(80);
static const char OTAUser[] = "AirPurifierTower";
static const char OTAPwd[]  = "Tank86";
static const char hostname[] = "AirPurifierTower";

CredentialsManager credentials;
PurifierManager    airManager;
LedStrip           ledStrip;
FanControl         fanControl;
DustSensor         sensorDust{PINS_DUSTSENSOR_ILED, PINS_DUSTSENSOR_VOUT};
AirSensor          sensorAir;

#ifdef HOMEASSISTANT
WiFiClient espClient;
HADevice   device;
HAMqtt     mqtt(espClient, device, 15); // Max 15 sensors
// See https://www.home-assistant.io/integrations/#search/mqtt
// See https://www.home-assistant.io/integrations/sensor/#device-class
#if defined(HA_COLOR_HEXA_MODE)
HALight leds("ledStrip", HALight::BrightnessFeature | HALight::RGBFeature | HALight::RGBHEXAFeature);
#else
HALight leds("ledStrip", HALight::BrightnessFeature | HALight::RGBFeature);
#endif
HAFan          purifierMotor("motor", HAFan::SpeedsFeature); // AirPurifier Motor
HASelect       purifierMode("mode");                         // Represent the mode of the purifier (Off/Manual/Automatic/NightMode)
HASensorNumber wifiRSSI("wrssi", HABaseDeviceType::PrecisionP0);
HASensorNumber dustPM25("pm25", HABaseDeviceType::PrecisionP0);
HASensorNumber temp("temperature", HABaseDeviceType::PrecisionP1);
HASensorNumber humidity("humidity", HABaseDeviceType::PrecisionP0);
HASensorNumber pressure("pressure", HABaseDeviceType::PrecisionP1);
HASensorNumber iaqAccuracy("iaqAccuracy", HABaseDeviceType::PrecisionP0); // 0: stabilisation, 1, low, 2, medium, 3, high
HASensorNumber iaq("iaq", HABaseDeviceType::PrecisionP0);                 // quality air index
HASensorNumber co2("co2_equivalent", HABaseDeviceType::PrecisionP0);
HASensorNumber vocEquivalent("vocEquivalent", HABaseDeviceType::PrecisionP2); // breath voc equivalent (ppm)
#else
// TODO some basic mqtt objects
#endif

///////// Manager Callbacks/////////////////

void onManagerMotorSpeedChanged(uint8_t motorSpeedPercent)
{
    // Speed really changes
    if (purifierMotor.getCurrentSpeed() != motorSpeedPercent)
    {
        Serial.println("Fan speed Changed " + String(motorSpeedPercent));

        fanControl.setSpeed(motorSpeedPercent);

        // Update mqtt state
        purifierMotor.setState(motorSpeedPercent != 0);
        purifierMotor.setSpeed(motorSpeedPercent);
    }
}

void onManagerLedColorChanged(uint8_t red, uint8_t green, uint8_t blue)
{
    // Color really changes
    auto color = HALight::RGBColor(red, green, blue);
    if (leds.getCurrentRGBColor() != color)
    {
        Serial.println("Led Strip Color Changed " + color.toString());
        const uint8_t brightness = (red == 0 && green == 0 && blue == 0) ? 0 : 64;
        ledStrip.set(brightness, color.red, color.green, color.blue);

        // Update mqtt state
        leds.setBrightness(brightness);
        leds.setRGBColor(color);
        leds.setState(brightness != 0);
    }
}

///////// MQTT Callbacks/////////////////
void onBMEDataChanged(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
    if (!outputs.nOutputs)
    {
        Serial.println("BSEC NO DATA");
        return;
    }

    Serial.println("BSEC outputs:\n\ttimestamp = " + String((int)(outputs.output[0].time_stamp / INT64_C(1000000))));
    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
        const bsecData output = outputs.output[i];
        switch (output.sensor_id)
        {
            case BSEC_OUTPUT_IAQ:
                Serial.println("\tiaq = " + String(output.signal));
                Serial.println("\tiaq accuracy = " + String((int)output.accuracy));
                iaqAccuracy.setValue(output.accuracy);
                if (output.accuracy >= 1) // at least accuray low
                    iaq.setValueFloat(output.signal);
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                Serial.println("\tc02 equivalent = " + String(output.signal));
                if (output.accuracy != 0) co2.setValueFloat(output.signal);
                break;
            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                Serial.println("\tbreath voc equivalent = " + String(output.signal));
                if (output.accuracy != 0) vocEquivalent.setValueFloat(output.signal);
                break;
            case BSEC_OUTPUT_RAW_TEMPERATURE: Serial.println("\ttemperature = " + String(output.signal)); break;
            case BSEC_OUTPUT_RAW_PRESSURE:
                Serial.println("\tpressure = " + String(output.signal));
                pressure.setValueFloat(output.signal / 100.0f); // convert to mbar
                break;
            case BSEC_OUTPUT_RAW_HUMIDITY: Serial.println("\thumidity = " + String(output.signal)); break;
            case BSEC_OUTPUT_RAW_GAS: Serial.println("\tgas resistance = " + String(output.signal)); break;
            case BSEC_OUTPUT_STABILIZATION_STATUS: Serial.println("\tstabilization status = " + String(output.signal)); break;
            case BSEC_OUTPUT_RUN_IN_STATUS: Serial.println("\trun in status = " + String(output.signal)); break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                Serial.println("\ttemperature comp = " + String(output.signal));
                temp.setValueFloat(output.signal);
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                Serial.println("\thumidity comp = " + String(output.signal));
                humidity.setValueFloat(output.signal);
                break;
            case BSEC_OUTPUT_COMPENSATED_GAS: Serial.println("\tcompensated gas = " + String(output.signal)); break;
            case BSEC_OUTPUT_GAS_PERCENTAGE: Serial.println("\tgas percentage = " + String(output.signal)); break;
            case BSEC_OUTPUT_GAS_ESTIMATE_1: Serial.println("\tgas estimate 1 = " + String(output.signal)); break;
            case BSEC_OUTPUT_GAS_ESTIMATE_2: Serial.println("\tgas estimate 2 = " + String(output.signal)); break;
            case BSEC_OUTPUT_GAS_ESTIMATE_3: Serial.println("\tgas estimate 3 = " + String(output.signal)); break;
            case BSEC_OUTPUT_GAS_ESTIMATE_4: Serial.println("\tgas estimate 4 = " + String(output.signal)); break;
            default: Serial.println("\t!!!OUCH SENSOR UNKNOWN = " + String(output.sensor_id) + String(output.signal)); break;
        }

        // Send sensor value to process
        airManager.process(output);
    }
}

void onDustChanged(float dust)
{
    static uint32_t lastSendTimer = millis();

    // Process automatic mode
    airManager.process(dust);

    // Manage mqtt send
    if ((millis() - lastSendTimer) > (5 * 60 * 1000))
    {
        lastSendTimer = millis();
        dustPM25.setValueFloat(dust);
    }
}

void onPurifierModeChanged(int8_t index, HASelect* sender)
{
    if (!airManager.setMode(index))
    {
        Serial.println("Invalid/unknown mode");
    }
    else
    {
        Serial.println("Mode changed to: " + String(airManager.getModeStr()));

        if (airManager.isAutoModeActive() && (airManager.getCurrentMode() != PurifierManager::Modes::Night)) ledStrip.setMode(LedStrip::Mode::Breath);
        else ledStrip.setMode(LedStrip::Mode::Normal);

        sender->setState(index);
    }
}

void onPurifierMotorSpeedChanged(uint16_t speed, HAFan* sender)
{
    Serial.println("Fan speed received " + String(speed));

    if (!airManager.isAutoModeActive())
    {
        // Auto mode disabled, but manual override => disable auto mode
        // airManager.setMode(PurifierManager::Modes::Manual);
        // purifierMode.setState(airManager.getModeIndex());
    }

    fanControl.setSpeed((uint8_t)speed);

    sender->setState(fanControl.getCurrentSpeed() != 0);
    sender->setSpeed(fanControl.getTargetSpeed());
}

void onPurifierMotorStateChanged(bool state, HAFan* sender)
{
    Serial.println("Fan state received " + String(state));

    if (!airManager.isAutoModeActive())
    {
        // Auto mode disabled, but manual override => disable auto mode
        // airManager.setMode(PurifierManager::Modes::Manual);
        // purifierMode.setState(airManager.getModeIndex());
    }

    // Set fanControl
    state ? fanControl.on() : fanControl.off();

    // send instant feedback
    sender->setState(state);
}

void onLedBrightnessChanged(uint8_t brightness, HALight* sender)
{
    Serial.println("Led Strip Brightness Received " + String(brightness));
    ledStrip.setBrightness(brightness);

    sender->setBrightness(brightness);
}

void onLedColorChanged(const HALight::RGBColor& color, HALight* sender)
{
    Serial.println("Led Strip Color Received " + color.toString());
    ledStrip.setColor(color.red, color.green, color.blue);

    // Send feedback
    sender->setRGBColor(color);
}

void onLedStateChanged(bool state, HALight* sender)
{
    Serial.println("Led Strip State Received " + String(state));
    ledStrip.setState(state);

    // Send feedback
    sender->setState(state);
}

////////////////////////// WIFI ////////////////////////
#if defined(HARDCODED_CREDENTIALS)
// const char* wifi_ssid = "";
// const char* wifi_password = "";
// const char* mqtt_server = "homeassistant.local";
// const char* mqtt_user = "";
// const char* mqtt_pwd = "";
// const uint16_t   mqtt_port = 1883;
/* /!\ The previous data has to be filled here of inside a private wifiCredentials.hpp /!\ */
#include <wifiCredentials.hpp>
#endif

void initWIFI(const char* ssid, const char* password)
{
    // Unique ID must be set!
    byte mac[6]; // WL_MAC_ADDR_LENGTH];
    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));

    // Connect to wifi
    WiFi.setHostname(hostname);
    Serial.println("Connecting to " + String(ssid));
    WiFi.persistent(false);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500); // waiting for the connection
    }

    Serial.println("Connected !");
    Serial.println("IP address: " + WiFi.localIP().toString());
}

///////// MQTT communication ////////
#if !defined(HOMEASSISTANT)
WiFiClient   espClient;
PubSubClient client(espClient);
long         lastMsg = 0;

void callbackMQTT(char* topic, byte* message, unsigned int length)
{
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;

    for (int i = 0; i < length; i++)
    {
        Serial.print((char)message[i]);
        messageTemp += (char)message[i];
    }
    Serial.println();

    // If a message is received on the topic air-manager/motor-speed, you check if the motor speed.
    // Changes the output state according to the message
    if (String(topic) == "air-manager/motor-speed")
    {
        Serial.print("Changing motor speed to ");
        if (messageTemp == "on")
        {
            Serial.println("on");
        }
        else if (messageTemp == "off")
        {
            Serial.println("off");
        }
    }
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("air-manager", mqtt_user, mqtt_pwd))
        {
            Serial.println("connected");
            // Subscribe
            client.subscribe("air-manager/motor-speed");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void initMQTT(const char* address, uint16_t port, const char* user = nullptr, const char* pwd = nullptr)
{
    client.setServer(address, port, user, pwd);
    client.setCallback(callbackMQTT);
}

void loopMQTT()
{
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    long now = millis();
    if ((now - lastMsg) > 60000)
    {
        lastMsg = now;

        // Convert the value to a char array
        char tempString[8];
        dtostrf(iaqSensor.temperature, 1, 2, tempString);
        Serial.print("Temperature: ");
        Serial.println(tempString);
        client.publish("air-manager/temperature", tempString);

        // Convert the value to a char array
        char humString[8];
        dtostrf(iaqSensor.humidity, 1, 2, humString);
        Serial.print("Humidity: ");
        Serial.println(humString);
        client.publish("air-manager/humidity", humString);

        // Convert the value to a char array
        char pressString[8];
        dtostrf(iaqSensor.pressure, 1, 2, pressString);
        Serial.print("Pressure: ");
        Serial.println(pressString);
        client.publish("air-manager/pressure", pressString);

        // Send co2 & iaq only when stabilisation is at least at low state (1)
        if (iaqSensor.iaqAccuracy >= 1)
        {
            // TODO Send iaqSensor.iaq
            // TODO Send iaqSensor.co2Equivalent
            // TODO Send iaqSensor.breathVocEquivalent
        }
    }
}
#else
/////////////////// Home assistant device ///////////

void initMQTT(const char* address, uint16_t port, const char* user = nullptr, const char* pwd = nullptr)
{
    Serial.println("Connection to mqtt server " + String(address) + ":" + String(port) + ((user != nullptr) ? ("@" + String(user)) : ""));

    // set device's details (optional)
    device.setName("Air Manager");
    device.setModel("Air Tower 1");
    device.setManufacturer("Tank86 electronics");
    device.setSoftwareVersion("2.0.5");

    // Use last will message
    device.enableLastWill();

    purifierMode.setIcon("mdi:refresh-auto");
    purifierMode.setName("AirPurifier mode");
    purifierMode.setOptions(airManager.getModeListStr());
    purifierMode.onCommand(onPurifierModeChanged);

    purifierMotor.onSpeedCommand(onPurifierMotorSpeedChanged);
    purifierMotor.onStateCommand(onPurifierMotorStateChanged);
    purifierMotor.setSpeedRangeMin(1);   // In percent
    purifierMotor.setSpeedRangeMax(100); // In percent
    purifierMotor.setIcon("mdi:fan");    // mdi:air-purifier");
    purifierMotor.setName("AirPurifier speed");

    leds.onStateCommand(onLedStateChanged);
    leds.onBrightnessCommand(onLedBrightnessChanged);
    leds.onRGBColorCommand(onLedColorChanged);
    leds.setIcon("mdi:lightbulb");
    leds.setName("AirPurifier strip");

    temp.setUnitOfMeasurement("°C");
    temp.setDeviceClass("temperature");
    temp.setIcon("mdi:temperature-celsius");
    temp.setName("AirPurifier temperature");

    humidity.setUnitOfMeasurement("%");
    humidity.setDeviceClass("humidity");
    humidity.setIcon("mdi:water-percent");
    humidity.setName("AirPurifier humidity");

    pressure.setUnitOfMeasurement("mbar");
    pressure.setDeviceClass("pressure");
    pressure.setIcon("mdi:weather-cloudy");
    pressure.setName("AirPurifier pressure");

    co2.setUnitOfMeasurement("ppm");
    co2.setDeviceClass("carbon_dioxide");
    co2.setIcon("mdi:molecule-co2");
    co2.setName("AirPurifier co2 equivalent");

    iaq.setDeviceClass("aqi");
    iaq.setIcon("mdi:quality-high");
    iaq.setName("AirPurifier iaq");

    // iaqAccuracy.setDeviceClass("none");
    iaqAccuracy.setIcon("mdi:poll");
    iaqAccuracy.setName("AirPurifier IAQ accuracy");

    vocEquivalent.setUnitOfMeasurement("ppm");
    vocEquivalent.setDeviceClass("volatile_organic_compounds");
    vocEquivalent.setIcon("mdi:emoticon-poop");
    vocEquivalent.setName("AirPurifier VOC equivalent");

    dustPM25.setUnitOfMeasurement("µg/m³");
    dustPM25.setDeviceClass("pm25");
    dustPM25.setIcon("mdi:weather-dust");
    dustPM25.setName("AirPurifier PM2.5 (dust sensor)");

    wifiRSSI.setDeviceClass("signal_strength");
    wifiRSSI.setUnitOfMeasurement("dBm");
    wifiRSSI.setName("AirPurifier Wifi signal strength");
    wifiRSSI.setIcon("mdi:signal-variant");

    // Connect to MQTT server
    mqtt.begin(address, port, user, pwd);

    // set mode accordingly to the manager
    purifierMode.setCurrentState(airManager.getCurrentMode());
}

void loopMQTT()
{
    static uint32_t lastUpdate = millis();

    mqtt.loop();

    uint32_t now = millis();
    // Every 15 minutes
    if ((now - lastUpdate) > (15 * 60 * 1000))
    {
        lastUpdate = now;
        if (WiFi.isConnected()) wifiRSSI.setValue(WiFi.RSSI());
    }
}

#endif
/////////////////////////////////////////////////////////////////////

////////// Purifier Code /////////////
void initPurifierPins()
{
#if defined(CONFIG_IDF_TARGET_ESP32C3)
    // Lolin ESP32_C3 has builtin RGB led
    // TODO Init RGB
    pinMode(PINS_STATUS_LED, OUTPUT);
#else
    pinMode(PINS_STATUS_LED, OUTPUT);
#endif
}

/////////////////// ARDUINO CODE //////////////
void setup()
{
    Serial.begin(115200);

    initPurifierPins();
    fanControl.init();
    ledStrip.init();
    sensorDust.init();

    // Power ON Led Effects
    ledStrip.setMode(LedStrip::Mode::PowerOn);

    /* Valid for boards with USB-COM. Wait until the port is open */
    // while(!Serial) delay(10);
    for (int i = 8; i > 0; i--)
    {
        Serial.println(i);
        delay(1000);
    }

    // No need to run at 240MHz (80 is more than enought for the application)
    setCpuFrequencyMhz(80);

    // Start 1024 bytes of EEPROM
    EEPROM.begin(1024); // Need BSEC_MAX_STATE_BLOB_SIZE(197 bytes) for BME680 +  512 bytes for Wifi credentials

#if !defined(HARDCODED_CREDENTIALS)
    // Start try to get wifi/mqtt credentials,
    // This method is blocking while all data are not initalized.
    ledStrip.setMode(LedStrip::Mode::Kitt);
    credentials.init("Air Purifier AP", "airpurifier");
    bool forcedMode = false;
    credentials.loadParameters(256, forcedMode); // EPROM Base address //BME680 need 197 bytes, so start at @256
#endif

    sensorDust.registercallBack(onDustChanged);
    sensorAir.init(0); // EEPROM Base Address 0
    sensorAir.attachCallback(onBMEDataChanged);

    airManager.registerMotorSpeedcallBack(onManagerMotorSpeedChanged);
    airManager.registerLedColorcallBack(onManagerLedColorChanged);
    if(airManager.isAutoModeActive()) ledStrip.setMode(LedStrip::Mode::Breath);
    else ledStrip.setMode(LedStrip::Mode::Normal);

#if defined(HARDCODED_CREDENTIALS)
    initWIFI(wifi_ssid, wifi_password);
    initMQTT(mqtt_server, mqtt_port, strlen(mqtt_user) ? mqtt_user : NULL, strlen(mqtt_pwd) ? mqtt_pwd : NULL);
#else
    initWIFI(credentials.getWifiSSID(), credentials.getWifiPWD());
    initMQTT(credentials.getMqttAddress(), credentials.getMqttPort(),
             credentials.getMqttUser()[0] == 0 ? nullptr : credentials.getMqttUser(),
             credentials.getMqttPwd()[0] == 0 ? nullptr : credentials.getMqttPwd());
#endif

    // Start AsyncElegantOTA
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) { request->send(200, "text/plain", "Hi, go to /update to see the page"); });
    AsyncElegantOTA.begin(&server, OTAUser, OTAPwd);
    server.begin();
}

void loop()
{
    // Sensors loop
    sensorAir.loop();
    sensorDust.loop();

    // Actuators
    // ledStrip.loop(); //done its in own freertos task
    fanControl.loop();

    // Communication
    loopMQTT();
}

/////////////////// ARDUINO CODE //////////////
