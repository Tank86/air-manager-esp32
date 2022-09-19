#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <ArduinoHA.h>
#include <PubSubClient.h>
#include <MCP40xx.h>
#include "dustSensor.hpp"
#include "airSensor.hpp"
#include "ledStrip.hpp"
#include "purifierManager.hpp"


// MCP40xx
const uint8_t POT_UDPin = 16;   // UP/DOWN
const uint8_t POT_CSPin = 18;   // CHIP SELECT
MCP40xx pot = MCP40xx(POT_CSPin, POT_UDPin);

const uint8_t Relay1_Pin = 11;
const uint8_t Relay2_Pin = 12;

PurifierManager manager;
DustSensor  dust;
AirSensor   airStation;
LedStrip    ledStrip;

#ifdef HOMEASSISTANT
WiFiClient espClient;
HADevice device;
HAMqtt mqtt(espClient, device);
// See https://www.home-assistant.io/integrations/#search/mqtt
// See https://www.home-assistant.io/integrations/sensor/#device-class
// Need to add MQTT Light https://www.home-assistant.io/integrations/light.mqtt/
//HASwitch led("led", false); // "led" is unique ID of the switch. You should define your own ID.
HALight leds("ledStrip"); // Is always automatic
HAFan purifierMotor("motor", HAFan::SpeedsFeature); //AirPurifier Motor
HASwitch autoMode("AutoMode", true);    //Represent the mode of the purifier (Automatic/manual)
HASensor temp("temperature");
HASensor humidity("humidity");
HASensor pressure("pressure");
HASensor co2("co2_equivalent");
HASensor iaq("iaq");                      // quality air index
HASensor iaqAccuracy("iaqAccuracy");      // 0: stabilisation, 1, low, 2, medium, 3, high
HASensor vocEquivalent("vocEquivalent");  // breath voc equivalent (ppm)
HASensor dustPM25("pm25");
#else
//TODO some basic mqtt objects
#endif

///////// Manager Callbacks/////////////////

void onManagerMotorSpeedChanged(uint8_t motorSpeedPercent)
{
    Serial.println("Fan speed Changed " + String(motorSpeedPercent));

    //Saturation
    if (motorSpeedPercent > 100) motorSpeedPercent = 100;

    uint16_t potPos = ((motorSpeedPercent * 64) / 100);

    //Assign pot value according to speed
    pot.setTap(potPos);

    //Update mqtt state
    purifierMotor.setState(motorSpeedPercent != 0);
    purifierMotor.setSpeed(motorSpeedPercent);
}

void onManagerLedColorChanged(uint8_t red, uint8_t green, uint8_t blue)
{
    Serial.println("Led Strip Color Changed " + String(red) + ", " + String(green) + ", " + String(blue));
    ledStrip.set(64, red, green, blue);

    //Update mqtt state
    leds.setBrightness(64);
    leds.setColor(red, green, blue);
    leds.setState(true);
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
            if (output.accuracy >= 1) //at least accuray low
                iaq.setValue(output.signal, 0);
            break;
        case BSEC_OUTPUT_CO2_EQUIVALENT:
            Serial.println("\tc02 equivalent = " + String(output.signal));
            if (output.accuracy != 0)
                co2.setValue(output.signal, 0);
            break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            Serial.println("\tbreath voc equivalent = " + String(output.signal));
            if (output.accuracy != 0)
                vocEquivalent.setValue(output.signal, 2);
            break;
        case BSEC_OUTPUT_RAW_TEMPERATURE:
            Serial.println("\ttemperature = " + String(output.signal));
            break;
        case BSEC_OUTPUT_RAW_PRESSURE:
            Serial.println("\tpressure = " + String(output.signal));
            pressure.setValue(output.signal / 100.0f, 1);  //convert to mbar
            break;
        case BSEC_OUTPUT_RAW_HUMIDITY:
            Serial.println("\thumidity = " + String(output.signal));
            break;
        case BSEC_OUTPUT_RAW_GAS:
            Serial.println("\tgas resistance = " + String(output.signal));
            break;
        case BSEC_OUTPUT_STABILIZATION_STATUS:
            Serial.println("\tstabilization status = " + String(output.signal));
            break;
        case BSEC_OUTPUT_RUN_IN_STATUS:
            Serial.println("\trun in status = " + String(output.signal));
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
            Serial.println("\ttemperature comp = " + String(output.signal));
            temp.setValue(output.signal, 1);
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
            Serial.println("\thumidity comp = " + String(output.signal));
            humidity.setValue(output.signal, 0);
            break;
        case BSEC_OUTPUT_COMPENSATED_GAS:
            Serial.println("\tcompensated gas = " + String(output.signal));
            break;
        case BSEC_OUTPUT_GAS_PERCENTAGE:
            Serial.println("\tgas percentage = " + String(output.signal));
            break;
        case BSEC_OUTPUT_GAS_ESTIMATE_1:
            Serial.println("\tgas estimate 1 = " + String(output.signal));
            break;
        case BSEC_OUTPUT_GAS_ESTIMATE_2:
            Serial.println("\tgas estimate 2 = " + String(output.signal));
            break;
        case BSEC_OUTPUT_GAS_ESTIMATE_3:
            Serial.println("\tgas estimate 3 = " + String(output.signal));
            break;
        case BSEC_OUTPUT_GAS_ESTIMATE_4:
            Serial.println("\tgas estimate 4 = " + String(output.signal));
            break;
        default:
            Serial.println("\t!!!OUCH SENSOR UNKNOWN = " + String(output.sensor_id) + String(output.signal));
            break;
        }

        //Send sensor value to process
        manager.process(output);
    }
}

void onDustChanged(float dust)
{
    static uint32_t lastSendTimer = millis();

    //Process automatic mode
    manager.process(dust);

    //Manage mqtt send
    if ((millis() - lastSendTimer) > (5 * 60 * 1000))
    {
        lastSendTimer = millis();
        dustPM25.setValue(dust, 0);
    }
}

void onAutoModeChanged(bool state, HASwitch* s)
{
    if(state)
        Serial.println("Automatic mode enabled");
    else
        Serial.println("Automatic mode disabled");

    s->setState(state);
    manager.setAutoMode(state);
}

void onPurifierMotorSpeedChanged(uint16_t speed)
{
    Serial.println("Fan speed received " + String(speed));

    if(autoMode.getState())
    {
        //Auto mode disabled
        //autoMode.setState(false); //FIXME: in the lib ????
        //manager.setAutoMode(false);  //FIXME: in the lib ????
    }

    //Saturation
    if (speed > 100) speed = 100;

    uint16_t potPos = ((speed * 64) / 100);
    //Assign pot value according to speed
    pot.setTap(potPos);
}

void onPurifierMotorStateChanged(bool state)
{
    Serial.println("Fan state received " + String(state));

    if(autoMode.getState())
    {
        //Auto mode disabled
        //autoMode.setState(false); //FIXME: in the lib ????
        //manager.setAutoMode(false);  //FIXME: in the lib ????
    }

    //send instant feedback
    purifierMotor.setState(state);

    //Set relay on/off
    digitalWrite(Relay1_Pin, state);
    digitalWrite(Relay2_Pin, state);
}

void onLedBrightnessChanged(uint8_t brightness)
{
    Serial.println("Led Strip Brightness Received " + String(brightness));
    ledStrip.setBrightness(brightness);
}

void onLedColorChanged(uint8_t red, uint8_t green, uint8_t blue)
{
    Serial.println("Led Strip Color Received " + String(red) + ", " + String(green) + ", " + String(blue));
    ledStrip.setColor(red, green, blue);
}

void onLedStateChanged(bool state)
{
    Serial.println("Led Strip State Received " + String(state));
    ledStrip.setState(state);

    //Send feedback
    leds.setState(state);
}




///////// MQTT communication ////////
//const char* wifi_ssid = "";
//const char* wifi_password = "";
//const char* mqtt_server = "homeassistant.local";
//const char* mqtt_user = "";
//const char* mqtt_pwd = "";
//const uint16_t   mqtt_port = 1883;
/* /!\ The previous data has to be filled here of inside a private wifiCredentials.hpp /!\ */
#include <wifiCredentials.hpp>

void initWIFI() 
{
    // Unique ID must be set!
    byte mac[6]; //WL_MAC_ADDR_LENGTH];
    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));

    // connect to wifi
    Serial.println("Connecting to " + String(wifi_ssid));
    WiFi.begin(wifi_ssid, wifi_password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500); // waiting for the connection
    }

    Serial.println("Connected !");
    Serial.println("IP address: " + WiFi.localIP().toString());
}

#ifndef HOMEASSISTANT
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

void callbackMQTT(char* topic, byte* message, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;

    for (int i = 0; i < length; i++) {
        Serial.print((char)message[i]);
        messageTemp += (char)message[i];
    }
    Serial.println();

    // If a message is received on the topic air-manager/motor-speed, you check if the motor speed. 
    // Changes the output state according to the message
    if (String(topic) == "air-manager/motor-speed") {
        Serial.print("Changing motor speed to ");
        if (messageTemp == "on") {
            Serial.println("on");
            digitalWrite(LED_BUILTIN, HIGH);
        }
        else if (messageTemp == "off") {
            Serial.println("off");
            digitalWrite(LED_BUILTIN, LOW);
        }
    }
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("air-manager", mqtt_user, mqtt_pwd)) {
            Serial.println("connected");
            // Subscribe
            client.subscribe("air-manager/motor-speed");
        }
        else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void initMQTT(void) {
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callbackMQTT);
}

void loopMQTT() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    long now = millis();
    if ((now - lastMsg) > 60000) {
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

        //Send co2 & iaq only when stabilisation is at least at low state (1)
        if (iaqSensor.iaqAccuracy >= 1) {
            //TODO Send iaqSensor.iaq
            //TODO Send iaqSensor.co2Equivalent
            //TODO Send iaqSensor.breathVocEquivalent
        }

    }
}
#else
/////////////////// Home assistant device ///////////

void initMQTT() 
{
    // set device's details (optional)
    device.setName("air-manager");
    device.setManufacturer("Tank86 electronics");
    device.setModel("Air purifier station");
    device.setSoftwareVersion("1.0.0");

    autoMode.setIcon("mdi:refresh-auto");
    autoMode.setName("Automatic mode");
    autoMode.onStateChanged(onAutoModeChanged);

    //purifierMotor.setRetain(true);
    purifierMotor.onSpeedChanged(onPurifierMotorSpeedChanged);
    purifierMotor.onStateChanged(onPurifierMotorStateChanged);
    purifierMotor.setSpeedRangeMin(1);
    purifierMotor.setSpeedRangeMax(100);
    purifierMotor.setIcon("mdi:fan"); //mdi:air-purifier");
    purifierMotor.setName("AirPurifier speed");

    leds.onStateChanged(onLedStateChanged);
    leds.onBrightnessChanged(onLedBrightnessChanged);
    leds.onColorChanged(onLedColorChanged);
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

    pressure.setUnitOfMeasurement("mBar");
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

    //iaqAccuracy.setDeviceClass("none");
    iaqAccuracy.setIcon("mdi:poll");
    iaqAccuracy.setName("AirPurifier IAQ accuracy");

    vocEquivalent.setUnitOfMeasurement("ppb");
    vocEquivalent.setDeviceClass("volatile_organic_compounds");
    vocEquivalent.setIcon("mdi:emoticon-poop");
    vocEquivalent.setName("AirPurifier VOC equivalent");

    dustPM25.setUnitOfMeasurement("ug/m3");
    dustPM25.setDeviceClass("pm25");
    dustPM25.setIcon("mdi:weather-dust");
    dustPM25.setName("AirPurifier PM2.5 (dust sensor)");

    // start server
    mqtt.begin(mqtt_server, mqtt_port, mqtt_user, mqtt_pwd);
}

void loopMQTT() 
{
    mqtt.loop();
}

#endif
/////////////////////////////////////////////////////////////////////

////////// Purifier Code /////////////
void initPurifierPins()
{
    pinMode(BUILTIN_LED, OUTPUT);
    pinMode(Relay1_Pin, OUTPUT);
    pinMode(Relay2_Pin, OUTPUT);
}

void initDigitalPot() 
{
    pot.setup();
    pot.begin(10000);   // MCP4011 10Kohm installed
    pot.zeroWiper();
}

/////////////////// ARDUINO CODE //////////////
void setup()
{
    Serial.begin(115200);
    /* Valid for boards with USB-COM. Wait until the port is open */
    //while(!Serial) delay(10);
    for (int i = 8; i > 0; i--)
    {
        Serial.println(i);
        delay(1000);
    }

    initPurifierPins();
    initDigitalPot();
    dust.init();
    dust.registercallBack(onDustChanged);
    airStation.init();
    airStation.attachCallback(onBMEDataChanged);
    ledStrip.init();

    manager.registerMotorSpeedcallBack(onManagerMotorSpeedChanged);
    manager.registerLedColorcallBack(onManagerLedColorChanged);

    initWIFI();
    initMQTT();
}

void loop()
{
    //Sensors loop
    airStation.loop();
    dust.loop();

    //Actuators
    ledStrip.loop();

    //Communication
    loopMQTT();
}

/////////////////// ARDUINO CODE //////////////



