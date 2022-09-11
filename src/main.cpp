#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <ArduinoHA.h>
#include <PubSubClient.h>
#include <bsec2.h>
#include <MCP40xx.h>


///////////////////////////////////////////////////////////////////////////
/* Configuration for two class classification used here
 * For four class classification please use configuration under config/FieldAir_HandSanitizer_Onion_Cinnamon
 */
#include "config/FieldAir_HandSanitizer/FieldAir_HandSanitizer.h"
/* Gas estimate names will be according to the configuration classes used */
const String gasName[] = { "Field Air", "Hand sanitizer", "Undefined 3", "Undefined 4"};


#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day

// Helper functions declarations
void checkBsecStatus(Bsec2 bsec);
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);
void errLeds(void);
bool loadState(Bsec2 bsec);
bool saveState(Bsec2 bsec);


// Create an object of the class Bsec
Bsec2 iaqSensor;
static uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

void initBME680(void)
{
  String output;

  Wire.begin();
  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);

  /* Desired subscription list of BSEC2 outputs */
  bsecSensor sensorList[] = {
          BSEC_OUTPUT_IAQ,
          BSEC_OUTPUT_CO2_EQUIVALENT,                         /*!< co2 equivalent estimate [ppm] */   
          BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,                  /*!< breath VOC concentration estimate [ppm] */    	
          BSEC_OUTPUT_RAW_TEMPERATURE,
          BSEC_OUTPUT_RAW_PRESSURE,
          BSEC_OUTPUT_RAW_HUMIDITY,
          BSEC_OUTPUT_RAW_GAS,
          BSEC_OUTPUT_STABILIZATION_STATUS, //ongoing (0) or stabilization is finished (1)
          BSEC_OUTPUT_RUN_IN_STATUS, //ongoing (0) or stabilization is finished (1)
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
  if (!iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire))
  {
    checkBsecStatus(iaqSensor);
  }

  /* Load the configuration string that stores information on how to classify the detected gas */
  if(0)// (!iaqSensor.setConfig(FieldAir_HandSanitizer_config))
  {
    checkBsecStatus (iaqSensor);
  }

  /* Copy state from the EEPROM to the algorithm */
  if(!loadState(iaqSensor))
  {
    checkBsecStatus (iaqSensor);
  }

  /* Subsribe to the desired BSEC2 outputs */
  if (!iaqSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_ULP));// BSEC_SAMPLE_RATE_HIGH_PERFORMANCE))
  {
    checkBsecStatus(iaqSensor);
  }

  /* Whenever new data is available call the newDataCallback function */
  iaqSensor.attachCallback(newDataCallback);

  Serial.println("BSEC library version " + \
      String(iaqSensor.version.major) + "." \
    + String(iaqSensor.version.minor) + "." \
    + String(iaqSensor.version.major_bugfix) + "." \
    + String(iaqSensor.version.minor_bugfix));
}

void updateBsecState(Bsec2 bsec)
{
    static uint16_t stateUpdateCounter = 0;
    bool update = false;

    if (!stateUpdateCounter || (stateUpdateCounter * STATE_SAVE_PERIOD) < millis())
    {
        /* Update every STATE_SAVE_PERIOD minutes */
        update = true;
        stateUpdateCounter++;
    }

    if (update && !saveState(bsec))
        checkBsecStatus(bsec);
}


void loopBME680(void)
{
    /* Call the run function often so that the library can 
     * check if it is time to read new data from the sensor  
     * and process it.
     */
    if (!iaqSensor.run())
    {
        checkBsecStatus(iaqSensor);
    } 
}

// Helper function definitions
void checkBsecStatus(Bsec2 bsec)
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


bool loadState(Bsec2 bsec)
{
    if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
    {
        /* Existing state in EEPROM */
        Serial.println("Reading state from EEPROM");
        Serial.print("State file: ");
        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
        {
            bsecState[i] = EEPROM.read(i + 1);
            Serial.print(String(bsecState[i], HEX) + ", ");
        }
        Serial.println();

        if (!bsec.setState(bsecState))
            return false;
    } else
    {
        /* Erase the EEPROM with zeroes */
        Serial.println("Erasing EEPROM");

        for (uint8_t i = 0; i <= BSEC_MAX_STATE_BLOB_SIZE; i++)
            EEPROM.write(i, 0);

        EEPROM.commit();
    }

    return true;
}

bool saveState(Bsec2 bsec)
{
    if (!bsec.getState(bsecState))
        return false;

    Serial.println("Writing state to EEPROM");
    Serial.print("State file: ");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
        EEPROM.write(i + 1, bsecState[i]);
        Serial.print(String(bsecState[i], HEX) + ", ");
    }
    Serial.println();

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();

    return true;
}


void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}


///////////Sharp DustSensor GP2Y1010AU0F | GP2Y1014AU0F////////
#define COV_RATIO        0.17//0.2    //ug/mmm / mv
#define NO_DUST_VOLTAGE  600//400    //mv
#define SYS_VOLTAGE      3300   


/* I/O define */
const uint8_t iled = 3; //A2;  //drive the led of sensor
const uint8_t vout = 5; //A4;  //analog input

float dust_density = nanf("");

uint16_t Filter(uint16_t m)
{
  const size_t _buff_max = 10;
  static uint16_t _buff[10];
  static int flag_first = 0, sum;
  uint16_t i;
  
  if(flag_first == 0)
  {
    flag_first = 1;
    for(i = 0, sum = 0; i < _buff_max; i++)
    {
      _buff[i] = m;
      sum += _buff[i];
    }
    return m;
  }
  else
  {
    sum -= _buff[0];
    for(i = 0; i < (_buff_max - 1); i++)
    {
      _buff[i] = _buff[i + 1];
    }
    _buff[9] = m;
    sum += _buff[9];
    
    i = sum / 10.0;
    return i;
  }
}


void initDustSensor()
{
  // iled default closed
  pinMode(iled, OUTPUT);
  digitalWrite(iled, LOW);
  
  Serial.println("Dust sensor initialised");
}

void loopDustSensor(void)
{
  static uint32_t lastTimer = millis();

  if((millis() - lastTimer) > 60000)
  {
    lastTimer = millis();

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
    float voltage = (adcvalue * (1.0/0.5));
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
    dust_density = ((0.17 * (voltage/1000.0)) - 0.1) * 1000.0;
    if(dust_density < 0) 
      dust_density = 0;
    Serial.print("The current dust concentration is: ");
    Serial.print(dust_density);
    Serial.print(" ug/m3 ");
    if(dust_density <= 50)        Serial.print("- Air quality: Excelent");
    else if(dust_density <= 100)  Serial.print("- Air quality: Average");
    else if(dust_density <= 150)  Serial.print("- Air quality: Light pollution");
    else if(dust_density <= 200)  Serial.print("- Air quality: Moderatre pollution");
    else if(dust_density <= 300)  Serial.print("- Air quality: Heavy pollution");
    else if(dust_density > 300)   Serial.print("- Air quality: Serious pollution");
    Serial.println("");

  //  PM2.5 density  |  Air quality   |  Air quality   | Air quality 
  // value(μg/m3)    |  index  (AQi)  |     level      |  evaluation
  //     0-35        |      0-50      |        I       |  Excellent
  //     35-75       |      51-100    |       II       |  Average
  //     75-115      |      101-150   |      III       |  Light pollution
  //     115-150     |      151-200   |       IV       |  Moderate pollution
  //     150-250     |      201-300   |        V       |  Heavy pollution
  //     250-500     |      ≥300 	    |       VI       |  Serious pullution 
  }
}
//////////////////////////////

/////////// MCP 40xx digital potentiometer ////////

// Pins declaration
const uint8_t POT_UDPin = 16;   // UP/DOWN
const uint8_t POT_CSPin = 18;  // CHIP SELECT

MCP40xx pot = MCP40xx(POT_CSPin, POT_UDPin);

void initDigitalPot() 
{
  pot.setup();
  // MCP4011 10Kohm installed
  pot.begin(10000);

  pot.zeroWiper();
}

//////////////////////////////

#include <wifiCredentials.hpp>
//const char* ssid = "";
//const char* password = "";
//const char* mqtt_server = "homeassistant.local";
//const char* mqtt_user = "";
//const char* mqtt_pwd = "";
///////////////////////////////////////

#ifndef HOMEASSISTANT

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

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
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else if(messageTemp == "off"){
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
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void initMQTT(void) {
  setup_wifi();
  client.setServer(mqtt_server, 1883);
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

    //Send co2 & iaq only when stabilisation is at least at medium state (2)
    if(iaqSensor.iaqAccuracy > 1) {
      //TODO Send iaqSensor.iaq
      //TODO Send iaqSensor.co2Equivalent
      //TODO Send iaqSensor.breathVocEquivalent
    }

  }
}
#else
/////////////////// Home assistant device ///////////

WiFiClient espClient;
HADevice device;
HAMqtt mqtt(espClient, device);
// See https://www.home-assistant.io/integrations/#search/mqtt
// See https://www.home-assistant.io/integrations/sensor/#device-class
// Need to add MQTT Light https://www.home-assistant.io/integrations/light.mqtt/
//HASwitch led("led", false); // "led" is unique ID of the switch. You should define your own ID.
HALight ledStrip("ledStrip");
HAFan purifierMotor("motor", HAFan::SpeedsFeature); // "motor" is unique ID of the switch.
HASensor temp("temperature"); // "temperature" is unique ID of the sensor.
HASensor humidity("humidity"); // "humidity" is unique ID of the sensor.
HASensor pressure("pressure"); // "pressure" is unique ID of the sensor.
HASensor co2("co2_equivalent"); // "co2_equivalent" is unique ID of the sensor.
HASensor iaq("iaq"); // quality air index
HASensor iaqAccuracy("iaqAccuracy"); // 0: stabilisation, 1, low, 2, medium, 3, high
HASensor vocEquivalent("vocEquivalent"); // breath voc equivalent (ppm)
HASensor dustPM25("pm2.5");

const uint8_t Relay1_Pin = 11;
const uint8_t Relay2_Pin = 12;


void onPurifierMotorSpeedChanged(uint16_t speed)
{
    Serial.println("Fan speed received "+  String(speed));
    
    //Assign pot value according to speed
    //pot.setValue()
}

void onPurifierMotorStateChanged(bool state)
{
    Serial.println("Fan state received "+  String(state));

    //Set relay on/off
    digitalWrite(Relay1_Pin, state);
    digitalWrite(Relay2_Pin, state);
}

void onLedBrightnessChanged(uint8_t brightness)
{
    Serial.println("Led Strip Brightness Changed "+  String(brightness));
}

void onLedColorChanged(uint8_t red, uint8_t green, uint8_t blue)
{
    Serial.println("Led Strip Color Changed "+  String(red) + ", " +  String(green) + ", " +  String(blue));
}

void onLedStateChanged(bool state)
{
    Serial.println("Led Strip State Changed "+  String(state));
}


void initPurifierPins()
{
    pinMode(BUILTIN_LED, OUTPUT);
    pinMode(Relay1_Pin, OUTPUT);
    pinMode(Relay2_Pin, OUTPUT);
}


void initHAMQTTDevice() {
    // Unique ID must be set!
    byte mac[6]; //WL_MAC_ADDR_LENGTH];
    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));

    // connect to wifi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500); // waiting for the connection
    }
    Serial.println();
    Serial.println("Connected to the network");

    // set device's details (optional)
    device.setName("air-manager");
    device.setManufacturer("Tank86 electronics");
    device.setModel("Air purifier station");
    device.setSoftwareVersion("1.0.0");

    //purifierMotor.setRetain(true);
    purifierMotor.onSpeedChanged(onPurifierMotorSpeedChanged);
    purifierMotor.onStateChanged(onPurifierMotorStateChanged);
    purifierMotor.setSpeedRangeMin(30); 
    purifierMotor.setSpeedRangeMax(95);
    //purifierMotor.setIcon("mdi:air-purifier");
    purifierMotor.setName("AirPurifier speed");

    ledStrip.onStateChanged(onLedStateChanged);
    ledStrip.onBrightnessChanged(onLedBrightnessChanged);
    ledStrip.onColorChanged(onLedColorChanged);
    ledStrip.setIcon("mdi:lightbulb"); //optional
    ledStrip.setName("AirPurifier strip"); // optional

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

    vocEquivalent.setUnitOfMeasurement("ppm");
    vocEquivalent.setDeviceClass("volatile_organic_compounds");
    vocEquivalent.setIcon("mdi:emoticon-poop");
    vocEquivalent.setName("AirPurifier VOC equivalent");

    dustPM25.setUnitOfMeasurement("ug/m3");
    dustPM25.setDeviceClass("pm25");
    dustPM25.setIcon("mdi:weather-dust");
    dustPM25.setName("AirPurifier PM2.5 (dust sensor)");

    // start server
    mqtt.begin(mqtt_server, 1883, mqtt_user, mqtt_pwd);
}

void loopHAMQTTDevice() {
    static unsigned long lastSentAt = millis();

    mqtt.loop();

    // Update sensors values every minute
    if ((millis() - lastSentAt) >= 60000)
        //Send dust density
        if(!isnan(dust_density))
            dustPM25.setValue(dust_density);

        //Send co2 & iaq only when stabilisation is finished
        //purifierMotor.setSpeed(iaqSensor.temperature);
}

#endif
////////////////////////////////////////////////

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
    if (!outputs.nOutputs)
    {
        return;
    }

    Serial.println("BSEC outputs:\n\ttimestamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
        const bsecData output  = outputs.output[i];
        switch (output.sensor_id)
        {
            case BSEC_OUTPUT_IAQ:
                Serial.println("\tiaq = " + String(output.signal));
                Serial.println("\tiaq accuracy = " + String((int) output.accuracy));
                iaqAccuracy.setValue(output.accuracy);
                if(output.accuracy > 1) //at least accuray medium
                  iaq.setValue(output.signal);
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                Serial.println("\tc02 equivalent = " + String(output.signal));
                if(output.accuracy != 0)
                  co2.setValue(output.signal);
                break;
            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                Serial.println("\tbreath voc equivalent = " + String(output.signal));
                if(output.accuracy != 0)
                  vocEquivalent.setValue(output.signal);
                break;
            case BSEC_OUTPUT_RAW_TEMPERATURE:
                Serial.println("\ttemperature = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
                Serial.println("\tpressure = " + String(output.signal));
                pressure.setValue(output.signal / 100.0f);  //convert to mbar
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
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE :
                Serial.println("\ttemperature comp = " + String(output.signal));
                temp.setValue(output.signal);
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY :
                Serial.println("\thumidity comp = " + String(output.signal));
                humidity.setValue(output.signal);
                break;
            case BSEC_OUTPUT_COMPENSATED_GAS : 
                Serial.println("\tcompensated gas = " + String(output.signal));
                break;
            case BSEC_OUTPUT_GAS_PERCENTAGE :
                Serial.println("\tgas percentage = " + String(output.signal));
                break;
            case BSEC_OUTPUT_GAS_ESTIMATE_1 :
                Serial.println("\tgas estimate 1 = " + String(output.signal));
                break;
            case BSEC_OUTPUT_GAS_ESTIMATE_2 :
                Serial.println("\tgas estimate 2 = " + String(output.signal));
                break;
          	case BSEC_OUTPUT_GAS_ESTIMATE_3 :
                Serial.println("\tgas estimate 3 = " + String(output.signal));
                break;
          	case BSEC_OUTPUT_GAS_ESTIMATE_4 :
                Serial.println("\tgas estimate 4 = " + String(output.signal));
                break;
            default:
                Serial.println("\t!!!OUCH SENSOR UNKNOWN = " + String(output.sensor_id) + String(output.signal));
                break;
        }
    }
    
    updateBsecState(iaqSensor);
}

/////////////////// ARDUINO CODE //////////////
void setup() 
{
  Serial.begin(115200);
  /* Valid for boards with USB-COM. Wait until the port is open */
  //while(!Serial) delay(10);
  //Serial.available
  for(int i = 10; i >0; i--)
  {
    Serial.println(i);
    delay(1000);
  }

  initPurifierPins();
  initBME680();
  initDustSensor();
  initDigitalPot();

  //initMQTT();
  initHAMQTTDevice();
}

void loop()
{
  loopBME680();
  loopDustSensor();

  //loopMQTT();
  loopHAMQTTDevice();
  
  //digitalWrite(BUILTIN_LED, false);
  //sleep(500);
  //digitalWrite(BUILTIN_LED, true);
  //sleep(500);
}

/////////////////// ARDUINO CODE //////////////



