#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <ArduinoHA.h>
#include <PubSubClient.h>
#include <bsec2.h>
#include <MCP40xx.h>


///////////////////////////////////////////////////////////////////////////
/* Configure the BSEC library with information about the sensor
    18v/33v = Voltage at Vdd. 1.8V or 3.3V
    3s/300s = BSEC operating mode, BSEC_SAMPLE_RATE_LP or BSEC_SAMPLE_RATE_ULP
    4d/28d = Operating age of the sensor in days
    generic_18v_3s_4d
    generic_18v_3s_28d
    generic_18v_300s_4d
    generic_18v_300s_28d
    generic_33v_3s_4d
    generic_33v_3s_28d
    generic_33v_300s_4d
    generic_33v_300s_28d
*/
//const uint8_t bsec_config_iaq[] = {
//#include "config/generic_33v_3s_4d/bsec_iaq.txt"
//};

#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day

// Helper functions declarations
void checkBsecStatus(Bsec2 bsec);
void errLeds(void);
void loadState(void);
void updateState(void);

// Create an object of the class Bsec
Bsec2 iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

void initBME680(void)
{
  String output;

  //iaqSensor.begin(SS, SPI);
  Wire.begin();

  if(!iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire));
  {
    checkBsecStatus(iaqSensor);
  }
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);

  //TODO 
  if(0);//!iaqSensor.setConfig(bsec_config_iaq))
  {
    checkBsecStatus(iaqSensor);
  }

  EEPROM.begin(256);
  loadState();
  
  bsec_virtual_sensor_t sensorList[] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    //BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  if(!iaqSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_LP))
  {
    checkBsecStatus(iaqSensor);
  }

  // Print the header
  output = "Timestamp [ms], IAQ accuracy[0-3], IAQ, temperature [°C], pressure [mBar], humidity [%], CO2 equivalent, breath VOC equivalent";
  Serial.println(output);
}

void loopBME680(void)
{
  unsigned long time_trigger = millis();
  if (iaqSensor.run()) { // If new data is available
    String output = String(time_trigger);
    auto sensors = iaqSensor.getOutputs();
    //output += ", " + String(sensors->output[].signal iaqSensor.rawTemperature);
    //output += ", " + String(sensors->output[].signal iaqSensor.rawHumidity);
    //output += ", " + String(sensors->output[].signal iaqSensor.gasResistance);
    output += ", " + String(sensors->output[BSEC_OUTPUT_STABILIZATION_STATUS].signal); // 0: Stabilization 1: Low, 2:Medium : auto-trimming ongoing, 3: High accuracy
    output += ", " + String(sensors->output[BSEC_OUTPUT_IAQ].signal);         // IAQ scale ranges from 0 (clean air) to 500 (heavily polluted air)
//    output += ", " + String(sensors->output[]iaqSensor.staticIaq); // Unscaled IaQ
    output += ", " + String(sensors->output[BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE].signal) + "°C";
    output += ", " + String(sensors->output[BSEC_OUTPUT_RAW_PRESSURE].signal  / 100) + "mBar";
    output += ", " + String(sensors->output[BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY].signal) + "%";
    output += ", " + String(sensors->output[BSEC_OUTPUT_CO2_EQUIVALENT].signal) + "ppm";
    output += ", " + String(sensors->output[BSEC_OUTPUT_BREATH_VOC_EQUIVALENT].signal);
    Serial.println(output);
    updateState();
  } else {
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


void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.println(bsecState[i], HEX);
    }

    if(!iaqSensor.setState(bsecState))
      checkBsecStatus(iaqSensor);
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}

void updateState(void)
{
  bool update = false;
  /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
  if (stateUpdateCounter == 0) {
    //if (iaqSensor.iaqAccuracy >= 3) {
    if(iaqSensor.getOutputs()->output[BSEC_OUTPUT_STABILIZATION_STATUS].signal != 0) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    if(!iaqSensor.getState(bsecState))
      checkBsecStatus(iaqSensor);
    else
    {
      Serial.println("Writing state to EEPROM");

      for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
        EEPROM.write(i + 1, bsecState[i]);
        Serial.println(bsecState[i], HEX);
      }

      EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
      EEPROM.commit();
    }
  }
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
const uint8_t iled = A5;  //drive the led of sensor
const uint8_t vout = A4;  //analog input

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

  delay(1000);
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
    if (((millis() - lastSentAt) >= 60000) &&
        (iaqSensor.status == BSEC_OK)) {
        lastSentAt = millis();

        //Send commom values
        auto sensors = iaqSensor.getOutputs();
        temp.setValue(sensors->output[BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE].signal);
        humidity.setValue(sensors->output[BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY].signal);
        pressure.setValue(sensors->output[BSEC_OUTPUT_RAW_PRESSURE].signal / 100.0);
        iaqAccuracy.setValue(sensors->output[BSEC_OUTPUT_STABILIZATION_STATUS].signal);

        if(!isnan(dust_density))
            dustPM25.setValue(dust_density);

        //Send co2 & iaq only when stabilisation is finished
        if(sensors->output[BSEC_OUTPUT_STABILIZATION_STATUS].signal != 0) {
            iaq.setValue(sensors->output[BSEC_OUTPUT_IAQ].signal);
            co2.setValue(sensors->output[BSEC_OUTPUT_CO2_EQUIVALENT].signal);
            vocEquivalent.setValue(sensors->output[BSEC_OUTPUT_BREATH_VOC_EQUIVALENT].signal);
        }

        //purifierMotor.setSpeed(iaqSensor.temperature);
    }
}

#endif
////////////////////////////////////////////////


/////////////////// ARDUINO CODE //////////////
void setup() 
{
  Serial.begin(115200);

  initPurifierPins();
  initBME680();
  //initDustSensor();
  initDigitalPot();

  //initMQTT();
  initHAMQTTDevice();
}

void loop()
{
  loopBME680();
  //loopDustSensor();

  //loopMQTT();
  loopHAMQTTDevice();
}

/////////////////// ARDUINO CODE //////////////



