#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <bsec.h>
#include <VL53L1X.h>


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
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
void updateState(void);

// Create an object of the class Bsec
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

void initBME680(void)
{
  String output;

  //iaqSensor.begin(SS, SPI);
  Wire.begin();

  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();

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

  iaqSensor.updateSubscription(sensorList, sizeof(sensorList)/sizeof(sensorList[0]), BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], IAQ accuracy[0-3], IAQ, temperature [°C], pressure [mBar], humidity [%], CO2 equivalent, breath VOC equivalent";
  Serial.println(output);
}

void loopBME680(void)
{
  unsigned long time_trigger = millis();
  if (iaqSensor.run()) { // If new data is available
    String output = String(time_trigger);
    //output += ", " + String(iaqSensor.rawTemperature);
    //output += ", " + String(iaqSensor.rawHumidity);
    //output += ", " + String(iaqSensor.gasResistance);
    output += ", " + String(iaqSensor.iaqAccuracy); // 0: Stabilization 1: Low, 2:Medium : auto-trimming ongoing, 3: High accuracy
    output += ", " + String(iaqSensor.iaq);         // IAQ scale ranges from 0 (clean air) to 500 (heavily polluted air)
//    output += ", " + String(iaqSensor.staticIaq); // Unscaled IaQ
    output += ", " + String(iaqSensor.temperature) + "°C";
    output += ", " + String(iaqSensor.pressure /100) + "mBar";
    output += ", " + String(iaqSensor.humidity) + "%";
    output += ", " + String(iaqSensor.co2Equivalent) + "ppm";
    output += ", " + String(iaqSensor.breathVocEquivalent);
    Serial.println(output);
    updateState();
  } else {
    checkIaqSensorStatus();
  }
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      Serial.println("BSEC error code : " + String(iaqSensor.status));
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      Serial.println("BSEC warning code : " + String(iaqSensor.status));
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      Serial.println("BME680 error code : " + String(iaqSensor.bme680Status));
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      Serial.println("BME680 warning code : " + String(iaqSensor.bme680Status));
    }
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

    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
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
    if (iaqSensor.iaqAccuracy >= 3) {
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
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
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



///////////VL53L1X////////
VL53L1X sensor;

void initVL53L1X()
{
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  ;sensor.setROISize(4,4);
  ;sensor.setROICenter(155);

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  
  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);
}
void loopVL53L1X()
{
  sensor.read();

  Serial.print("range: ");
  Serial.print(sensor.ranging_data.range_mm);
  Serial.print("\tstatus: ");
  Serial.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  Serial.print("\tpeak signal: ");
  Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tambient: ");
  Serial.print(sensor.ranging_data.ambient_count_rate_MCPS);

  Serial.println();
}
/////////////////////////

///////////Sharp DustSensor GP2Y1010AU0F | GP2Y1014AU0F////////
#define COV_RATIO        0.17//0.2    //ug/mmm / mv
#define NO_DUST_VOLTAGE  600//400    //mv
#define SYS_VOLTAGE      3300   


/* I/O define */
const uint8_t iled = A15;  //drive the led of sensor
const uint8_t vout = A14;  //analog input


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
  analogSetAttenuation(ADC_11db);  //No attenuation (range 100mV => 950mV)
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
 	float density = ((0.17 * (voltage/1000.0)) - 0.1) * 1000.0;
    
  Serial.print("The current dust concentration is: ");
  Serial.print(density);
  Serial.print(" ug/m3 ");
  if(density <= 50)        Serial.print("- Air quality: Excelent");
  else if(density <= 100)  Serial.print("- Air quality: Average");
  else if(density <= 150)  Serial.print("- Air quality: Light pollution");
  else if(density <= 200)  Serial.print("- Air quality: Moderatre pollution");
  else if(density <= 300)  Serial.print("- Air quality: Heavy pollution");
  else if(density > 300)   Serial.print("- Air quality: Serious pollution");
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

// Replace the next variables with your SSID/Password combination
const char* ssid = "";
const char* password = "";

// Add your MQTT Broker IP address:
const char* mqtt_server = "homeassistant.local";//"192.168.1.52";
const char* mqtt_user = "espModule_air";
const char* mqtt_pwd = "espmodule_air";

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

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
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
    if (client.connect("esp-air-manager", mqtt_user, mqtt_pwd)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
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
  if ((now - lastMsg) > 5000) {
    lastMsg = now;
    
    // Convert the value to a char array
    char tempString[8];
    dtostrf(iaqSensor.temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString);
    
    // Convert the value to a char array
    char humString[8];
    dtostrf(iaqSensor.humidity, 1, 2, humString);
    Serial.print("Humidity: ");
    Serial.println(humString);
    client.publish("esp32/humidity", humString);
  }
}



/////////////////// ARDUINO CODE //////////////
void setup() 
{
  Serial.begin(115200);

  initMQTT();
  initBME680();
  //initVL53L1X();
  initDustSensor();
}

void loop()
{
  loopMQTT();
  loopBME680();
  //loopVL53L1X();
  loopDustSensor();
}

/////////////////// ARDUINO CODE //////////////



