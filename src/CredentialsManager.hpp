#ifndef INCLUDE_CREDENTIALS_MANAGER_HPP
#define INCLUDE_CREDENTIALS_MANAGER_HPP

#include <Arduino.h>
#include <WebServer.h>


class ParameterManager
{
public:
    ParameterManager() = default;

    void loadParameters(uint16_t EEPROM_BaseAddress, bool forcedShowAP);
    String getWifiSSID() const { return Wifi_SSID; }
    String getWifiPWD() const  { return Wifi_Pwd; }
    uint16_t getMqttPort() const { return Mqtt_Port; }
    String getMqttAddress() const  { return Mqtt_Address; }
    String getMqttUser() const { return Mqtt_User; }
    String getMqttPwd() const  { return Mqtt_Pwd; }

private:
    void runAPServer();
    bool waitReboot();

    static void handleNotFound();
    static void handleRoot();

    static bool writeCredentials(String wifi_ssid, String wifi_pwd, String mqtt_add, uint16_t mqtt_port, String mqtt_usr, String mqtt_pwd);
    void initEEPROMArea();
    bool isEEPROMContentValid();
    void loadEEPROMData();

    static uint16_t EEPROM_BaseAddress;
    static WebServer server;

    enum : uint16_t
    {
        EE_MAGICKEY = 4486,
        EE_OFFSET_MAGICKEY = 0,
        EE_OFFSET_MQTT_PORT = 4,
        EE_OFFSET_MQTT_ADDRESS = 10,
        EE_OFFSET_MQTT_USER = 100,
        EE_OFFSET_MQTT_PWD = 200,
        EE_OFFSET_WIFI_SSID = 300,
        EE_OFFSET_WIFI_PWD = 400,
    };

    String Wifi_SSID{""};
    String Wifi_Pwd{""};
    uint16_t Mqtt_Port{1883};
    String Mqtt_Address{""};
    String Mqtt_User{""};
    String Mqtt_Pwd{""};
};

#endif

