#ifndef INCLUDE_CREDENTIALS_MANAGER_HPP
#define INCLUDE_CREDENTIALS_MANAGER_HPP

#include <Arduino.h>
#include <DNSServer.h>
#include <WebServer.h>

class CredentialsManager
{
  public:
    CredentialsManager() = default;

    void init(const char *SSID, const char *hostname);
    void loadParameters(uint16_t EEPROM_BaseAddress, bool forcedShowAP);
    bool isWifiReacheable();

    String   getWifiSSID() const { return Wifi_SSID; }
    String   getWifiPWD() const { return Wifi_Pwd; }
    uint16_t getMqttPort() const { return Mqtt_Port; }
    String   getMqttAddress() const { return Mqtt_Address; }
    String   getMqttUser() const { return Mqtt_User; }
    String   getMqttPwd() const { return Mqtt_Pwd; }

  private:
    void runAPServer();
    bool waitReboot();

    static void handleRoot();
    static void handleWifiScan();
    static void handleConfig();
    static void handleConfigSave();
    static void handleNotFound();
    static bool captivePortal();

    // Utilities
    static bool isIp(String str);

    // EEPROM read/Save
    void        initEEPROMArea();
    bool        isEEPROMContentValid();
    void        loadEEPROMData();
    static bool writeCredentials(const String &wifi_ssid, const String &wifi_pwd, const String &mqtt_add, uint16_t mqtt_port,
                                 const String &mqtt_usr, const String &mqtt_pwd);

    // EEPROM Data
    static uint16_t EEPROM_BaseAddress;

    // Soft AP network parameters
    static String    apSSID;
    static IPAddress apIP;
    static IPAddress netMsk;
    static WebServer server;
    static struct wifiList_t
    {
        bool     refreshed;
        uint16_t scanCount;
    } wifiList;

    // hostname for DNS. like http://esp32portal.local
    static String  serverHostname;
    const uint16_t DNS_PORT{53};
    DNSServer      dnsServer;

    enum : uint16_t
    {
        EE_MAGICKEY            = 4486,
        EE_OFFSET_MAGICKEY     = 0,
        EE_OFFSET_MQTT_PORT    = 4,
        EE_OFFSET_MQTT_ADDRESS = 10,
        EE_OFFSET_MQTT_USER    = 100,
        EE_OFFSET_MQTT_PWD     = 200,
        EE_OFFSET_WIFI_SSID    = 300,
        EE_OFFSET_WIFI_PWD     = 400,
    };

    String   Wifi_SSID{""};
    String   Wifi_Pwd{""};
    uint16_t Mqtt_Port{1883};
    String   Mqtt_Address{""};
    String   Mqtt_User{""};
    String   Mqtt_Pwd{""};
};

#endif
