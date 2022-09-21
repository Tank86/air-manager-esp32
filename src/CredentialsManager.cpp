#include "WiFiManager.hpp"
#include "WifiPage.h"
#include "EEPROM.h"

uint16_t ParameterManager::EEPROM_BaseAddress{0};
WebServer ParameterManager::server{IPAddress{192,168,4,1}, 80};

/*
* Function to handle unknown URLs
*/
void ParameterManager::handleNotFound()
{
    String message{"File Not Found\n\n"};
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i = 0; i < server.args(); i++)
    {
        message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }

    server.send(404, "text/plain", message);
}

/*
* Function for home page
*/
void  ParameterManager::handleRoot()
{
    if (server.hasArg("wifi_ssid") && 
        server.hasArg("wifi_pwd") && 
        server.hasArg("mqtt_port") && 
        server.hasArg("wifi_ssid") && 
        server.hasArg("mqtt_user") && 
        server.hasArg("mqtt_pwd") )
    {
        String wifi_ssid{server.arg("wifi_ssid")};
        String wifi_pwd{server.arg("wifi_pwd")};
        uint16_t mqtt_port = String{server.arg("mqtt_port")}.toInt();
        String mqtt_address{server.arg("wifi_ssid")};
        String mqtt_user{server.arg("mqtt_user")};
        String mqtt_pwd{server.arg("mqtt_pwd")};
        Serial.println("Server received submit (" + wifi_ssid + " , " + wifi_pwd + " , " + mqtt_address + " , " + mqtt_port + " , " + mqtt_user + " , " + mqtt_pwd +  " )");

        if(!wifi_ssid.isEmpty() &&
        !wifi_pwd.isEmpty() &&
            mqtt_port != 0 &&
        !mqtt_address.isEmpty() )
        {
            if (writeCredentials(wifi_ssid, wifi_pwd, mqtt_address, mqtt_port, mqtt_user, mqtt_pwd)) 
            {
                String response_success = "<h1>Success</h1>";
                response_success += "<h2>Device will restart in 3 seconds</h2>";
                server.send(200, "text/html", response_success);

                Serial.println("Rebooting ...");
                delay(3000);
                ESP.restart();
            }
            else 
            {
                String response_error = "<h1>Error</h1>";
                response_error += "<h2><a href='/'>Data ok, but saving fails,</a>to try again";
                server.send(200, "text/html", response_error);
            }
        }
        else 
        {
            String response_error = "<h1>Error</h1>";
            response_error += "<h2><a href='/'>data is invalid</a>to try again";
            server.send(200, "text/html", response_error);
        }
    }
    else 
    {
        Serial.println("Server missing some parameters, retry enter them");
        server.send(200, "text/html", INDEX_HTML);
    }
}

/*
* Function for loading form
* Returns: false if no WiFi creds in EEPROM
*/
void ParameterManager::runAPServer()
{
    const char* ssid = "ESP32 Air Purifier";

    Serial.println("Starting Access Point...");
    WiFi.softAP(ssid);
    //WiFi.softAPConfig()   //TODO custom address

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    server.on("/", handleRoot);
    server.onNotFound(handleNotFound);
    server.begin();

    Serial.println("HTTP server started");
}

bool ParameterManager::waitReboot()
{
    while (true) 
    {
        server.handleClient();
        delay(100);
    }

    return false;
}

void ParameterManager::initEEPROMArea() 
{
    for (uint32_t i = 0; i < 512; i++) 
    {
        EEPROM.writeByte(EEPROM_BaseAddress + i, 0xFF);
    }
    EEPROM.commit();
}


/*
* Function checking WiFi creds in memory
* Returns: true if not empty, false if empty
*/
bool  ParameterManager::isEEPROMContentValid()
{
    Serial.println("Checking EEPROM content");
    uint32_t magicKey = EEPROM.readUInt(EEPROM_BaseAddress + EE_OFFSET_MAGICKEY);

    return (magicKey == EE_MAGICKEY);
}


void ParameterManager::loadEEPROMData()
{
    char buffer[100];

    EEPROM.readString(EEPROM_BaseAddress + EE_OFFSET_WIFI_SSID, buffer, sizeof(buffer));
    Wifi_SSID = String(buffer);
    EEPROM.readString(EEPROM_BaseAddress + EE_OFFSET_WIFI_PWD, buffer, sizeof(buffer));
    Wifi_Pwd = String(buffer);

    Mqtt_Port = EEPROM.readUShort(EEPROM_BaseAddress + EE_OFFSET_MQTT_PORT);
    EEPROM.readString(EEPROM_BaseAddress + EE_OFFSET_MQTT_ADDRESS, buffer, sizeof(buffer));
    Mqtt_Address = String(buffer);
    EEPROM.readString(EEPROM_BaseAddress + EE_OFFSET_MQTT_USER, buffer, sizeof(buffer));
    Mqtt_User = String(buffer);
    EEPROM.readString(EEPROM_BaseAddress + EE_OFFSET_MQTT_PWD, buffer, sizeof(buffer));
    Mqtt_Pwd = String(buffer);
}

/*
* Function for writing creds to EEPROM
* Returns: true if save successful, false if unsuccessful
*/
bool ParameterManager::writeCredentials(String wifi_ssid, String wifi_pwd, String mqtt_add, uint16_t mqtt_port, String mqtt_usr, String mqtt_pwd)
{
    char buffer[100];

    wifi_ssid.toCharArray(buffer, 100);
    EEPROM.writeString(EEPROM_BaseAddress + EE_OFFSET_WIFI_SSID, buffer);
    wifi_pwd.toCharArray(buffer, 100);
    EEPROM.writeString(EEPROM_BaseAddress + EE_OFFSET_WIFI_PWD, buffer);
    mqtt_add.toCharArray(buffer, 100);
    EEPROM.writeString(EEPROM_BaseAddress + EE_OFFSET_MQTT_ADDRESS, buffer);
    EEPROM.writeUShort(EEPROM_BaseAddress + EE_OFFSET_MQTT_PORT, mqtt_port);
    mqtt_usr.toCharArray(buffer, 100);
    EEPROM.writeString(EEPROM_BaseAddress + EE_OFFSET_MQTT_USER, buffer);
    mqtt_pwd.toCharArray(buffer, 100);
    EEPROM.writeString(EEPROM_BaseAddress + EE_OFFSET_MQTT_PWD, buffer);

    bool success = EEPROM.commit();
    if(success)
    {
        delay(100); //just in case
        EEPROM.writeUInt(EEPROM_BaseAddress + EE_OFFSET_MAGICKEY, EE_MAGICKEY);
        Serial.println("Stored Wifi credentials");
        return EEPROM.commit();
    }
    else
    {
        Serial.println("Error when storing wifi credentials");
        return false;
    }
}


void ParameterManager::loadParameters(uint16_t EE_BaseAddr, bool forcedShowAP)
{
    EEPROM_BaseAddress = EE_BaseAddr;

    // Check if EEPROM content is valid
    if (forcedShowAP || !isEEPROMContentValid())
    {
        if(!forcedShowAP)
        {
            initEEPROMArea();
            Serial.println("Initializing EEPROM Credentials area...");
        }

        Serial.println("Starting WIFI AP server mode to enter credentials");
        runAPServer();

        //Run server and wait for user to enter credentials
        while (waitReboot());
    }
    else
    {
        Serial.println("Parameters are valid, start directly without showing ap webserver");

        // Load EEPROM area
        loadEEPROMData();
    }
}

