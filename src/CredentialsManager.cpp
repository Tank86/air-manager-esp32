#include "CredentialsManager.hpp"
#include <EEPROM.h>
#include <ESPmDNS.h>

String CredentialsManager::apSSID{"esp32 access point"};
IPAddress CredentialsManager::apIP{192, 168, 4, 1};
IPAddress CredentialsManager::netMsk{255, 255, 255, 0};
WebServer CredentialsManager::server{apIP, 80};
String CredentialsManager::serverHostname{"esp32portal"};
CredentialsManager::wifiList_t CredentialsManager::wifiList{false, 0};

uint16_t CredentialsManager::EEPROM_BaseAddress{0};

/** Handle root or redirect to captive portal */
void CredentialsManager::handleRoot()
{
    if (captivePortal())
    {
        // If caprive portal redirect instead of displaying the page.
        return;
    }

    // At root go to configuration page
    handleConfig();
}

void CredentialsManager::handleWifiScan()
{
    Serial.print("wifi list refreshing ...");
    while(!wifiList.refreshed);
    wifiList.scanCount = WiFi.scanNetworks();
    wifiList.refreshed = true;
    Serial.println("done");

    //TODO show a waiting page during scanning

    //Once done, go to config page
    handleConfig();
}

/** All config page handler */
void CredentialsManager::handleConfig()
{
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");

    String Page;
    Page += F("<!DOCTYPE html><html lang='en'><head>"
              "<meta name='viewport' content='width=device-width'>"
              "<title>Credential Page</title></head>"
              "<body bgcolor='grey'>"
              "<h1>Device configuration</h1>"
              "<h4><tr><th align='left'>Found WLAN list (<a href='/scan'>Click to refresh</a>)</th></tr></h4>"
              "<ul>");

    if (wifiList.scanCount > 0)
    {
        for (uint16_t i = 0; i < wifiList.scanCount; i++)
            Page += String(F("\r\n<li><tr><td>")) + WiFi.SSID(i) + ((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? F(" ") : F(" &#128272")) +
                    F(" (") + WiFi.RSSI(i) + F(")</td></tr></li>");
    }
    else
    {
        Page += F("<li><tr><td>No WLAN found</td></tr></li>");
    }

    Page += F("</ul>"
              "\r\n<br /><form method='POST' action='configsave'><h4>Enter wifi info:</h4>"
              "<input type='text' maxlength='50' placeholder='ssid' name='wifi_ssid'/>"
              "<br /><input type='password' maxlength='50' placeholder='password' name='wifi_pwd'/>"
              "<h4>Enter MQTT info:</h4>"
              "<input type='text' maxlength='50' placeholder='address(homeassistant.local)' name='mqtt_addr'/>"
              "<br /><input type='number' min='0' max='65535' placeholder='port(1883)' value='1883' name='mqtt_port'/>"
              "<br /><input type='text' maxlength='50' placeholder='user' name='mqtt_user'/>"
              "<br /><input type='text' maxlength='50' placeholder='password' name='mqtt_pwd'/>"
              "<br /><input type='submit' value='Save'/></form>"
              "</body></html>");
    server.send(200, "text/html", Page);
    server.client().stop(); // Stop is needed because we sent no content length
}

/** Handle saving of full credential configuration */
void CredentialsManager::handleConfigSave()
{
    String wifi_ssid{server.arg("wifi_ssid")};
    String wifi_pwd{server.arg("wifi_pwd")};
    uint16_t mqtt_port = String{server.arg("mqtt_port")}.toInt();
    String mqtt_address{server.arg("wifi_ssid")};
    String mqtt_user{server.arg("mqtt_user")};
    String mqtt_pwd{server.arg("mqtt_pwd")};
    Serial.println("Server received submit (" + wifi_ssid + " , " + wifi_pwd + " , " + mqtt_address + " , " + mqtt_port + " , " +
                   mqtt_user + " , " + mqtt_pwd + " )");

    server.sendHeader("Location", "config", true);
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");

    // server.send(302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
    // server.client().stop();             // Stop is needed because we sent no content length

    if (!wifi_ssid.isEmpty() /*&& !wifi_pwd.isEmpty()*/ && mqtt_port != 0 && !mqtt_address.isEmpty())
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
            response_error += "<h2><a href='/'>Data ok, but saving fails,</a> to try again";
            server.send(200, "text/html", response_error);
        }
    }
    else
    {
        String response_error = "<h1>Error</h1>";
        response_error += "<h2><a href='/'>data is invalid</a> to try again";
        server.send(200, "text/html", response_error);
    }

    // connect = strlen(ssid) > 0; // Request WLAN connect with new credentials if there is a SSID
}


/** Redirect to captive portal if we got a request for another domain. Return true in that case so the page handler do not try to handle the
 * request again. */
bool CredentialsManager::captivePortal()
{
    if (!isIp(server.hostHeader()) && server.hostHeader() != (String(serverHostname) + ".local"))
    {
        Serial.println("Request redirected to captive portal");
        server.sendHeader("Location", String("http://") + server.client().localIP().toString(), true);
        server.send(302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
        server.client().stop();             // Stop is needed because we sent no content length
        return true;
    }
    return false;
}

/**
 * Handle unknown URLs
 */
void CredentialsManager::handleNotFound()
{
    if (captivePortal())
    {
        // If caprive portal redirect instead of displaying the error page.
        return;
    }

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

    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.send(404, "text/plain", message);
}

/** Is this an IP? */
bool CredentialsManager::isIp(String str)
{
    for (size_t i = 0; i < str.length(); i++)
    {
        int c = str.charAt(i);
        if (c != '.' && (c < '0' || c > '9'))
        {
            return false;
        }
    }
    return true;
}

void CredentialsManager::runAPServer()
{
    Serial.println("Starting Access Point...");
    WiFi.softAPConfig(apIP, apIP, netMsk);
    WiFi.softAP(apSSID.c_str());
    delay(500); // Without delay I've seen the IP address blank
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Setup the DNS server redirecting all the domains to the apIP
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", IP);

    server.on("/", handleRoot);
    server.on("/scan", handleWifiScan);
    server.on("/config", handleConfig);
    server.on("/configsave", handleConfigSave);

    server.onNotFound(handleNotFound);
    server.on("/generate_204", handleRoot); // Android captive portal. Maybe not needed. Might be handled by notFound handler.
    server.on("/fwlink", handleRoot);       // Microsoft captive portal. Maybe not needed. Might be handled by notFound handler.

    server.begin();
    Serial.println("HTTP server started");
}

bool CredentialsManager::waitReboot()
{
    bool mdnsaction = false;

    while (true)
    {
        if(!wifiList.refreshed)
        {
            int16_t status = WiFi.scanComplete();
            if(status == WIFI_SCAN_FAILED) 
                 status = WiFi.scanNetworks(true);
                

            if(status == WIFI_SCAN_RUNNING) { Serial.print("."); } //wait
            else if(status == WIFI_SCAN_FAILED) { } //retry
            else
            {
                wifiList.scanCount = status;
                wifiList.refreshed = true;
                Serial.println("wifi scan done");
            }
        }

        if (!mdnsaction && (WiFi.status() == WL_CONNECTED))
        {
            // Setup MDNS responder
            if (!MDNS.begin(serverHostname.c_str()))
            {
                Serial.println("Error setting up MDNS responder!");
            }
            else
            {
                Serial.println("mDNS responder started");
                // Add service to MDNS-SD
                MDNS.addService("http", "tcp", 80);
                mdnsaction = true;
            }
        }

        // DNS
        dnsServer.processNextRequest();
        // HTTP
        server.handleClient();

        delay(100);
    }

    return false;
}

void CredentialsManager::initEEPROMArea()
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
bool CredentialsManager::isEEPROMContentValid()
{
    Serial.println("Checking EEPROM content");
    uint32_t magicKey = EEPROM.readUInt(EEPROM_BaseAddress + EE_OFFSET_MAGICKEY);

    return (magicKey == EE_MAGICKEY);
}

void CredentialsManager::loadEEPROMData()
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
bool CredentialsManager::writeCredentials(const String &wifi_ssid, const String &wifi_pwd, const String &mqtt_add, uint16_t mqtt_port,
                                          const String &mqtt_usr, const String &mqtt_pwd)
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
    if (success)
    {
        delay(100); // Avoid double write instantly, so wait a little (not sure it is needed, but it does not hurts)
        EEPROM.writeUInt(EEPROM_BaseAddress + EE_OFFSET_MAGICKEY, EE_MAGICKEY);
        Serial.println("Stored credentials");
        return EEPROM.commit();
    }
    else
    {
        Serial.println("Error when storing credentials");
        return false;
    }
}

void CredentialsManager::init(const char *SSID, const char *hostname)
{
    serverHostname = hostname;
    apSSID = SSID;
}

void CredentialsManager::loadParameters(uint16_t EE_BaseAddr, bool forcedShowAP)
{
    EEPROM_BaseAddress = EE_BaseAddr;

    // Check if EEPROM content is valid
    if (forcedShowAP || !isEEPROMContentValid())
    {
        if (!forcedShowAP)
        {
            initEEPROMArea();
            Serial.println("Initializing EEPROM Credentials area...");
        }

        Serial.println("Starting WIFI AP server mode to enter credentials");
        runAPServer();

        // Run server and wait for user to enter credentials
        while (waitReboot())
            ;
    }
    else
    {
        Serial.println("Parameters are valid, start directly without showing ap webserver");

        // Load EEPROM area
        loadEEPROMData();
    }
}

bool CredentialsManager::isWifiReacheable()
{
    if(!Wifi_SSID.isEmpty())
    {
        // Try Connect to wifi
        Serial.println("Trying conenction to  " + Wifi_SSID);
        WiFi.begin(Wifi_SSID.c_str(), Wifi_Pwd.c_str());

        bool connected = false;
        for(uint32_t retry = 0; (retry < 30) && !connected; retry++)
        {
            Serial.print(".");
            delay(500); //Waiting between tries
            connected = (WiFi.status() == WL_CONNECTED);
        }

        return connected;
    }
    else
    {
        return false;
    }
}