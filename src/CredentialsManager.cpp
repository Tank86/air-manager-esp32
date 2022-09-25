#include "CredentialsManager.hpp"
#include <EEPROM.h>

String                         CredentialsManager::apSSID{"esp32 access point"};
IPAddress                      CredentialsManager::apIP{192, 168, 4, 1};
IPAddress                      CredentialsManager::netMsk{255, 255, 255, 0};
WebServer                      CredentialsManager::server{apIP, 80};
String                         CredentialsManager::serverHostname{"esp32portal"};
CredentialsManager::wifiList_t CredentialsManager::wifiList{false, 0};

uint16_t CredentialsManager::EEPROM_BaseAddress{0};

#define COLOR_FORM "#4f4f4f"
#define COLOR_INPUT "#dddddd"
#define COLOR_INPUT_TEXT "#000000"
//#define COLOR_CONSOLE "#1f1f1f"
//#define COLOR_CONSOLE_TEXT "#65c115"
#define COLOR_BACKGROUND "#252525"
#define COLOR_TEXT "#eaeaea"
#define COLOR_SSI COLOR_TEXT
#define COLOR_BUTTON_BG "#1fa3ec"
#define COLOR_BUTTON "#faffff"

static const char HTTP_HEAD_ROOT[] PROGMEM = // HTTP HEADER
    "<!DOCTYPE html><html lang='en'><head>"
    "<meta name='viewport' content='width=device-width'>"
    "<link rel='icon' href='data:image/x-icon;base64,AAABAAEAEA8AAAEAIAAkBAAAFgAAACgAAAAQAAAAHgAAAAEAIAAAAAAAwAMAAMMOAADDDgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA09PUR8DAwSAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAANTU1f+6urv/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAADT09P/uLi5/wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA0dHS/7e2uP8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAADCmjvPxKFD/8SfQbcAAAAAAAAAAM/P0P+1tbb/AAAAAAAAAADDnT9fxJ5A/8KaO88AAAAAAAAAAAAAAADEoUKXxJ9B/8OeP/8AAAAAwpk7XwAAAADNzc7/s7O0/wAAAADBmDmHAAAAAMKbPP/DnD3/xKFClwAAAAAAAAAAxJ9B/8OcPv8AAAAAwZc4EMGXOP/BlTf/zMzN/7Kxs//BlDb/wZU2/8CVNkcAAAAAwpk6/8KbPP8AAAAAw55Ab8OcPf/CmjsIAAAAAMGVNv/AkzT/AAAAAOi5TYPouU2DAAAAAL+QMv/AkjP/AAAAAAAAAADBlzn/wpo7z8OcPv/CmTr/AAAAAAAAAADFokT/x6tNKAAAAADjsUX/36tA/wAAAAAAAAAAwpk6/8GXOAgAAAAAwJQ1/8GXOP/Cmjv/zLlc/wAAAADLtVdAy7RW/wAAAADir0Mc2qQ6/9SbMv8AAAAAAAAAAMisTv/Iq02fAAAAAMirTf/BlDb/zLlc/8y3Wf8AAAAAAAAAAMqxVP/Jr1JfAAAAANSbMv/HhyD/AAAAAMeoShDHqEr/AAAAAAAAAADHp0n/x6hK/wAAAADLtVf/yrJV9wAAAADJr1H/ya1P/wAAAAAAAAAAAAAAAAAAAADGpUf/xaRG/wAAAADFpEW3xaRF/8WkRhwAAAAAyrJV/8qwUv/JrlBvAAAAAMiqS//HqEqXAAAAAAAAAADEokQ3xKFD/wAAAADDn0AnxKBB/8SgQf8AAAAAAAAAAAAAAADJrlD/yKxO/wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAw5w+/8OcPv8AAAAAAAAAAAAAAAAAAAAAAAAAAMipS//Hp0lLAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAwpk7EMKYOv8AAAAAAAAAAAAAAAD+fwAA/n8AAP5/AAD+fwAAxmMAAIpRAACQCQAAEkwAADJkAAAkZAAAMkwAAJPIAACJkQAAz/MAAOfnAAA='>"
    "<title>Credential Page</title>";

static const char HTTP_HEAD_SCRIPT[] PROGMEM = // JAVASCRIPTS
    "<script>eb=s=>document.getElementById(s);"
    "function c(l){eb('s1').value=l.innerText;eb('p1').focus();}</script>";

static const char HTTP_HEAD_STYLE[] PROGMEM =
    "<style>"
    "div,fieldset,input,select{padding:5px;font-size:1em}"
    "fieldset{background:" COLOR_FORM "}" // COLOR_FORM, Also update HTTP_TIMER_STYLE
    "p{margin:0.5em 0}"
    "input{width:100%;box-sizing:border-box;-webkit-box-sizing:border-box;-moz-box-sizing:border-box;background:" COLOR_INPUT
    ";color:" COLOR_INPUT_TEXT "}"
    //"input[type=checkbox],input[type=radio]{width:1em;margin-right:6px;vertical-align:-1px;}"
    "input[type=range]{width:99%}"
    "select{width:100%;background:" COLOR_INPUT ";color:" COLOR_INPUT_TEXT "}" // COLOR_INPUT, COLOR_INPUT_TEXT
    //"textarea{resize:vertical;width:98%;height:318px;padding:5px;overflow:auto;background:" COLOR_CONSOLE ";color:" COLOR_CONSOLE_TEXT "}"
    "body{text-align:center;font-family:verdana,sans-serif;background:" COLOR_BACKGROUND "}" // COLOR_BACKGROUND
    "td{padding:0px}"
    "button {border:0;border-radius:0.3rem;background:" COLOR_BUTTON_BG ";color:" COLOR_BUTTON
    ";line-height:2.4rem;font-size:1.2rem;width:100%;-webkit-transition-duration:0.4s;transition-duration:0.4s;cursor:pointer}"
    "button:hover {background:#0e70a4}"
    ".bgrn {background:#47c266} .bgrn:hover {background:#5aaf6f}"
    //".bred {background:#d43535} .bred:hover {background:#931f1f}"
    "a {color:" COLOR_BUTTON_BG ";text-decoration:none}"
    ".p {float:left;text-align:left}"
    ".q {float:right; text-align:right}"
    ".si{display:inline-flex;align-items:flex-end;height:15px;padding:0}"
    ".si i{width:3px;margin-right:1px;border-radius:3px;background-color:" COLOR_SSI "}"
    ".si .b0{height:25%}.si .b1{height:50%}.si .b2{height:75%}.si .b3{height:100%}.o30{opacity:.3}"
    "</style>";

static String getRSSIIcon(int32_t rssi)
{
    String quality{};
    // rssi received in dbm (0 = perfect, -100 = bad)
    if (rssi < -100) rssi = -100;
    if (rssi > 0) rssi = 0;
    uint32_t num_bars = map(rssi, 0, -100, 4, 0);

    // Print signal strength indicator (has 4 bars)
    quality += "<div class='si'>";
    for (uint32_t k = 0; k < 4; ++k)
        quality += String("<i class='b" + String(k) + ((num_bars < k) ? PSTR(" o30") : PSTR("")) + String("'></i>"));
    quality += "</div>";

    // Serial.println(quality);
    return quality;
}

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
    while (!wifiList.refreshed)
        ;
    wifiList.scanCount = WiFi.scanNetworks();
    wifiList.refreshed = true;
    Serial.println("done");

    // TODO show a waiting page during scanning

    // Once done, go to config page
    handleConfig();
}

/** All config page handler */
void CredentialsManager::handleConfig()
{
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");

    String Page;
    Page += F(HTTP_HEAD_ROOT);
    Page += F(HTTP_HEAD_SCRIPT);
    Page += F(HTTP_HEAD_STYLE);
    Page += F("</head><body>");
    Page += F("<div style='text-align:left;display:inline-block;color:" COLOR_TEXT ";min-width:340px'>");
    Page += F("<div style='text-align:center;color:" COLOR_TEXT "'><h1>Device configuration</h1></div>"
              "<h4>Found WLAN list (<a href='/scan'>Click to refresh</a>)</h4>");

    if (wifiList.scanCount > 0)
    {
        for (uint16_t i = 0; i < wifiList.scanCount; i++)
            Page += String(F("\r\n<div><a href='#p' onclick='c(this)'>")) + WiFi.SSID(i) + String(F("</a><span class='q'>")) +
                    getRSSIIcon(WiFi.RSSI(i)) + ((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? F(" &#128275") : F(" &#128272")) +
                    F("</span></div>");
    }
    else
    {
        Page += F("<div><h4>No WLAN found</h4></div>");
    }

    Page += F("\r\n<br />"
              "<fieldset><legend><b>Parameters</b></legend>"
              "<form method='POST' action='configsave'>"
              "<p><b>WiFi Network</b><br><input id='s1' placeholder='Type or Select your WiFi Network' name='wifi_ssid'></p>"
              "<p><b>WiFi Password</b><br><input id='p1' type='password' placeholder='Enter your WiFi Password' name='wifi_pwd'></p>"
              "<p><b>Mqtt Address</b><br><input placeholder='e.g. homeassistant.local' name='mqtt_addr'></p>"
              "<p><b>Mqtt Port</b><br><input type='number' min='0' max='65535' placeholder='e.g. 1883' value='1883' name='mqtt_port'></p>"
              "<p><b>Mqtt User</b><br><input placeholder='Type Mqtt user (or let empty)' name='mqtt_user'></p>"
              "<p><b>Mqtt Password</b><br><input type='password' placeholder='Type Mqtt password (or let empty)' name='mqtt_pwd'></p>"
              "<br><button class='button bgrn' name='save' type='submit'>Save</button>"
              "</form></fieldset>"
              "</div></body></html>");

    server.send(200, "text/html", Page);
    server.client().stop(); // Stop is needed because we sent no content length
}

/** Handle saving of full credential configuration */
void CredentialsManager::handleConfigSave()
{
    String   wifi_ssid{server.arg("wifi_ssid")};
    String   wifi_pwd{server.arg("wifi_pwd")};
    uint16_t mqtt_port = String{server.arg("mqtt_port")}.toInt();
    String   mqtt_address{server.arg("mqtt_addr")};
    String   mqtt_user{server.arg("mqtt_user")};
    String   mqtt_pwd{server.arg("mqtt_pwd")};
#if _DEBUG
    Serial.println("Server received submit (" + wifi_ssid + ", " + wifi_pwd + ", " + mqtt_address + ", " + mqtt_port + ", " + mqtt_user +
                   ", " + mqtt_pwd + ")");
#endif

    server.sendHeader("Location", "config", true);
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");

    String Page;
    Page += F(HTTP_HEAD_ROOT);
    Page += F(HTTP_HEAD_STYLE);
    Page += F("</head><body>");
    Page += F("<div style='text-align:left;display:inline-block;color:" COLOR_TEXT ";min-width:340px'>");

    if (!wifi_ssid.isEmpty() /*&& !wifi_pwd.isEmpty()*/ && mqtt_port != 0 && !mqtt_address.isEmpty())
    {
        if (writeCredentials(wifi_ssid, wifi_pwd, mqtt_address, mqtt_port, mqtt_user, mqtt_pwd))
        {
            Page += F("<h1>Success</h1><br><h2>Device will restart in 3 seconds</h2>"
                      "</div></body></html>");
            server.send(200, "text/html", Page);

            Serial.println("Rebooting ...");
            delay(3000);
            ESP.restart();
        }
        else
        {
            Page += F("<h1>Error</h1><br><h2><a href='/'>Data ok, but saving fails,</a> click to try again"
                      "</div></body></html>");
            server.send(200, "text/html", Page);
        }
    }
    else
    {
        Page += F("<h1>Error</h1><br><h2><a href='/'>data is invalid,</a> click to try again"
                  "</div></body></html>");
        server.send(200, "text/html", Page);
    }

    // server.send(302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
    // server.client().stop();             // Stop is needed because we sent no content length
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
    while (true)
    {
        if (!wifiList.refreshed)
        {
            int16_t status = WiFi.scanComplete();
            if (status == WIFI_SCAN_FAILED) status = WiFi.scanNetworks(true);

            if (status == WIFI_SCAN_RUNNING)
            {
                // wait
                Serial.print(".");
            }
            else if (status == WIFI_SCAN_FAILED)
            {
                // retry
            }
            else
            {
                wifiList.scanCount = status;
                wifiList.refreshed = true;
                Serial.println("wifi scan done");
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
    Serial.println("Checking EEPROM magickey");
    uint32_t magicKey = EEPROM.readUInt(EEPROM_BaseAddress + EE_OFFSET_MAGICKEY);

    return (magicKey == EE_MAGICKEY);
}

void CredentialsManager::loadEEPROMData()
{
    String str;
    str = EEPROM.readString(EEPROM_BaseAddress + EE_OFFSET_WIFI_SSID);
    memcpy(credentials.Wifi_SSID, str.c_str(), str.length());
    str = EEPROM.readString(EEPROM_BaseAddress + EE_OFFSET_WIFI_PWD);
    memcpy(credentials.Wifi_Pwd, str.c_str(), str.length());
    str = EEPROM.readString(EEPROM_BaseAddress + EE_OFFSET_MQTT_ADDRESS);
    memcpy(credentials.Mqtt_Server, str.c_str(), str.length());
    credentials.Mqtt_Port = EEPROM.readUShort(EEPROM_BaseAddress + EE_OFFSET_MQTT_PORT);
    str                   = EEPROM.readString(EEPROM_BaseAddress + EE_OFFSET_MQTT_USER);
    memcpy(credentials.Mqtt_User, str.c_str(), str.length());
    str = EEPROM.readString(EEPROM_BaseAddress + EE_OFFSET_MQTT_PWD);
    memcpy(credentials.Mqtt_Pwd, str.c_str(), str.length());
}

/*
 * Function for writing creds to EEPROM
 * Returns: true if save successful, false if unsuccessful
 */
bool CredentialsManager::writeCredentials(const String& wifi_ssid, const String& wifi_pwd, const String& mqtt_add, uint16_t mqtt_port,
                                          const String& mqtt_usr, const String& mqtt_pwd)
{
    EEPROM.writeString(EEPROM_BaseAddress + EE_OFFSET_WIFI_SSID, wifi_ssid);
    EEPROM.writeString(EEPROM_BaseAddress + EE_OFFSET_WIFI_PWD, wifi_pwd);
    EEPROM.writeString(EEPROM_BaseAddress + EE_OFFSET_MQTT_ADDRESS, mqtt_add);
    EEPROM.writeUShort(EEPROM_BaseAddress + EE_OFFSET_MQTT_PORT, mqtt_port);
    EEPROM.writeString(EEPROM_BaseAddress + EE_OFFSET_MQTT_USER, mqtt_usr);
    EEPROM.writeString(EEPROM_BaseAddress + EE_OFFSET_MQTT_PWD, mqtt_pwd);

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

void CredentialsManager::init(const char* SSID, const char* hostname)
{
    serverHostname = hostname;
    apSSID         = SSID;
}

void CredentialsManager::loadParameters(uint16_t EE_BaseAddr, bool forcedShowAP)
{
    EEPROM_BaseAddress = EE_BaseAddr;

    // Check if EEPROM content is valid, (initial power-on)
    if (!isEEPROMContentValid())
    {
        initEEPROMArea();
        Serial.println("Initializing EEPROM credentials area...");
    }
    else
    {
        loadEEPROMData();
        Serial.println("MagicKey found, load credentials");
    }

    // Check if wifi can be connected, else automatically setup ap mode
    if (forcedShowAP || !isWifiReacheable())
    {
        Serial.println("Starting WIFI AP server mode to enter credentials");
        runAPServer();

        // Run server and wait for user to enter credentials
        waitReboot();
    }
    else
    {
        Serial.println("Wifi connected, => start directly without showing ap webserver");
    }
}

bool CredentialsManager::isWifiReacheable()
{
    if (!credentials.Wifi_SSID[0] == 0)
    {
        // Try Connect to wifi
        Serial.println("Trying conenction to  " + String(credentials.Wifi_SSID));
        WiFi.begin(credentials.Wifi_SSID, credentials.Wifi_Pwd);

        bool connected = false;
        for (uint32_t retry = 0; (retry < 30) && !connected; retry++)
        {
            Serial.print(".");
            delay(500); // Waiting between tries
            connected = (WiFi.status() == WL_CONNECTED);
        }

        return connected;
    }
    else
    {
        return false;
    }
}