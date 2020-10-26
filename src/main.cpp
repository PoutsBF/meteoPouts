#include <Arduino.h>

#include "RainSensor.h"
#include "WindSensor.h"
//#include "sslCertificate.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Preferences.h>
#include <SDS011.h>

#include <freertos/FreeRTOSConfig.h>

#include <FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <freertos/timers.h>

//#include <ESPAsyncWebServer.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include "sslCertificate.h"
#include <WebServer.h>

#include <FS.h>
#include <SPIFFS.h>

#include <Wire.h>

#include "WifiConfig.h"

#define solarRelay 18
#define measBatt 34

#define APPin 22
#define APLed 19
#define STALed 23
#define windDirPin 32
#define windSpeedPin 33
#define rainPin 27

#define battLow 11
#define battFull 13.5
#define battInterval 2000

#define bmeAddress 0x76
#define bmeInterval 5000        //ms = 5 seconds

#define sdsInterval 60000        //ms = 1 minute

#define lastConnectedTimeout 600000        //ms = 10 minutes: 10 * 60 * 1000

#define hourMs 3600000        //ms (60 * 60 * 1000 ms)

unsigned long uploadInterval = hourMs;

bool prevWindPinVal = false;
bool prevRainPinVal = false;

//current sensor values
int windDir = 0;            //degrees
float windSpeed = 0;        //m/s
int beaufort = 0;
String beaufortDesc = "";
float windSpeedAvg = 0;
float windDirAvg = 0;
float rainAmountAvg = 0;

float temperature = 0;        //*C
float humidity = 0;           //%
float pressure = 0;           //hPa
bool bmeRead = 0;

float PM10 = 0;        //particle size: 10 um or less
float PM25 = 0;        //particle size: 2.5 um or less

float batteryVoltage = 0;        //v
float batteryCharging = false;

//serial variables
String serialIn;
bool serialRdy = false;

unsigned long lastBMETime = 0;
unsigned long lastSDSTime = 0;
unsigned long lastUploadTime = 0;
unsigned long lastAPConnection = 0;
unsigned long lastBattMeasurement = 0;

//const char *ssid = YOUR_WIFI_SSID;
const char *password = YOUR_WIFI_PASSWD;
const char *hostName = "esp-async";
const char *http_username = "admin";
const char *http_password = "admin";

//network settings
String APSSID = "ESP32";
String ssid;
String pass;

//AsyncWebServer server(80);

WindSensor ws(windSpeedPin, windDirPin);
RainSensor rs(rainPin);
Adafruit_BME680 bme;        // I2C
SDS011 sds(Serial1);

//webserver, client and secure client pointers
WebServer *server = NULL;
WiFiClient *client = NULL;
WiFiClientSecure *clientS = NULL;

Preferences pref;

//reads the windsensor and stores the values in global variables
void readWindSensor();
void readRainSensor();
void readBME();
void handleSerial();
void checkSerial();
/*
 * Note: preference names can't be longer than 15 characters
 */
void loadNetworkCredentials();
void storeNetworkCredentials();
void loadUploadSettings();
void storeUploadSettings();

//-----------------------------------------------------------------------------
//          gestion WIFI
//-----------------------------------------------------------------------------
String SSIDList(String separator);
//String SSIDList(String separator = ",");
//send a list of available networks to the client connected to the webserver
void getSSIDList();
//store the wifi settings configured on the webpage and restart the esp to connect to this network
void setWiFiSettings();
//send the wifi settings to the connected client of the webserver
void getWiFiSettings();
//store the upload settings configured on the webpage
void setUploadSettings();
void getUploadSettings();
//send the weather data to the connected client of the webserver
void getWeatherData();
//send the battery data to the connected client of the webserver
void getBatteryData();
//toggle the charging of the battery
void toggleCharging();
//restart the esp as requested on the webpage
void restart();
//get the content type of a filename
String getContentType(String filename);
//send a file from the SPIFFS to the connected client of the webserver
void sendFile();
//send data to the connected client of the webserver
void sendData(String data);
//initialize wifi by connecting to a wifi network or creating an accesspoint
void initWiFi();
//connect the esp32 to a wifi network
bool connectWiFi();
//configure the access point of the esp32
void configureSoftAP();
//initialize the webserver on port 80
void configureServer();
void configureClient();
//request the specified url of the specified host
String performRequest(bool secure, String host, String url, int port, 
                        String method, 
                        String headers, 
                        String data);
String WiFiStatusToString();

//-----------------------------------------------------------------------------
//      SETUP -----------------------------------------------------------------
//-----------------------------------------------------------------------------
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial1.begin(9600);

    SPIFFS.begin();
    Serial.println("SPIFFS started");
    pinMode(APPin, INPUT_PULLUP);
    pinMode(APLed, OUTPUT);
    pinMode(STALed, OUTPUT);
    pinMode(solarRelay, OUTPUT);
    pinMode(measBatt, ANALOG);

    digitalWrite(APLed, LOW);
    digitalWrite(STALed, LOW);
    digitalWrite(solarRelay, LOW);

    ws.initWindSensor();
    rs.initRainSensor();

    Wire.begin(25, 26, 100000);        //sda, scl, freq=100kHz
    bme.begin(bmeAddress);
    //recommended settings for weather monitoring
    //    configuration du capteur (sur échantillonage, filtres)
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);        // 320*C for 150 ms

    sds.setMode(SDS_SET_QUERY);

    loadNetworkCredentials();
    loadUploadSettings();
    initWiFi();
}

void loop()
{
    //serial debugging
    checkSerial();

    //handle WiFi
    if (server != NULL)
        server->handleClient();

    //if the current wifi mode isn't STA and the timout time has passed -> restart
    if ((WiFi.getMode() != WIFI_STA) && ((lastAPConnection + lastConnectedTimeout) < millis()))
    {
        if (digitalRead(APPin))
        {
            Serial.println("Last connection was more than 10 minutes ago. Restart.");
            ESP.restart();
        }
        else
        {                                       //button still pressed
            lastAPConnection = millis();        //wait some time before checking again
        }
    }

    //reinit if the wifi connection is lost
    if ((WiFi.getMode() == WIFI_STA) && (WiFi.status() == WL_DISCONNECTED))
    {
        digitalWrite(STALed, LOW);
        Serial.println("Lost connection. Trying to reconnect.");
        initWiFi();
    }

    //read sensors
    readWindSensor();
    readRainSensor();

    //read bme280 every 5 seconds
    if ((lastBMETime + bmeInterval) < millis())
    {
        lastBMETime = millis();
        readBME();
    }

    //read SDS011 every minute
    if ((lastSDSTime + sdsInterval) < millis())
    {
        lastSDSTime = millis();
        if (!sds.getData(&PM25, &PM10))
            Serial.println("Failed to read SDS011");
    }

    //upload data if the uploadperiod has passed and if WiFi is connected
    if (((lastUploadTime + uploadInterval) < millis()) && (WiFi.status() == WL_CONNECTED))
    {
        lastUploadTime = millis();
        Serial.println("Upload interval time passed");

        windSpeedAvg = ws.getWindSpeedAvg();
        windDirAvg = ws.getWindDirAvg();
        rainAmountAvg = rs.getRainAmount() * hourMs / uploadInterval;
    }

    //handle battery (resistor divider: vBatt|--[470k]-+-[100k]--|gnd)
    if ((lastBattMeasurement + battInterval) < millis())
    {
        lastBattMeasurement = millis();
        float adcVoltage = ((float)analogRead(measBatt) / 4096) * 3.3 + 0.15;        //0.15 offset from real value
        batteryVoltage = adcVoltage * 570 / 100 + 0.7;                               //analog read between 0 and 3.3v * resistor divider + 0.7v diode drop
        Serial.println("adc voltage: " + String(adcVoltage) + ", batt voltage: " + String(batteryVoltage) + ", currently charging: " + String(batteryCharging ? "yes" : "no"));
        if (batteryVoltage > battFull)
            batteryCharging = false;
        if (batteryVoltage < battLow)
            batteryCharging = true;

        digitalWrite(solarRelay, batteryCharging);
    }
}

//reads the windsensor and stores the values in global variables
void readWindSensor()
{
    if (digitalRead(windSpeedPin) && !prevWindPinVal)
    {
        ws.calcWindSpeed();
    }
    prevWindPinVal = digitalRead(windSpeedPin);

    ws.updateWindSensor();
    windSpeed = ws.getWindSpeed();
    beaufort = ws.getBeaufort();
    beaufortDesc = ws.getBeaufortDesc();

    ws.determineWindDir();
    windDir = ws.getWindDir();
}

void readRainSensor()
{
    //inverted logic
    if (!digitalRead(rainPin) && prevRainPinVal)
    {
        Serial.println("Rainbucket tipped");
        rs.calcRainAmount();
    }
    prevRainPinVal = digitalRead(rainPin);
}

void readBME()
{
//    bme.takeForcedMeasurement();
    bme.beginReading();
    if (bme.endReading())
    {
        temperature = bme.readTemperature();
        humidity = bme.readHumidity();
        pressure = bme.readPressure() / 100.0;
    }
}

void checkSerial()
{
    while (Serial.available())
    {
        char in = Serial.read();
        serialIn += in;
        if (in == '\n')
            serialRdy = true;
    }
    handleSerial();
}

void handleSerial()
{
    if (serialRdy)
    {
        Serial.println(serialIn);
        if (serialIn.indexOf("wifiStatus") != -1)
        {
            Serial.println("WiFi status: " + WiFiStatusToString());
        }
        else if (serialIn.indexOf("printData") != -1)
        {
            Serial.println("Wind speed:         " + String(ws.getWindSpeed()) + "m/s, " + String(ws.getWindSpeed() * 3.6) + "km/h");
            Serial.println("Beaufort:           " + String(ws.getBeaufort()) + " (" + ws.getBeaufortDesc() + ")");
            Serial.println("Wind speed avg:     " + String(ws.getWindSpeedAvg(false)));
            Serial.println("Wind direction:     " + ws.getWindDirString() + " (" + String(ws.getWindDir()) + ")");
            Serial.println("Wind direction avg: " + String(ws.getWindDirAvg(false)));
            Serial.println("Rain amount:        " + String(rs.getRainAmount(false)) + "mm");
            Serial.println("Temperature:        " + String(temperature) + "*C");
            Serial.println("Humidity:           " + String(humidity) + "%");
            Serial.println("Pressure:           " + String(pressure) + "hPa");
            Serial.println("Dust 10um:          " + String(PM10) + "ug/m3");
            Serial.println("Dust 2.5um:         " + String(PM25) + "ug/m3");
        }
        else if (serialIn.indexOf("WifiSettings") != -1)
        {
            if (serialIn.indexOf("set") != -1)
            {
                ssid = serialIn.substring(serialIn.indexOf(":") + 1, serialIn.indexOf(","));
                pass = serialIn.substring(serialIn.indexOf(",") + 1, serialIn.indexOf("\n"));
                storeNetworkCredentials();
            }
            Serial.println("WiFi settings: SSID: " + ssid + ", pass: " + pass);
        }
        else if (serialIn.indexOf("restart") != -1)
        {
            ESP.restart();
        }
        else if (serialIn.indexOf("timeActive") != -1)
        {
            unsigned long time = millis();
            Serial.println("Time active: " + String(time) + "ms, " + String(time / 1000 / 60) + " minutes");
        }

        serialRdy = false;
        serialIn = "";
    }
}

/*
 * Note: preference names can't be longer than 15 characters
 */
void loadNetworkCredentials()
{
    pref.begin("weather", false);
    ssid = pref.getString("ssid");
    pass = pref.getString("pass");
    pref.end();
}

void storeNetworkCredentials()
{
    pref.begin("weather", false);
    pref.putString("ssid", ssid);
    pref.putString("pass", pass);
    pref.end();
}

void loadUploadSettings()
{
    pref.begin("weather", false);

    uploadInterval = ((pref.getUInt("uploadInterval") != 0) ? pref.getUInt("uploadInterval") : hourMs);
    pref.end();
}

void storeUploadSettings()
{
    pref.begin("weather", false);

    pref.putUInt("uploadInterval", uploadInterval);
    pref.end();
}

//-----------------------------------------------------------------------------
//          gestion WIFI
//-----------------------------------------------------------------------------
String SSIDList(String separator = ",")
{
    Serial.println("Scanning networks");
    String ssidList;
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++)
    {
        String ssid = WiFi.SSID(i);
        Serial.println(String(i) + ": " + ssid);
        if (ssidList.indexOf(ssid) != -1)
        {
            Serial.println("SSID already in list");
        }
        else
        {
            if (ssidList != "")
                ssidList += separator;
            ssidList += ssid;
        }
    }
    return ssidList;
}

//send a list of available networks to the client connected to the webserver
void getSSIDList()
{
    Serial.println("SSID list requested");
    sendData(SSIDList());
}

//store the wifi settings configured on the webpage and restart the esp to connect to this network
void setWiFiSettings()
{
    Serial.println("WiFi settings received");
    Serial.println(server->uri());
    ssid = server->arg("ssid");
    pass = server->arg("pass");
    String response = "Attempting to connect to '" + ssid + "'. The WiFi module restarts and tries to connect to the network.";
    sendData(response);
    Serial.println("Saving network credentials and restart.");
    storeNetworkCredentials();
    delay(1000);
    ESP.restart();
}

//send the wifi settings to the connected client of the webserver
void getWiFiSettings()
{
    Serial.println("WiFi settings requested");
    String response;
    response += ssid + ",";
    response += SSIDList(";");
    sendData(response);
}

//store the upload settings configured on the webpage
void setUploadSettings()
{
    Serial.println("Upload settings received");
    Serial.println(server->uri());
    long recvInterval = server->arg("interval").toInt();
    uploadInterval = ((recvInterval <= 0) ? hourMs : (recvInterval * 60 * 1000));        //convert to ms (default is 1 hr)
    storeUploadSettings();
    sendData("Upload settings stored");
}

void getUploadSettings()
{
    Serial.println("Upload settings requested");
    String response;
    response += String(uploadInterval / 1000 / 60);        //convert to minutes
    sendData(response);
}

//send the weather data to the connected client of the webserver
void getWeatherData()
{
    Serial.println("Weather data requested");
    String response;
    response += String(windSpeed * 3.6) + ",";        //km/h
    response += String(beaufort) + " (" + beaufortDesc + "),";
    response += String(windDir) + ",";
    response += String(temperature) + ",";
    response += String(humidity) + ",";
    response += String(pressure) + ",";
    response += String(PM25) + ",";
    response += String(PM10);
    sendData(response);
}

//send the battery data to the connected client of the webserver
void getBatteryData()
{
    Serial.println("Battery data requested");
    String response;
    response += String(batteryVoltage) + ",";
    response += String(batteryCharging ? "1" : "0");
    sendData(response);
}

//toggle the charging of the battery
void toggleCharging()
{
    Serial.println("Toggle charging requested");
    batteryCharging = !batteryCharging;
    sendData("Battery charging " + String(batteryCharging ? "started" : "stopped"));
}

//restart the esp as requested on the webpage
void restart()
{
    sendData("The ESP32 will restart and you will be disconnected from the '" + APSSID + "' network.");
    delay(1000);
    ESP.restart();
}

//get the content type of a filename
String getContentType(String filename)
{
    if (server->hasArg("download"))
        return "application/octet-stream";
    else if (filename.endsWith(".htm"))
        return "text/html";
    else if (filename.endsWith(".html"))
        return "text/html";
    else if (filename.endsWith(".css"))
        return "text/css";
    else if (filename.endsWith(".js"))
        return "application/javascript";
    else if (filename.endsWith(".png"))
        return "image/png";
    else if (filename.endsWith(".gif"))
        return "image/gif";
    else if (filename.endsWith(".jpg"))
        return "image/jpeg";
    else if (filename.endsWith(".ico"))
        return "image/x-icon";
    else if (filename.endsWith(".xml"))
        return "text/xml";
    else if (filename.endsWith(".pdf"))
        return "application/x-pdf";
    else if (filename.endsWith(".zip"))
        return "application/x-zip";
    else if (filename.endsWith(".gz"))
        return "application/x-gzip";
    return "text/plain";
}

//send a file from the SPIFFS to the connected client of the webserver
void sendFile()
{
    String path = server->uri();
    Serial.println("Got request for: " + path);
    if (path.endsWith("/")) path += "index.html";
    String contentType = getContentType(path);
    if (SPIFFS.exists(path))
    {
        Serial.println("File " + path + " found");
        File file = SPIFFS.open(path, "r");
        server->streamFile(file, contentType);
        file.close();
    }
    else
    {
        Serial.println("File '" + path + "' doesn't exist");
        server->send(404, "text/plain", "The requested file doesn't exist");
    }

    lastAPConnection = millis();
}

//send data to the connected client of the webserver
void sendData(String data)
{
    Serial.println("Sending: " + data);
    server->send(200, "text/plain", data);

    lastAPConnection = millis();
}

//initialize wifi by connecting to a wifi network or creating an accesspoint
void initWiFi()
{
    digitalWrite(APLed, LOW);
    digitalWrite(STALed, LOW);
    Serial.print("WiFi: ");
    if (!digitalRead(APPin))
    {
        Serial.println("AP");
        configureSoftAP();
    }
    else
    {
        Serial.println("STA");
        if (!connectWiFi())
        {
            Serial.println("Connecting failed. Starting AP");
            configureSoftAP();
        }
        else
        {
            configureClient();
            digitalWrite(STALed, HIGH);
        }
    }
}

//connect the esp32 to a wifi network
bool connectWiFi()
{
    if (ssid == "")
    {
        Serial.println("SSID unknown");
        return false;
    }
    WiFi.mode(WIFI_STA);
    Serial.println("Attempting to connect to " + ssid + ", pass: " + pass);
    WiFi.begin(ssid.c_str(), pass.c_str());
    for (int timeout = 0; timeout < 15; timeout++)
    {        //max 15 seconds
        int status = WiFi.status();
        if ((status == WL_CONNECTED) || (status == WL_CONNECT_FAILED) || (status == WL_NO_SSID_AVAIL))
            break;
        Serial.print(".");
        delay(1000);
    }
    Serial.println();
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Failed to connect to " + ssid);
        Serial.println("WiFi status: " + WiFiStatusToString());
        WiFi.disconnect();
        return false;
    }
    Serial.println("Connected to " + ssid);
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
    return true;
}

//configure the access point of the esp32
void configureSoftAP()
{
    Serial.println("Configuring AP: " + String(APSSID));
    WiFi.mode(WIFI_AP);
    WiFi.softAP(APSSID.c_str(), NULL, 1, 0, 1);
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP IP: ");
    Serial.println(ip);
    lastAPConnection = millis();
    digitalWrite(APLed, HIGH);

    configureServer();
}

//initialize the webserver on port 80
void configureServer()
{
    server = new WebServer(80);
    server->on("/getWeatherData", HTTP_GET, getWeatherData);
    server->on("/getBatteryData", HTTP_GET, getBatteryData);
    server->on("/toggleCharging", HTTP_GET, toggleCharging);
    server->on("/setWiFiSettings", HTTP_GET, setWiFiSettings);
    server->on("/getWiFiSettings", HTTP_GET, getWiFiSettings);
    server->on("/setUploadSettings", HTTP_GET, setUploadSettings);
    server->on("/getUploadSettings", HTTP_GET, getUploadSettings);
    server->on("/getSSIDList", HTTP_GET, getSSIDList);
    server->on("/restart", HTTP_GET, restart);
    server->onNotFound(sendFile);        //handle everything except the above things
    server->begin();
    Serial.println("Webserver started");
}

void configureClient()
{
    client = new WiFiClient();
    clientS = new WiFiClientSecure();
    clientS->setCACert(certificate);
}

//request the specified url of the specified host
String performRequest(bool secure, 
                        String host, 
                        String url, 
                        int port = 80, 
                        String method = "GET", 
                        String headers = "Connection: close\r\n", 
                        String data = "")
{
    WiFiClient *c = (secure ? clientS : client);
    Serial.println("Connecting to host '" + host + "' on port " + String(port));
    c->connect(host.c_str(), port);        //default ports: http: port 80, https: 443
    String request = method + " " + url + " HTTP/1.1\r\n" +
                     "Host: " + host + "\r\n" +
                     headers + "\r\n";
    Serial.println("Requesting url: " + request);
    c->print(request);
    if (data != "")
    {
        Serial.println("Data: " + data);
        c->print(data + "\r\n");
    }

    unsigned long timeout = millis();
    while (c->available() == 0)
    {
        if (timeout + 5000 < millis())
        {
            Serial.println("Client timeout");
            c->stop();
            return "";
        }
    }
    //read client reply
    String response;
    while (c->available())
    {
        response = c->readStringUntil('\r');
    }
    Serial.println("Response: " + response);
    c->stop();
    return response;
}

String WiFiStatusToString()
{
    switch (WiFi.status())
    {
        case WL_IDLE_STATUS: return "IDLE"; break;
        case WL_NO_SSID_AVAIL: return "NO SSID AVAIL"; break;
        case WL_SCAN_COMPLETED: return "SCAN COMPLETED"; break;
        case WL_CONNECTED: return "CONNECTED"; break;
        case WL_CONNECT_FAILED: return "CONNECT_FAILED"; break;
        case WL_CONNECTION_LOST: return "CONNECTION LOST"; break;
        case WL_DISCONNECTED: return "DISCONNECTED"; break;
        case WL_NO_SHIELD: return "NO SHIELD"; break;
        default: return "Undefined: " + String(WiFi.status()); break;
    }
}
