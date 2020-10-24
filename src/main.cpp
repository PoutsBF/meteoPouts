#include <Arduino.h>

#include "RainSensor.h"
#include "WindSensor.h"
#include "sslCertificate.h"
//#include <Adafruit_BME280.h>
#include <Preferences.h>
#include <SDS011.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

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

//network settings
String APSSID = "ESP32";
String ssid;
String pass;

//thingspeak api and fields numbers
//https://thingspeak.com/channels/535447/private_show
bool thingspeakEnabled;
String thingspeakApi;
int tsfWindSpeed = 0;
int tsfWindDir = 0;
int tsfRainAmount = 0;
int tsfTemperature = 0;
int tsfHumidity = 0;
int tsfAirpressure = 0;
int tsfPM25 = 0;
int tsfPM10 = 0;

//senseBox IDs
bool senseBoxEnabled;
String senseBoxStationId;
String senseBoxTempId;
String senseBoxHumId;
String senseBoxPressId;
String senseBoxWindSId;
String senseBoxWindDId;
String senseBoxRainId;
String senseBoxPM25Id;
String senseBoxPM10Id;

unsigned long uploadInterval = hourMs;
bool uploaded = false;

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

WindSensor ws(windSpeedPin, windDirPin);
RainSensor rs(rainPin);
Adafruit_BME280 bme;
SDS011 sds(Serial1);

//webserver, client and secure client pointers
WebServer *server = NULL;
WiFiClient *client = NULL;
WiFiClientSecure *clientS = NULL;

Preferences pref;

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
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,        // temperature
                    Adafruit_BME280::SAMPLING_X1,        // pressure
                    Adafruit_BME280::SAMPLING_X1,        // humidity
                    Adafruit_BME280::FILTER_OFF);

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

        if (thingspeakEnabled)
        {
            if (uploadToThingspeak())
                Serial.println("Uploaded successfully");
            else
                Serial.println("Uploading failed");
        }
        else
            Serial.println("Thingspeak disabled");

        if (senseBoxEnabled)
        {
            if (uploadToSenseBox())
                Serial.println("Uploaded successfully");
            else
                Serial.println("Uploading failed");
        }
        else
            Serial.println("SenseBox disabled");
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
    bme.takeForcedMeasurement();
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0;
}
