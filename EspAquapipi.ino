#include "libraries/TickerScheduler/TickerScheduler.h"
#include "libraries/LightController/ChannelManager.h"

#include <Wire.h>
#include <RTClib.h>

#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

const int LED_VALUE_REFRESH_RATE = 1000; //led will be rereshed every x mS


void setupWifiConfigAP() {
  WiFiManager wifiManager;
  //first parameter is name of access point, second optional is the password
  wifiManager.autoConnect("EspAquapipi");
  wifiManager.setDebugOutput(true);
}

RTC_DS3231 Rtc;
void setupRTC() {
  Wire.begin();
  Rtc.begin(); 

  DateTime compiled = DateTime(__DATE__, __TIME__);
  if (Rtc.lostPower()) 
  {
    //set the compilation time as actual time
    //TODO add a call to a NTP server to synch the time from internet instead of compiled time
    Serial.println("RTC lost power and lost confidence in the DateTime!");
    Rtc.adjust(compiled);
  }
  DateTime now = Rtc.now();
  if (now.secondstime() < compiled.secondstime()) 
  {
    Serial.println("RTC is older than compile time!  (Updating DateTime)");
    Rtc.adjust(compiled);
  }
}

const int MaxChannels = 2;   // Max number of channels, change if more or less are required
const int MaxPoints = 10;    // Max number of light intensity points, change if more or less are required
Channel Channels[MaxChannels];
Point Points[MaxChannels][MaxPoints];

void InitializeChannels(int channels) {
  // (Lights on 08:30 - 19:00 w. 30 minutes of sunrise / sunset added)
    
  // Channel 0:
  int channelNo = 0;  // Currently editing channel 0
  int pin = 10;   // Channel 0 uses pin 10
  Channels[channelNo] = Channel(pin, MaxPoints, fademode_linear, Points[channelNo]);  // Initialize channel and choose FadeMode
  Channels[channelNo].AddPoint(8, 0, 0);  // Add Point (can also use decimal values ranging from 0 to 1 if you prefer)
  Channels[channelNo].AddPoint(9, 0, 255);
  Channels[channelNo].AddPoint(19, 30, 255);
  Channels[channelNo].AddPoint(20, 0, 0);
  
  // Channel 1:
  channelNo = 1;  // Currently editing channel 1
  pin = 11;   // Channel 1 uses pin 11
  Channels[channelNo] = Channel(pin, MaxPoints, fademode_linear, Points[channelNo]);
  Channels[channelNo].AddPoint(8, 0, 0);  // Add Point (can also use decimal values ranging from 0 to 1 if you prefer)
  Channels[channelNo].AddPoint(9, 0, 255);
  Channels[channelNo].AddPoint(19, 30, 255);
  Channels[channelNo].AddPoint(20, 0, 0);
}

void setupLedManager() {
  InitializeChannels(MaxChannels);
}

// Convert HH:mm:ss -> Seconds since midnight
long Seconds(int hours, int minutes, int seconds) {
  return ((long)hours * 60 * 60) + (minutes * 60) + seconds ;
}

DateTime CurrentTime;
void refreshLedValue() {
  CurrentTime = Rtc.now();
  long now = Seconds(CurrentTime.hour(), CurrentTime.minute(), CurrentTime.second());  // Convert current time to seconds since midnight
  //function execute on time every LED_VALUE_REFRESH_RATE thanks to the scheduler
  for(int channel = 0; channel < MaxChannels; channel++)        // For each Channel
  {
    analogWrite(Channels[channel].GetPin(), Channels[channel].GetLightIntensityInt(now)); // Get updated light intensity and write value to pin (update is performed when reading value)
  }
}

TickerScheduler ts(5);
void setupScheduler() {
    ts.add(0, LED_VALUE_REFRESH_RATE, refreshLedValue, false);
}

void setup () 
{
  Serial.begin(115200);
  setupWifiConfigAP();
  setupLedManager();
  setupRTC();
  setupScheduler();
}


void loop () 
{
  ts.update();
}

