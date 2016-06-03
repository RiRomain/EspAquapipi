#include "libraries/TickerScheduler/TickerScheduler.h"
#include "libraries/LightController/ChannelManager.h"

//Comment following line to use the real RTC_DS3231 RTC and uncomment to use compiled time.
#define VIRTUAL_RTC

//Comment following line to deactivate Debug serial output, uncomment to activate.
#define DEBUG

#include <Wire.h>
#include <RTClib.h>

#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

//OLED Screen configuration
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 0  // GPIO0
//OLED Display use D1 (SCL) and D2 (SDA)
Adafruit_SSD1306 display(OLED_RESET);

const int LED_VALUE_REFRESH_RATE = 5000; //led will be rereshed every x mS
const int RELAY_PIN = D8; //TODO Should be D1, but D1 already used by the OLED screen. See what to do here.

#ifdef DEBUG
  #define DEBUG_PRINT(x)      Serial.print (x)
  #define DEBUG_PRINTLN(x)    Serial.println (x)
  #define DEBUG_PRINTDEC(x)   Serial.print (x, DEC)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTDEC(x)
#endif

void setupWifiConfigAP() {
  WiFiManager wifiManager;
  //first parameter is name of access point, second optional is the password
  wifiManager.autoConnect("EspAquapipi");
  #ifdef DEBUG
    wifiManager.setDebugOutput(true);
  #else
    wifiManager.setDebugOutput(false);
  #endif
}

#ifdef VIRTUAL_RTC
RTC_Millis Rtc;
void setupRTC() {
  Rtc.begin(DateTime(__DATE__, __TIME__));
  DEBUG_PRINTLN("Started virtual RTC with last compilation time");
}

#else
RTC_DS3231 Rtc;
void setupRTC() {
  Wire.begin();
  Rtc.begin(); 

  DateTime compiled = DateTime(__DATE__, __TIME__);
  if (Rtc.lostPower()) 
  {
    //set the compilation time as actual time
    //TODO add a call to a NTP server to synch the time from internet instead of compiled time
    DEBUG_PRINTLN("RTC lost power and lost confidence in the DateTime!");
    Rtc.adjust(compiled);
  }
  DateTime now = Rtc.now();
  if (now.secondstime() < compiled.secondstime()) 
  {
    DEBUG_PRINTLN("RTC is older than compile time!  (Updating DateTime)");
    Rtc.adjust(compiled);
  }
}
#endif

const int MaxChannels = 2;   // Max number of channels, change if more or less are required
const int MaxPoints = 10;    // Max number of light intensity points, change if more or less are required
Channel Channels[MaxChannels];
Point Points[MaxChannels][MaxPoints];

void InitializeChannels(int channels) {
  // (Lights on 08:30 - 19:00 w. 30 minutes of sunrise / sunset added)
    
  // Channel 0:
  int channelNo = 0;  // Currently editing channel 0
  int pin = D3;   // Channel 0 uses pin 10
  pinMode(pin, OUTPUT);   // sets the pin as output
  Channels[channelNo] = Channel(pin, MaxPoints, fademode_linear, Points[channelNo]);  // Initialize channel and choose FadeMode
  Channels[channelNo].AddPoint(8, 0, 0);  // Add Point (can also use decimal values ranging from 0 to 1 if you prefer)
  Channels[channelNo].AddPoint(9, 0, 255);
  Channels[channelNo].AddPoint(19, 30, 255);
  Channels[channelNo].AddPoint(20, 0, 0);
  
  // Channel 1:
  channelNo = 1;  // Currently editing channel 1
  pin = D4;   // Channel 1 uses pin BUILTIN_LED (D4, GPIO2);
  pinMode(pin, OUTPUT);   // sets the pin as output
  Channels[channelNo] = Channel(pin, MaxPoints, fademode_linear, Points[channelNo]);
  Channels[channelNo].AddPoint(8, 0, 0);  // Add Point (can also use decimal values ranging from 0 to 1 if you prefer)
  Channels[channelNo].AddPoint(9, 0, 255);
  Channels[channelNo].AddPoint(14, 30, 255);
  Channels[channelNo].AddPoint(14, 35, 0);
  Channels[channelNo].AddPoint(15, 30, 255);
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

boolean tick = false;
DateTime CurrentTime;
void refreshLedValue() {
  CurrentTime = Rtc.now();
  long now = Seconds(CurrentTime.hour(), CurrentTime.minute(), CurrentTime.second());  // Convert current time to seconds since midnight
  //function execute on time every LED_VALUE_REFRESH_RATE thanks to the scheduler
  display.clearDisplay();
  display.setTextSize(0);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  boolean relay = false;
  for(int channel = 0; channel < MaxChannels; channel++)        // For each Channel
  {
    int intensity = Channels[channel].GetLightIntensityInt(now);
    int pin = Channels[channel].GetPin();
    DEBUG_PRINT("Settings led channel ");
    DEBUG_PRINTDEC(channel);
    DEBUG_PRINT(" on pin ");
    DEBUG_PRINTDEC(pin);
    DEBUG_PRINT(" to value ");
    DEBUG_PRINTDEC(intensity);
    DEBUG_PRINTLN(".");
    display.print("Pin "); display.print(pin); display.print(": "); display.println(intensity);
    analogWrite(pin, intensity); // Get updated light intensity and write value to pin (update is performed when reading value)
    if (0 != intensity) {
      relay = true;
    }
  }
  if (true == relay) {
    display.println("Relay: ON");
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    display.println("Relay: OFF");
    digitalWrite(RELAY_PIN, LOW);
  }
  if (true == tick) {
    tick = false;
    display.print(".");
  } else {
    tick = true;
  }
  display.display();
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("");
}

TickerScheduler ts(1);
void setupScheduler() {
    DEBUG_PRINTLN("Scheduler setup begin");
    ts.add(0, LED_VALUE_REFRESH_RATE, refreshLedValue, false);
    DEBUG_PRINTLN("Scheduler setup finished");
}

void setupOled() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  display.clearDisplay();
   // text display tests
  display.setTextSize(0);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Starting!");
  display.display();
  DEBUG_PRINTLN("Finished oled setup");
}

void setupRelay() {
  pinMode(RELAY_PIN ,OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
}

void setup () 
{
  Serial.begin(115200);
  DEBUG_PRINTLN("Begin setup");
 // setupWifiConfigAP();
  setupRelay();
  setupLedManager();
  setupRTC();
  setupOled();
  setupScheduler();
  DEBUG_PRINTLN("Finished setup");
}


void loop () 
{
  ts.update();
}

