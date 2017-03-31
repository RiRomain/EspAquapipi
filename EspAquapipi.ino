#include "libraries/TickerScheduler/TickerScheduler.h"
#include "libraries/LightController/ChannelManager.h"
#include <OneWire.h>
// http://pubsubclient.knolleary.net/
#include <PubSubClient.h>
#include <ArduinoJson.h>

//Comment following line to deactivate Debug serial output, uncomment to activate.
#define DEBUG
//Comment following line to use the real RTC_DS3231 RTC and uncomment to use compiled time.
//#define VIRTUAL_RTC


//PIN MAP
//D0  Not used - IO Only
//D1  12V Relay
//D2  LED 1 and 2 - ETH1
//D3  (Push button)
//D4  DS18B20 (PullUp only!)
//D5  LED 3 and 4 - ETH2
//D6  RTC Clock SDA
//D7  RTC Clock SCL
//D8  LED Blue - ETH1 (Green and Red)

#define led_1_state_topic "fishtank/light/light1"
#define led_2_state_topic "fishtank/light/light2"
#define led_blue_state_topic "fishtank/light/lightblue"
#define led_1_set_topic "fishtank/light/light1/set"
#define led_2_set_topic "fishtank/light/light2/set"
#define led_blue_set_topic "fishtank/light/lightblue/set"
const int BUFFER_SIZE = JSON_OBJECT_SIZE(8);

byte led1 = 255;
byte led2 = 255;
byte ledBlue = 255;

#define SDA_PIN D6
#define SCL_PIN D7

#define LED1 D2
#define LED2 D5
#define LED_BLUE D8
#define RELAY_PIN D1
#define DS18B20_PIN D4

#include <Wire.h>
#include <RTClib.h>

#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

const int LED_VALUE_REFRESH_RATE = 5000;    //led will be rereshed every 5 secondes
const int TEMP_VALUE_REFRESH_RATE = 300000; //temperature will be refreshed every 5 minutes
const int MQTT_REFRESH_RATE = 100;          //refresh MQTT every 100mS

#ifdef DEBUG
#define DEBUG_PRINT(x)      Serial.print (x)
#define DEBUG_PRINTLN(x)    Serial.println (x)
#define DEBUG_PRINTDEC(x)   Serial.print (x, DEC)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTDEC(x)
#endif

#define mqtt_server "192.168.0.112"
#define mqtt_user "homeassistant"
#define mqtt_password "XXXXXX"

WiFiClient espClient;

//adding this to bypass to problem of the arduino builder issue 50
void callback(char*topic, byte* payload, unsigned int length);

PubSubClient client(mqtt_server, 1883, callback, espClient);
//PubSubClient client(mqtt_server, 1883, espClient);

//MQTT last attemps reconnection number
long lastReconnectAttempt = 0;


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
  Wire.begin(SDA_PIN, SCL_PIN);
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

const int MaxChannels = 3;   // Max number of channels, change if more or less are required
const int MaxPoints = 10;    // Max number of light intensity points, change if more or less are required
Channel Channels[MaxChannels];
Point Points[MaxChannels][MaxPoints];

void addPointToChannels(int pin, int channelNo) {
  pinMode(pin, OUTPUT);   // sets the pin as output
  Channels[channelNo] = Channel(pin, MaxPoints, fademode_linear, Points[channelNo]);
  Channels[channelNo].AddPoint(12, 0, 0); // Add Point (can also use decimal values ranging from 0 to 1 if you prefer)
  Channels[channelNo].AddPoint(12, 30, 255);
  Channels[channelNo].AddPoint(23, 0, 255);
  Channels[channelNo].AddPoint(23, 59, 0);
}

void addPointToBlueChannels(int pin, int channelNo) {
  pinMode(pin, OUTPUT);   // sets the pin as output
  Channels[channelNo] = Channel(pin, MaxPoints, fademode_linear, Points[channelNo]);
  Channels[channelNo].AddPoint(12, 0, 0); // Add Point (can also use decimal values ranging from 0 to 1 if you prefer)
  Channels[channelNo].AddPoint(12, 30, 255);
  Channels[channelNo].AddPoint(23, 0, 255);
  Channels[channelNo].AddPoint(23, 55, 0);
}

void initializeChannels() {
  addPointToChannels(LED1, 0);
  addPointToChannels(LED2, 1);
  addPointToBlueChannels(LED_BLUE, 2);
}

void refreshChannelsMaxValue(int intensity1, int intensity2, int intensityBlue) {
  DEBUG_PRINT("Storing new value intensity1 "); DEBUG_PRINTDEC(intensity1);
  DEBUG_PRINT(" intensity2 "); DEBUG_PRINTDEC(intensity2);
  DEBUG_PRINT(" and intensityBlue "); DEBUG_PRINTDEC(intensityBlue); DEBUG_PRINTLN("");
  refreshPointOfChannels(0, intensity1);
  refreshPointOfChannels(1, intensity2);
  refreshPointOfBlueChannels(2, intensityBlue);
}

void refreshPointOfChannels(int channelNo, int intensity) {
  DEBUG_PRINT("Changing intensity "); DEBUG_PRINTDEC(Channels[channelNo].GetPoint(2).GetIntensity());
  DEBUG_PRINT(" to intensity "); DEBUG_PRINTDEC(intensity); DEBUG_PRINTLN("");
  Channels[channelNo].SetPoint(1, 12, 0, 0); // Add Point (can also use decimal values ranging from 0 to 1 if you prefer)
  Channels[channelNo].SetPoint(2, 12, 30, intensity);
  Channels[channelNo].SetPoint(3, 23, 0, intensity);
  Channels[channelNo].SetPoint(4, 23, 59, 0);
  Channels[channelNo].Reset();
}

void refreshPointOfBlueChannels(int channelNo, int intensity) {
  DEBUG_PRINT("Changing intensity "); DEBUG_PRINTDEC(Channels[channelNo].GetPoint(2).GetIntensity());
  DEBUG_PRINT(" to intensity "); DEBUG_PRINTDEC(intensity); DEBUG_PRINTLN("");
  Channels[channelNo].SetPoint(1, 12, 0, 0); // Add Point (can also use decimal values ranging from 0 to 1 if you prefer)
  Channels[channelNo].SetPoint(2, 12, 30, intensity);
  Channels[channelNo].SetPoint(3, 23, 0, intensity);
  Channels[channelNo].SetPoint(4, 23, 55, 0);
  Channels[channelNo].Reset();
}

void setupLedManager() {
  analogWriteRange(255);
  initializeChannels();
}

// Convert HH:mm:ss -> Seconds since midnight
long Seconds(int hours, int minutes, int seconds) {
  long actualSecond = ((long)hours * 60 * 60) + (minutes * 60) + seconds;
  return actualSecond;
}

void digitalClockDisplay(DateTime dateTime) {
  // digital clock display of current time
#ifdef DEBUG
  Serial.print(dateTime.year(), DEC);
  Serial.print('/');
  Serial.print(dateTime.month(), DEC);
  Serial.print('/');
  Serial.print(dateTime.day(), DEC);
  Serial.print(' ');
  Serial.print(dateTime.hour(), DEC);
  Serial.print(':');
  Serial.print(dateTime.minute(), DEC);
  Serial.print(':');
  Serial.print(dateTime.second(), DEC);
  Serial.println();
#endif
}


DateTime CurrentTime;
void refreshLedValue() {
  CurrentTime = Rtc.now();
  DEBUG_PRINT("Actual time: ");
  digitalClockDisplay(CurrentTime);
  long now = Seconds(CurrentTime.hour(), CurrentTime.minute(), CurrentTime.second());  // Convert current time to seconds since midnight
  //function execute on time every LED_VALUE_REFRESH_RATE thanks to the scheduler
  boolean relay = false;
  for (int channel = 0; channel < MaxChannels; channel++)       // For each Channel
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
    analogWrite(pin, intensity); // Get updated light intensity and write value to pin (update is performed when reading value)
    sendLigthValue(channel, intensity);
    if (0 != intensity) {
      relay = true;
    }
  }
  if (true == relay) {
    DEBUG_PRINTLN("Relay: ON");
    digitalWrite(RELAY_PIN, HIGH);
    sendRelayValue("ON");
  } else {
    DEBUG_PRINTLN("Relay: OFF");
    digitalWrite(RELAY_PIN, LOW);
    sendRelayValue("OFF");
  }
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("");
}

float tempC = 0;
int tempCasInt = 0;
OneWire ds(DS18B20_PIN);
void refreshTempValue() {
  byte addr[8];
  int actualSensorIndex = 0;
  while (ds.search(addr)) {
    if ( OneWire::crc8( addr, 7) != addr[7]) {
      DEBUG_PRINTLN("CRC is not valid!");
    } else if ( addr[0] != 0x10 && addr[0] != 0x28) {
      DEBUG_PRINTLN("Device is not recognized, ignoring device.");
    } else {
      ds.reset();
      ds.select(addr);
      ds.write(0x44, 1); // start conversion, with parasite power on at the end

      byte present = ds.reset();
      ds.select(addr);
      ds.write(0xBE); // Read Scratchpad

      byte data[12];
      for (int i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = ds.read();
      }

      byte MSB = data[1];
      byte LSB = data[0];

      float tempRead = ((MSB << 8) | LSB); //using two's compliment
      tempC = tempRead / 16;
      tempCasInt = (int) tempC;
      String readTempAsString = String(tempC, DEC);
      if (String("85.0000000000") != readTempAsString) {
        String result = "channel=TEMP" + String(actualSensorIndex, DEC) + "&temp=" + readTempAsString;
        DEBUG_PRINTLN("Sending " + result);
        if (client.connected()) {
          sendTemperatureToMqtt(actualSensorIndex, readTempAsString);
        } else if (reconnect()) {
          DEBUG_PRINTLN("Sending temp value to MQTT after reconnect");
          sendTemperatureToMqtt(actualSensorIndex, readTempAsString);
          lastReconnectAttempt = 0;
        }

      } else {
        DEBUG_PRINTLN("Got error value 85 from sensor " + String(actualSensorIndex, DEC) + " !");
      }

      actualSensorIndex = actualSensorIndex + 1;
    }
  }
  if (actualSensorIndex == 0) {
    DEBUG_PRINTLN("No valid device or error.");
  }
  ds.reset_search();
}


TickerScheduler ts(3);
void setupScheduler() {
  DEBUG_PRINTLN("Scheduler setup begin");
  ts.add(0, LED_VALUE_REFRESH_RATE, refreshLedValue, false);
  ts.add(1, TEMP_VALUE_REFRESH_RATE, refreshTempValue, true);
  ts.add(2, MQTT_REFRESH_RATE, refreshMQTTValueState, true);
  DEBUG_PRINTLN("Scheduler setup finished");
}

void setupRelay() {
  pinMode(RELAY_PIN , OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
}

void setupDS18B20() {
  ds.reset();
}

void setup ()
{
  Serial.begin(115200);
  DEBUG_PRINTLN("Begin setup");
  setupWifiConfigAP();
  setupRelay();
  setupLedManager();
  setupRTC();
  setupDS18B20();
  setupScheduler();
  DEBUG_PRINTLN("Finished setup");
}


void loop ()
{
  ts.update();
}

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

boolean reconnect() {
  // Loop until we're reconnected
  // Generate client name based on MAC address and last 8 bits of microsecond counter
  DEBUG_PRINTLN("Reconnecting MQTT");
  String clientName;
  clientName += "tank";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);
  if (!client.connected()) {
    DEBUG_PRINTLN("Attempting MQTT connection...");
    // Attempt to connect
    // If you  want to use a username and password, uncomment next line and comment the line if (client.connect("433toMQTTto433")) {
    if (client.connect((char*) clientName.c_str(), mqtt_user, mqtt_password)) {
      client.publish("outTopic", "fishtank started or reconnected");
      DEBUG_PRINTLN("connected or reconnected");
      //Subscribing to topic(s)
      client.subscribe(led_1_set_topic);
      client.subscribe(led_2_set_topic);
      client.subscribe(led_blue_set_topic);
    } else {
      DEBUG_PRINT("failed, rc=");
      DEBUG_PRINTLN(String(client.state()));
      DEBUG_PRINTLN("try again in the next loop");
      // The scheduler will only call that in 2 seconds
      // We avoid blocking here to still manage the light in the abscence of MQTT brokers
    }
  }
  return client.connected();
}

bool valueNeedToRefresh = false;

void refreshMQTTValueState() {
  //MQTT client connexion management
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      DEBUG_PRINTLN("client mqtt not connected, trying to connect");
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  // MQTT loop
  client.loop();

  if (valueNeedToRefresh) {
    refreshChannelsMaxValue(led1, led2, ledBlue);
    valueNeedToRefresh = false;
  }
}


#define temperature_topic "fishtank/sensor/temperature"
void sendTemperatureToMqtt(int index, String temperature) {
  String topic = temperature_topic + String(index, DEC);
  char* topicPointer = (char*) topic.c_str();
  client.publish(topicPointer, temperature.c_str(), true);
  DEBUG_PRINT("New value published to topic: ");
  DEBUG_PRINT(topic);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(temperature.c_str());
}

#define light_sensor_topic "fishtank/sensor/light"
void sendLigthValue(int index, int value) {
  String topic = light_sensor_topic + String(index, DEC);
  char* topicPointer = (char*) topic.c_str();
  client.publish(topicPointer, String(value, DEC).c_str(), true);
  DEBUG_PRINT("New value published to topic: ");
  DEBUG_PRINT(topic);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(String(value, DEC).c_str());
}

#define relay_sensor_topic "fishtank/sensor/relay"
void sendRelayValue(String value) {
  client.publish(relay_sensor_topic, value.c_str(), true);
  DEBUG_PRINT("New value published to topic: ");
  DEBUG_PRINT(relay_sensor_topic);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(value.c_str());
}


/*
  SAMPLE PAYLOAD:
    {
      "brightness": 120,
      "flash": 2,
      "transition": 5,
      "state": "ON"
    }
*/


byte tempVal = 255;
void callback(char* topic, byte* payload, unsigned int length) {
  DEBUG_PRINT("Message arrived [");
  DEBUG_PRINT(topic);
  DEBUG_PRINT("] ");
  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  DEBUG_PRINTLN(message);
  if (strcmp(topic, led_1_set_topic) == 0) {
    if (!processJson(message)) {
      return;
    }
    led1 = tempVal;
  }
  if (strcmp(topic, led_2_set_topic) == 0) {
    if (!processJson(message)) {
      return;
    }
    led2 = tempVal;
  }
  if (strcmp(topic, led_blue_set_topic) == 0) {
    if (!processJson(message)) {
      return;
    }
    ledBlue = tempVal;
  }
  valueNeedToRefresh = true;
  sendState();
}


bool processJson(char* message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(message);
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }
  if (root.containsKey("brightness")) {
    tempVal = root["brightness"];
  }
  return true;
}

const char* on_cmd = "ON";
const char* off_cmd = "OFF";

void sendState() {
  sendSingleState(led_1_state_topic, led1);
  sendSingleState(led_2_state_topic, led2);
  sendSingleState(led_blue_state_topic, ledBlue);
}

void sendSingleState(char* topic, byte brightnessIn) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  boolean stateOn = brightnessIn != 0;
  root["state"] = (stateOn) ? on_cmd : off_cmd;
  root["brightness"] = brightnessIn;

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  client.publish(topic, buffer, true);
}


