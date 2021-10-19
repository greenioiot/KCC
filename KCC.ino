#include <FS.h>
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

//Library ThingIO board
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "BluetoothSerial.h"

WiFiManager wifiManager;
//Constant MQTT , WIFI
//const char* ssid = "Precast_05";     //SSID Name
//const char* password =  "Pre@2021";   //SSID Password
const char* mqttServer = "192,168,0,100";   //IP MQTT
const int mqttPort = 1883;  //Port MQTT
const char* mqttUser = "";  //MQTT User
const char* mqttPassword = "";  //MQTT Password
long lastReconnectAttempt = 0;

//Set Interval time to Restart MCU
unsigned long period = 3600000; //Period Time for Restart = period/1000   unit "Sec"   {1000 = 1 sec} 3600000
unsigned long last_time = 0; //Start variable
int T=0; // Time counter reset ESP when disconnect wifi

//Set WIFI Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

boolean reconnect() {
  Serial.println("Reconnecting to mqtt");
  if (mqttClient.connect("MQTT_Bridge")) {
    // Once connected, publish an announcement...
    Serial.println("Connect To MQTT");
  }
  return mqttClient.connected();
}



#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
 
 
 
long interval = 1000;  //millisecond
unsigned long previousMillis = 0;
 
String dataJson = "";
 
boolean validEpoc = false;



Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

int16_t adc0, adc1, adc2, adc3;
 
void _init() {

 

}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

char mqtt_url[50];
char mqtt_topic[50];
char frequency[4];
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup(void)
{
  Serial.begin(115200);
 
  Serial.println("Load config...");
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          //strcpy(output, json["output"]);
          strcpy(mqtt_url, json["mqtt_url"]);
          strcpy(mqtt_topic, json["mqtt_topic"]);
          strcpy(frequency, json["frequency"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

  WiFiManagerParameter mqtt_url_param("MQTT_URL", "MQTT_URL", mqtt_url, 50);
  WiFiManagerParameter mqtt_topic_param("MQTT_TOPIC", "MQTT_TOPIC", mqtt_topic, 50);
  WiFiManagerParameter frequency_param("FREQUENCY", "FREQUENCY", frequency, 4);
  
///---------------------
  wifiManager.setTimeout(120);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.addParameter(&mqtt_url_param);
  wifiManager.addParameter(&mqtt_topic_param);
  wifiManager.addParameter(&frequency_param);
  String wifiName = "@ESP32-";
  wifiName.concat(String((uint32_t)ESP.getEfuseMac(), HEX));
  if (!wifiManager.autoConnect(wifiName.c_str())) {
    //Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    //    ESP.reset();
    //delay(1000);
    ESP.restart();
    delay(1);
  }
  strcpy(mqtt_url, mqtt_url_param.getValue());
  strcpy(mqtt_topic, mqtt_topic_param.getValue());
  strcpy(frequency, frequency_param.getValue());
  interval = 1000 * atoi(frequency);
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_url"] = mqtt_url;
    json["mqtt_topic"] = mqtt_topic;
    json["frequency"] = frequency;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
  Serial.println("Connected to the WiFi network");
  mqttClient.setServer(mqtt_url,1883);
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
   
    //Specify Sensor Node Number "07aFA07"///////////////////////////////////////////////////////////////////////////////////
    if (mqttClient.connect("07aFA07", mqttUser, mqttPassword )) {
 
      Serial.println("connected");
 
    } else {
 
      Serial.print("failed with state ");
      Serial.print(mqttClient.state());
      delay(2000);
      T=T+1;
      delay(100);
      if (T==30){
      Serial.println("Restart ESP");
      ESP.restart();
    }
    }
  }
  
//----------------------

  SerialBT.begin("07aFA07"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  ads.begin();
  
  previousMillis = millis();
  
}

//====mapfloat====

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  if ((in_max - in_min) + out_min != 0) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  } else {
    return 0;
  }
}

void loop(void)
{

//======
//Restert ESP 

    if(millis() >= period) {
     Serial.println("Restart ESP");
     ESP.restart();
 }

 //Reconnect MQTT
  if (!mqttClient.connected()) {
    Serial.println("mqtt NOT connected");
        T=T+1;
    delay(100);
    if (T==30){
    Serial.println("Restart ESP");
    ESP.restart();
    }
    Serial.println("-------------");
    long now = millis();
    if (now - lastReconnectAttempt > 60000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected
    mqttClient.loop();
  }





//=======
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    adc3 = ads.readADC_SingleEnded(3);

    Serial.print(adc0); Serial.print(" ");  Serial.print(adc1); Serial.print(" ");   Serial.print(adc2); Serial.print(" ");  Serial.println(adc3);
    float val0 = mapfloat(adc0, 2128, 10560, 0, 150);//Temperature 0 - 150
    float val1 = mapfloat(adc1, 2128, 10560, 0, 150);//Temperature 0 - 150
    float val2 = mapfloat(adc2, 2128, 10560, 0, 20);//Vibration 0 - 20
    float val3 = mapfloat(adc3, 2128, 10560, 0, 20);//Vibration 0 - 20

//=======
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   DynamicJsonBuffer jbuffer;
   JsonObject& root = jbuffer.createObject();
      root["N011"] = (val0);// setting up the variable sensor to hold JSON object root]
      root["N012"] = (val1);// setting up the variable sensor to hold JSON object root]
      root["N013"] = (val2);
      root["N014"] = (val3);

 
  //char JSONmessageBuffer[100];
  //JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  char JSONmessageBuffer[100];
  root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);

  if (mqttClient.publish(mqtt_topic, JSONmessageBuffer) == true) {
    Serial.println("Success sending message");
  } else {
    Serial.println("Error sending message");
    T=T+1;
    delay(100);
    if (T==30){
    Serial.println("Restart ESP");
    ESP.restart();
    }
  }
 
  mqttClient.loop();
  Serial.println("-------------");
 
 // delay(10000);
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//=======  
    String udpData2 = "{\"IoT\":\"07aFA07\",\"Sensor1\":";
    udpData2.concat(val0);
    udpData2.concat(",\"Sensor2\":");
    udpData2.concat(val1);
    udpData2.concat(",\"Sensor3\":");
    udpData2.concat(val2);
    udpData2.concat(",\"Sensor4\":");
    udpData2.concat(val3);
 
    udpData2.concat("}");
    Serial.println(udpData2);
    SerialBT.println(udpData2);
 
    previousMillis = currentMillis;

 

    
  }
    delay(10000);
  
}//Closed void loop