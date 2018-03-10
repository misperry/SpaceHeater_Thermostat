/*
 *  File Name: TempHumSensor.ino
 *  
 *  Application: HomeAssistant Space Heater Thermostat
 *  
 *  Description: This code is for the ESP8266 WiFi enabled arduino
 *  compatible device.  This will relay the temperature information 
 *  of the DHT11 device to the HASS frontend for processing.
 *  
 *  Author: M. Sperry - http://www.youtube.com/misperry
 *  Date: 03/09/2018
 *  Revision: 1.0
 *  
 *  
 */

#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define CON_TIME_OUT 20                  //Timeout of no connection to wifi
#define MQTT_TIME_OUT 10                  //Timeout of no connection to MQTT server

#define DHTPIN 0                         //Pin which is connected to the DHT sensor
#define DHTTYPE DHT11                    //Type of sensor is the DHT11, you can change it to DHT22 (AM2302), DHT21 (AM2301)

#define mqtt_server "" // Enter your MQTT server adderss or IP. I use my DuckDNS adddress (yourname.duckdns.org) in this field
#define mqtt_user "" //enter your MQTT username
#define mqtt_password "" //enter your password
#define MQTT_SENSOR_TOPIC "ha/bedroom_temp"  //Enter topic for your MQTT

// Wifi: SSID and password
const char* ssid     = "";
const char* password = "";

//DHT SEtup
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// function called to publish the temperature and the humidity
void publishData(float p_temperature) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["temperature"] = (String)p_temperature;
  root.prettyPrintTo(Serial);
  Serial.println("");

  char data[200];
  root.printTo(data, root.measureLength() + 1);
  client.publish(MQTT_SENSOR_TOPIC, data, true);
}

// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESPBlindstl", mqtt_user, mqtt_password)) {
      Serial.println("INFO: connected");
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.print(client.state());
      Serial.println("DEBUG: try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup(void) {

    Serial.begin(9600);

    // We start by connecting to a WiFi network
   
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(800);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  
  // init the MQTT connection
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Initialize DHT sensor
  dht.begin();
  Serial.println("DHT11 Unified Sensor Data");

  //Print temp sensor details
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");

  //Set delay between sensor readings based on sensor details
  delayMS = sensor.min_delay / 1000;
}

void loop(void) {

  float temperature;

  if (!client.connected())
  {
    reconnect();
  }

  delay(delayMS);

  // Get temperature event and print its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
    temperature = 0.00;
  }
  else {
    temperature = event.temperature;
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C");
  }
  
  // publish to MQTT
  publishData(temperature);
      
}
