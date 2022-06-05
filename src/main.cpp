/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor
  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "myconfig.h"

#define SEALEVELPRESSURE_HPA 1013.25
#define MSG_BUFFER_SIZE  50
#define BAUD_RATE 115200
#define MQTT_PORT 1883
#define LOOP_DELAY 60000

// DECLARATIONS
bool DEBUG_MODE = true;
Adafruit_BME280 bme; // I2C
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];
int value = 0;
float sensorTemperature = 0.0;
float sensorPressure = 0.0;
float sensorHumidity = 0.0;
String temperatureUrl = String("/sensors/" + sensorName + "/Temperature");
String humidityUrl = String("/sensors/" + sensorName + "/Humidity");
String pressureUrl = String("/sensors/" + sensorName + "/Pressure");

// FONCTIONS
void setup_wifi() 
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

}

void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void printValues() 
{
  Serial.print("Temperature = ");
  Serial.print(sensorTemperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(sensorPressure / 100.0F);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(sensorHumidity);
  Serial.println(" %");

  Serial.println("---------------------------");
}

void setup() 
{
  Serial.begin(BAUD_RATE);
  delay(10);
  setup_wifi();
  client.setServer(mqtt_server, MQTT_PORT);
  client.setCallback(callback);
  
  if (!bme.begin(0x76, &Wire)) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }

  Serial.println("-- Default Test --");
  Serial.println("normal mode, 16x oversampling for all, filter off,");
  Serial.println("0.5ms standby period");
  Serial.println();
}


void loop() 
{  
  if (!client.connected()) 
    reconnect();
  
  client.loop();
  
  // Only needed in forced mode! In normal mode, you can remove the next line.
  bme.takeForcedMeasurement(); // has no effect in normal mode


  unsigned long now = millis();
  if (now - lastMsg > 2000) 
  {
    lastMsg = now;
    sensorTemperature = bme.readTemperature();
    sensorPressure = bme.readPressure();
    sensorHumidity = bme.readHumidity();
    snprintf(msg, MSG_BUFFER_SIZE, "%f", sensorTemperature);
    client.publish(temperatureUrl.c_str(), msg);
    snprintf(msg, MSG_BUFFER_SIZE, "%f", sensorHumidity);
    client.publish(humidityUrl.c_str(), msg);
    snprintf(msg, MSG_BUFFER_SIZE, "%f", sensorPressure / 100.0F);
    client.publish(pressureUrl.c_str(), msg);

    if(DEBUG_MODE)
      printValues();
  }

  delay(LOOP_DELAY);
  // deepSlepp for 60s
  // ESP.deepSleep(60 * 1000000);
}
