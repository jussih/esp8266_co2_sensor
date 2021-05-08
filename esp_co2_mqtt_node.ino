#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "MHZ19.h"

//#define DEBUG_ENABLED
#include "debug.h"

// CO2 sensor UART pins
#define MH_Z19_RX 4           // D2 on NodeMCU devkit
#define MH_Z19_TX 5           // D1 on NodeMCU devkit
#define MH_Z19_BAUDRATE 9600
const unsigned long MH_Z19_PREHEAT_TIME = 3 * 60 * 1000;
const unsigned long MEASUREMENT_INTERVAL = 2 * 60 * 1000;

// Temperature and humidity sensor
#define DHT_PIN 12            // DHT data, D6 on NodeMCU devkit
#define DHT_TYPE DHT21        // AM2301 sensor type

// Connect to the WiFi
const char* ssid = "";
const char* password = "";
const char* mqtt_server = "";
const char* mqtt_user = "";
const char* mqtt_password = "";
const char* mqtt_co2_state_topic = "homeassistant/sensor/sensorBedroomCO2/state";
const char* mqtt_temp_state_topic = "homeassistant/sensor/sensorBedroomTemperature/state";
const char* mqtt_humidity_state_topic = "homeassistant/sensor/sensorBedroomHumidity/state";

#define MQTT_BUFFER_SIZE (50)
char mqttBuffer[MQTT_BUFFER_SIZE];
unsigned long lastMsg = 0;
 
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
MHZ19 co2;
SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX);
bool isCO2SensorReady = false;
DHT dht(DHT_PIN, DHT_TYPE);

void setup_co2() {
  co2Serial.begin(MH_Z19_BAUDRATE);
  co2.begin(co2Serial);
  co2.autoCalibration(false);
  #ifdef DEBUG_ENABLED
  co2.printCommunication(true, true);
  #endif
}

void setup_dht() {
  dht.begin();
}

void setup_wifi() {
  delay(10);
  DEBUG2("Connecting to", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  randomSeed(micros());

  DEBUG("WiFi connected");
  DEBUG2("IP address:", WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    DEBUG("Attempting MQTT connection...");
    // Create a random mqttClient ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      DEBUG("connected");
    } else {
      DEBUG2("failed, rc=", mqttClient.state());
      DEBUG(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void measureCO2() {
  int ppm_uart = co2.getCO2(false);  // limited read (default), force = false
  if (co2.errorCode != RESULT_OK) {
    DEBUG2("Error reading CO2 sensor measurement. Error code:", co2.errorCode);
    return;
  }
  snprintf(mqttBuffer, MQTT_BUFFER_SIZE, "%d", ppm_uart);
  DEBUG2("Publish CO2 message:", mqttBuffer);
  mqttClient.publish(mqtt_co2_state_topic, mqttBuffer);  
}

void measureTemperature() {
  float temp = dht.readTemperature();
  if (isnan(temp))
  {
    DEBUG("Error reading temperature sensor measurement.");
    return;
  }
  // convert float to str, 3 digits + 2 decimal digits
  dtostrf(temp, 3, 1, mqttBuffer);
  DEBUG2("Publish temperature message:", mqttBuffer);
  mqttClient.publish(mqtt_temp_state_topic, mqttBuffer);  
}

void measureHumidity() {
  float humidity = dht.readHumidity();
  if (isnan(humidity))
  {
    DEBUG("Error reading humidity sensor measurement.");
    return;
  }
  // convert float to str, 3 digits + 2 decimal digits
  dtostrf(humidity, 3, 1, mqttBuffer);
  DEBUG2("Publish humidity message:", mqttBuffer);
  mqttClient.publish(mqtt_humidity_state_topic, mqttBuffer);  
}

void setup() {
  #ifdef DEBUG_ENABLED
  Serial.begin(115200);
  #endif
  setup_wifi();
  setup_co2();
  setup_dht();
  mqttClient.setServer(mqtt_server, 1883);
}

void loop()
{
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();
  unsigned long now = millis();
  bool measurementDue = now - lastMsg > MEASUREMENT_INTERVAL;

  if (!isCO2SensorReady && (now > MH_Z19_PREHEAT_TIME)) {
    isCO2SensorReady = true;
  }
  if (isCO2SensorReady && measurementDue) {
    measureCO2();
  }
  if (measurementDue) {
    lastMsg = now;
    measureTemperature();
    measureHumidity();
  }
  delay(1000);
}
