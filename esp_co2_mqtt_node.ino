#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "MHZ19.h"
//#define DEBUG
#include "debug.h"

#define MEASUREMENT_INTERVAL 120000

// CO2 sensor UART pins
#define MH_Z19_RX 4
#define MH_Z19_TX 5
#define MH_Z19_BAUDRATE 9600
const unsigned int MH_Z19_PREHEAT_TIME = 3 * 60 * 1000;

// Connect to the WiFi
const char* ssid = "";
const char* password = "";
const char* mqtt_server = "";
const char* mqtt_user = "";
const char* mqtt_password = "";
const char* mqtt_config_topic = "homeassistant/sensor/sensorBedroomCO2/config";
const char* mqtt_state_topic = "homeassistant/sensor/sensorBedroomCO2/state";

#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
unsigned long lastMsg = 0;
 
WiFiClient espClient;
PubSubClient client(espClient);
MHZ19 co2;
SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX);
bool isCO2SensorReady = false;

void setup_co2() {
  co2Serial.begin(MH_Z19_BAUDRATE);
  co2.begin(co2Serial);
  co2.autoCalibration(false);
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
  while (!client.connected()) {
    DEBUG("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      DEBUG("connected");
    } else {
      DEBUG2("failed, rc=", client.state());
      DEBUG(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publishMeasurement() {
  int ppm_uart = co2.getCO2();
  if (ppm_uart < 0) {
    DEBUG("Error reading sensor measurement");
    return;
  }
  snprintf(msg, MSG_BUFFER_SIZE, "%d", ppm_uart);
  DEBUG2("Publish message:", msg);
  client.publish(mqtt_state_topic, msg);  
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  setup_co2();
  client.setServer(mqtt_server, 1883);
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  unsigned long now = millis();
  if (!isCO2SensorReady && (now > MH_Z19_PREHEAT_TIME)) {
    isCO2SensorReady = true;
  }
  if (isCO2SensorReady && (now - lastMsg > MEASUREMENT_INTERVAL)) {
    lastMsg = now;
    publishMeasurement();
  }
  delay(1000);
}
