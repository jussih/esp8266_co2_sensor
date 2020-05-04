#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "MHZ.h"

#define MEASUREMENT_INTERVAL 120000

// CO2 sensor UART pins
#define MH_Z19_RX 4
#define MH_Z19_TX 5

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
MHZ co2(MH_Z19_RX, MH_Z19_TX, MHZ19B);

void setup_wifi() {
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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publishDiscovery() {
  // Publish HomeAssistant MQTT autodiscovery message
  const unsigned char discovery[] = "{\"name\": \"CO2\", \"state_topic\": \"homeassistant/sensor/sensorBedroomCO2/state\", \"unit_of_measurement\": \"PPM\"}";
  Serial.print("Publishing discovery message...");
  //Serial.println(discovery);
  client.beginPublish(mqtt_config_topic, sizeof(discovery), false);
  client.write(discovery, sizeof(discovery));
  bool result = client.endPublish();
  
  if (!result) {
    Serial.println("Discovery publish failed!");
  }
}

void publishMeasurement() {
  if (!co2.isReady()) {
    Serial.println("Sensor not ready for measurement...");
    return;
  }
  int ppm_uart = co2.readCO2UART();
  if (ppm_uart < 0) {
    Serial.println("Error reading sensor measurement");
    return;
  }
  snprintf(msg, MSG_BUFFER_SIZE, "%d", ppm_uart);
  Serial.print("Publish message: ");
  Serial.println(msg);
  client.publish(mqtt_state_topic, msg);  
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop()
{
  if (!client.connected()) {
    reconnect();
    publishDiscovery();
  }
  client.loop();
  unsigned long now = millis();
  if (now - lastMsg > MEASUREMENT_INTERVAL) {
    lastMsg = now;
    publishMeasurement();
  }
  delay(1000);
}
