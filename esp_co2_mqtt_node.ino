#include <DHT.h>
#include <ESP8266WiFi.h>
#include <MHZ19.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "secrets.h"

//#define DEBUG_ENABLED
#include "debug.h"

// CO2 sensor
#define MH_Z19_RX 4           // D2 on NodeMCU devkit
#define MH_Z19_TX 5           // D1 on NodeMCU devkit
#define MH_Z19_BAUDRATE 9600
const unsigned long MH_Z19_PREHEAT_TIME = 3 * 60 * 1000;

// Temperature and humidity sensor
#define DHT_PIN 12            // DHT data, D6 on NodeMCU devkit
#define DHT_TYPE DHT21        // AM2301 sensor type

// WiFi
const char* ssid = WLAN_SSID;
const char* password = WLAN_PASSWORD;

// MQTT
const char* mqtt_server = MQTT_SERVER;
const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PASSWORD;
const char* mqtt_co2_state_topic = MQTT_CO2_TOPIC;
const char* mqtt_temp_state_topic = MQTT_TEMPERATURE_TOPIC;
const char* mqtt_humidity_state_topic = MQTT_HUMIDITY_TOPIC;
#define MQTT_BUFFER_SIZE (10)
char mqttBuffer[MQTT_BUFFER_SIZE];

const unsigned long MEASUREMENT_INTERVAL = 15 * 1000;
const unsigned long PUBLISH_INTERVAL = 4 * 60 * 1000;
unsigned long lastMeasurement = 0;
unsigned long lastPublish = 0;

// measurement buffers
#define MEASUREMENT_BUFFER_SIZE (10)
float co2Buffer[MEASUREMENT_BUFFER_SIZE];
short co2BufferWriteIndex = 0;
float tempBuffer[MEASUREMENT_BUFFER_SIZE];
short tempBufferWriteIndex = 0;
float humidityBuffer[MEASUREMENT_BUFFER_SIZE];
short humidityBufferWriteIndex = 0;
const float NULL_MEASUREMENT = -300.0;
 
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
MHZ19 co2;
SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX);
bool isCO2SensorReady = false;
DHT dht(DHT_PIN, DHT_TYPE);

float calculateAverage(float buffer[], short bufferSize);

void setupSerial() {
  #ifdef DEBUG_ENABLED
  Serial.begin(115200);
  #endif
}

void setupCO2() {
  co2Serial.begin(MH_Z19_BAUDRATE);
  co2.begin(co2Serial);
  co2.autoCalibration(false);
  #ifdef DEBUG_ENABLED
  co2.printCommunication(true, true);
  #endif
}

void setupDHT() {
  dht.begin();
}

void setupWifi() {
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

void setupMQTT() {
  mqttClient.setServer(mqtt_server, 1883);
}

void setupBuffers() {
  for (short i = 0; i < MEASUREMENT_BUFFER_SIZE; i++) {
    co2Buffer[i] = NULL_MEASUREMENT;
    tempBuffer[i] = NULL_MEASUREMENT;
    humidityBuffer[i] = NULL_MEASUREMENT;
  }
}

void reconnectMQTT() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    DEBUG("Attempting MQTT connection...");
    // Create a random mqttClient ID
    String clientId = "CO2Sensor-";
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

void measureCO2(unsigned long now) {
  if (!isCO2SensorReady && (now > MH_Z19_PREHEAT_TIME)) {
    isCO2SensorReady = true;
  } 
  if (!isCO2SensorReady) {
    DEBUG("CO2 sensor warming up...");
    return;
  }
  int co2ppm = co2.getCO2(false);  // limited read (minimum value 400), force = false
  if (co2.errorCode != RESULT_OK) {
    DEBUG2("Error reading CO2 sensor measurement. Error code:", co2.errorCode);
    co2Buffer[co2BufferWriteIndex] = NULL_MEASUREMENT;
  } else {
    DEBUG2("CO2 measurement: ", co2ppm);
    co2Buffer[co2BufferWriteIndex] = co2ppm;
  }
  co2BufferWriteIndex = (co2BufferWriteIndex + 1) % MEASUREMENT_BUFFER_SIZE;
}

void publishCO2() {
  float average = calculateAverage(co2Buffer, MEASUREMENT_BUFFER_SIZE);
  if (average == NULL_MEASUREMENT) {
    DEBUG("No valid CO2 measurements to publish");
    return;
  }
  dtostrf(average, 4, 0, mqttBuffer);
  DEBUG2("Publish CO2 message:", mqttBuffer);
  mqttClient.publish(mqtt_co2_state_topic, mqttBuffer);  
}

void measureTemperature() {
  float temp = dht.readTemperature();
  if (isnan(temp))
  {
    DEBUG("Error reading temperature sensor measurement.");
    temp = NULL_MEASUREMENT;
  }
  tempBuffer[tempBufferWriteIndex] = temp;
  tempBufferWriteIndex = (tempBufferWriteIndex + 1) % MEASUREMENT_BUFFER_SIZE;
  #ifdef DEBUG_ENABLED
  char strval[5];
  dtostrf(temp, 3, 1, strval);
  DEBUG2("Temperature measurement:", strval);
  #endif
}

void publishTemperature() {
  float average = calculateAverage(tempBuffer, MEASUREMENT_BUFFER_SIZE);
  if (average == NULL_MEASUREMENT) {
    DEBUG("No valid temperature measurements to publish");
    return;
  }
  // convert float to str, 3 digits + 1 decimal digits
  dtostrf(average, 3, 1, mqttBuffer);
  DEBUG2("Publish temperature message:", mqttBuffer);
  mqttClient.publish(mqtt_temp_state_topic, mqttBuffer);  
}

void measureHumidity() {
  float humidity = dht.readHumidity();
  if (isnan(humidity))
  {
    DEBUG("Error reading humidity sensor measurement.");
    humidity = NULL_MEASUREMENT;
  }
  humidityBuffer[humidityBufferWriteIndex] = humidity;
  humidityBufferWriteIndex = (humidityBufferWriteIndex + 1) % MEASUREMENT_BUFFER_SIZE;
  #ifdef DEBUG_ENABLED
  char strval[5];
  dtostrf(humidity, 3, 1, strval);
  DEBUG2("Humidity measurement:", strval);
  #endif
}

void publishHumidity() {
  float average = calculateAverage(humidityBuffer, MEASUREMENT_BUFFER_SIZE);
  if (average == NULL_MEASUREMENT) {
    DEBUG("No valid humidity measurements to publish");
    return;
  }
  // convert float to str, 3 digits + 1 decimal digits
  dtostrf(average, 3, 1, mqttBuffer);
  DEBUG2("Publish humidity message:", mqttBuffer);
  mqttClient.publish(mqtt_humidity_state_topic, mqttBuffer);  
}

void setup() {
  setupSerial();
  setupWifi();
  setupCO2();
  setupDHT();
  setupMQTT();
  setupBuffers();
}

void loop()
{
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  unsigned long now = millis();
  bool measurementDue = (unsigned long)(now - lastMeasurement) >= MEASUREMENT_INTERVAL;
  bool publishDue = (unsigned long)(now - lastPublish) >= PUBLISH_INTERVAL;

  if (measurementDue) {
    measureCO2(now);
    measureTemperature();
    measureHumidity();
    lastMeasurement = now;
  }
  if (publishDue) {
    publishCO2();
    publishTemperature();
    publishHumidity();
    lastPublish = now;
  }
  delay(500);
}

float calculateAverage(float buffer[], short bufferSize) {
  // exclude highest and lowest value, calculate average from remaining
  // if no valid measurements to average, returns NULL_MEASUREMENT
  short maxIndex = 0;
  short minIndex = 0;
  for (short i = 0; i < bufferSize; i++) {
    maxIndex = buffer[i] > buffer[maxIndex] ? i : maxIndex;
    if (buffer[minIndex] == NULL_MEASUREMENT) {
      minIndex = i;
    } else if (buffer[i] > NULL_MEASUREMENT) {
      minIndex = buffer[i] < buffer[minIndex] ? i : minIndex;
    }
  }

  float sum = 0.0;
  short validMeasurementCount = 0;
  for (short i = 0; i < bufferSize; i++) {
    if (
      buffer[i] > NULL_MEASUREMENT &&
      i != minIndex &&
      i != maxIndex
    ) {
      sum += buffer[i];
      validMeasurementCount++;
    }
  }
  return validMeasurementCount > 0
    ? sum / (float)validMeasurementCount
    : NULL_MEASUREMENT;
}
