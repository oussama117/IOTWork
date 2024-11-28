#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define WIFI_SSID "iPhone"
#define WIFI_PASSWORD "ranim1289"
#define MQTT_HOST IPAddress(172, 20, 10, 2)
#define MQTT_PORT 1883
#define MQTT_PUB_DATA "esp32/sensors/data"
#define ID_MOUTON 

const int oneWireBus = 4;          
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

Adafruit_MPU6050 mpu;

const String idNecklace = "N0001";
float temp;
const int PulseSensorPin = 34;
int heartRateValue = 0;
int analogValue = 0;
int maxAnalogValue = 4095;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;
const long interval = 10000; // 10 seconds interval for sending data
int TimePerSample = 0 ;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged. PacketId: ");
  Serial.println(packetId);
}

int getHeartRate() {
  analogValue = analogRead(PulseSensorPin);  // Read the analog value from Pulse Sensor
  // Apply a simple scaling to simulate a heartbeat signal (BPM range 60-120)
  heartRateValue = map(analogValue, 0, maxAnalogValue, 60, 120);  // Scale it to BPM range
  return heartRateValue;
}

// Function to get acceleration values
void getAcceleration(float &ax, float &ay, float &az) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
}

// Function to get gyroscope values
void getGyroscope(float &gx, float &gy, float &gz) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
}

// Function to detect movement (acceleration)
float isMoving() {
  float ax, ay, az;
  float ax1, ay1, az1;
  getAcceleration(ax, ay, az);
  delay(500);
  getAcceleration(ax1, ay1, az1);
  float accx = ax1 - ax ;
  float accy = ay1 - ay ;
  float accz = az1 - az ;
  
  return sqrt(accx * accx + accy * accy + accz * accz);
}

// Function to detect rotation (gyroscope)
float isRotating() {
  float gx, gy, gz;
  float gx1, gy1, gz1;
  getGyroscope(gx, gy, gz);
  delay(500);
  getGyroscope(gx1, gy1, gz1);
  float gyrx = gx1 - gx ;
  float gyry = gy1 - gy ;
  float gyrz = gz1 - gz ;
  
  return sqrt(gyrx * gyrx + gyry * gyry + gyrz * gyrz);
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    while (1);
  }
  pinMode(PulseSensorPin, INPUT);
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}

void loop() {
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sensors.requestTemperatures(); 
    temp = sensors.getTempCByIndex(0);
    heartRateValue = getHeartRate();
    String accelerationMagnitude = String(isMoving());
    String jsonMessage = 
          "{\"IdNecklace\":\""+idNecklace+
          "\",\"acc\":"+String(isMoving())+
          ",\"gyr\":"+String(isRotating())+
          ",\"pulse\":"+String(heartRateValue)+
          ",\"temp\":"+String(temp)+
          ",\"time\":"+String(millis())+"}";
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_DATA, 1, true, jsonMessage.c_str());
    Serial.println("");
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_DATA, packetIdPub1);
    Serial.println("");
    Serial.print("Sent Message: ");
    Serial.println(jsonMessage);
    TimePerSample += 4;
  }
  //delay(1000);
}
