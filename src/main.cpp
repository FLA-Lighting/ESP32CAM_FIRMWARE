#include "secrets.h"
#include "camera_config.h"
#include <MQTTClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// I2C Configuration
#define I2C_SDA 14  // SDA connected to GPIO 14
#define I2C_SCL 15  // SCL connected to GPIO 15
TwoWire I2CSensors = TwoWire(0);

// BME280 Sensor Object
Adafruit_BME280 bme;  // I2C

#define SEALEVELPRESSURE_HPA (1013.25)

// Buffer size for MQTT client
const int bufferSize = 1024 * 23;  // 23552 bytes

// WiFi and MQTT client instances
WiFiClient net;
MQTTClient client = MQTTClient(bufferSize);

// Function to connect to Wi-Fi
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("\n\n=====================");
  Serial.println("Connecting to Wi-Fi");
  Serial.println("=====================\n\n");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n\n=====================");
  Serial.println("Connected to Wi-Fi!");
  Serial.println("=====================\n\n");
}

// Function to connect to MQTT
void connectMQTT() {
  client.begin(mqtt_broker, 1883, net);
  client.setCleanSession(true);

  Serial.println("\n\n=====================");
  Serial.println("Connecting to MQTT Broker");
  Serial.println("=====================\n\n");

  while (!client.connect(ID, mqtt_login, mqtt_password)) {
    Serial.print(".");
    delay(100);
  }

  if (!client.connected()) {
    Serial.println("Timed out while connecting to MQTT!");
    ESP.restart();
    return;
  }

  Serial.println("\n\n=====================");
  Serial.println("Connected to MQTT!");
  Serial.println("=====================\n\n");
}

// Function to capture and publish an image
void grabImage() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb != NULL && fb->format == PIXFORMAT_JPEG && fb->len < bufferSize) {
    Serial.print("Image Length: ");
    Serial.print(fb->len);
    Serial.print("\t Published Image: ");
    bool result = client.publish(ESP32CAM_PUBLISH_TOPIC, (const char *)fb->buf, fb->len);
    Serial.println(result);

    if (!result) {
      ESP.restart();
    }
  }
  esp_camera_fb_return(fb);
  delay(1);
}

// Function to publish sensor data
void publishSensorData() {
  char payload[100];
  snprintf(payload, sizeof(payload), "{\"temperature\": %.2f, \"pressure\": %.2f, \"altitude\": %.2f, \"humidity\": %.2f}",
           bme.readTemperature(), bme.readPressure() / 100.0F, bme.readAltitude(SEALEVELPRESSURE_HPA), bme.readHumidity());

  client.publish("esp32/bme280", payload);
}

// Function to connect to WiFi, MQTT, and initialize sensors
void setup() {
  Serial.begin(115200);
  cameraInit(); // Initialize the camera
  connectWiFi(); // Connect to Wi-Fi
  connectMQTT(); // Connect to MQTT Broker
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  bool status = bme.begin(0x76, &I2CSensors); // Initialize BME280 sensor

  if (!status) {
    Serial.println("Unable to find a valid BME280 sensor, check the connection.");
    while (1);
  }
}

// Function to publish sensor data only once every minute
unsigned long lastSensorDataTime = 0;  // Variable to track the last time sensor data was sent
const unsigned long sensorDataInterval = 60000;  // Interval in milliseconds (1 minute)

void publishSensorDataWithInterval() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastSensorDataTime >= sensorDataInterval) {
    publishSensorData();  // Publish sensor data
    lastSensorDataTime = currentMillis;  // Update the last time sensor data was sent
  }
}

// Main loop
void loop() {
  client.loop(); // Handle MQTT events

  // If connected to MQTT, capture and publish the image
  if (client.connected()) {
    grabImage();

    // Publish sensor data only once every minute
    publishSensorDataWithInterval();
  }
}
