#include "secrets.h"
#include "camera_config.h"
#include <MQTTClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Definições de pinos para I2C
#define I2C_SDA 14  // SDA conectado ao GPIO 14
#define I2C_SCL 15  // SCL conectado ao GPIO 15
TwoWire I2CSensors = TwoWire(0);

// Objeto do sensor BME280
Adafruit_BME280 bme;  // I2C

#define SEALEVELPRESSURE_HPA (1013.25)

// Tamanho do buffer para o cliente MQTT
const int bufferSize = 1024 * 23;  // 23552 bytes

// Instâncias do cliente WiFi e MQTT
WiFiClient net;
MQTTClient client = MQTTClient(bufferSize);

// Função para conectar ao Wi-Fi
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("\n\n=====================");
  Serial.println("Conectando no Wi-Fi");
  Serial.println("=====================\n\n");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n\n=====================");
  Serial.println("Conectado ao Wi-Fi!");
  Serial.println("=====================\n\n");
}

// Função para conectar ao MQTT
void connectMQTT() {
  // Inicializa o cliente MQTT
  client.begin(mqtt_broker, 1883, net);
  client.setCleanSession(true);

  Serial.println("\n\n=====================");
  Serial.println("Conectando no MQTT BROKER");
  Serial.println("=====================\n\n");

  while (!client.connect(ID, mqtt_login, mqtt_password)) {
    Serial.print(".");
    delay(100);
  }

  if (!client.connected()) {
    Serial.println("Tempo esgotado ao conectar ao MQTT!");
    ESP.restart();
    return;
  }

  Serial.println("\n\n=====================");
  Serial.println("Conectado ao MQTT!");
  Serial.println("=====================\n\n");
}

// Função para capturar e publicar uma imagem
void grabImage() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb != NULL && fb->format == PIXFORMAT_JPEG && fb->len < bufferSize) {
    Serial.print("Comprimento da Imagem: ");
    Serial.print(fb->len);
    Serial.print("\t Imagem Publicada: ");
    bool result = client.publish(ESP32CAM_PUBLISH_TOPIC, (const char *)fb->buf, fb->len);
    Serial.println(result);

    if (!result) {
      ESP.restart();
    }
  }
  esp_camera_fb_return(fb);
  delay(1);
}

// Função para publicar dados do sensor BME280
void publishSensorData() {
  char payload[100];
  snprintf(payload, sizeof(payload), "{\"temperatura\": %.2f, \"pressao\": %.2f, \"altitude\": %.2f, \"umidade\": %.2f}",
           bme.readTemperature(), bme.readPressure() / 100.0F, bme.readAltitude(SEALEVELPRESSURE_HPA), bme.readHumidity());

  client.publish("esp32/bme280", payload);
}

// Função de configuração
void setup() {
  Serial.begin(115200);
  cameraInit(); // Inicializa a câmera
  connectWiFi(); // Conecta ao Wi-Fi
  connectMQTT(); //// Conecta ao MQTT BROKER
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  bool status = bme.begin(0x76, &I2CSensors); // Inicializa o sensor BME280
  if (!status) {
    Serial.println("Não foi possível encontrar um sensor BME280 válido, verifique a conexão!");
    while (1);
  }
}

// Função principal de execução contínua
void loop() {
  client.loop(); // Lida com os eventos do cliente MQTT

  // Se conectado ao MQTT, captura e publica a imagem e publica os dados do sensor
  if (client.connected()) {
    grabImage();
    publishSensorData();
  }
}
