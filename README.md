# Módulo Mestre

Este projeto representa o firmware do módulo mestre do sistema de postes inteligentes. O módulo tem como princípio capturar imagens, enviá-las via MQTT e posteriormente passá-las por um processo de análise de inteligência artificial para detectar a presença de pessoas. Simultaneamente, o módulo captura dados do ambiente por meio do sensor BME280.

Este é um código para um projeto usando um ESP32 com uma câmera (ESP32-CAM), um sensor de pressão, temperatura e umidade BME280, e comunicação MQTT para enviar dados da câmera e informações do sensor para um broker MQTT. Vamos passar pelo código passo a passo:

1. **Inclusão de Bibliotecas:**
   ```cpp
   #include "secrets.h"
   #include "camera_config.h"
   #include <MQTTClient.h>
   #include <Wire.h>
   #include <Adafruit_Sensor.h>
   #include <Adafruit_BME280.h>
   ```

   - `secrets.h`: Este arquivo contém informações sensíveis como as credenciais Wi-Fi (`WIFI_SSID` e `WIFI_PASSWORD`), o endereço do broker MQTT (`mqtt_broker`), login e senha MQTT (`mqtt_login` e `mqtt_password`).
   - `camera_config.h`: Configurações relacionadas à câmera.
   - `MQTTClient.h`: Biblioteca para implementar a funcionalidade MQTT.
   - `Wire.h`: Biblioteca para comunicação I2C.
   - `Adafruit_Sensor.h` e `Adafruit_BME280.h`: Bibliotecas para o sensor BME280.

2. **Configuração de Pinos I2C:**
   ```cpp
   #define I2C_SDA 14  // SDA conectado ao GPIO 14
   #define I2C_SCL 15  // SCL conectado ao GPIO 15
   TwoWire I2CSensors = TwoWire(0);
   ```

   - Define os pinos SDA e SCL para a comunicação I2C.

3. **Objeto do Sensor BME280:**
   ```cpp
   Adafruit_BME280 bme;  // I2C
   ```

   - Cria um objeto para o sensor BME280.

4. **Definição de Constantes:**
   ```cpp
   #define SEALEVELPRESSURE_HPA (1013.25)
   const int bufferSize = 1024 * 23;  // 23552 bytes
   ```

   - Define a pressão ao nível do mar para o cálculo da altitude.
   - Define o tamanho do buffer para o cliente MQTT.

5. **Instâncias de Cliente MQTT e Cliente Wi-Fi:**
   ```cpp
   WiFiClient net;
   MQTTClient client = MQTTClient(bufferSize);
   ```

   - Cria instâncias do cliente MQTT e do cliente Wi-Fi.

6. **Função para Conectar ao Wi-Fi:**
   ```cpp
   void connectWiFi() { ... }
   ```

   - Conecta o dispositivo ao Wi-Fi.

7. **Função para Conectar ao MQTT:**
   ```cpp
   void connectMQTT() { ... }
   ```

   - Conecta o dispositivo ao broker MQTT.

8. **Função para Capturar e Publicar uma Imagem:**
   ```cpp
   void grabImage() { ... }
   ```

   - Captura uma imagem da câmera e a publica no tópico MQTT.

9. **Função para Publicar Dados do Sensor:**
   ```cpp
   void publishSensorData() { ... }
   ```

   - Lê dados do sensor BME280 e os publica no tópico MQTT.

10. **Configuração Inicial no `setup()`:**
   ```cpp
   void setup() { ... }
   ```

   - Inicializa a comunicação serial, a câmera, conecta ao Wi-Fi, ao MQTT e inicializa o sensor BME280.

11. **Loop Principal no `loop()`:**
   ```cpp
   void loop() { ... }
   ```

   - Lida com eventos MQTT, captura e publica imagens, e publica dados do sensor com intervalo de um minuto.
