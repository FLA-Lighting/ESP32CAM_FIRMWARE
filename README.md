# Módulo Mestre dos Postes Inteligentes

Este projeto representa o firmware do módulo mestre do sistema de postes inteligentes baseado em ESP32CAM. O módulo tem como princípio capturar imagens, enviá-las via MQTT e posteriormente passá-las por um processo de análise de inteligência artificial para detectar a presença de pessoas. Simultaneamente, o módulo captura dados do ambiente por meio do sensor BME280. Além disso, está em desenvolvimento a funcionalidade do módulo atenuador, que permitirá o controle da intensidade luminosa dos postes.

## secrets.h (Arquivo de Configuração)

```cpp

// Arquivo: secrets.h

// Definição de identificador único para o dispositivo ESP32CAM
#define ID "ESP32CAM_0"

// Tópico MQTT para publicação de imagens do ESP32CAM
#define ESP32CAM_PUBLISH_TOPIC "esp32/cam_0"

// Configurações de rede Wi-Fi
const char WIFI_SSID[] = "MATHEUS ";     // Nome da rede Wi-Fi
const char WIFI_PASSWORD[] = "12213490"; // Senha da rede Wi-Fi

// Configurações do servidor MQTT
const char mqtt_broker[] = "ec2-13-58-196-72.us-east-2.compute.amazonaws.com"; // Endereço do servidor MQTT
const char mqtt_login[] = "admin";        // Nome de usuário MQTT
const char mqtt_password[] = "1221";      // Senha do usuário MQTT

```
## camera_config.h (Configurações da Câmera)

Este arquivo contém as definições de pinos e a função cameraInit(), responsável pela inicialização da câmera.

## Código Principal (main.ino)

Este arquivo é a parte principal do código. Ele se conecta à rede Wi-Fi, ao servidor MQTT e, em seguida, entra em um loop em que captura imagens da câmera. As imagens são publicadas no servidor MQTT para posterior análise de IA, enquanto os dados do sensor BME280 são enviados para o tópico "esp32/bme280".
