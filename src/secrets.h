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
