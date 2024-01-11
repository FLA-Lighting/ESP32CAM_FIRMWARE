// Arquivo: secrets.h

// Definição de identificador único para o dispositivo ESP32CAM
#define ID "ESP32CAM_0"

// Tópico MQTT para publicação de imagens do ESP32CAM
#define ESP32CAM_PUBLISH_TOPIC "esp32/cam_0"


// Configurações do servidor MQTT
const char mqtt_broker[] = "35.208.123.29"; // Endereço do servidor MQTT
const char mqtt_login[] = "admin";        // Nome de usuário MQTT
const char mqtt_password[] = "1221";      // Senha do usuário MQTT
