/*
 * These are the setting you need to customize for your own hardware and MQTT server
 * At some point this could be part of the WiFi setup process
 */

#define MQTT_SERVER       "192.168.1.20"
#define MQTT_SERVERPORT   1883
#define MQTT_USERNAME     "YOUR_MQTT_USERNAME"
#define MQTT_PASSWORD     "YOUR_MQTT_PASSWORD"
#define MQTT_TOPIC        "your/mqtt/topic"

// The current SHA1 fingerprint is used to validate the MQTT server certificate and can be found by running
// echo -n | openssl s_client -connect 192.168.73.15:8883 | openssl x509 -fingerprint -noout
#define MQTT_SSL_FINGERPRINT   "YOUR_MQTT_SERVER_CERT_FINGERPRINT"

// these are the IDs printed to the console by the DS18B20_setup function
#define FRIDGE_TEMP_SENSOR_ID  { 0x28, 0xFF, 0xD7, 0xD4, 0x31, 0x18, 0x01, 0xF9 }
#define FREEZER_TEMP_SENSOR_ID { 0x28, 0x89, 0x27, 0x3D, 0x0A, 0x00, 0x00, 0x74 }
#define ROOM_TEMP_SENSOR_ID    { 0x28, 0x42, 0x5D, 0x3D, 0x0A, 0x00, 0x00, 0xC3 }
