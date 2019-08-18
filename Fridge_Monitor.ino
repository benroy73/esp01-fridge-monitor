/*
 * language ref https://www.arduino.cc/reference/en/
 * 
 * To do:
 * - get TLS working to MQTT server with WiFiClientSecure
 * - Battery health reporting
 * - DS18b20 id discovery
 * - certificate validation
 * - Use a task schedule to run do things on different intervals  https://github.com/arkhipenko/TaskScheduler
 * 
 * 
 * https://developer.xively.com/docs/esp8266
 * https://www.esp8266.com/viewtopic.php?f=32&t=14301
 * https://www.enricobassetti.it/2018/03/nodemcu-esp8266-and-ssl-tls-https
 * https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/WiFiClientSecureAxTLS.h
 * https://github.com/esp8266/Arduino/issues/1851
 * https://letsencrypt.org/certificates/
 */

#define DEBUG false // flag to turn on/off debugging set to true or false
#define Serial if(DEBUG)Serial // Serial output is only needed for debugging

#include <ESP8266WiFi.h>

// for OTA
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// for WifiManager
#include <DNSServer.h>            // Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     // Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <MQTT.h>

#include <OneWire.h>              // http://www.pjrc.com/teensy/td_libs_OneWire.html
#include <DallasTemperature.h>    // http://milesburton.com/Main_Page?title=Dallas_Tem...

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#include "config.h"               // this is the file where your customize your own settings

#define ESP_LED           1  // the blue LED is on the same pin as TX GPIO1, the red LED is always on
#define ESP_GP0           0  // GP0 gpio 0
#define ESP_GP2           2  // GP2 gpio 2
#define ESP_TX            1  // TXD gpio 1
#define ESP_RX            3  // RXD gpio 3

#define ONE_WIRE_BUS      ESP_GP2
#define I2C_BUS_1         ESP_RX
#define I2C_BUS_2         ESP_GP0
#define SPEAKER_PIN       ESP_TX

unsigned long lastMillis = 0;

// DS18b20 stuff
// instead of hardcoding the sensor IDs they should be discovered and set in the DS18B20_setup function
DeviceAddress fridgeTempSensor  = FRIDGE_TEMP_SENSOR_ID;
DeviceAddress freezerTempSensor = FREEZER_TEMP_SENSOR_ID;
DeviceAddress roomTempSensor    = ROOM_TEMP_SENSOR_ID;


OneWire oneWire(ONE_WIRE_BUS); // oneWire bus pins
DallasTemperature sensors(&oneWire);  // Pass address of our oneWire instance to Dallas Temperature.

byte degree[8] = { B00110, B01001, B01001, B00110, B00000, B00000, B00000, B00000 };  // Custom degree character
LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//WiFiClientSecure tls_client;  // WiFiClientSecure for SSL/TLS support -- TODO this dosen't work but it did previously
WiFiClient tls_client;  // WiFiClient with no TLS encrypton
MQTTClient mqtt_client;

//boolean verify_MQTT_server_fingerprint() {
//    // check the TSL certificate fingerprint to verify the server identity
//    // we can't validate with a CA bundle so this is the next best option
//    const char* host = MQTT_SERVER;
//    Serial.print("Connecting to ");
//    Serial.println(host);
//    if (! tls_client.connect(host, MQTT_SERVERPORT)) {
//        Serial.println("Connection failed.");
//        return false;
//    }
//    if (tls_client.verify(MQTT_SSL_FINGERPRINT, host)) {
//        Serial.println("Secure connection. Certificate matches.");
//        return true;
//    } else {
//        Serial.println("Insecure connection! Certificate did not match.");
//        lcd_display("Cert mismatch", "");
//        return false;
//    }
//}


boolean MQTT_connect() {
    if (mqtt_client.connected()) {
        return true;
    }

    // this type of hard coded fingerprint verifying breaks when the MQTT cert is renewed
    // unfortunately this happens frequently for a LetsEncrypt cert so finding a TLS library that
    // really does cert verification will be much better
//    if (! verify_MQTT_server_fingerprint()) {
//        lcd_display("MQTT connection", "SSL FAILED");
//        return false;
//    }

    delay(1000);
    delay(1000);
    delay(1000);
    
    Serial.println("Connecting to MQTT... ");
    Serial.println(String(ESP.getChipId()).c_str());
    Serial.println(MQTT_USERNAME);
    Serial.println(MQTT_PASSWORD);
 
    if (mqtt_client.connect(String(ESP.getChipId()).c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
      lcd_display("MQTT connected", "");
      return true;
    }
    else {
        lcd_display("MQTT connection", "FAILED");
//        Serial.println(mqtt_client.lastError());
//        Serial.println(mqtt_client.returnCode());
//        delay(1000);
//        lcd_display(mqtt_client.lastError(), mqtt_client.returnCode());
        return false;
    }
}


void MQTT_send(char *msg) {
    if (! MQTT_connect()) {
        return;
    }
  
    // Now we can publish stuff!
    Serial.print("Sending mqtt msg: ");
    Serial.println(msg);

    lcd.setCursor(15,0);
    lcd.print(".");
    delay(1000);

    if (mqtt_client.publish(MQTT_TOPIC, msg)) {
        Serial.println("OK!");
        lcd.setCursor(15,0);
        lcd.print(" ");
    } else {
        Serial.println("Failed");
        lcd.setCursor(15,0);
        lcd.print("!");
    }
}


void lcd_setup() {
    Serial.println("Setting up LCD");
    Wire.pins(I2C_BUS_1, I2C_BUS_2);  // i2c bus pins
    Wire.begin();
    lcd.init();
    lcd.createChar(0, degree);
    lcd.backlight();
}


void lcd_display(char *line_one, char *line_two) {
    // clear the screen
    lcd.setCursor(0,0);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print("                ");

    // show line 1
    lcd.setCursor(0,0);
    lcd.print(line_one);

    // show line 2
    lcd.setCursor(0,1);
    lcd.print(line_two);

    Serial.println("----------------");
    Serial.println(line_one);
    Serial.println(line_two);
    Serial.println("----------------");
}


void lcd_display_temps(float temp1, float temp2) {
    char temp_fridge[8];
    char temp_freezer[8];
    sprintf(temp_fridge, "%3.1f", temp1);
    sprintf(temp_freezer, "%3.1f", temp2);

    lcd.setCursor(0,0);
    lcd.print("Fridge:  ");
    lcd.print(temp_fridge);
    lcd.write((byte)0);
    lcd.print("F ");
  
    lcd.setCursor(0,1);
    lcd.print("Freezer: ");
    lcd.print(temp_freezer);
    lcd.write((byte)0);
    lcd.print("F ");
}


void sound_alarm() {
    pinMode(SPEAKER_PIN, OUTPUT);  //sets the speaker pin to be an output

    const int NOTE = 466;  // b is 466 Hz  double to increase octave
    const int DURATION = 700;
    const int COUNT = 3;

    for (uint8_t i = 0; i < COUNT; i++) {
      tone(SPEAKER_PIN, NOTE * 3, DURATION); 
      delay(DURATION);
      delay(DURATION/2);
    }
}


void updateTemperatures() {
    sensors.requestTemperatures();    // request temp update from all devices on the bus
    float temp_fridge = sensors.getTempF(fridgeTempSensor);
    float temp_freezer = sensors.getTempF(freezerTempSensor);
    float temp_room = sensors.getTempF(roomTempSensor);

    if (temp_fridge < -190 || temp_freezer < -190 || temp_room < -190) { // reports -196 when no sensor info
        // TODO: this should be more flexible and handle some working sensors
        lcd_display("Temp sensor error", "");
        return;
    }

    lcd_display_temps(temp_fridge, temp_freezer);

    char msg_json[128];
    sprintf(msg_json, "{\"fridge\":%3.1f,\"freezer\":%3.1f,\"room\":%3.1f}", temp_fridge, temp_freezer, temp_room);
    MQTT_send(msg_json);

    // if temp is too high or low sound the alarm and send MQTT message
    if (temp_fridge > 40) {
//        sound_alarm();
    }
    if (temp_freezer > 20) {
//        sound_alarm();
    }
}


void check_for_power() {
//   if power is out sound the alarm and send MQTT message
//      sound_alarm();
}


void printAddress(DeviceAddress deviceAddress) {
    // print a onewire device address
    Serial.print("{ ");
    for (uint8_t i = 0; i < 8; i++) {
        if (deviceAddress[i] < 16) Serial.print("0");
        Serial.print("0x");
        Serial.print(deviceAddress[i], HEX);
        Serial.print(", ");
    }
    Serial.print("}");
}


void DS18B20_setup() {
    Serial.println("");

    Serial.print("Locating DS18B20 devices...");
    sensors.begin();
    Serial.print("Found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");
  
    // report parasite power requirements
    Serial.print("Parasite power is: "); 
    if (sensors.isParasitePowerMode()) Serial.print("ON");
    else Serial.print("OFF");
    Serial.println("");
    
    DeviceAddress tempDeviceAddress;
    oneWire.reset_search();
    uint8_t device_count = sensors.getDeviceCount();
    for (uint8_t i = 0; i < device_count; i++) {
//      // Search for devices on the bus and assign based on an index.
//      if (!sensors.getAddress(tempSensor1, 0)) Serial.println("Unable to find address for Device 0"); 
//      Serial.print("Device 0 Address: ");
//      printAddress(tempSensor1);
//      Serial.println();

      // get the address for each device
      if (!oneWire.search(tempDeviceAddress)) Serial.println("Unable to find address for tempSensor");
      Serial.print("tempDeviceAddress: ");
      printAddress(tempDeviceAddress);
      Serial.println();

      // set the resolution (9 to 12 bits, lower is faster)
      sensors.setResolution(tempDeviceAddress, 9); 
    }
}


void setup_wifi() {
    lcd_display("Wifi connecting", "");

    WiFiManager wifiManager;
//    wifiManager.resetSettings();  //reset saved settings for testing

    //get ssid and password from eeprom and try to connect
    //if it does not connect it starts an access point with the specified name
    //"AutoConnectAP"and goes into a blocking loop awaiting configuration
//    wifiManager.autoConnect("AutoConnectAP");

    // specify the AP name and password
//    wifiManager.autoConnect("AP-NAME", "AP-PASSWORD");

    // auto generate the AP name as "ESP" + ChipID
    wifiManager.autoConnect();

//    Serial.print("Connecting to ");
//    Serial.println(ssid);
//    WiFi.mode(WIFI_STA);
//    WiFi.begin(ssid, password);
//    
//    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
//        Serial.println("Connection Failed! Rebooting...");
//        delay(5000);
//        ESP.restart();
//    }

    char row_two[16];
    sprintf(row_two, "%-16s", WiFi.localIP().toString().c_str());
    lcd_display("Wifi connected", row_two);
}


void setup_ota() {
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);
  
    // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setHostname("myesp8266");
  
    // No authentication by default
    // ArduinoOTA.setPassword("admin");
  
    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";
    
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("OTA Ready");
}


void setup() {
    Serial.begin(115200);
    Serial.println("Booting");
    delay(1000); // Allow the hardware to sort itself out
  
    lcd_setup();
    lcd_display("Booting", "");

    setup_wifi();
    setup_ota();

    DS18B20_setup();

    mqtt_client.begin(MQTT_SERVER, MQTT_SERVERPORT, tls_client);
    MQTT_connect();
    lastMillis = millis();
}

void loop() {
    delay(1000); // repeat the loop every 1s
    ArduinoOTA.handle();  // watch for OTA updates
    mqtt_client.loop();   // process MQTT subscriptions and keep the connection alive

    if (millis() - lastMillis > 15000) {  // do this every 15s
        lastMillis = millis();

        updateTemperatures(); // read the sensors and update the LCD and MQTT
        check_for_power();    // check for power outages
    }
}
