/*
 * Description:  Control LED with AskSensors and ESP32 dev board over MQTT
 *  Author: https://asksensors.com, 2020
 *  github: https://github.com/asksensors
 */
 
 
#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#define EEPROM_SIZE 6


const char* ssid = "Your ssid"; // Wifi SSID
const char* password = "password"; // Wifi Password
const char* subTopic = "TOPIC"; // This is your choice,This is the key for your mqtt communication
const int LED_pin = 22; // LEd pin As per your controller
const char* mqtt_server = "192.168.8.120";//IP address as you available
unsigned int mqtt_port = 1883;
unsigned int currentMillis1 ;
unsigned int previousMillis1=0; 
unsigned int interval1=3000;
int address=0;


WiFiClient ESP32Client No;//if you can connect mutipple device in single MQTT Broker You must give a uniqe no for each one.
                          //example: ESP32Client 1, ESP32Client 2, ESP32client3 like
                          //If you give same to all ,Only one device can communicate at a time not work all in symmultenosly
                          //Conclution: you must need a unique client id for each device you connect in same mqtt
PubSubClient client(ESP32Client No);

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  Serial.println("*****************************************************");
  Serial.println("********** Program Start : ESP32 controls LED with mqtt.icfoss.org over MQTT");
  Serial.println("Set LED as output");
  
  pinMode(LED_pin, OUTPUT);   // set led as output
  if(EEPROM.read(address)!=0)
  {
    digitalWrite(LED_pin,HIGH);
  }
  
  Serial.print("********** connecting to WIFI : ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("->WiFi connected");
  Serial.println("->IP address: ");
  Serial.println(WiFi.localIP());
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  if (!client.connected()) 
    reconnect();
  Serial.print("********** Subscribe to swathantra light management  topic:");
  Serial.print(subTopic);
  // susbscribe
  client.subscribe(subTopic);
}

void loop() {
   delay(200); 
     
  if (!client.connected()) {
    reconnect();
  }

  client.loop();
  
}
String s;
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Command received from mqtt.icfoss.org[");
  Serial.print(topic);
  Serial.println("] ");
  int i = 0;

  for (i = 0; i < length; i++) {
    Serial.print((char)payload[i]); 
  }
  payload[i]='\0';
  Serial.println("");
  Serial.println("********** Parse Actuator command");  
  Serial.print("$$$");
  Serial.println((char *)payload);
  //Serial.println((char*)"module1=1");
  Serial.println("$$$");
  
  if(strcmp((char *)payload,(char*)"module1=1")==0){ 
    digitalWrite(LED_pin, HIGH);
    Serial.println("LED is OFF");  
    EEPROM.write(address, LOW);
    EEPROM.commit();
    Serial.println("eeprom=");
    Serial.print(byte(EEPROM.read(address)));  
  } 
  else{
    digitalWrite(LED_pin, LOW);
    Serial.println("LED is ON");
    EEPROM.write(address, HIGH);
    EEPROM.commit();
    Serial.println("eeprom=");
    Serial.print(byte(EEPROM.read(address))); 
   }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    WiFiretake();
    Serial.print("********** Attempting MQTT connection..."); // Attempt to connect
    if (client.connect("ESP32Client No", username, pass)) {  
      Serial.println("-> MQTT client connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("-> try again in 5 seconds"); // Wait 5 seconds before retrying
      delay(5000);
    }
    client.subscribe(subTopic);
  }
}
void WiFiretake(){
  unsigned long currentMillis1 = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis1 - previousMillis1 >=interval1)) {
    //Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    Serial.println("wifi is struggling");
    WiFi.reconnect();
    delay(5000);
    Serial.println("wifi is rejoin");
    previousMillis1 = currentMillis1;
    Serial.println("initWifi previousMillis = currentMillis;");
  }
  }
