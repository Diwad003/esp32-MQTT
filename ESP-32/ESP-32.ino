#include <ArduinoMqttClient.h>
#include <WiFiClientSecure.h>
//#include <esp_task_wdt.h>      // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/misc_system_api.html
#include "esp_intr_types.h"
#include "HardwareSerial.h"
#include "esp_rom_gpio.h"¨




void setup();
void loop();
void connectToWiFi();
void mqttConnect();
void write_mqtt(MqttClient mqttClient, char* topic, char* data);
void read_mqtt(MqttClient mqttClient);
void uart_init();
void uart_Tx(const char data);
void uart_Tx_string(const char* data);
byte uart_Rx();


  
char ssid[] = "iPSK-UMU";    // your network SSID (name)
char password[] = "HejHopp!!";    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;


MqttClient mqttClient(wifiClient);
const char broker[] = "tfe.iotwan.se";
int        port     = 1883;
char* send_topic  = "JD/controller";
char* receive_topic = "JD/vehicle"; 
#define mqtt_username "intro23"
#define mqtt_password "outro"


#define Rx 16
#define Tx 17


void setup() 
{
  Serial.begin(9600);
  
  uart_init();
  connectToWiFi();
  mqttConnect();
  mqttClient.subscribe(send_topic);
}

void loop()
{
  read_mqtt(mqttClient);
}

void connectToWiFi()
{
    // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WiFi SSID: ");
  Serial.println(ssid);


  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    // wait 1 second for re-trying
    delay(1000);
  }

  Serial.println("You're connected to the network\n");
}

void mqttConnect()
{  
  //Connect to MQTT broker
  mqttClient.setUsernamePassword(mqtt_username, mqtt_password);

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) 
  {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!\n");
}

void write_mqtt(MqttClient mqttClient, char* topic, char* data)
{
  mqttClient.beginMessage(topic);
  mqttClient.print(data);
  mqttClient.endMessage();
}

void read_mqtt(MqttClient mqttClient)
{
  int messageSize = mqttClient.parseMessage();
  if (messageSize) 
  {
    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");
    
    byte message;
    while (mqttClient.available()) 
    {
       message = mqttClient.read();
       Serial.print((char)message);
       uart_Tx(message);
    }
  }
}

void uart_init()
{
  Serial1.begin(9600);
  Serial1.setPins(Rx, Tx);
}

void uart_Tx(const char data)
{
  Serial1.print(data);
  Serial1.flush();
}

void uart_Tx_string(const char* data)
{
  for(int i = 0; i<strlen(data); i++)
  {
    uart_Tx(data[i]);
  }
}
byte uart_Rx()
{
  byte incomingByte = 0;
  if (Serial1.available() > 0) 
  {
    incomingByte = Serial1.read();
  }

  return incomingByte;

}
