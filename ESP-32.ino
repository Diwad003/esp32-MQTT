#include <ArduinoMqttClient.h>
#include <WiFiClientSecure.h>
//#include <esp_task_wdt.h>      // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/misc_system_api.html
#include "HardwareSerial.h"
#include "esp_intr_alloc.h"




void setup();
void loop();
void connectToWiFi();
void mqttConnect();
void write_mqtt(MqttClient mqttClient, char* topic, const char* data);
void read_mqtt(MqttClient mqttClient);


void uart_init();
void uart_Tx(const char* data);
char uart_Rx();
void uart_Rx_to_broker(char* messageBuffer);

#define BUFFER_SIZE 8
char messageBuffer[BUFFER_SIZE];
uint8_t uart_Rx_index = 0;

  

char ssid[] = "iPSK-UMU";    // your network SSID (name)
char password[] = "HejHopp!!";    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;


MqttClient mqttClient(wifiClient);
const char broker[] = "tfe.iotwan.se";
int        port     = 1883;
char* send_topic  = "JD/sensor";
char* receive_topic = "JD/joystick"; 
#define mqtt_username "intro23"
#define mqtt_password "outro"


#define Rx 16
#define Tx 17

void setup() 
{
  Serial.begin(38400);
  
  uart_init();
  connectToWiFi();
  mqttConnect();
  mqttClient.subscribe(receive_topic);

  uart_Tx("Setup Done\n\r");
}

void loop()
{
  mqttClient.poll();

  uart_Rx_to_broker(messageBuffer);
  read_mqtt(mqttClient);


  if(WiFi.status() != WL_CONNECTED || !mqttClient.connect(broker, port)) 
  {
    ESP.restart();
  }
}

void connectToWiFi()
{
  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WiFi SSID: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    timeout++;
    if(timeout > 20)
    {
      ESP.restart();
    }
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
    Serial.println("Try again");
    mqttConnect();
  }

  Serial.println("You're connected to the MQTT broker!\n");
}

void write_mqtt(MqttClient mqttClient, char* topic, const char* data)
{
  mqttClient.beginMessage(topic);
  mqttClient.print(data);
  mqttClient.endMessage();

  Serial.println(data);
}

void read_mqtt(MqttClient mqttClient)
{
  int messageSize = mqttClient.parseMessage();
  if (messageSize) 
  {
    char message;
    while (mqttClient.available()) 
    {
      message = mqttClient.read();
      Serial.print(message);
      uart_Tx((char*)message);
    }
  }
}

void uart_init()
{
  Serial1.begin(38400, SERIAL_8N1, Rx, Tx);
}

void uart_Tx(const char* data)
{
  Serial1.print(data);
  Serial1.print('&');
  Serial1.flush();

  Serial.println(data);
}

char uart_Rx()
{
  char incomingByte = 0;
  if (Serial1.available() > 0) 
  {
    incomingByte = Serial1.read();
  }

  return incomingByte;
}

void uart_Rx_to_broker(char* messageBuffer)
{
  char data;
  do
  {
    data = uart_Rx();
    if(data == 0)
    {
      return;
    }
    else if (data == '&')
    {
      messageBuffer[uart_Rx_index] = '&';
      uart_Rx_index++;
      messageBuffer[uart_Rx_index] = '\0';
      uart_Rx_index = 0;
      write_mqtt(mqttClient, send_topic, messageBuffer);
      return;
    }
    else
    {
      messageBuffer[uart_Rx_index] = data;
      uart_Rx_index++;
    }

  } while(uart_Rx_index >= BUFFER_SIZE - 1);
}


