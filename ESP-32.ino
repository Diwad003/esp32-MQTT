#include <ArduinoMqttClient.h>
#include <WiFiClientSecure.h>
//#include <esp_task_wdt.h>      // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/misc_system_api.html
#include "HardwareSerial.h"
#include "esp_intr_alloc.h"




void setup();
void loop();
void connectToWiFi();
void mqttConnect();
void write_mqtt(MqttClient mqttClient, const char* topic, char* data);
void onMqttMessage(int messageSize);


void uart_init();
void uart_Tx(const char data);
char uart_Rx();
void uart_Rx_to_Tx_broker(char* messageBuffer);

#define BUFFER_SIZE 8
char messageBuffer[BUFFER_SIZE];
uint8_t uart_Rx_index = 0;

void destroy(char* buffer);

  

char ssid[] = "iPSK-UMU";    // your network SSID (name)
char password[] = "HejHopp!!";    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;


MqttClient mqttClient(wifiClient);
const char broker[] = "tfe.iotwan.se";
int        port     = 1883;
const char send_topic[]  = "JD/sensor";
const char receive_topic[] = "JD/joystick"; 
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

  //uart_Tx("F:N:N\n\r");
}

void loop()
{
  if (!mqttClient.connected())
  {
    mqttConnect();
  }
  else if (WiFi.status() != WL_CONNECTED)
  {
    ESP.restart();
  }
  uart_Rx_to_Tx_broker(messageBuffer);
  mqttClient.poll();
}

void onMqttMessage(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.println("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) 
  {
    //Serial.print((char)mqttClient.read());
    uart_Tx((char)mqttClient.read());
  }
  Serial.println();

  Serial.println();
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

  mqttClient.onMessage(onMqttMessage); 
  mqttClient.subscribe(receive_topic);
}

void write_mqtt(MqttClient mqttClient, const char* topic, char* data)
{
  mqttClient.beginMessage(topic);
  mqttClient.print(data);
  mqttClient.endMessage();

  Serial.println(data);
}

void uart_init()
{
  Serial2.begin(38400, SERIAL_8N1, Rx, Tx);
}

void uart_Tx(const char data)
{
  Serial2.print(data);
  Serial2.flush();

  Serial.print(data);
}

char uart_Rx()
{
  char incomingByte = 0;
  if (Serial2.available() > 0) 
  {
    incomingByte = Serial2.read();
  }

  return incomingByte;
}

void uart_Rx_to_Tx_broker(char* messageBuffer)
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

void destroy(char* buffer)
{
  for(int i = 0; i<strlen(buffer); i++)
  {
    buffer[i] = ' ';
  }
}
