#include <Arduino.h>
#include "SPIFFS.h"
#include <WiFiClientSecure.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "utility.h"
#include "device/device.h"
#include "device/device_web_server.h"
#include "device/device_global_variables.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "comms/i2c/I2CCommanderMaster.h"


void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

AsyncWebServer ws_server(80);
AsyncWebSocket ws("/ws");


#define TARGET_I2C_ADDRESS 0x60
#include <ESPmDNS.h>
#include <Smartcar.h>
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_Freq 100000UL


// our RosmoESC's address...
//#define TARGET_I2C_ADDRESS 0x60


// global instance of commander - controller device version
I2CCommanderMaster commander;

void setup() {
    	// slow start - give RosmoESC a chance to boot up, serial to connect, etc...
    	delay(1000);
	 // this is a debug setup so initialize and wait for serial connection
	Serial.begin(115200);
	while (!Serial);
	esp32_setup_peripherals();
	ws.onEvent(onWsEvent);
	ws_server.addHandler(&ws);
	esp32_setup_web_server(&ws_server);


 while (!Wire.begin(I2C_SDA, I2C_SCL, 100000ul))  {    // standard wire pins 
       Serial.println("I2C failed to initialize");
        delay(1000);
    }
    commander.addI2CMotors(TARGET_I2C_ADDRESS, 0); // only one motor in my test setup - *Need to add a second motor
    commander.addI2CMotors(TARGET_I2C_ADDRESS, 1); // Not sure if I've done this correctly
    commander.init();
    Serial.println("I2C Commander intialized.");
}

//Original spec: 
//    messages are always 3 bytes long;
//    every message starts with a value 97 - this is a start code of the message. 97 is ASCII 'a' I was just testing it with UART console, that's why I chose this value;
//    the second byte represents the right joystick. If it is in the center we send 97 if it is up 98 ('b') and 99 ('c') if it is down;
//    the third byte represents the left joystick. If it is in the center we send 97, if it is left 98 ('b') and 99 ('c') if it is right.
//    If we loose the websocket connection we command robot to stop.




void setSpeeds(float speed1, float speed2) {
  if (commander.writeRegister(0, REG_TARGET, &speed1, 4)!=4) { // 0 is the motor number
	Serial.println("Error Motor 0");
  }
  if (commander.writeRegister(1, REG_TARGET, &speed2, 4)!=4) { // 1 is the motor number
  	Serial.println("Error Motor 1");
  }
}


//I2C version:
uint8_t status[4];


void loop()
{
  delay(100);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  String payload_string = (const char *)data;

  if (type == WS_EVT_CONNECT)
  {
    Serial.println("Websocket client connection received");
    char json_message[50] = "{\"hostName\":\"";
    int32_t index = 0;
    const char* ssid = get_ssid();
    while (ssid[index])
    {
      json_message[index + 13] = ssid[index];
      index += 1;
    }
    json_message[index + 13] = '\"';
    json_message[index + 14] = '}';
    Serial.println(json_message);
    ws.textAll(json_message);
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    Serial.println("Client disconnected");
    setSpeeds(0.0f, 0.0f);
  }
  else if (type == WS_EVT_DATA) // receive text from client
  {
    char direction_1 = payload_string[2];
    char direction_2 = payload_string[0];

    if (direction_1 == 'U') //Forwards
    {
	setSpeeds(1.0f, 1.0f);
    }
    else if (direction_1 == 'D') //Backwards
    {
	setSpeeds(-1.0f, -1.0f);
    }
    else
    {
    	setSpeeds(0.0f, 0.0f);
    }

    if (direction_2 == 'L') //left
    {
	setSpeeds(-1.0f, 1.0f);  
    }
    else if (direction_2 == 'R') //right
    {
	setSpeeds(1.0f, -1.0f);
    }
    else
    {
	setSpeeds(0.0f, 0.0f);
    }

    Serial.println("direction 1: " + String(direction_1) + " direction 2: " + String(direction_2));
  }
}
