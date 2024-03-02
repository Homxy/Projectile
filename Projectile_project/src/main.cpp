#include <Arduino.h>
#include <WiFi.h>
#include <MQTT.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>



#define BUTTON 0


int potpin = 32;  
int val;    
int state = 15;
int State;

Servo deServo;
Servo shotServo;


int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

const char ssid[] = "@JumboPlusIoT";
const char pass[] = "abcdefgh";

const char mqtt_broker[]="test.mosquitto.org";
const char mqtt_topic[]="g35/payload";
const char mqtt_cmd[]="g35/cmd";
const char mqtt_degree[]="g35/degree";
const char mqtt_client_id[]="35"; // must change this string to a unique value
int MQTT_PORT=1883;

int counter=0;

WiFiClient net;
MQTTClient client;



unsigned long lastMillis = 0;


void IRAM_ATTR IO_INT_ISR()
{
  static int angle = 0;
  angle = (angle == 0) ? 90 : 0;
  shotServo.write(angle);
}

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect(mqtt_client_id)) {  
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe(mqtt_topic);
  // client.unsubscribe("/hello");
  client.subscribe(mqtt_cmd);
  client.subscribe(mqtt_degree);
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  if(payload == "r"){
    IO_INT_ISR();
  }
  if(payload == "analog"){

  }

  if(payload.toInt() >= 0 && payload.toInt() <= 70){
    deServo.write(payload.toInt()+12);
  }
}

void setup() {
  Serial.begin(9600);
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  WiFi.begin(ssid, pass);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin(mqtt_broker, MQTT_PORT, net);
  client.onMessage(messageReceived);

  connect();

  deServo.attach(19); 
  shotServo.attach(18);
  pinMode(state,INPUT);
  pinMode(BUTTON, INPUT);
  shotServo.write(0);

  attachInterrupt(BUTTON, IO_INT_ISR, RISING);
}

void loop() {
  State = digitalRead(state);
  Serial.println(State);
  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

  val = analogRead(potpin);
  val = map(val, 0, 2047, 12, 70);     // scale it to use it with the servo (value between 0 and 180)
  //Serial.println(val);            // reads the value of the potentiometer (value between 0 and 1023)
  deServo.write(val);                  // sets the servo position according to the scaled value
  delay(15);           
}