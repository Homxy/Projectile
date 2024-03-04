#include <Arduino.h>
#include <WiFi.h>
#include <MQTT.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <LCD_I2C.h>
#include "math.h"
LCD_I2C lcd(0x27, 16, 2);

#define BUTTON 12 //button to shot servo
//shotservo 18
//degreeservo 19
int potpin = 32;  //potentiometer pin 
int val;
int valA;    
double angle;
int degree;

int state = 15;// mqtt or manual analog
int State_M;
int manual = 2;// Botton or Auto pin 
int Manual;

int trig = 16;// trig ultrasonic pin
int echo = 17;// echo ultrasonic pin


long duration,distance_1;
Servo deServo;
Servo shotServo;


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


void lcd_print(int distance,int degree){
lcd.backlight();
lcd.clear();

lcd.print("degree :");
lcd.print(degree);
lcd.setCursor(0,1);
lcd.print("distance :");
lcd.print(distance);
}

long dis_tance(){
  digitalWrite(echo, LOW);
  delayMicroseconds(2);
  digitalWrite(echo, HIGH);
  delayMicroseconds(5);
  digitalWrite(echo, LOW);
  duration = pulseIn(trig, HIGH);
  return duration/58;
}

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

  if(payload == "200"){//shot
    IO_INT_ISR();
  }
  if(payload == "300"){ //Mode analog
    State_M = false;
  }

  if(payload.toInt() >= 0 && payload.toInt() <= 70){
    val = (payload.toInt()+12);
    Serial.print(val);
  }
}

void setup() {
  pinMode(echo, OUTPUT);
  pinMode(trig, INPUT);
  pinMode(state,INPUT_PULLUP);
  pinMode(manual,INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
  
  Serial.begin(9600);
  lcd.begin();
  lcd.backlight();
  WiFi.begin(ssid, pass);
  
  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin(mqtt_broker, MQTT_PORT, net);
  client.onMessage(messageReceived);

  /* connect(); */

  deServo.attach(19); 
  shotServo.attach(18);
  
  shotServo.write(0);

  attachInterrupt(BUTTON, IO_INT_ISR, FALLING);
  
}

void loop() {
  State_M = digitalRead(state);
  Manual = digitalRead(manual);
  //Serial.println(State);
  distance_1 = dis_tance();
  //Serial.println(distance_1);
  

  client.loop();

  if (!client.connected()) {
    connect();
  }

  valA = analogRead(potpin);// scale it to use it with the servo (value between 0 and 180)
  valA = map(valA, 0, 2047, 12, 70);// reads the value of the potentiometer (value between 0 and 1023)

  if(Manual == true){
  if(State_M  == false){ //push botton down to control by analog
    deServo.write(valA); 
    degree =valA;
  }else{
    deServo.write(val);
    degree =val;
  }    
           
  }else{//Auto code
    angle = asin(distance_1*9.8/(4.12*4.12*2*100));//(0.5);
    angle = (angle*180/3.14);
    Serial.print("angle :");
    Serial.println(angle);
    if(angle < 58){
    deServo.write(angle+12);
    degree = angle;
    }
  }
  lcd_print(distance_1,degree);
  client.publish(mqtt_degree, String(degree));
  Serial.print(Manual);
  Serial.print(" "); 
  Serial.print(State_M);
  Serial.print(" ");  
  Serial.print(valA);
  Serial.print(" ");  
  Serial.println(distance_1);                  // sets the servo position according to the scaled value
  delay(50);  // <- fixes some issues with WiFi stability       
}
