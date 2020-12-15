#include "SparkFunLSM6DS3.h"
#include <math.h>

#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <WiFiEspUdp.h>

#include "Wire.h"
#include "SPI.h"

#define BUTTON_PIN 2
#define SMOKE_PIN A0
#define SMOKE_THRESH 30 // treshold to detect some toxic gas (Lowered for demonstration purposes)
#define SMOKE_POLL  1000 // interval in ms to check the gas sensor
#define SERVER_POLL 1000 // interval in ms to see if the arduino needs to send an alert to the server

// =================== INTERNET INIT START ========================

#include "WiFiEsp.h"
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(10, 11); // RX, TX
#endif
#define CLIENT_TIMEOUT 5000
/*
 * Please note that some of the Wifiesp connectivity code is heavily inspired from
 * The example found here: https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/examples/WiFiClient/WiFiClient.ino
 * And the starter code given to us by Sina :) (THANK YOU SINA)
 */
char ssid[] = ""; // your network SSID (name)
char pass[] = ""; // your network password
int status = WL_IDLE_STATUS; // the Wifi radio's status
char server[] = ""; // server IP address
char var[] = {'-','-','-'}; // button, smoke sensor, and imu sensor readings
char get_request[200];

// Initialize the Ethernet client object
WiFiEspClient client;

/*
 * Tries to connect to the given server given the server
 */
void attemptToConnect(){
  Serial.print("Attempting to connect to: ");
  Serial.println(ssid);
  client.connect(server, CLIENT_TIMEOUT);
  Serial.println("Connected!");
}
// =================== INTERNET INIT END ==========================

// =================== IMU INIT BEGIN  ============================
LSM6DS3 imu(I2C_MODE, 0x6B);

// sample time readings
long prev_time = 0;
long debug_time = 0;

float svm   = 0;
float theta = 0;

float read_x = 0;
float read_y = 0;
float read_z = 0;

float prev_x = 0;
float prev_y = 0;
float prev_z = 0;

// =================== IMU INIT END ===========================

/*
 * Using a fall detecting algorithm found here:
 * https://www.hindawi.com/journals/jam/2014/896030/
 */
float calculateSvm(){
  return sqrt(read_x*read_x + read_y*read_y + read_z*read_z);
}

float calculateAngle(){
  float to_return = atan(sqrt(read_y*read_y + read_z*read_z)/read_x);
  return to_return*(180/M_PI);
}

float calculateDSVM(){
  float x_component = pow(read_x - prev_x, 2);
  float y_component = pow(read_y - prev_y, 2);
  float z_component = pow(read_z - prev_z, 2);
  return sqrt(x_component + y_component + z_component);
  
}
// end fall detecting algorithm

/*
 * Reads from the accelerometer and updates the values 
 */
void updateAccValues(){
  // first, set the previous read values to the current ones
  prev_x = read_x;
  prev_y = read_y;
  prev_z = read_z;

  // then, read from the accelerometer and update the values
  read_x = imu.readFloatAccelX();
  read_y = imu.readFloatAccelY();
  read_z = imu.readFloatAccelZ();
  
}

void setup() {
  /*
   * General setup
   */
  Serial.begin(9600);
  
  /*
   * Button setup
    */
  pinMode(BUTTON_PIN, INPUT);

  /*
   * Accelerometer setup
   */
  Serial.println("Initializing IMU...");
  if(imu.begin()){
    Serial.println("Something went wrong with the IMU.");
    while(1){}
  }
  Serial.println("IMU Initialized!");
  updateAccValues();

  /*
   * Wifi module setup
   */
  // Sinas starting code for the esp8226 wifi module 
  Serial1.begin(115200);
  WiFi.init(&Serial1);
  if(WiFi.status() == WL_NO_SHIELD){
    Serial.println("Cannot find ESP8266");
    while(1){}
  }

  while(status != WL_CONNECTED){
    Serial.print("Connecting to: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
  }
  Serial.println("Connected!");  
  Serial.print("You're connected to: ");
  Serial.println(ssid);

  /* 
   *  Warm up the gas sensor 
   */
  pinMode(SMOKE_PIN, INPUT);
  Serial.println("Warming up gas sensor...");
  delay(5000);  // wait 5 seconds for the gas sensor to get warmed up
  Serial.println("Initting done!");
}

float mq2Reading = 0;
float smoke_time = 0;
float prev_server_check = 0;

char fall_event = '-';
char smoke_event = '-';
char button_event = '-';

void clearEvents(){
  fall_event = '-';
  smoke_event = '-';
  button_event = '-';
}

bool checkEventOccurence(){
  return (fall_event != '-' || smoke_event != '-' || button_event != '-');
}

void loop() {
  // continuously check the inputs for an event

  // first check to see if the button is being pressed
  if(digitalRead(BUTTON_PIN)){
    Serial.println("BUTTON EVENT");
    button_event = 'b';
  }
  // then check to see if the a fall has been detected
  svm = calculateSvm();
  theta = calculateAngle();
  
  if(abs(theta) > 65 and abs(svm) > 2.5){
    Serial.println("FALL EVENT");
    fall_event = 'f';
    Serial.print("Theta: ");
    Serial.println(abs(theta));

    Serial.print("SVM: ");
    Serial.println(svm);
  }
  
  // Then check to see if the mq2 has been checked
  if((millis() - smoke_time) > SMOKE_POLL){
    mq2Reading = analogRead(SMOKE_PIN);
    if(mq2Reading > SMOKE_THRESH){
      Serial.print("SMOKE EVENT: ");
      Serial.println(mq2Reading);
      smoke_event = 's';
    }
    smoke_time = millis();
  }

  
  // Check to see if its time to send information to the server
  
  if((millis() - prev_server_check) > SERVER_POLL){
    prev_server_check = millis();
    if(checkEventOccurence()){
      attemptToConnect();
      var[2] = button_event;
      var[1] = fall_event;
      var[0] = smoke_event;
      sprintf(get_request, "GET /?var=%s HTTP/1.1\r\nHost: 18.221.147.67\r\nConnection: close\r\n\r\n", var);
      client.print(get_request);
      clearEvents();
      //client.print("EVENT");
      //clearEvents();
      delay(1000);
      client.stop(); // close the socket that way we dont run out of sockets :)
      //client.print("PRINTED");         
    }
  }
  
  // update accelerometer values
  updateAccValues();
}
