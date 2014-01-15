#include "Platypus.h"
#include <Servo.h> 
#include <adk.h>

// TODO: remove me
#include "Board.h"

/** ADK USB Host configuration */
// Accessory descriptor. 
char applicationName[] = "Platypus Server"; // the app on Android
char accessoryName[] = "Platypus Control Board"; // your Arduino board
char companyName[] = "Platypus LLC";
char versionNumber[] = "0.1";
char serialNumber[] = "1";
char url[] = "http://senseplatypus.com";

USBHost Usb;
ADK adk(&Usb, companyName, applicationName, accessoryName,versionNumber,url,serialNumber);

Servo servo;

void setup() {
  Serial.begin(9600);
  while (!Serial); // TODO: probably not needed for Due
 
  // Charging control
  digitalWrite(board::CHG_CTRL, LOW);
  pinMode(board::CHG_CTRL, INPUT);
  digitalWrite(board::CHG_CTRL, HIGH);
  
  // put your setup code here, to run once:

  digitalWrite(board::M2_PWR, LOW);
  pinMode(board::M1_PWR, OUTPUT);
  digitalWrite(board::M1_PWR, LOW);
  pinMode(board::M2_PWR, OUTPUT);
  
  servo.attach(board::M1_SERVO);
  servo.write(90);
  Serial.println("Servo started.");
  
  delay(1000);
  /*
  Serial.println("Motor arming.");
  digitalWrite(M2_PWR, HIGH);
  digitalWrite(M1_PWR, HIGH);
  
  // run motor arming sequence
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, HIGH);

  servo.write(180);
  delay(5500);

  servo.write(0);
  delay(3500);

  servo.write(90);
  delay(8500);
 
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, LOW);
  Serial.println("Motor armed.");
  */
  
  // Enable charging
  //digitalWrite(CHG_CTRL, HIGH);
}

const size_t RECEIVE_BUFFER_SIZE = 512;
unsigned char input_buffer[RECEIVE_BUFFER_SIZE+1];
uint32_t bytes_read = 0;

void loop() {
  Usb.Task();
  input_buffer[RECEIVE_BUFFER_SIZE] = 0;
  
  if (adk.isReady()){
        adk.read(&bytes_read, RECEIVE_BUFFER_SIZE, input_buffer);
        if (bytes_read > 0){
            adk.write(bytes_read, input_buffer);
            Serial.print("RCV: ");
            Serial.println((const char*)input_buffer);
        }    
  } else{

  }
  
/*
  // Test RGB led
  delay(100);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, LOW);
  delay(100);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  delay(100);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, HIGH);
 */
 
 /*
  // Test battery voltage monitoring
  float voltage = analogRead(V_BATT);
  voltage *= (3.3f / 1024.0f) * 11.0f;
  Serial.print("Battery voltage: ");
  Serial.print(voltage);
  Serial.println();
  */
  
  /*
  // run motor at some slow speed
  //delay(1000);
  //servo.write(100);
  */
}
