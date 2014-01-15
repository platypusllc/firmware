#include "Platypus.h"
#include <adk.h>

// Include necessary Arduino headers to inform the IDE to link these libraries
#include <Servo.h>

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
ADK adk(&Usb, companyName, applicationName, accessoryName,versionNumber, url, serialNumber);

platypus::LED rgb_led;
platypus::VaporPro motor(0);

void setup() {
  Serial.begin(9600);
  while (!Serial); // TODO: probably not needed for Due
   
  // put your setup code here, to run once:

  digitalWrite(board::M2_PWR, LOW);
  pinMode(board::M1_PWR, OUTPUT);
  digitalWrite(board::M1_PWR, LOW);
  pinMode(board::M2_PWR, OUTPUT);
    
  rgb_led.set(1,0,0);
  motor.arm();
  rgb_led.set(1,1,0);
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
  input_buffer[RECEIVE_BUFFER_SIZE] = 0;
  
  /*
  Usb.Task();  
  if (adk.isReady()){
        adk.read(&bytes_read, RECEIVE_BUFFER_SIZE, input_buffer);
        if (bytes_read > 0){
            adk.write(bytes_read, input_buffer);
            Serial.print("RCV: ");
            Serial.println((const char*)input_buffer);
        }    
  } else{

  }
  */
  rgb_led.set(1,0,0);
  delay(200);
  rgb_led.set(0,1,0);
  delay(200);
  rgb_led.set(0,0,1);
  delay(200);
  
  if (motor.velocity() < 1.0) {
    motor.velocity(motor.velocity() + 0.1);
  } else {
    motor.velocity(-1.0);    
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
