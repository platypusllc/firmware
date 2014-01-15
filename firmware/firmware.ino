#include <Servo.h> 
#include <adk.h>

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

// Pin definitions
// Left side of Arduino
const int LED_R = 54;
const int LED_G = 55;
const int LED_B = 56;

const int V_BATT = A3;

const int S4_ANALOG = A4;
const int S3_ANALOG = A5;
const int S2_ANALOG = A6;
const int S1_ANALOG = A7;

const int S1_CURRENT = A8;
const int S2_CURRENT = A9;
const int S3_CURRENT = A10;
const int S4_CURRENT = A11;

const int S1_PWR = 66;
const int S2_PWR = 67;
const int S3_PWR = 68;
const int S4_PWR = 69;

const int M2_PWR = 48;
const int M1_PWR = 50;

const int CHG_CTRL = 51;

// Right side of Arduino
const int M2_SERVO = 12;
const int M1_SERVO = 11;

const int S4_B = 9;
const int S4_A = 8;
const int S3_B = 7;
const int S3_A = 6;
const int S2_B = 5;
const int S2_A = 4;
const int S1_B = 3;
const int S1_A = 2;

Servo servo;

void setup() {
  Serial.begin(9600);
  while (!Serial); // TODO: probably not needed for Due
 
  // Charging control
  digitalWrite(CHG_CTRL, LOW);
  pinMode(CHG_CTRL, INPUT);
  digitalWrite(CHG_CTRL, HIGH);
  
  // put your setup code here, to run once:
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  digitalWrite(M2_PWR, LOW);
  pinMode(M1_PWR, OUTPUT);
  digitalWrite(M1_PWR, LOW);
  pinMode(M2_PWR, OUTPUT);
  
  servo.attach(M1_SERVO);
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
