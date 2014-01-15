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
char versionNumber[] = "3.0";
char serialNumber[] = "3";
char url[] = "http://senseplatypus.com";

USBHost Usb;
ADK adk(&Usb, companyName, applicationName, accessoryName,versionNumber, url, serialNumber);

// Android receive buffer
const size_t RECEIVE_BUFFER_SIZE = 512;
unsigned char input_buffer[RECEIVE_BUFFER_SIZE+1];

// TODO: move this to platypus header in wrapper object
platypus::LED rgb_led;
platypus::VaporPro motor(0);

void setup() {
  Serial.begin(9600);
  while (!Serial); // TODO: probably not needed for Due
  
  // Make the ADK buffer a null terminated string  
  input_buffer[RECEIVE_BUFFER_SIZE] = 0;
}

void loop() {
  uint32_t bytes_read = 0;

  Usb.Task();
  if (adk.isReady()) {
    rgb_led.set(0,1,0);
    
    adk.read(&bytes_read, RECEIVE_BUFFER_SIZE, input_buffer);
    if (bytes_read > 0) {
      adk.write(bytes_read, input_buffer);
      Serial.print("RCV: ");
      Serial.println((const char*)input_buffer);
    }    
  } else{
    rgb_led.set(1,0,0);
  }
}
