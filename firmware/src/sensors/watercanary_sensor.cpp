/**
 * Airboat Control Firmware - Watercanary
 *
 * Contains control and update code interfacing with the Watercanary sensor
 * The data is directly forwarded over serial.
 */
 
#include <stdio.h>
#include <string.h>

// Define the char code for the Amarino callback
#define RECV_WATERCANARY_FN 'w'

// Stack variables (safe for use in protothreads)
static String wcSample;
static char wcBuffer[5];

void initWaterCanary()  {  
  // Enable serial port
  Serial3.begin(9600);
}

// Calculate the watercanary checksum and return if it is correct or not
boolean wcChecksum(String& fluorescence) {
  
  char checksum = 0;
  char buf[36], cs[2], ccs[2];
  String strcs; 
  
  fluorescence.toCharArray(wcBuffer, 36);
  char *indx = &wcBuffer[1];
  
  for (int i = 1; i < 33; i++, indx++)
       checksum ^= *indx;
       
  sprintf(cs, "%02x",checksum);
  
  strcs = fluorescence.substring(34);
  strcs.toCharArray(ccs, 2);
  
  return (strcmp(cs, ccs));
}

// Wrapper function that will start a sensor reading
void updateWaterCanary() {
  
  // Get bytes from serial buffer 
  while (Serial3.available()) {   
    char c = Serial3.read();
    wcSample += c;
   
    // Parse if we receive end-of-line characters   
    if (c == '\r' || c == '\n' || wcSample.length() > 45)   {       
      if (wcSample.length() == 40)
      {
        amarino.send(RECV_WATERCANARY_FN);
        for (int i = 0; i < 40; i += 5) {
          wcSample.substring(i,i+5).toCharArray(wcBuffer, 5);
          amarino.send(wcBuffer);
        }
        amarino.sendln();       
      }
      wcSample = "";
    } 
  }
}

