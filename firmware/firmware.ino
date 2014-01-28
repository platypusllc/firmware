#include "Platypus.h"
#include <adk.h>

// Arduino headers used in Platypus.h
// (informs the IDE to link these libraries)
#include <Servo.h>

// JSON parsing library
#include "jsmn.h"

// TODO: remove me
#include "Board.h"

// ADK USB Host configuration 
char applicationName[] = "Platypus Server"; // the app on Android
char accessoryName[] = "Platypus Control Board"; // your Arduino board
char companyName[] = "Platypus LLC";
char versionNumber[] = "3.0";
char serialNumber[] = "3";
char url[] = "http://senseplatypus.com";

// ADK USB Host
USBHost Usb;
ADK adk(&Usb, companyName, applicationName, accessoryName, versionNumber, url, serialNumber);

// Android send/receive buffers
const size_t INPUT_BUFFER_SIZE = 512;
char input_buffer[INPUT_BUFFER_SIZE+1];

const size_t OUTPUT_BUFFER_SIZE = 576;
char output_buffer[OUTPUT_BUFFER_SIZE+1];

// JSON parsing buffers
const size_t NUM_JSON_TOKENS = 32;
jsmn_parser json_parser;
jsmntok_t json_tokens[NUM_JSON_TOKENS];

// TODO: move this to platypus header in wrapper object
platypus::LED rgb_led;
platypus::VaporPro motor(0);

void reportJsonError(jsmnerr_t error)
{
  char *error_message;
  
  // Fill in the appropriate JSON error description.
  switch(error) {
    case JSMN_ERROR_NOMEM:
      error_message = "Insufficient memory.";
      break;
    case JSMN_ERROR_INVAL:
      error_message = "Invalid JSON string.";
      break;
    case JSMN_ERROR_PART:
      error_message = "Incomplete JSON string.";
      break;
    default:
      error_message = "Unknown JSON error.";
      break;
  }
  
  // Construct a JSON error message.
  snprintf(output_buffer, OUTPUT_BUFFER_SIZE, 
           "{"
           "\"error\": \"%s\","
           "\"args\": \"%s\""
           "}",
           error_message, input_buffer);
  
  // Report error over USB. 
  adk.write(strnlen(output_buffer, OUTPUT_BUFFER_SIZE), (uint8_t*)output_buffer);
  
  // TODO: if performance issue, remove this line.
  // Report error to debugging console.
  Serial.println(output_buffer);
}

void setup() 
{
  // Initialize debugging serial console.
  Serial.begin(115200);
  
  // Make the ADK buffers into null terminated string.
  input_buffer[INPUT_BUFFER_SIZE] = '\0';
  output_buffer[OUTPUT_BUFFER_SIZE] = '\0';
  
  // Initialize the JSON parser.
  jsmn_init(&json_parser);
}

void loop() 
{
  // Number of bytes received from USB.
  uint32_t bytes_read = 0;
  
  // Signed char pointer to head of buffer.
  char *input_buffer_ptr = (char *)input_buffer;
  
  // Result of JSON parsing.
  jsmnerr_t json_result;

  // Attempt to create USB connection.
  Usb.Task();
  
  // Shutdown system if not connected to USB.
  if (!adk.isReady())
  {
    // Set LED to red.
    rgb_led.set(1,0,0);
    
    // Turn off motors.
    motor.disable();
    
    // Wait for USB connection again.
    return;
  }
  
  // Set LED to green.
  rgb_led.set(0,1,0); 
    
  // Read next command from USB.
  adk.read(&bytes_read, INPUT_BUFFER_SIZE, (uint8_t*)input_buffer);
  if (bytes_read <= 0) return;
  
  // TODO: remove this debug print
  Serial.print("RCV: ");
  Serial.println(input_buffer);
    
  // Parse command as JSON string
  json_result = jsmn_parse(&json_parser, input_buffer, json_tokens, NUM_JSON_TOKENS);
  
  // Check for valid result, report error on failure.
  if (json_result != JSMN_SUCCESS)
  {
    reportJsonError(json_result);
    return;
  }
  
  // TODO: read these inputs and do things.
  
  // Send back response.
  //adk.write(bytes_read, input_buffer);
}
