#include "Platypus.h"
#include "Components.h"
#include <adk.h>

// Arduino headers used in Platypus.h
// (informs the IDE to link these libraries)
#include <Servo.h>
#include <Scheduler.h>

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
char debug_buffer[INPUT_BUFFER_SIZE+1];

const size_t OUTPUT_BUFFER_SIZE = 576;
char output_buffer[OUTPUT_BUFFER_SIZE+3];

// Define the systems on this board
// TODO: move this board.h?
platypus::Led rgb_led;

const size_t NUM_MOTORS = 2;
platypus::Motor *motor[NUM_MOTORS];

const size_t NUM_SENSORS = 4;
platypus::Sensor *sensor[NUM_SENSORS];

/**
 * Wrapper for ADK send command that copies data to debug port.
 * Requires a null-terminated char* pointer.
 */
void send(char *str) 
{ 
  // Add newline termination
  // TODO: Make sure we don't buffer overflow
  size_t len = strlen(str);
  str[len++] = '\r';
  str[len++] = '\n';
  str[len] = '\0';
  
  // Write string to USB.
  if (adk.isReady()) adk.write(len, (uint8_t*)str);
  
  // Copy string to debugging console.
  Serial.print("-> ");
  Serial.print(str);
}

/**
 * Returns a JSON error message to the connected USB device and the
 * serial debugging console.
 */
void reportError(const char *error_message, const char *buffer)
{
  // Construct a JSON error message.
  snprintf(output_buffer, OUTPUT_BUFFER_SIZE, 
           "{"
           "\"error\": \"%s\","
           "\"args\": \"%s\""
           "}",
           error_message, buffer);
  send(output_buffer);
}

/**
 * Constructs a JSON error message related to the parsing of a JSON string.
 */
void reportJsonError(jsmnerr_t error, const char* buffer)
{
  char *error_message;
  
  // Fill in the appropriate JSON error description.
  switch(error) {
    case JSMN_SUCCESS:
      // If we were successful, there is nothing to report.
      return;
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

  // Send appropriate error message
  reportError(error_message, buffer);  
}

/**
 * Copies a JSON string token into a provided char* buffer.
 */
void json_string(const jsmntok_t &token, const char *json_str, char *output_str, size_t output_len)
{
  size_t len = min(token.end - token.start, output_len - 1);
  strncpy(output_str, &json_str[token.start], len);
  output_str[len] = '\0';
}

/**
 * Handler to respond to incoming JSON commands and dispatch them to
 * configurable hardware components.
 */
void handleCommand(const char *buffer)
{
  // JSON parser structure
  jsmn_parser json_parser;
  
  // JSON token buffer
  const size_t NUM_JSON_TOKENS = 64;
  jsmntok_t json_tokens[NUM_JSON_TOKENS];
  
  // Result of JSON parsing.
  jsmnerr_t json_result;

  // Initialize the JSON parser.
  jsmn_init(&json_parser);
  
  // Parse command as JSON string
  json_result = jsmn_parse(&json_parser, buffer, json_tokens, NUM_JSON_TOKENS);
  
  // Check for valid result, report error on failure.
  if (json_result != JSMN_SUCCESS)
  {
    reportJsonError(json_result, buffer);
    return;
  }
  
  // Get the first token and make sure it is a JSON object.
  jsmntok_t *token = json_tokens;
  if (token->type != JSMN_OBJECT) 
  {
    reportError("Commands must be JSON objects.", buffer);
    return;
  }
  
  // There should always be an even number of key-value pairs
  if (token->size & 1) {
    reportError("Command entries must be key-value pairs.", buffer);
    return;      
  }
  
  // Read each field of the JSON object and act accordingly.
  size_t num_entries = token->size / 2;
  for (size_t entry_idx = 0; entry_idx < num_entries; entry_idx++)
  {
    // Get the name field for this entry.
    token++;
    char entry_name[64];
    if (token->type != JSMN_STRING)
    {
      reportError("Expected name field for entry.", buffer);
      return;
    }
    json_string(*token, buffer, entry_name, 64);
    
    // Attempt to decode the configurable object for this entry.
    platypus::Configurable *entry_object;
    
    // If it is a motor, it must take the form 'm1'.
    if (entry_name[0] == 'm')
    {
      size_t motor_idx = entry_name[1] - '0';
      if (motor_idx >= NUM_MOTORS || entry_name[2] != '\0') 
      {
        reportError("Invalid motor index.", buffer);
        return;
      }
      entry_object = motor[motor_idx];
    }
    // If it is a motor, it must take the form 's1'.
    else if (entry_name[0] == 's')
    {
      size_t sensor_idx = entry_name[1] - '0';
      if (sensor_idx >= NUM_SENSORS || entry_name[2] != '\0') 
      {
        reportError("Invalid sensor index.", buffer);
        return;
      }
      entry_object = sensor[sensor_idx];
    }
    // Report parse error if unable to identify this entry.
    else {
      reportError("Unknown command entry.", buffer);
      return;
    }
    
    // The following token should always be the entry object.
    token++;
    if (token->type != JSMN_OBJECT) {
      reportError("Command entries must be objects.", buffer);
      return;      
    }

    // There should always be an even number of key-value pairs
    if (token->size & 1) {
      reportError("Command parameters must be key-value pairs.", buffer);
      return;      
    }
    
    // Iterate through each parameter.
    size_t num_params = token->size / 2;
    for (size_t param_idx = 0; param_idx < num_params; param_idx++)
    {
      // Get the name field for this parameter.
      token++;
      char param_name[64];
      if (token->type != JSMN_STRING)
      {
        reportError("Expected name field for parameter.", buffer);
        return;
      }
      json_string(*token, buffer, param_name, 64);
      
      // Get the value field for this parameter.
      token++;
      char param_value[64];
      json_string(*token, buffer, param_value, 64);
      
      // Pass this parameter to the entry object.
      if (!entry_object->set(param_name, param_value)) {
        reportError("Invalid parameter set.", buffer);
        return;
      }
    }
  } 
}

void setup() 
{
  // Latch power shutdown line high to keep board from turning off.
  pinMode(board::PWR_KILL, OUTPUT);
  digitalWrite(board::PWR_KILL, HIGH);
  
  // Initialize sensors
  sensor[0] = new platypus::AnalogSensor(0);
  sensor[1] = new platypus::Hdf5(1);
  sensor[2] = new platypus::Winch(2);
  sensor[3] = new platypus::AnalogSensor(3);
  
  // Initialize motors
  motor[0] = new platypus::HobbyKingBoat(0); 
  motor[1] = new platypus::HobbyKingBoat(1);

  // Initialize debugging serial console.
  Serial.begin(115200);
  
  // Make the ADK buffers into null terminated string.
  debug_buffer[INPUT_BUFFER_SIZE] = '\0';
  input_buffer[INPUT_BUFFER_SIZE] = '\0';
  output_buffer[OUTPUT_BUFFER_SIZE] = '\0';
  
  // Create secondary tasks for system.
//  Scheduler.startLoop(motorDecayLoop);
  Scheduler.startLoop(serialDebugLoop);
  Scheduler.startLoop(testLoop);
  
  // Print header indicating that board successfully initialized
  Serial.println("------------------------------");
  Serial.println(companyName);
  Serial.println(url);
  Serial.println(accessoryName);
  Serial.println(versionNumber);
  Serial.println("------------------------------");
  
  // Turn LED off
  // TODO: Investigate how this gets turned on in the first place
  rgb_led.set(0, 0, 0);
  delay(1000);
}

void loop() 
{
  // Number of bytes received from USB.
  uint32_t bytes_read = 0;
  
  yield();
  return;
  
  Usb.Task();
  
  // Shutdown system if not connected to USB.
  if (!adk.isReady())
  {
    // Set LED to red when USB is not connected.
    rgb_led.R(1);
    rgb_led.G(0);
    
    // Turn off motors.
    for (size_t motor_idx = 0; motor_idx < NUM_MOTORS; ++motor_idx) {
      motor[motor_idx]->disable();
    }
    
    // Wait for USB connection again.
    yield();
    return;
  }
  
  // Rearm motors if necessary
  for (size_t motor_idx = 0; motor_idx < NUM_MOTORS; ++motor_idx) {
    if (!motor[motor_idx]->enabled()) {
      Serial.print("Arming motor ");
      Serial.println(motor_idx);
      
      motor[motor_idx]->arm();
    }
  }
  
  // Set LED to green when USB is connected.
  rgb_led.R(0);
  rgb_led.G(1);
    
  // Read next command from USB.
  adk.read(&bytes_read, INPUT_BUFFER_SIZE, (uint8_t*)input_buffer);
  if (bytes_read <= 0) 
  {
    yield();
    return;
  }
  
  // Properly null-terminate the buffer.
  input_buffer[bytes_read] = '\0';
  
  // Copy incoming message to debug console.
  Serial.print("<- ");
  Serial.println(input_buffer);
    
  // Attempt to parse command
  handleCommand(input_buffer);
}

/**
 * Decays motor velocity exponentially in case of communications loss.
 */
void motorDecayLoop()
{
  // Only run while connected via USB.
  if (!adk.isReady()) {
    yield();
    return;
  }
  
  // Wait for a fixed time period.
  delay(100);
  
  // Decay all motors exponentially towards zero speed.
  for (size_t motor_idx = 0; motor_idx < NUM_MOTORS; ++motor_idx) {
    motor[motor_idx]->velocity(motor[motor_idx]->velocity() * 0.9);
  }
  
  // TODO: move this to another location (e.g. Motor)
  // Send motor status update over USB
  snprintf(output_buffer, OUTPUT_BUFFER_SIZE,
    "{"
      "\"m0\":{"
        "\"v\":%f,"
        "\"c\":%f"
      "},"
      "\"m1\":{"
        "\"v\":%f,"
        "\"c\":%f"
      "}"
    "}",
    motor[0]->velocity(), motor[0]->current(),
    motor[1]->velocity(), motor[1]->current()
  );
  send(output_buffer);
}

/**
 * Reads from serial debugging console and attempts to execute commands.
 */
void serialDebugLoop()
{
  // Index to last character in debug buffer.
  static size_t debug_buffer_idx = 0;
  
  // Wait until characters are received.
  while (!Serial.available()) yield();

  // Put the new character into the buffer.  
  char c = Serial.read();
  debug_buffer[debug_buffer_idx++] = c;

  // If it is the end of a line, or we are out of space, parse the buffer.
  if (debug_buffer_idx >= INPUT_BUFFER_SIZE || c == '\n' || c == '\r') 
  {
    // Properly null-terminate the buffer.
    debug_buffer[debug_buffer_idx] = '\0';
    debug_buffer_idx = 0;
    
    // Echo incoming message on debug console.
    Serial.print("## ");
    Serial.println(debug_buffer);
    
    // Attempt to parse command.
    handleCommand(debug_buffer); 
  }
}

void testLoop()
{
  // Attempt to create USB connection.
  rgb_led.R(1);
  rgb_led.G(0);
  ((platypus::Winch*)sensor[2])->send(128,6,96);
  delay(1000);
  rgb_led.R(0);
  rgb_led.G(1);
  ((platypus::Winch*)sensor[2])->send(128,6,32);
  delay(1000);
  rgb_led.R(1);
  rgb_led.G(1);
  ((platypus::Winch*)sensor[2])->send(128,6,64);
  delay(1000); 
}


