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

// System state enumeration
enum SystemState
{
  /** There is no ADK USB device currently plugged in. */
  DISCONNECTED,
  /** There is an ADK USB device detected, but it is unresponsive. */
  CONNECTED,
  /** There is a Platypus Server currently communicating. */
  RUNNING  
};
SystemState system_state = DISCONNECTED;

// Time betweeen commands before we consider the Android
// server to be unresponsive.
const size_t RESPONSE_TIMEOUT_MS = 500;

// Define the systems on this board
// TODO: move this board.h?
platypus::Led rgb_led;


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
      if (motor_idx >= board::NUM_MOTORS || entry_name[2] != '\0') 
      {
        reportError("Invalid motor index.", buffer);
        return;
      }
      entry_object = platypus::motors[motor_idx];
    }
    // If it is a sensor, it must take the form 's1'.
    else if (entry_name[0] == 's')
    {
      size_t sensor_idx = entry_name[1] - '0';
      if (sensor_idx >= board::NUM_SENSORS || entry_name[2] != '\0') 
      {
        reportError("Invalid sensor index.", buffer);
        return;
      }
      entry_object = platypus::sensors[sensor_idx];
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
  delay(2000);
  
  // Latch power shutdown line high to keep board from turning off.
  pinMode(board::PWR_KILL, OUTPUT);
  digitalWrite(board::PWR_KILL, HIGH);

  // Initialize debugging serial console.
  Serial.begin(115200);

  // Start the system in the disconnected state
  system_state = DISCONNECTED;
  
  // TODO: replace this with smart hooks.
  // Initialize sensors
  platypus::sensors[0] = new platypus::ServoSensor(0);
  platypus::sensors[1] = new platypus::AtlasDO(1);
  platypus::sensors[2] = new platypus::HDS(2);
  platypus::sensors[3] = new platypus::ES2(3);
  
  // Initialize motors
  platypus::motors[0] = new platypus::Dynamite(0);
  platypus::motors[1] = new platypus::Dynamite(1);

  // Make the ADK buffers into null terminated string.
  debug_buffer[INPUT_BUFFER_SIZE] = '\0';
  input_buffer[INPUT_BUFFER_SIZE] = '\0';
  output_buffer[OUTPUT_BUFFER_SIZE] = '\0';

  // Set ADC Precision:
  analogReadResolution(12);
  
  // Create secondary tasks for system.
  Scheduler.startLoop(motorUpdateLoop);
  //Scheduler.startLoop(serialConsoleLoop);
  Scheduler.startLoop(batteryUpdateLoop);

  // Initialize Platypus library.
  platypus::init();
  
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
  // Keep track of how many reads we haven't made so far.
  static unsigned long last_command_time = 0;
  
  // Number of bytes received from USB.
  uint32_t bytes_read = 0;
  
  // Do USB bookkeeping.
  Usb.Task();
  
  // Report system as shutdown if not connected to USB.
  if (!adk.isReady())
  {
    // If not connected to USB, we are 'DISCONNECTED'.
    if (system_state != DISCONNECTED)
    {
      Serial.println("STATE: DISCONNECTED");
      system_state = DISCONNECTED;
    }
    
    // Wait for USB connection again.
    yield();
    return;
  } 
  else 
  {  
    // If connected to USB, we are now 'CONNECTED'!
    if (system_state == DISCONNECTED)
    {
      Serial.println("STATE: CONNECTED");
      system_state = CONNECTED; 
    }
  }
        
  // Attempt to read command from USB.
  adk.read(&bytes_read, INPUT_BUFFER_SIZE, (uint8_t*)input_buffer);
  unsigned long current_command_time = millis();
  if (bytes_read <= 0) 
  {
    // If we haven't received a response in a long time, maybe 
    // we are 'CONNECTED' but the server is not running.
    if (current_command_time - last_command_time >= RESPONSE_TIMEOUT_MS)
    {
      if (system_state == RUNNING)
      {
        Serial.println("STATE: CONNECTED");
        system_state = CONNECTED; 
      }
    }
    
    // Wait for more USB data again.
    yield();
    return;
  } 
  else 
  {
    // If we received a command, the server must be 'RUNNING'.
    if (system_state == CONNECTED) 
    {
      Serial.println("STATE: RUNNING");
      system_state = RUNNING; 
    }
    
    // Update the timestamp of last received command.
    last_command_time = current_command_time;
  }
  
  // Properly null-terminate the buffer.
  input_buffer[bytes_read] = '\0';
  
  // Copy incoming message to debug console.
  Serial.print("<- ");
  Serial.println(input_buffer);
  
  // Attempt to parse command
  handleCommand(input_buffer);
}

void batteryUpdateLoop()
{
  int rawVoltage = analogRead(board::V_BATT);
  double voltageReading = 0.008879*rawVoltage + 0.09791;
  //0.03516*rawVoltage+0.05135;

  char output_str[128];
  snprintf(output_str, 128,
           "{"
           "\"s4\":{"
           "\"type\":\"battery\","
           "\"data\":\"%.3f %f %f\""
           "}"
           "}",
           voltageReading, 
           platypus::motors[0]->velocity(),
           platypus::motors[1]->velocity()
          );
  send(output_str);  
  delay(1000);
}

/**
 * Periodically sends motor velocity updates.
 */
void motorUpdateLoop()
{
  // Wait for a fixed time period.
  delay(100);
  
  // Set the LED for current system state
  switch (system_state)
  {
  case DISCONNECTED:
    // Red blink
    rgb_led.set((millis() >> 8) & 1, 0, 0);
    break;
  case CONNECTED:
    // Green blink
    rgb_led.set(0, (millis() >> 8) & 1, 0);
    break;
  case RUNNING:
    // Solid green
    rgb_led.set(0, 1, 0);
    break;
  }
  
  // Handle the motors appropriately for each system state.
  switch (system_state)
  {
  case DISCONNECTED:
    // Turn off motors.
    for (size_t motor_idx = 0; motor_idx < board::NUM_MOTORS; ++motor_idx) 
    {
      platypus::Motor* motor = platypus::motors[motor_idx];
      if (motor->enabled())
      {
        Serial.print("Disabling motor [");
        Serial.print(motor_idx);
        Serial.println("]");
        motor->disable();
      }
    }
    break;
  case CONNECTED:
    // Decay all motors exponentially towards zero speed.
    for (size_t motor_idx = 0; motor_idx < board::NUM_MOTORS; ++motor_idx) 
    {
      platypus::Motor* motor = platypus::motors[motor_idx];
      motor->velocity(motor->velocity() * 0.8);
    }
    // NOTE: WE DO NOT BREAK OUT OF THE SWITCH HERE!
  case RUNNING:
    // Rearm motors if necessary.
    for (size_t motor_idx = 0; motor_idx < board::NUM_MOTORS; ++motor_idx) 
    {
      platypus::Motor* motor = platypus::motors[motor_idx];
      if (!motor->enabled()) 
      {
        Serial.print("Arming motor [");
        Serial.print(motor_idx);
        Serial.println("]");
        //Serial.println("I've turned off the fucking motor arming");
        motor->arm();
        Serial.println("Motor Armed");
        //motor->enable
      }
    }
    break;
  }
  
  // Send status updates while connected to server.
  if (system_state == RUNNING)
  {
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
      platypus::motors[0]->velocity(), platypus::motors[0]->current(),
      platypus::motors[1]->velocity(), platypus::motors[1]->current()
    );
    send(output_buffer);
  }
}

/**
 * Periodically sends winch position updates.
 */
void winchUpdateLoop()
{
  // Wait for a fixed time period.
  delay(300);
  
  // Send status updates while connected to server.
  if (system_state == RUNNING)
  {  
    // TODO: Remove this hack
    // Send encoder status update over USB
    bool valid = false;
    long pos = ((platypus::Winch*)platypus::sensors[2])->encoder(&valid);
    
    if (valid)
    {
      snprintf(output_buffer, OUTPUT_BUFFER_SIZE,
        "{"
          "\"s2\":{"
            "\"type\":\"winch\","
            "\"depth\":%ld"
          "}"
        "}",
        pos
      );
      send(output_buffer);
    } 
  }
}

/**
 * Reads from serial debugging console and attempts to execute commands.
 */
void serialConsoleLoop()
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

