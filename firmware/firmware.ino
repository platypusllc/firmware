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

// Number of empty reads before we consider the Android
// server to be unresponsive.
const size_t RESPONSE_TIMEOUT = 255;

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
  motor[0] = new platypus::Seaking(0); 
  motor[1] = new platypus::Seaking(1);

  // Initialize debugging serial console.
  Serial.begin(115200);
  
  // Start the system in the disconnected state
  system_state = DISCONNECTED;
  
  // Make the ADK buffers into null terminated string.
  debug_buffer[INPUT_BUFFER_SIZE] = '\0';
  input_buffer[INPUT_BUFFER_SIZE] = '\0';
  output_buffer[OUTPUT_BUFFER_SIZE] = '\0';
  
  // Create secondary tasks for system.
  Scheduler.startLoop(motorUpdateLoop);
  Scheduler.startLoop(serialConsoleLoop);
//  Scheduler.startLoop(winchTestLoop);
//  Scheduler.startLoop(hd5SimLoop);
  
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
  uint8_t response_counter;
  
  // Number of bytes received from USB.
  uint32_t bytes_read = 0;
  
  // Do USB bookkeeping.
  Usb.Task();
  
  // Report system as shutdown if not connected to USB.
  if (!adk.isReady())
  {
    // If not connected to USB, we are 'DISCONNECTED'.
    system_state = DISCONNECTED;
    
    // Wait for USB connection again.
    yield();
    return;
  } 
  else 
  {  
    // If connected to USB, we are now 'CONNECTED'!
    if (system_state == DISCONNECTED)
    {
      system_state = CONNECTED; 
    }
  }
        
  // Attempt to read command from USB.
  adk.read(&bytes_read, INPUT_BUFFER_SIZE, (uint8_t*)input_buffer);
  if (bytes_read <= 0) 
  {
    // If we haven't received a response in a long time, maybe 
    // we are 'CONNECTED' but the server is not running.
    ++response_counter;
    if (response_counter > RESPONSE_TIMEOUT)
    {
      system_state = CONNECTED; 
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
      system_state = RUNNING; 
    }
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
    for (size_t motor_idx = 0; motor_idx < NUM_MOTORS; ++motor_idx) 
    {
      if (motor[motor_idx]->enabled())
      {
        Serial.print("Disabling motor [");
        Serial.print(motor_idx);
        Serial.println("]");
        motor[motor_idx]->disable();
      }
    }
    break;
  case CONNECTED:
    // Decay all motors exponentially towards zero speed.
    for (size_t motor_idx = 0; motor_idx < NUM_MOTORS; ++motor_idx) 
    {
      motor[motor_idx]->velocity(motor[motor_idx]->velocity() * 0.8);
    }
    // NOTE: WE DO NOT BREAK OUT OF THE SWITCH HERE!
  case RUNNING:
    // Rearm motors if necessary.
    for (size_t motor_idx = 0; motor_idx < NUM_MOTORS; ++motor_idx) 
    {
      if (!motor[motor_idx]->enabled()) 
      {
        Serial.print("Arming motor [");
        Serial.print(motor_idx);
        Serial.println("]");
        motor[motor_idx]->arm();
      }
    }
    break;
  }
  
  // Send system status updates while connected to server.
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
      motor[0]->velocity(), motor[0]->current(),
      motor[1]->velocity(), motor[1]->current()
    );
    send(output_buffer);
    
    // TODO: Remove this hack
    // Send encoder status update over USB
    long pos = ((platypus::Winch*)sensor[2])->position();
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

void winchTestLoop()
{ 
  // Test encoders
  Serial.println("SET ENC");
  ((platypus::Winch*)sensor[2])->reset();
  delay(100);
  
  Serial.println("GET ENC");
  long pos = ((platypus::Winch*)sensor[2])->position();
  Serial.println(pos);
  delay(100);
  return;

  // Test PID
  Serial.println("SET PID");
  platypus::Winch::PidCommand pid = {
    platypus::swap(100),
    platypus::swap(0),
    platypus::swap(0),
    platypus::swap(0),
    platypus::swap(0),
    platypus::swap(0),
    platypus::swap(0),
  };
  //((platypus::Winch*)sensor[2])->write(128, 61, (uint8_t*)&pid, sizeof(pid));
  delay(100);

  Serial.println("GET PID");
  bool valid = ((platypus::Winch*)sensor[2])->read(128, 63, (uint8_t*)&pid, sizeof(pid));
  Serial.print("P: ");
  Serial.print(valid);
  Serial.print(":");
  Serial.println(platypus::swap(pid.P));
  return;
  
  // Test position
  Serial.println("SET POS");
  ((platypus::Winch*)sensor[2])->position(10000);
  delay(2000);

  // Test velocity command
  ((platypus::Winch*)sensor[2])->write(128,6,50); // Backwards
  delay(1000);
  ((platypus::Winch*)sensor[2])->write(128,6,64); // STOP
  delay(1000);
  ((platypus::Winch*)sensor[2])->write(128,6,80); // Forwards
  delay(1000);
  ((platypus::Winch*)sensor[2])->write(128,6,64); // STOP
  delay(1000);
}

const unsigned int NUM_HD5_STRINGS = 20;
const char *hd5_strings[] = {
  "$GPAPB,,,,,,,,,,,,,,,N*26",
  "$SDHDG,,,,,*70",
  "$GPRMB,,,,,*70",
  "$GPAPB,,,,,,,,,,,,,,,N*26",
  "$SDHDG,,,,,*70",
  "$GPRMB,,,,,,,,,,,,,,N*04",
  "$SDHDG,,,,,*70",
  "$GPXTE,,,,,N,N*5E",
  "$SDHDG,,,,,*70",
  "$SDDBT,8.3,f,2.5,M,1.3,F*08",
  "$SDHDG,,,,,*70",
  "$SDMTW,21.4,C*03",
  "$SDHDG,,,,,*70",
  "$GPGGA,,,,,,0,00,,,M,,M,,*66",
  "$SDHDG,,,,,*70",
  "$GPGLL,,,,,,V,N*64",
  "$SDHDG,,,,,*70",
  "$GPVTG,,T,,M,,N,,K,N*2C",
  "$SDHDG,,,,,*70",
  "$SDHDG,,,,,*70"
};

static unsigned int hd5_sim_index = 0;
void hd5SimLoop()
{
  char output_str[128+3];
  snprintf(output_str, 128,
    "{"
     "\"s%u\":{"
       "\"type\":\"hdf5\","
       "\"nmea\":\"%s\""
     "}"
    "}",
    1,
    hd5_strings[hd5_sim_index]
  );  
  send(output_str);
  delay(100);
  
  hd5_sim_index++;
  if (hd5_sim_index >= NUM_HD5_STRINGS)
  {
    hd5_sim_index = 0;
  }
}


