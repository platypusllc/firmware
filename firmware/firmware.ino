#include "Platypus.h"
#include "Components.h"
#include <adk.h>

// Arduino headers used in Platypus.h
// (informs the IDE to link these libraries)
#include <Servo.h>
#include <Scheduler.h>

// JSON parsing library
#include <ArduinoJson.h>

// TODO: remove me
#include "Board.h"

// ADK USB Host configuration
char applicationName[] = "Platypus Server"; // the app on Android
char accessoryName[] = "Platypus Control Board"; // your Arduino board
char companyName[] = "Platypus LLC";
char versionNumber[] = "3.0";
char serialNumber[] = "3";
char url[] = "http://senseplatypus.com";

// pointer to RC
platypus::RC * pRC = NULL;

// ADK USB Host
USBHost Usb;
ADK adk(&Usb, companyName, applicationName, accessoryName, versionNumber, url, serialNumber);

// Android send/receive buffers
const size_t INPUT_BUFFER_SIZE = 512;
char input_buffer[INPUT_BUFFER_SIZE + 1];
char debug_buffer[INPUT_BUFFER_SIZE + 1];

const size_t OUTPUT_BUFFER_SIZE = 576;
char output_buffer[OUTPUT_BUFFER_SIZE + 3];

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

// Time to wait before dropping into DISCONNECTED state when USB cable is disconnected
// Deals with dodgy USB connections and improves USB C support for all cables
const size_t CONNECTION_TIMEOUT_MS = 500;

// Define the systems on this board
// TODO: move this board.h?
platypus::Led rgb_led;

void RC_listener()
{
  //Serial.println("RC_listener loop...");
  if (pRC != NULL)
  {
    pRC->update();
    delay(100);
    if (pRC->isOverrideEnabled())
    {
      if(!platypus::motors[0]->enabled()) platypus::motors[0]->enable();
      if(!platypus::motors[1]->enabled()) platypus::motors[1]->enable();
      pRC->motorSignals(); // set motor velocities
    }
  }
  yield();
}

/**
   Wrapper for ADK send command that copies data to debug port.
   Requires a null-terminated char* pointer.
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
  if (str[2] != 'm') // don't print motor commands
  {
    Serial.print("-> ");
    Serial.print(str);
  }
}

/**
   Returns a JSON error message to the connected USB device and the
   serial debugging console.
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
   Handler to respond to incoming JSON commands and dispatch them to
   configurable hardware components.
*/
void handleCommand(char *buffer)
{
  // Allocate buffer for JSON parsing
  StaticJsonBuffer<200> jsonBuffer;

  // Attempt to parse JSON in buffer
  JsonObject& command = jsonBuffer.parseObject(buffer);

  // Check for parsing error
  if (!command.success())
  {
    // Parsing Failure
    reportError("Failed to parse JSON command.", buffer);
    return;
  }

  for (JsonObject::iterator it = command.begin(); it != command.end(); ++it)
  {
    const char * key = it->key;

    platypus::Configurable * target_object;
    size_t object_index;

    // Determine target object
    switch (key[0]){
    case 'm': // Motor command
      object_index = key[1] - '0';

      if (object_index >= board::NUM_MOTORS){
        reportError("Invalid motor index.", buffer);
        return;
      }

      if (pRC != NULL)
      {
        if (pRC->isOverrideEnabled()) continue; // ignore motor signals from the phone if RC is enabled
      }

      target_object = platypus::motors[object_index];
      break;
      
    case 's': // Sensor command
      object_index = key[1] - '0';

      if (object_index >= board::NUM_SENSORS){
        reportError("Invalid sensor index.", buffer);
        return;
      }

      if (object_index == 0) // the airboat ServoSensor
      {
        if (pRC != NULL)
        {
          if (pRC->isOverrideEnabled()) continue; // ignore servo signals from the phone if RC is enabled
        } 
      }     

      target_object = platypus::sensors[object_index];
      break;
    
    case 'i': // Sensor instantiation command
      object_index = key[1] - '0';
      Serial.print("Sensor instantiation command for S"); Serial.print(object_index); 
      Serial.print(".  Current name = "); Serial.println(platypus::sensors[object_index]->name());

      if (strcmp(platypus::sensors[object_index]->name(), "dummy") != 0)
      {
        Serial.println("   Not a dummy sensor. Freeing the memory.");
        if ((strcmp(platypus::sensors[object_index]->name(), "RC_SBUS") == 0)
           ||
           (strcmp(platypus::sensors[object_index]->name(), "RC_PWM") == 0))
        {
          pRC = NULL;
        }
        delete platypus::sensors[object_index]; // free the memory      
        platypus::sensors[object_index] = &(platypus::Sensor::dummy()); // temporarily point to the dummy sensor again
        platypus::SerialHandler_t handler = {platypus::Sensor::onSerialDummy_, platypus::sensors[object_index]};
        platypus::SERIAL_HANDLERS[object_index] = handler;
      }      
      { // enclosing scope for new objects in switch-case
        const char * sensor_type = it->value;
        if (strcmp(sensor_type, "AtlasDO") == 0)
        {
          platypus::sensors[object_index] = new platypus::AtlasDO(object_index);
          Serial.print("    S"); Serial.print(object_index); Serial.println(" is now AtlasDO");
        }
        else if (strcmp(sensor_type, "AtlasPH") == 0)
        {
          platypus::sensors[object_index] = new platypus::AtlasPH(object_index);
          Serial.print("    S"); Serial.print(object_index); Serial.println(" is now AtlasPH");
        }
        else if (strcmp(sensor_type, "ES2") == 0)
        {
          platypus::sensors[object_index] = new platypus::ES2(object_index);
          Serial.print("    S"); Serial.print(object_index); Serial.println(" is now ES2");
        }        
        else if (strcmp(sensor_type, "GY26Compass") == 0)
        {
          platypus::sensors[object_index] = new platypus::GY26Compass(object_index);
          Serial.print("    S"); Serial.print(object_index); Serial.println(" is now GY26Compass");
        }  
        else if (strcmp(sensor_type, "HDS") == 0)
        {
          platypus::sensors[object_index] = new platypus::HDS(object_index);
          Serial.print("    S"); Serial.print(object_index); Serial.println(" is now HDS");
        }  
        else if (strcmp(sensor_type, "RC_SBUS") == 0)
        {
          platypus::RC_SBUS * ptemp = new platypus::RC_SBUS(object_index);
          pRC = ptemp;          
          platypus::sensors[object_index] = ptemp;          
          Serial.print("    S"); Serial.print(object_index); Serial.println(" is now RC_SBUS");
        }
        else if (strcmp(sensor_type, "RC_PWM") == 0)
        {
          platypus::RC_PWM * ptemp = new platypus::RC_PWM(object_index);
          pRC = ptemp;          
          platypus::sensors[object_index] = ptemp;
          Serial.print("    S"); Serial.print(object_index); Serial.println(" is now RC_PWM");
        }
        else if (strcmp(sensor_type, "Sampler") == 0)
        {
          platypus::sensors[object_index] = new platypus::JSONPassThrough(object_index);
          Serial.print("    S"); Serial.print(object_index); Serial.println(" is now Sampler");
        }
      }
      continue;    

    case 't': // vehicle type command
      { // enclosing scope for new objects in switch-case
        const char * type = it->value;
        if (strcmp(type, "Prop") == 0)
        {
          rc::vehicle_type = rc::VehicleType::PROP;
          Serial.println("Changed vehicle type to propboat");
        }
        else if (strcmp(type, "Air") == 0)
        {
          rc::vehicle_type = rc::VehicleType::AIR;
          Serial.println("Changed vehicle type to airboat");
        }    
        else
        {
          Serial.print("WARNING: unknown vehicle type: "); Serial.println(type); 
        }
      }
      continue;

    default: // Unrecognized target
      reportError("Unknown command target.", buffer);
      //return; // needs to be continue so we don't throw away a JSON that may have other valid commands in it
      continue;
    }

    // Extract JsonObject with param:value pairs
    JsonObject& params = it->value;

    // TODO: Move this parsing to specific components and pass ref to params instead
    // Iterate over and set parameter:value pairs on target object
    for (JsonObject::iterator paramIt = params.begin(); paramIt != params.end(); ++paramIt)
    {
      const char * param_name = paramIt->key;
      const char * param_value = paramIt->value;

      /* Debugging Output
        Serial.print("Sending command to ");
        Serial.print(key);
        Serial.print(": ");
        Serial.print(param_name);
        Serial.print(" : ");
        Serial.println(param_value);
      */

      if (!target_object->set(param_name, param_value)) {
        reportError("Invalid parameter set.", buffer);
        continue; // Todo: Should we return or continue?
      }
    }
  }
}

void setup()
{
  delay(1000);

  // Latch power shutdown line high to keep board from turning off.
  pinMode(board::PWR_KILL, OUTPUT);
  digitalWrite(board::PWR_KILL, HIGH);

  // Initialize debugging serial console.
  Serial.begin(115200);

  // Start the system in the disconnected state
  system_state = DISCONNECTED;
    
  // Initialize sensors
  platypus::sensors[0] = new platypus::ServoSensor(0);
  for (int i = 1; i < 4; i++)
  {
    platypus::sensors[i] = &(platypus::Sensor::dummy());
  }
  // RC_SBUS on s2 by default (for running the boat without a phone)
  platypus::RC_SBUS * ptemp = new platypus::RC_SBUS(2);
  pRC = ptemp;          
  platypus::sensors[2] = ptemp;

  // Initialize motors
  platypus::motors[0] = new platypus::Dynamite(0);
  platypus::motors[1] = new platypus::Dynamite(1);

  // Power all peripherals
  platypus::motors[0]->enablePower(true);
  platypus::motors[1]->enablePower(true);

  // Make the ADK buffers into null terminated string.
  debug_buffer[INPUT_BUFFER_SIZE] = '\0';
  input_buffer[INPUT_BUFFER_SIZE] = '\0';
  output_buffer[OUTPUT_BUFFER_SIZE] = '\0';

  // Set ADC Precision:
  analogReadResolution(12);

  // Create secondary tasks for system.
  Scheduler.startLoop(motorUpdateLoop);
  Scheduler.startLoop(serialConsoleLoop);
  Scheduler.startLoop(batteryUpdateLoop);
  Scheduler.startLoop(RC_listener);

  // Initialize Platypus library.
  platypus::init();

  // Print header indicating that board successfully initialized
  Serial.println(F("------------------------------"));
  Serial.println(companyName);
  Serial.println(url);
  Serial.println(accessoryName);
  Serial.println(versionNumber);
  Serial.println(F("------------------------------"));
  
  // Turn LED off
  // TODO: Investigate how this gets turned on in the first place
  rgb_led.set(0, 0, 0);
  delay(1000);
}

void loop()
{
  // Keep track of how many reads we haven't made so far.
  static unsigned long last_command_time = 0;

  // Keep track of last time USB connection was detected
  static unsigned long last_usb_connection_time = 0;

  // Number of bytes received from USB.
  uint32_t bytes_read = 0;

  // Do USB bookkeeping.
  Usb.Task();

  // Report system as shutdown if not connected to USB.
  if (!adk.isReady())
  {
    unsigned long current_time = millis();
    // If not connected to USB, we are 'DISCONNECTED'.
    if (system_state != DISCONNECTED)
    {
      Serial.println("WARNING: USB connection state fault");
      if (current_time - last_usb_connection_time >= CONNECTION_TIMEOUT_MS) {
        Serial.println("STATE: DISCONNECTED");
        system_state = DISCONNECTED;
      }
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
      last_usb_connection_time = millis();
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
    last_usb_connection_time = current_command_time;
  }

  // Properly null-terminate the buffer.
  input_buffer[bytes_read] = '\0';

  // Copy incoming message to debug console.
  if (!(input_buffer[12]=='}' && input_buffer[2]=='m')) // don't print pure zero motor commands
  {
    Serial.print("<- ");
    Serial.println(input_buffer);
  }

  // Attempt to parse command
  handleCommand(input_buffer);
}

void batteryUpdateLoop()
{
  int rawVoltage = analogRead(board::V_BATT);
  double voltageReading = 0.008879 * rawVoltage + 0.09791;

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
  yield();
}

/**
   Periodically sends motor velocity updates.
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
      bool should_disable = true;
      if (pRC != NULL)
      {
        if (pRC->isOverrideEnabled())
        {
          should_disable = false;
        }
      }
      if (should_disable)
      {
        if (motor->enabled())
        {
          Serial.print("Disabling motor "); Serial.println(motor_idx);
          motor->disable();
        }
      }
    }
    break;
  case CONNECTED:
      // Decay all motors exponentially towards zero speed.
      for (size_t motor_idx = 0; motor_idx < board::NUM_MOTORS; ++motor_idx)
      {
        platypus::Motor* motor = platypus::motors[motor_idx];
        motor->set("v", "0.0");
      }
    // NOTE: WE DO NOT BREAK OUT OF THE SWITCH HERE!
  case RUNNING:
      // Rearm motors if necessary.
      for (size_t motor_idx = 0; motor_idx < board::NUM_MOTORS; ++motor_idx)
      {
        platypus::Motor* motor = platypus::motors[motor_idx];
        if (!motor->enabled())
        {
          Serial.print("Arming motor "); Serial.print(motor_idx);
          motor->arm();
          Serial.println(F("Motor Armed"));
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
   Periodically sends winch position updates.
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
  yield();
}

/**
   Reads from serial debugging console and attempts to execute commands.
*/
void serialConsoleLoop()
{
  // Index to last character in debug buffer.
  static size_t debug_buffer_idx = 0;

  // Wait until characters are received.
  while (!Serial.available()) yield();

  // Put the new character into the buffer, ignore \n and \r
  char c = Serial.read();
  if (c != '\n' && c != '\r') {
    debug_buffer[debug_buffer_idx++] = c;
  }

  // If it is the end of a line, or we are out of space, parse the buffer.
  if (debug_buffer_idx >= INPUT_BUFFER_SIZE || c == '\n' || c == '\r')
  {
    // Properly null-terminate the buffer.
    debug_buffer[debug_buffer_idx] = '\0';
    debug_buffer_idx = 0;

    Serial.println(debug_buffer);
    if (strcmp(debug_buffer, "DOc") == 0) {
      platypus::sensors[1]->calibrate(1);
    } else if (strcmp(debug_buffer, "DOc0") == 0) {
      platypus::sensors[1]->calibrate(0);
    } else if (strcmp(debug_buffer, "PHcm") == 0) {
      platypus::sensors[2]->calibrate(0.0);
    } else if (strcmp(debug_buffer, "PHcl") == 0) {
      platypus::sensors[2]->calibrate(-1);
    } else if (strcmp(debug_buffer, "PHch") == 0) {
      platypus::sensors[2]->calibrate(1);
    }
    // Attempt to parse command.
    handleCommand(debug_buffer);
  }
}

