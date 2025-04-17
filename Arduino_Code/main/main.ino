#include <Servo.h>

// Define which digital pins the servos are connected to.
const int SERVO_PITCH_PIN = 9;        // Servo for first pitch command
const int SERVO_PITCH2_PIN = 10;      // Servo for second pitch command

// Create servo objects.
Servo servoPitch;   // Controlled via SERVO:PITCH:<angle>
Servo servoPitch2;  // Controlled via SERVO:PITCH2:<angle>

// Use the built-in LED for pump simulation.
const int LED_PIN = LED_BUILTIN;

void setup() {
  Serial.begin(9600); // Must match the baud rate used by the flight computer.
  while (!Serial) {
    ; // Wait for the serial port to connect (for native USB boards).
  }
  
  // Attach servos to their designated pins.
  servoPitch.attach(SERVO_PITCH_PIN);
  servoPitch2.attach(SERVO_PITCH2_PIN);

  // Initialize LED pin.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Arduino ready. Waiting for commands...");
}

void loop() {
  if (Serial.available() > 0) {
    // Read a complete command line from the serial buffer.
    String commandLine = Serial.readStringUntil('\n');
    commandLine.trim(); // Remove any extra whitespace/newline characters.
    if (commandLine.length() > 0) {
      processCommand(commandLine);
    }
  }
}

void processCommand(String cmd) {
  // Remove the "AR:" prefix if present.
  if (cmd.startsWith("AR:")) {
    cmd = cmd.substring(3);
  }

  Serial.print("Received command: ");
  Serial.println(cmd);

  // Find the first colon, which separates the command type.
  int firstColon = cmd.indexOf(':');
  if (firstColon == -1) {
    Serial.println("Invalid command format.");
    return;
  }

  // Get the command type (e.g., SERVO or PUMP).
  String commandType = cmd.substring(0, firstColon);
  commandType.toUpperCase(); // Convert for case-insensitive comparison

  if (commandType == "SERVO") {
    // Expected format: SERVO:<AXIS>:<ANGLE>
    int secondColon = cmd.indexOf(':', firstColon + 1);
    if (secondColon == -1) {
      Serial.println("Invalid servo command format.");
      return;
    }
    
    // Extract the axis.
    String axis = cmd.substring(firstColon + 1, secondColon);
    axis.toUpperCase();

    // Extract the angle as a string and convert to an integer.
    String angleStr = cmd.substring(secondColon + 1);
    angleStr.trim();
    int angle = angleStr.toInt();
    // Constrain the angle between 0 and 180.
    angle = constrain(angle, 0, 180);

    // Process the servo command based on the axis keyword.
    if (axis == "PITCH") {
      servoPitch.write(angle);
      Serial.print("Set servo PITCH to ");
      Serial.print(angle);
      Serial.println(" degrees.");
    }
    else if (axis == "PITCH2") {
      servoPitch2.write(angle);
      Serial.print("Set servo PITCH2 to ");
      Serial.print(angle);
      Serial.println(" degrees.");
    }
    else {
      Serial.println("Unknown servo axis. Expected PITCH or PITCH2.");
    }
  }
  else if (commandType == "PUMP") {
    // Expected format: PUMP:<STATE> where STATE is ON or OFF.
    String state = cmd.substring(firstColon + 1);
    state.trim();
    state.toUpperCase();

    if (state == "ON") {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("Pump ON (LED ON).");
    }
    else if (state == "OFF") {
      digitalWrite(LED_PIN, LOW);
      Serial.println("Pump OFF (LED OFF).");
    }
    else {
      Serial.println("Invalid pump command state.");
    }
  }
  else {
    Serial.println("Unknown command type.");
  }
}
