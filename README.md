# Xbotix


#include <QTRSensors.h>
#include <Adafruit_APDS9960.h>
#include <Servo.h>

//~~~~~~~~~~~~~~~~~~~~~
// _PIN CONFIGURATION_
//~~~~~~~~~~~~~~~~~~~~~

// --- LINE SENSOR SETUP--- 
#define NUM_SENSORS 8
#define EMITTER_PIN 10 

// --- MOTOR DRIVER PINS (L298N) ---
// used analog pins as Digital pins :) 
#define ENA 1  // Left Motor Speed (PWM)
#define IN1 A0 // Left Motor Direction 1
#define IN2 A1 // Left Motor Direction 2
#define ENB 12  // Right Motor Speed (PWM)
#define IN3 A2 // Right Motor Direction 1 
#define IN4 A3 // Right Motor Direction 2

// --- GRIPPER SERVO PIN(optional) ---
#define GRIPPER_PIN 11 // Changed to avoid conflict

//~~~~~~~~~~~~~~~~~~~~~~~
// _OBJECTS & VARIABLES_ 
//~~~~~~~~~~~~~~~~~~~~~~~

// ------ LINE SENSOR -------
QTRSensors qtr;
unsigned int sensorValues[NUM_SENSORS];

// --- COLOR SENSOR SETUP ---
Adafruit_APDS9960 apds;

// --- GRIPPER SETUP ---
Servo gripper;
#define GRIP_OPEN 0   // needs to be adjusted
#define GRIP_CLOSE 90 // needs to be adjusted

// --- PID Control ---
float Kp = 0.07;  // Proportional gain. Tune this first.
float Ki = 0.0001; // Integral gain. Keep this very small.
float Kd = 0.6;    // Derivative gain. Tune this after Kp.

int error = 0;
int lastError = 0;
int integral = 0;

// --- ENHANCED BUTTON PRESS CONTROL ---

bool buttonPressed = false;
bool atButtonStation = false;
unsigned long buttonPressStartTime = 0;
const unsigned long BUTTON_PRESS_DURATION = 500; // milliseconds
const unsigned long BUTTON_RELEASE_DURATION = 500; // milliseconds

// --- MOTION LIBRARY with SERIAL COMMANDS ---
String serialCommand = "";
bool commandInProgress = false;
int commandSpeed = 120; // Default speed
int commandDuration = 1000; // Default duration in milliseconds
unsigned long commandStartTime = 0;

// --- ROBOT STATE ---
bool isCarryingCube = false;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ---- SETUP FUNCTIONS ----
//~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setupLineSensor() {
  qtr.setTypeRC(); //resistor capacitor type
  qtr.setSensorPins((const uint8_t[]){2,3,4,5,6,7,8,9}, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);

  Serial.println("Calibrating line sensors...");
  delay(50); 
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(20); 
  }
  Serial.println("Line sensor calibration done!");
}

void setupColorSensor() {
  if (!apds.begin()) {
    Serial.println("APDS-9960 not found!");
    while (1); // stop execution if sensor not detected
  }
  Serial.println("Color sensor ready!");
}

void setupGripper() {
  gripper.attach(GRIPPER_PIN); // servo signal pin
  gripOpen();         // start with gripper open
}

void setupMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();
  Serial.println("Motors ready!");
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~
// --- HELPER FUNCTIONS --- 
//~~~~~~~~~~~~~~~~~~~~~~~~~~

// --- Line position reading --- 
int readLinePosition() {
  return qtr.readLineBlack(sensorValues); // 0-7000 scale
}

// --- Color detection function ---
String detectColor() {
  uint16_t r, g, b, c;
  apds.getColorData(&r, &g, &b, &c);

  // Enhanced color detection with better thresholds
  if (c < 100) {
    return "NoLight"; // Not enough light to detect color
  }
  
  // Normalize RGB values
  float r_norm = (float)r / c * 255;
  float g_norm = (float)g / c * 255;
  float b_norm = (float)b / c * 255;
  
  // Determine color with better thresholds
  if (r_norm > 150 && r_norm > g_norm * 1.5 && r_norm > b_norm * 1.5) {
    return "Red";
  } else if (b_norm > 150 && b_norm > r_norm * 1.5 && b_norm > g_norm * 1.5) {
    return "Blue";
  } else if (g_norm > 150 && g_norm > r_norm * 1.5 && g_norm > b_norm * 1.5) {
    return "Green";
  }
  
  return "Unknown";
}

// --- Gripper Control ---
void gripOpen() {
  gripper.write(GRIP_OPEN);
  Serial.println("Gripper Opened");
}

void gripClose() {
  gripper.write(GRIP_CLOSE);
  Serial.println("Gripper Closed");
}

// ----- MOTOR CONTROL ----- 

void moveForward(int speed, int duration) {
  if (speed < 0 || speed > 255) {
    Serial.println("Invalid speed. Using default speed.");
    speed = 120;
  }
  
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Left motor Forward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Right motor Forward
  analogWrite(ENA, speed); analogWrite(ENB, speed); //Set speed
  
  if (duration > 0) {
    delay(duration);
    stopMotors();
  }
}

void moveBackward(int speed, int duration) {
  if (speed < 0 || speed > 255) {
    Serial.println("Invalid speed. Using default speed.");
    speed = 120;
  }
  
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // Left motor Backward
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Right motor Backward
  analogWrite(ENA, speed); analogWrite(ENB, speed); // Set speed
  
  if (duration > 0) {
    delay(duration);
    stopMotors();
  }
}

void turnRight(int speed, int duration) {
  if (speed < 0 || speed > 255) {
    Serial.println("Invalid speed. Using default speed.");
    speed = 120;
  }
  
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Left Motor Forward
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Right Motor Backward
  analogWrite(ENA, speed); analogWrite(ENB, speed);
  
  if (duration > 0) {
    delay(duration);
    stopMotors();
  }
}

void turnLeft(int speed, int duration) {
  if (speed < 0 || speed > 255) {
    Serial.println("Invalid speed. Using default speed.");
    speed = 120;
  }
  
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // Left Motor Backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Right Motor Forward
  analogWrite(ENA, speed); analogWrite(ENB, speed);
  
  if (duration > 0) {
    delay(duration);
    stopMotors();
  }
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
	// Set direction to forward for both
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  
  // Set the speeds
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

// ---- Process serial commands ----
// BULAK thiyenna puluwan meka thama amaruma una wade :( . Kama karanna!
// Chata ge sahaya gaththemi .

void processSerialCommand() {
  if (serialCommand.length() == 0) return;
  
  // Convert to uppercase for case-insensitive comparison
  serialCommand.toUpperCase();
  
  // Parse command
  char command = serialCommand.charAt(0);
  
  // For commands with parameters (e.g., "F120" for forward at speed 120)
  int param1 = 0;
  int param2 = 0;
  
  if (serialCommand.length() > 1) {
    // Extract first parameter (speed or duration)
    String param1Str = "";
    int i = 1;
    while (i < serialCommand.length() && isDigit(serialCommand.charAt(i))) {
      param1Str += serialCommand.charAt(i);
      i++;
    }
    param1 = param1Str.toInt();
    
    //Extract second parameter (duration) if present
    if (i < serialCommand.length() && serialCommand.charAt(i) == ',') {
      i++; // Skip comma
      String param2Str = "";
      while (i < serialCommand.length() && isDigit(serialCommand.charAt(i))) {
        param2Str += serialCommand.charAt(i);
        i++;
      }
      param2 = param2Str.toInt();
    }
  }
  
  // Execute command
  switch (command) {
    case 'F': // Forward
      if (param1 > 0) {
        if (param2 > 0) {
          moveForward(param1, param2);
        } else {
          moveForward(param1, 0); // Continuous movement
        }
      } else {
        moveForward(commandSpeed, commandDuration);
      }
      Serial.println("Moving forward");
      break;
      
    case 'B': // Backward
      if (param1 > 0) {
        if (param2 > 0) {
          moveBackward(param1, param2);
        } else {
          moveBackward(param1, 0); // Continuous movement
        }
      } else {
        moveBackward(commandSpeed, commandDuration);
      }
      Serial.println("Moving backward");
      break;
      
    case 'L': // Left
      if (param1 > 0) {
        if (param2 > 0) {
          turnLeft(param1, param2);
        } else {
          turnLeft(param1, 0); // Continuous movement
        }
      } else {
        turnLeft(commandSpeed, commandDuration);
      }
      Serial.println("Turning left");
      break;
      
    case 'R': // Right
      if (param1 > 0) {
        if (param2 > 0) {
          turnRight(param1, param2);
        } else {
          turnRight(param1, 0); // Continuous movement
        }
      } else {
        turnRight(commandSpeed, commandDuration);
      }
      Serial.println("Turning right");
      break;
      
    case 'S': // Stop
      stopMotors();
      Serial.println("Stopping motors");
      break;
      
    case 'G': // Gripper control
      if (param1 == 0) {
        gripOpen();
      } else {
        gripClose();
      }
      break;
      
    case 'P': // Press button
      pressButton();
      break;
      
    case 'A': // Approach button station
      approachButtonStation();
      break;
      
    case 'T': // Detect tower color
      Serial.println("Tower color: " + detectTowerColor());
      break;
      
    case 'C': // Detect cube color
      Serial.println("Cube color: " + detectColor());
      break;
      
    case 'D': // Set default speed
      if (param1 > 0 && param1 <= 255) {
        commandSpeed = param1;
        Serial.println("Default speed set to " + String(param1));
      } else {
        Serial.println("Invalid speed. Please use a value between 1 and 255.");
      }
      break;
      
    case 'U': // Set default duration
      if (param1 > 0) {
        commandDuration = param1;
        Serial.println("Default duration set to " + String(param1) + "ms");
      } else {
        Serial.println("Invalid duration. Please use a positive value.");
      }
      break;
      
    case 'H': // Help
      Serial.println("Available commands:");
      Serial.println("F[speed],[duration] - Move forward");
      Serial.println("B[speed],[duration] - Move backward");
      Serial.println("L[speed],[duration] - Turn left");
      Serial.println("R[speed],[duration] - Turn right");
      Serial.println("S - Stop motors");
      Serial.println("G[0|1] - Gripper control (0=open, 1=close)");
      Serial.println("P - Press button");
      Serial.println("A - Approach button station");
      Serial.println("T - Detect tower color");
      Serial.println("C - Detect cube color");
      Serial.println("D[speed] - Set default speed");
      Serial.println("U[duration] - Set default duration");
      Serial.println("H - Show this help");
      break;
      
    default:
      Serial.println("Unknown command. Type H for help.");
      break;
  }
  
  // Clear command
  serialCommand = "";
}

// --- READ SERIAL INPUTS ---
void readSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      // End of command
      processSerialCommand();
    } else if (c >= 32 && c <= 126) {
      // Printable character
      serialCommand += c;
    }
  }
}

// ----- PID LINE FOLLOWING -----
void followLine(int baseSpeed) {
  int position = readLinePosition(); // Read the line position
  error = position - 3500; // Calculate the error (Ideal position is 3500)

	// Calculate the PID terms
  int P = error * Kp;
  integral += error;
  int I = integral * Ki;
  int D = (error - lastError) * Kd;

	// Calculate the total correction
  int correction = P + I + D;

  // Adjust motor speeds based on the correction
  // If correction is positive (robot is to the right), slow down right motor
  // If correction is negative (robot is to the left), slow down left motor
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

	// Constrain speeds to be within the valid PWM range (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotorSpeeds(leftSpeed, rightSpeed); // Apply the new speeds to the motors
  lastError = error;  // Update lastError for the next iteration
}

// ---- TASK SEQUENCES ----
void approachButtonStation() {
  Serial.println("Approaching button station...");
  moveForward(80);
  delay(1500); // This should be replaced with a sensor-based stop
  stopMotors();
  atButtonStation = true;
  Serial.println("At button station");
}
void pressButton() {
	if (!atButtonStation) {
    Serial.println("Not at button station, approaching first...");
    approachButtonStation();
  }
  
  Serial.println("Pressing button...");
  buttonPressStartTime = millis();
  buttonPressed = true;
  moveForward(100); //move forward to press the button
  
  // We handle the timing in the main loop
}

// --- HANDLE BUTTON PRESS TIMING ---
void handleButtonPress() {
  if (buttonPressed) {
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - buttonPressStartTime;
    
    if (elapsed < BUTTON_PRESS_DURATION) {
      // Still pressing the button
      moveForward(100);
    } else if (elapsed < BUTTON_PRESS_DURATION + BUTTON_RELEASE_DURATION) {
      // Releasing the button
      moveBackward(100);
    } else {
      // Button press sequence complete
      stopMotors();
      buttonPressed = false;
      Serial.println("Button press complete");
    }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ---- MAIN SETUP & LOOP ----
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// --- MAIN SETUP ---

void setup() {
  Serial.begin(9600);  // initialize serial communication
  setupLineSensor();
  setupColorSensor();
  setupGripper();
  setupMotors();
  
  Serial.println("Robot is ready!");
  delay(1000);
}

// --- MAIN LOOP ---

void loop() {
	// --- Read serial input ---
	readSerialInput();
	
  // Handle button press if in progress
  handleButtonPress();
  
  // If no command is in progress, continue with normal operation
  if (!commandInProgress && !buttonPressed) {
    // --- Read line position ---
    int pos = readLinePosition();
    Serial.println("Line position: " + String(pos));

    // --- motor PID control using pos ---
    int baseSpeed = 120; // Adjust this for your robot's top speed
    followLine(baseSpeed);

    // --- Detect cube color ---
    String color = detectColor();
    if ((color == "Red" || color == "Blue") && !isCarryingCube) {
      stopMotors(); //stop motors here 
      Serial.println("Cube detected: " + color);
 
      gripClose();  // pick up cube
      delay(500);
      isCarryingCube = true;

      // TODO: move to appropriate socket
      Serial.println("Moving to " + color + " socket...");
      // Add code to move to the appropriate socket based on color
      
      gripOpen();   // drop cube
      delay(500);

      // TODO: return to main path
      Serial.println("Returning to path...");
      // Add code to return to the main path
    }
  }

  delay(100); // loop speed
}
