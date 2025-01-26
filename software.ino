#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Encoder.h>

// Motor and encoder setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1); 
const int encoderPinA = 2;
const int encoderPinB = 3;
Encoder myEnc(encoderPinA, encoderPinB);

// Relay setup
const int relayPin = 10; // Pin connected to relay

// Motor control variables
long targetPositionCounts = 0; // Target position in encoder counts
int targetSpeedRPM = 0; // Target speed in RPM
float gearRatio = 46.85; // Gear ratio of the motor
int encoderCPR = 48; // Encoder counts per revolution of the motor shaft
float outputCPR = encoderCPR * gearRatio; // Counts per revolution of the gearbox's output shaft

void setup() {
  Serial.begin(9600);
  AFMS.begin(); 
  pinMode(relayPin, OUTPUT); // Set relay pin as output
  digitalWrite(relayPin, LOW); // Ensure relay is off at startup
  Serial.println("Enter target speed in RPM followed by target position in revolutions:");
}

void loop() {
  static bool awaitingInput = true;
  if (awaitingInput && Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equalsIgnoreCase("STOP")) {
      myMotor->run(RELEASE); // Stop the motor
      Serial.println("Motor stopped");
      awaitingInput = true;
    } else if (command.equalsIgnoreCase("Laser_Relay_On")) {
      digitalWrite(relayPin, HIGH); // Turn on relay
      Serial.println("Relay turned ON");
      Serial.print("Relay pin state: ");
      Serial.println(digitalRead(relayPin)); // Print the relay pin state
    } else if (command.equalsIgnoreCase("Laser_Relay_Off")) {
      digitalWrite(relayPin, LOW); // Turn off relay
      Serial.println("Relay turned OFF");
      Serial.print("Relay pin state: ");
      Serial.println(digitalRead(relayPin)); // Print the relay pin state
    } else if (command.equalsIgnoreCase("m")) {
      // Handle motor control command
      Serial.println("Enter target speed in RPM followed by target position in revolutions:");
    } else {
      // Parse target speed and position from the serial input
      targetSpeedRPM = command.toInt();
      targetPositionCounts = (long)(Serial.parseFloat() * outputCPR);
    
      Serial.print("Target speed set to: ");
      Serial.print(targetSpeedRPM);
      Serial.print(" RPM, Target position set to: ");
      Serial.print(targetPositionCounts);
      Serial.println(" counts.");

      // Update control flags
      awaitingInput = false;
    }
  }

  if (!awaitingInput) {
    // Proceed with motor control to reach the target position at the specified speed
    controlMotor(targetSpeedRPM, targetPositionCounts);
  }
}

void controlMotor(int speedRPM, long positionCounts) {
  long currentCounts = myEnc.read();
  long error = positionCounts - currentCounts;
  
  Serial.print("Current encoder counts: ");
  Serial.println(currentCounts);
  Serial.print("Target encoder counts: ");
  Serial.println(positionCounts);
  Serial.print("Position error: ");
  Serial.println(error);
  
  // Example simplistic approach to start/stop motor based on position error
  if (abs(error) > 10) { // Simple threshold, adjust based on your system's resolution and noise level
    myMotor->setSpeed(map(speedRPM, 0, 300, 0, 255)); // Mapping RPM to PWM value, adjust as needed
    if (error > 0) {
      myMotor->run(FORWARD);
    } else {
      myMotor->run(BACKWARD);
    }
  } else {
    myMotor->run(RELEASE); // Stop the motor when the target position is reached
  }
}
