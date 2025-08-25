/*
  Automatic Railway Gate Control System - Dual IR Sensor & Dual Servo Version
  Components: 2 IR Sensors, Arduino, Buzzer, 2 Servo Motors, 2 Red LEDs, 2 Green LEDs
  
  System Logic:
  - 2 IR sensors placed on both sides of the railway gate
  - 2 Servo motors for better gate control (left and right gate arms)
  - Green LEDs ON = Gate Open (Normal state)
  - Red LEDs ON + Buzzer = Train detected, Gate closing/closed
  - Dual servo synchronized movement
*/

#include <Servo.h>

// ===== PIN CONNECTIONS (Organized by Function) =====

// IR Sensors
const int IR_SENSOR1_PIN = 2;    // IR sensor 1 (Side A) - Digital Pin 2
const int IR_SENSOR2_PIN = 3;    // IR sensor 2 (Side B) - Digital Pin 3

// LEDs (Status Indicators)
const int RED_LED1_PIN = 4;      // Red LED 1 - Digital Pin 4
const int RED_LED2_PIN = 5;      // Red LED 2 - Digital Pin 5
const int GREEN_LED1_PIN = 6;    // Green LED 1 - Digital Pin 6
const int GREEN_LED2_PIN = 7;    // Green LED 2 - Digital Pin 7

// Alert System (3-Pin Buzzer)
const int BUZZER_PIN = 8;        // Buzzer Signal Pin - Digital Pin 8
// Note: Connect VCC to 5V, GND to GND, Signal to Pin 8

// Servo Motors (Gate Control)
const int SERVO1_PIN = 9;        // Servo Motor 1 (Left Gate) - PWM Pin 9
const int SERVO2_PIN = 10;       // Servo Motor 2 (Right Gate) - PWM Pin 10

// Create servo objects
Servo leftGateServo;   // Controls left side of gate
Servo rightGateServo;  // Controls right side of gate

// System states
bool trainDetected = false;
bool gateOpen = true;
bool sensor1Active = false;
bool sensor2Active = false;
unsigned long lastDetectionTime = 0;
const unsigned long GATE_DELAY = 3000; // 3 seconds delay before opening gate

// Servo positions - Left servo correct, Right servo logic FIXED
const int LEFT_GATE_OPEN_ANGLE = 0;     // Left gate open (horizontal/parallel to track)
const int LEFT_GATE_CLOSED_ANGLE = 90;  // Left gate closed (swings to block track)

// Right servo positions - LOGIC CORRECTED (swapped open/closed)
const int RIGHT_GATE_OPEN_ANGLE = 0;    // Right gate open (horizontal/parallel to track) - SWAPPED
const int RIGHT_GATE_CLOSED_ANGLE = 90; // Right gate closed (swings to block track) - SWAPPED

// Detection tracking
String trainDirection = "";
int servoSpeedClosing = 3; // Fast closing - 3ms delay for emergency response
int servoSpeedOpening = 8; // Moderate opening - 8ms delay for safety

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // ===== PIN INITIALIZATION =====
  // IR Sensors
  pinMode(IR_SENSOR1_PIN, INPUT);
  pinMode(IR_SENSOR2_PIN, INPUT);
  
  // LEDs
  pinMode(RED_LED1_PIN, OUTPUT);
  pinMode(RED_LED2_PIN, OUTPUT);
  pinMode(GREEN_LED1_PIN, OUTPUT);
  pinMode(GREEN_LED2_PIN, OUTPUT);
  
  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Attach servo motors
  leftGateServo.attach(SERVO1_PIN);
  rightGateServo.attach(SERVO2_PIN);
  
  // Initialize system to open state
  openGate();
  
  Serial.println("=== DUAL SERVO RAILWAY GATE CONTROL SYSTEM ===");
  Serial.println("IR Sensor 1 (Pin 2): Side A | IR Sensor 2 (Pin 3): Side B");
  Serial.println("Left Servo (Pin 9) | Right Servo (Pin 10)");
  Serial.println("Gate Status: OPEN");
  Serial.println("System Ready - Monitoring both sensors...");
  printConnectionGuide();
}

void loop() {
  // Read both IR sensors (LOW when object detected, HIGH when clear)
  int sensor1Value = digitalRead(IR_SENSOR1_PIN);
  int sensor2Value = digitalRead(IR_SENSOR2_PIN);
  
  // Update sensor states
  bool sensor1Detecting = (sensor1Value == LOW);
  bool sensor2Detecting = (sensor2Value == LOW);
  
  // Check for new detections
  if (sensor1Detecting && !sensor1Active) {
    sensor1Active = true;
    trainDirection = "Side A";
    Serial.println("🚂 TRAIN DETECTED on Side A (Sensor 1)!");
  }
  
  if (sensor2Detecting && !sensor2Active) {
    sensor2Active = true;
    if (trainDirection == "") {
      trainDirection = "Side B";
    } else {
      trainDirection = "Both Sides";
    }
    Serial.println("🚂 TRAIN DETECTED on Side B (Sensor 2)!");
  }
  
  // Train detection logic
  if (sensor1Detecting || sensor2Detecting) {
    if (!trainDetected) {
      trainDetected = true;
      Serial.println("⚠️  DANGER: Train approaching from " + trainDirection);
      closeGate();
    }
    lastDetectionTime = millis();
  } else {
    // No train detected on either sensor
    if (trainDetected) {
      if (millis() - lastDetectionTime >= GATE_DELAY) {
        trainDetected = false;
        sensor1Active = false;
        sensor2Active = false;
        trainDirection = "";
        Serial.println("✅ Train cleared both sensors. Opening gate...");
        openGate();
      }
    }
  }
  
  // Status update every 5 seconds when gate is open
  static unsigned long lastStatusTime = 0;
  if (gateOpen && millis() - lastStatusTime >= 5000) {
    Serial.println("📊 Status: Gate OPEN | S1: " + String(sensor1Detecting ? "🔴 ACTIVE" : "🟢 Clear") + 
                  " | S2: " + String(sensor2Detecting ? "🔴 ACTIVE" : "🟢 Clear"));
    lastStatusTime = millis();
  }
  
  delay(50);
}

void closeGate() {
  if (gateOpen) {
    Serial.println("🚨 === EMERGENCY GATE CLOSURE ===");
    
    // IMMEDIATE LED response for instant visual warning
    digitalWrite(GREEN_LED1_PIN, LOW);
    digitalWrite(GREEN_LED2_PIN, LOW);
    digitalWrite(RED_LED1_PIN, HIGH);
    digitalWrite(RED_LED2_PIN, HIGH);
    
    // Start emergency warning tone IMMEDIATELY (non-blocking)
    tone(BUZZER_PIN, 1800, 500); // High urgent tone
    
    Serial.println("⚡ FAST CLOSING: Both gates moving to EMERGENCY CLOSE position...");
    
    // FAST gate closure with emergency speed
    moveGatesFast(LEFT_GATE_CLOSED_ANGLE, RIGHT_GATE_CLOSED_ANGLE, servoSpeedClosing);
    
    gateOpen = false;
    Serial.println("🔒 EMERGENCY CLOSURE COMPLETE - " + String(millis()) + "ms");
    Serial.println("🚫 CRITICAL: RAILWAY BLOCKED - TRAIN APPROACHING FROM " + trainDirection);
    
    // Start continuous warning after closure
    startContinuousWarning();
  }
}

void openGate() {
  if (!gateOpen || (!trainDetected && gateOpen)) {
    Serial.println("✅ === SAFE GATE OPENING ===");
    
    // Turn off buzzer first with proper stop function
    stopBuzzer();
    delay(300);
    
    // Brief opening signal (2 short beeps with different tones)
    tone(BUZZER_PIN, 1500, 150); // High tone
    delay(200);
    tone(BUZZER_PIN, 1000, 150); // Lower tone  
    delay(300);
    
    // Open both gates with moderate speed (safety opening)
    Serial.println("🔄 SAFE OPENING: Moving both gates to OPEN position...");
    moveGatesFast(LEFT_GATE_OPEN_ANGLE, RIGHT_GATE_OPEN_ANGLE, servoSpeedOpening);
    
    // Turn off red LEDs
    digitalWrite(RED_LED1_PIN, LOW);
    digitalWrite(RED_LED2_PIN, LOW);
    
    // Turn on green LEDs
    digitalWrite(GREEN_LED1_PIN, HIGH);
    digitalWrite(GREEN_LED2_PIN, HIGH);
    
    gateOpen = true;
    Serial.println("🟢 SAFE: Both Gates OPEN - Clear to Cross");
  } else {
    // Initialize to open state (startup)
    leftGateServo.write(LEFT_GATE_OPEN_ANGLE);
    rightGateServo.write(RIGHT_GATE_OPEN_ANGLE);
    digitalWrite(GREEN_LED1_PIN, HIGH);
    digitalWrite(GREEN_LED2_PIN, HIGH);
    digitalWrite(RED_LED1_PIN, LOW);
    digitalWrite(RED_LED2_PIN, LOW);
    stopBuzzer();
    delay(500);
  }
}

void moveGatesFast(int leftTarget, int rightTarget, int speed) {
  unsigned long startTime = millis();
  
  // Get current positions
  int leftCurrent = gateOpen ? LEFT_GATE_OPEN_ANGLE : LEFT_GATE_CLOSED_ANGLE;
  int rightCurrent = gateOpen ? RIGHT_GATE_OPEN_ANGLE : RIGHT_GATE_CLOSED_ANGLE;
  
  // Calculate step directions
  int leftStep = (leftTarget > leftCurrent) ? 2 : -2;  // 2-degree steps for faster movement
  int rightStep = (rightTarget > rightCurrent) ? 2 : -2; // 2-degree steps for faster movement
  
  // Move both servos simultaneously with fast motion
  while (leftCurrent != leftTarget || rightCurrent != rightTarget) {
    if (abs(leftCurrent - leftTarget) > 1) {
      leftCurrent += leftStep;
      // Clamp to target if we overshoot
      if ((leftStep > 0 && leftCurrent > leftTarget) || (leftStep < 0 && leftCurrent < leftTarget)) {
        leftCurrent = leftTarget;
      }
      leftGateServo.write(leftCurrent);
    }
    
    if (abs(rightCurrent - rightTarget) > 1) {
      rightCurrent += rightStep;
      // Clamp to target if we overshoot
      if ((rightStep > 0 && rightCurrent > rightTarget) || (rightStep < 0 && rightCurrent < rightTarget)) {
        rightCurrent = rightTarget;
      }
      rightGateServo.write(rightCurrent);
    }
    
    delay(speed); // Fast movement delay
  }
  
  unsigned long totalTime = millis() - startTime;
  Serial.println("⚡ Gate movement completed in " + String(totalTime) + "ms");
  delay(100); // Minimal settling time
}

void buzzerWarning() {
  Serial.println("🔊 Sounding warning alarm...");
  // Enhanced warning pattern for 3-pin buzzer - Multiple tones
  
  // Pattern 1: Rapid urgent beeps
  for (int i = 0; i < 5; i++) {
    tone(BUZZER_PIN, 1000, 200); // 1000Hz for 200ms
    delay(250);
    tone(BUZZER_PIN, 1500, 200); // 1500Hz for 200ms
    delay(250);
  }
  
  delay(300);
  
  // Pattern 2: Continuous warning tone
  tone(BUZZER_PIN, 800, 1000); // 800Hz for 1 second
  delay(1000);
}

void startContinuousWarning() {
  // Continuous alternating warning tones while gate is closed
  static unsigned long lastToneChange = 0;
  static bool highTone = true;
  
  if (millis() - lastToneChange >= 800) { // Change tone every 800ms
    if (highTone) {
      tone(BUZZER_PIN, 1200); // High warning tone
    } else {
      tone(BUZZER_PIN, 800);  // Low warning tone
    }
    highTone = !highTone;
    lastToneChange = millis();
  }
}

void stopBuzzer() {
  noTone(BUZZER_PIN); // Stop any ongoing tone
  digitalWrite(BUZZER_PIN, LOW);
}

void printConnectionGuide() {
  Serial.println("\n=== CONNECTION GUIDE ===");
  Serial.println("IR Sensors:");
  Serial.println("  Sensor 1 → Pin 2 (Side A)");
  Serial.println("  Sensor 2 → Pin 3 (Side B)");
  Serial.println("LEDs:");
  Serial.println("  Red LED 1 → Pin 4");
  Serial.println("  Red LED 2 → Pin 5");
  Serial.println("  Green LED 1 → Pin 6");
  Serial.println("  Green LED 2 → Pin 7");
  Serial.println("3-Pin Buzzer:");
  Serial.println("  VCC → 5V");
  Serial.println("  GND → GND"); 
  Serial.println("  Signal → Pin 8");
  Serial.println("Servo Motors (Pin 10 Logic CORRECTED):");
  Serial.println("  Left Gate Servo → Pin 9 (PWM) - CORRECT");
  Serial.println("  Right Gate Servo → Pin 10 (PWM) - LOGIC FIXED");
  Serial.println("Gate Movement:");
  Serial.println("  OPEN: Left=0°, Right=0° (both horizontal/clear)");
  Serial.println("  CLOSED: Left=90°, Right=90° (both block track)");
  Serial.println("  FIXED: Pin 10 now closes when train detected!");
  Serial.println("========================\n");
}

// Test function for fast dual servo system
void testFastServoSystem() {
  Serial.println("🧪 === TESTING FAST EMERGENCY SYSTEM ===");
  Serial.println("⚡ Testing EMERGENCY gate closure speed...");
  unsigned long startTime = millis();
  closeGate();
  unsigned long closeTime = millis() - startTime;
  Serial.println("🔒 Emergency closure time: " + String(closeTime) + "ms");
  
  delay(3000);
  
  Serial.println("✅ Testing safe gate opening...");
  startTime = millis();
  openGate();
  unsigned long openTime = millis() - startTime;
  Serial.println("🟢 Safe opening time: " + String(openTime) + "ms");
  
  delay(1000);
  Serial.println("✅ Fast servo system test completed.");
}

// Individual servo control for calibration
void calibrateServos() {
  Serial.println("🔧 === SERVO CALIBRATION ===");
  
  Serial.println("Moving Left Servo through range...");
  for (int angle = 0; angle <= 180; angle += 30) {
    leftGateServo.write(angle);
    Serial.println("Left Servo: " + String(angle) + "°");
    delay(1000);
  }
  
  Serial.println("Moving Right Servo through range...");
  for (int angle = 0; angle <= 180; angle += 30) {
    rightGateServo.write(angle);
    Serial.println("Right Servo: " + String(angle) + "°");
    delay(1000);
  }
  
  Serial.println("✅ Calibration complete - returning to open position");
  openGate();
}