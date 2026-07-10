#include <Arduino.h>

// ================= SWAPPED PIN DEFINITIONS =================
// Motor Driver - LEFT (Now using former Right pins)
#define L_R_EN 23
#define L_L_EN 4
#define L_RPWM 25
#define L_LPWM 32

// Motor Driver - RIGHT (Now using former Left pins)
#define R_R_EN 18
#define R_L_EN 19
#define R_RPWM 21
#define R_LPWM 22

// 16-CH MUX Sensor
#define MUX_EN 26
#define MUX_S0 27
#define MUX_S1 14
#define MUX_S2 16
#define MUX_S3 13
#define MUX_SIG 34

// Buttons
#define CALIBRATE_BTN 15  // Safe boot pin
#define START_BTN 33      // Safe boot pin (after erasing flash)

// ================= ROBOT PARAMETERS =================
const int NUM_SENSORS = 16;
const int MAX_PWM = 255;       
const int BASE_SPEED = 150;    

// PID GAINS 
float Kp = 0.15;  
float Ki = 0.0;   
float Kd = 1.2;   

// ================= VARIABLES =================
int sensorValues[NUM_SENSORS];
int calibratedMin[NUM_SENSORS];
int calibratedMax[NUM_SENSORS];
bool digitalSensors[NUM_SENSORS]; 

float lastError = 0;
float integral = 0;
int lastLinePosition = 0; 

// ================= MOTOR FUNCTIONS =================
void setupMotors() {
  pinMode(L_R_EN, OUTPUT); pinMode(L_L_EN, OUTPUT);
  pinMode(R_R_EN, OUTPUT); pinMode(R_L_EN, OUTPUT);
  
  // Enable both directions for both drivers
  digitalWrite(L_R_EN, HIGH); digitalWrite(L_L_EN, HIGH);
  digitalWrite(R_R_EN, HIGH); digitalWrite(R_L_EN, HIGH);

  // ESP32 Core v3.x PWM Setup
  ledcAttach(L_RPWM, 5000, 8); // Left Forward
  ledcAttach(L_LPWM, 5000, 8); // Left Reverse
  ledcAttach(R_RPWM, 5000, 8); // Right Forward
  ledcAttach(R_LPWM, 5000, 8); // Right Reverse
}

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);

  // Left Motor Logic (Pins 25 & 33)
  if (leftSpeed >= 0) {
    ledcWrite(L_RPWM, leftSpeed); ledcWrite(L_LPWM, 0);
  } else {
    ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, -leftSpeed);
  }

  // Right Motor Logic (Pins 21 & 22)
  if (rightSpeed >= 0) {
    ledcWrite(R_RPWM, rightSpeed); ledcWrite(R_LPWM, 0);
  } else {
    ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, -rightSpeed);
  }
}

void stopMotors() {
  ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, 0);
  ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, 0);
}

// ================= SENSOR FUNCTIONS =================
void setupMUX() {
  pinMode(MUX_EN, OUTPUT);
  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT); pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_SIG, INPUT);
  digitalWrite(MUX_EN, LOW); // Active LOW to enable MUX
}

void selectMuxChannel(int channel) {
  digitalWrite(MUX_S0, bitRead(channel, 0));
  digitalWrite(MUX_S1, bitRead(channel, 1));
  digitalWrite(MUX_S2, bitRead(channel, 2));
  digitalWrite(MUX_S3, bitRead(channel, 3));
}

void readSensorsRaw() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(i);
    delayMicroseconds(300); // Allow MUX to switch
    sensorValues[i] = analogRead(MUX_SIG);
  }
}

void calibrateSensors() {
  Serial.println("CALIBRATING... Sweep robot over line!");
  for (int i = 0; i < NUM_SENSORS; i++) {
    calibratedMin[i] = 4095; // ESP32 ADC is 12-bit
    calibratedMax[i] = 0;
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 4000) { // 4 seconds to calibrate
    readSensorsRaw();
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] < calibratedMin[i]) calibratedMin[i] = sensorValues[i];
      if (sensorValues[i] > calibratedMax[i]) calibratedMax[i] = sensorValues[i];
    }
  }
  Serial.println("CALIBRATION COMPLETE!");
}

void readDigitalSensors() {
  readSensorsRaw();
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Map the raw value between 0 and 1000 based on calibration
    int mappedVal = map(sensorValues[i], calibratedMin[i], calibratedMax[i], 0, 1000);
    // Threshold: >500 means on the line.
    digitalSensors[i] = (mappedVal > 500) ? 1 : 0; 
  }
}

// ================= LINE POSITION & LOGIC =================
int getLinePosition() {
  int onLineCount = 0;
  int positionSum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (digitalSensors[i] == 1) {
      onLineCount++;
      positionSum += i * 1000; // Weighted position
    }
  }

  // CROSS JUNCTION DETECTION: If many sensors see the line, it's a cross.
  if (onLineCount >= 8) {
    return 7500; // Center position (forces robot straight)
  }

  // 90-DEGREE TURN / LINE LOST RECOVERY
  if (onLineCount == 0) {
    // If we lose the line, check where it was last
    if (lastLinePosition < 6000) return -500;   // Force hard left turn
    else if (lastLinePosition > 9000) return 15500; // Force hard right turn
    else return 7500;                            // Was center, go straight
  }

  // Normal Line Following Calculation
  float position = (float)positionSum / onLineCount;
  lastLinePosition = position; // Save for lost line recovery
  return position;
}

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  
  setupMotors();
  setupMUX();

  pinMode(CALIBRATE_BTN, INPUT_PULLUP);
  pinMode(START_BTN, INPUT_PULLUP);

  stopMotors();

  // Wait for Calibrate Button
  Serial.println("Press CALIBRATE button to start calibration.");
  while (digitalRead(CALIBRATE_BTN) == HIGH) { delay(10); }
  calibrateSensors();
  
  // Wait for Start Button
  Serial.println("Press START button to run the robot!");
  while (digitalRead(START_BTN) == HIGH) { delay(10); }
  delay(500); // Small delay before takeoff
}

void loop() {
  readDigitalSensors();
  int position = getLinePosition();

  // Calculate Error: Center is 7500. Range: -7500 to +7500
  float error = position - 7500;

  // PID Calculations
  integral += error;
  integral = constrain(integral, -5000, 5000); // Anti-windup
  
  float derivative = error - lastError;
  lastError = error;

  float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Apply PID to motors
  int leftSpeed = BASE_SPEED + pidOutput;
  int rightSpeed = BASE_SPEED - pidOutput;

  // Handle sharp turns / 90 degrees - allow reverse motor spinning!
  if (error < -4500) {        // Hard Left / 90 Left
    leftSpeed = -BASE_SPEED;  // Reverse left motor
    rightSpeed = BASE_SPEED;  // Forward right motor
  } else if (error > 4500) {  // Hard Right / 90 Right
    leftSpeed = BASE_SPEED;   // Forward left motor
    rightSpeed = -BASE_SPEED; // Reverse right motor
  }

  setMotors(leftSpeed, rightSpeed);
}
