// ============================================
// MOTOR TEST: Safe Pins, Full Speed, 10s Intervals
// ============================================

// ----- Left Motor -----
#define L_R_EN 23
#define L_L_EN 4
#define L_RPWM 25
#define L_LPWM 33  // MOVED TO 12 (32 CANNOT do PWM!)

// ----- Right Motor -----
#define R_R_EN 18
#define R_L_EN 19
#define R_RPWM 21
#define R_LPWM 22

// ----- Buttons (Just for reference) -----
// #define CALIBRATE_BTN 15
// #define START_BTN 33    // Moving button to 33 is perfectly safe!

#define MAX_SPEED 255 

void setup() {
  // 1. Turn on BTS7960 Logic
  pinMode(L_R_EN, OUTPUT); digitalWrite(L_R_EN, HIGH);
  pinMode(L_L_EN, OUTPUT); digitalWrite(L_L_EN, HIGH);
  pinMode(R_R_EN, OUTPUT); digitalWrite(R_R_EN, HIGH);
  pinMode(R_L_EN, OUTPUT); digitalWrite(R_L_EN, HIGH);

  // 2. Setup PWM for ESP32 Core v3.x (Using safe PWM pins)
  ledcAttach(L_RPWM, 5000, 8);
  ledcAttach(L_LPWM, 5000, 8); // Pin 12
  ledcAttach(R_RPWM, 5000, 8);
  ledcAttach(R_LPWM, 5000, 8);
  
  delay(3000); // 3 seconds before it takes off
}

void loop() {
  // ---- FORWARD FOR 10 SECONDS ----
  ledcWrite(L_RPWM, MAX_SPEED); ledcWrite(L_LPWM, 0);
  ledcWrite(R_RPWM, MAX_SPEED); ledcWrite(R_LPWM, 0);
  
  delay(10000); 
  
  // Quick brake
  stopMotors();
  delay(200);

  // ---- BACKWARD FOR 10 SECONDS ----
  ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, MAX_SPEED); // Tests Pin 12
  ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, MAX_SPEED);
  
  delay(10000); 

  stopMotors();
  delay(200);
}

void stopMotors() {
  ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, 0);
  ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, 0);
}
