// ============================================
// MOTOR TEST: Swapped Pins, Full Speed, 10s Intervals
// ============================================

// ----- Left Motor (SWAPPED PINS) -----
#define L_R_EN 23
#define L_L_EN 4
#define L_RPWM 25
#define L_LPWM 33

// ----- Right Motor (SWAPPED PINS) -----
#define R_R_EN 18
#define R_L_EN 19
#define R_RPWM 21
#define R_LPWM 22

#define MAX_SPEED 255  // 100% Full Speed

void setup() {
  // 1. Turn on BTS7960 Logic
  pinMode(L_R_EN, OUTPUT); digitalWrite(L_R_EN, HIGH);
  pinMode(L_L_EN, OUTPUT); digitalWrite(L_L_EN, HIGH);
  pinMode(R_R_EN, OUTPUT); digitalWrite(R_R_EN, HIGH);
  pinMode(R_L_EN, OUTPUT); digitalWrite(R_L_EN, HIGH);

  // 2. Setup PWM for ESP32 Core v3.x
  ledcAttach(L_RPWM, 5000, 8);
  ledcAttach(L_LPWM, 5000, 8);
  ledcAttach(R_RPWM, 5000, 8);
  ledcAttach(R_LPWM, 5000, 8);
  
  // Give you 3 seconds to step back before it takes off
  delay(3000); 
}

void loop() {
  // ---- FORWARD FOR 10 SECONDS ----
  // Left Forward (RPWM = 255, LPWM = 0)
  ledcWrite(L_RPWM, MAX_SPEED); ledcWrite(L_LPWM, 0);
  // Right Forward (RPWM = 255, LPWM = 0)
  ledcWrite(R_RPWM, MAX_SPEED); ledcWrite(R_LPWM, 0);
  
  delay(10000); // Wait 10 seconds
  
  // Quick brake for 0.2 seconds (prevents gear grinding)
  stopMotors();
  delay(200);

  // ---- BACKWARD FOR 10 SECONDS ----
  // Left Reverse (RPWM = 0, LPWM = 255)
  ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, MAX_SPEED);
  // Right Reverse (RPWM = 0, LPWM = 255)
  ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, MAX_SPEED);
  
  delay(10000); // Wait 10 seconds

  // Quick brake before looping again
  stopMotors();
  delay(200);
}

void stopMotors() {
  ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, 0);
  ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, 0);
}
