// ==============================================================
// TINKER-TRACER: ULTIMATE 600RPM LINE FOLLOWER
// Optimized for: Bare ESP32 + 138mm Curved 16-IR + BTS7960
// Features: Auto-Calibrate, Dynamic Speed, 90° Pivots, Cross Ignoring
// Modified by Chinglen2080
// ==============================================================

// ----- Mux Pins -----
#define MUX_EN   26
#define MUX_S0   27
#define MUX_S1   14
#define MUX_S2   16
#define MUX_S3   13
#define MUX_SIG  34

// ----- Left Motor (Final Pins) -----
#define L_R_EN   23
#define L_L_EN   4
#define L_LPWM   25
#define L_RPWM   33  // Safe PWM pin  <-- that was a LIE

// ----- Right Motor (Final Pins) -----
#define R_R_EN   18
#define R_L_EN   19
#define R_LPWM   21
#define R_RPWM   22

// ----- Buttons -----
#define BTN_CAL  15
#define BTN_GO   32

// ----- System Constants -----
#define NUM_SENSORS     16
#define PWM_FREQ        5000
#define PWM_BITS        8
#define PWM_MAX         255
#define CALIB_TIME_MS   5000
#define CALIB_SPIN_SPD  80  //THATS FAST FAST (it was 160)

// ----- Advanced Track Tuning -----
#define LINE_THRESHOLD  200    // 0-1000 normalized value to trigger "Line Detected"
#define JUNCTION_COUNT  10     // If >10 sensors see line, assume cross junction

// ----- 600 RPM Dynamic Speed Tuning ----- // Modified
#define STRAIGHT_SPEED 150    // Max speed on perfect straights (max is 255, it was 200)
#define GENTLE_SPEED   100    // Slight curves
#define SHARP_SPEED    80    // Sharp curves
#define EXTREME_SPEED  90     // Recovering from edges
#define TURN_SPEED     70    // Outer wheel speed during 90° pivots

// ----- PID Tuning (High Inertia 600 RPM Setup) -----
#define KP  0.8               // Aggressive steering // YOU DONT EVEN TURN FOR GODS SAKE!!!
#define KI  0.0               // 0 to prevent high-speed oscillation // MAKE IT TURN FIRST
#define KD  4.5               // Heavy dampening to stop wobble // fine

// ----- LED -----
#define LED_PIN  2

// ----- State Machine -----
enum RobotState { IDLE, CALIBRATING, FOLLOWING, SEARCHING, HANDLING_JUNCTION };
RobotState currentState = IDLE;

// ----- PID State -----
float lastError = 0;
float integral  = 0;

// ----- Sensor Data -----
int rawVals[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
uint16_t lineMap = 0; // 16-bit bitmap (1 = line, 0 = no line)
int activeSensors = 0;

// ----- Flags & Timers -----
bool calibrated = false;
bool running    = false;
int lastKnownSide = 0; // -1 = left, 1 = right
unsigned long stateTimer = 0;

// ==============================================================
//                          SETUP
// ==============================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  
  // Mux Setup
  pinMode(MUX_EN, OUTPUT); digitalWrite(MUX_EN, LOW);
  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT); 
  pinMode(MUX_S2, OUTPUT); pinMode(MUX_S3, OUTPUT);

  // Motor Enable Setup (BTS7960 needs these HIGH to spin)
  pinMode(L_R_EN, OUTPUT); digitalWrite(L_R_EN, HIGH);
  pinMode(L_L_EN, OUTPUT); digitalWrite(L_L_EN, HIGH);
  pinMode(R_R_EN, OUTPUT); digitalWrite(R_R_EN, HIGH);
  pinMode(R_L_EN, OUTPUT); digitalWrite(R_L_EN, HIGH);

  // ESP32 Core v3.x PWM Setup
  ledcAttach(L_RPWM, PWM_FREQ, PWM_BITS);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_BITS);
  ledcAttach(R_RPWM, PWM_FREQ, PWM_BITS);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_BITS);

  // Button Setup
  pinMode(BTN_CAL, INPUT_PULLUP);
  pinMode(BTN_GO,  INPUT_PULLUP);

  // Init sensor min/max
  for (int i = 0; i < NUM_SENSORS; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }
  
  stopMotors();
  Serial.println("TINKER-TRACER READY");
}

// ==============================================================
//                          MAIN LOOP
// ==============================================================
void loop() {
  // --- Calibrate Button ---
  if (digitalRead(BTN_CAL) == LOW) { 
    delay(30); // Debounce
    if (digitalRead(BTN_CAL) == LOW) { 
      runCalibration(); 
      while(digitalRead(BTN_CAL) == LOW); // Wait for release
    }
  }

  // --- Start/Stop Button ---
  if (digitalRead(BTN_GO) == LOW)  { 
    delay(30); // Debounce
    if (digitalRead(BTN_GO) == LOW)  { 
      toggleRun(); 
      while(digitalRead(BTN_GO) == LOW); // Wait for release
    }
  }

  // --- Robot Brain ---
  if (running) {
    readAndMapSensors(); 
    
    switch (currentState) {
      case FOLLOWING:          handleFollowing(); break;
      case SEARCHING:          handleSearching(); break;
      case HANDLING_JUNCTION:  handleJunction();  break;
      default: break;
    }
  }
}

// ==============================================================
//                    STATE: FOLLOWING
// ==============================================================
void handleFollowing() {
  // 1. Cross Junction Detection (Blast straight over it)
  if (activeSensors >= JUNCTION_COUNT) {
    currentState = HANDLING_JUNCTION;
    stateTimer = millis();
    return;
  }

  // 2. Broken Line Detection
  if (activeSensors == 0) {
    currentState = SEARCHING;
    stateTimer = millis();
    return;
  }

  // 3. HARD 90° TURN DETECTION (Crucial for curved 138mm PCB)
  // Because the PCB curves outwards, the line hits the extreme edges fast.
  // Left 90°: Sensors 0,1,2 active AND 3-15 are DEAD
  if ((lineMap & 0x0007) && !(lineMap & 0xFFF8)) { 
    lastKnownSide = -1;
    setMotor(L_RPWM, L_LPWM, -TURN_SPEED); // Left wheel reverse
    setMotor(R_RPWM, R_LPWM,  TURN_SPEED); // Right wheel forward
    integral = 0; lastError = -7.5; // Reset PID so it doesn't spike when straightening out
    return;
  }
  // Right 90°: Sensors 13,14,15 active AND 0-12 are DEAD
  else if ((lineMap & 0xE000) && !(lineMap & 0x1FFF)) { 
    lastKnownSide = 1;
    setMotor(L_RPWM, L_LPWM,  TURN_SPEED); 
    setMotor(R_RPWM, R_LPWM, -TURN_SPEED); 
    integral = 0; lastError = 7.5;
    return;
  }

  // 4. Standard PID + Dynamic Speed
  float pos = getWeightedPosition();
  float error = pos - 7.5; // Center is 7.5
  
  if (error < -1.5) lastKnownSide = -1;
  else if (error > 1.5) lastKnownSide = 1;

  integral += error;
  float derivative = error - lastError;
  
  float correction = (KP * error) + (KI * integral) + (KD * derivative);
  lastError = error;

  // --- DYNAMIC SPEED LOGIC ---
  int currentBaseSpeed;
  float absError = abs(error); 

  //if (absError < 1.0)      currentBaseSpeed = STRAIGHT_SPEED; 
  if (absError < 1.0)      currentBaseSpeed = 225; //i want to see smth tbh
  else if (absError < 2.0) currentBaseSpeed = GENTLE_SPEED;  
  else if (absError < 3.5) currentBaseSpeed = SHARP_SPEED;   
  else                     currentBaseSpeed = EXTREME_SPEED; 

  // Apply dynamic speed with PID correction
  int leftSpd  = constrain(currentBaseSpeed + (int)correction, -PWM_MAX, PWM_MAX);
  int rightSpd = constrain(currentBaseSpeed - (int)correction, -PWM_MAX, PWM_MAX);

  setMotor(L_RPWM, L_LPWM, leftSpd);
  setMotor(R_RPWM, R_LPWM, rightSpd);
}

// ==============================================================
//                    STATE: SEARCHING (Broken Line)
// ==============================================================
void handleSearching() {
  unsigned long elapsed = millis() - stateTimer;

  // Did we find the line again?
  if (activeSensors > 0) {
    currentState = FOLLOWING;
    integral = 0; // Reset PID memory
    return;
  }

  if (elapsed < 250) { 
    // Pivot in place towards last known side
    if (lastKnownSide <= 0) { 
      setMotor(L_RPWM, L_LPWM, -140); setMotor(R_RPWM, R_LPWM,  140);
    } else { 
      setMotor(L_RPWM, L_LPWM,  140); setMotor(R_RPWM, R_LPWM, -140);
    }
  } 
  else if (elapsed < 450) { 
    // Inch forward slowly to bridge the gap
    setMotor(L_RPWM, L_LPWM,  90); setMotor(R_RPWM, R_LPWM,  90);
  }
  else {
    // Totally lost. Stop and wait.
    stopMotors();
  }
}

// ==============================================================
//                    STATE: JUNCTION (Cross / T-Junction)
// ==============================================================
void handleJunction() {
  // Lock wheels straight for 350ms to blast over the cross lines
  if (millis() - stateTimer < 350) {
    setMotor(L_RPWM, L_LPWM, STRAIGHT_SPEED); 
    setMotor(R_RPWM, R_LPWM, STRAIGHT_SPEED);
    return;
  }

  // Resume following once we are past the thick intersection lines
  if (activeSensors < JUNCTION_COUNT && activeSensors > 0) {
    currentState = FOLLOWING;
    integral = 0; lastError = 0; // Reset PID
  }
}

// ==============================================================
//                   SENSOR MAPPING & MATH
// ==============================================================
void readAndMapSensors() {
  lineMap = 0;
  activeSensors = 0;
  readAllSensors();
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (normalized(i) > LINE_THRESHOLD) {
      lineMap |= (1 << i); // Set bit to 1
      activeSensors++;
    }
  }
}

void selectChannel(uint8_t ch) {
  digitalWrite(MUX_S0, ch & 0x01);
  digitalWrite(MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(MUX_S2, (ch >> 2) & 0x01);
  digitalWrite(MUX_S3, (ch >> 3) & 0x01);
}

int readChannel(uint8_t ch) {
  selectChannel(ch);
  delayMicroseconds(150);
  return analogRead(MUX_SIG);
}

void readAllSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) rawVals[i] = readChannel(i);
}

int normalized(uint8_t idx) {
  int range = sensorMax[idx] - sensorMin[idx];
  if (range < 50) range = 50; // Prevent divide by zero
  long val = ((long)(rawVals[idx] - sensorMin[idx]) * 1000L) / range;
  return constrain(val, 0, 1000);
}

float getWeightedPosition() {
  long sum = 0, weighted = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (lineMap & (1 << i)) { 
      int v = normalized(i);
      weighted += (long)i * v;
      sum += v;
    }
  }
  if (sum == 0) return 7.5; // Fallback to center if nothing seen
  return (float)weighted / (float)sum;
}

// ==============================================================
//                   CALIBRATION & CONTROLS
// ==============================================================
void runCalibration() {
  currentState = CALIBRATING;
  Serial.println("CALIBRATING...");
  digitalWrite(LED_PIN, HIGH);
  for (int i = 0; i < NUM_SENSORS; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }

  unsigned long t0 = millis();
  while (millis() - t0 < CALIB_TIME_MS) {
    // Spin back and forth over the line
    bool firstHalf = (millis() - t0) < (CALIB_TIME_MS / 2);
    if (firstHalf) { setMotor(L_RPWM, L_LPWM, CALIB_SPIN_SPD); setMotor(R_RPWM, R_LPWM, -CALIB_SPIN_SPD); }
    else           { setMotor(L_RPWM, L_LPWM, -CALIB_SPIN_SPD); setMotor(R_RPWM, R_LPWM, CALIB_SPIN_SPD); }
    
    readAllSensors();
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (rawVals[i] < sensorMin[i]) sensorMin[i] = rawVals[i];
      if (rawVals[i] > sensorMax[i]) sensorMax[i] = rawVals[i];
    }
    delay(5);
  }
  
  stopMotors();
  calibrated = true;
  digitalWrite(LED_PIN, LOW);
  currentState = IDLE;
  Serial.println("CALIBRATION DONE!");
}

void toggleRun() {
  if (!calibrated) { 
    Serial.println("CALIBRATE FIRST!"); 
    return; 
  }
  
  running = !running;
  if (!running) { 
    stopMotors(); 
    digitalWrite(LED_PIN, LOW); 
    currentState = IDLE; 
    Serial.println("STOPPED"); 
  }
  else { 
    integral = 0; 
    lastError = 0; 
    currentState = FOLLOWING; 
    digitalWrite(LED_PIN, HIGH); 
    Serial.println("RUNNING!"); 
  }
}

// ==============================================================
//                   MOTOR HELPER FUNCTIONS
// ==============================================================
void setMotor(int pinFwd, int pinRev, int speed) {
  if (speed > 0) {
    ledcWrite(pinFwd, speed);
    ledcWrite(pinRev, 0);
  } else if (speed < 0) {
    ledcWrite(pinFwd, 0);
    ledcWrite(pinRev, -speed); 
  } else {
    ledcWrite(pinFwd, 0);
    ledcWrite(pinRev, 0);
  }
}

void stopMotors() {
  ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, 0);
  ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, 0);
}
