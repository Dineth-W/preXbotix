// ===== Motor driver pins (TB6612FNG) =====
#define AIN1 14
#define AIN2 12
#define PWMA 13
#define BIN1 26
#define BIN2 25
#define PWMB 33
#define STBY 27

// ===== IR sensor pins =====
#define S1 23   // left-most
#define S2 22   // left-mid
#define S3 21   // center
#define S4 19   // right-mid
#define S5 18   // right-most

// ===== Motor base speed =====
int baseSpeed = 130;      // max speed
int maxTurnAdjustment = 50; // max speed adjustment for turns (0-255 )    100

// ===== Line detection state =====
// 0 = unknown, 1 = black line, 2 = white line
int lineType = 0;

// ===== Robot dimensions and motion params =====
float sensorToCenter_mm = 145.0;    // distance from sensors to robot center
float wheelDiameter_mm = 43.0;      // wheel diameter
float wheelRadius_mm = wheelDiameter_mm/2.0;
float motorRPM = 500.0;             // motor RPM
float wheelCircum_mm = 3.1416 * wheelDiameter_mm;
float speed_mm_s = (motorRPM / 60.0) * wheelCircum_mm * (baseSpeed/255.0); // rough real speed estimation

// ===== Turning delay calculation =====
// For a gentle pre-turn delay: let robot move forward about the distance from sensor to center
int preTurnDelay_ms = int(sensorToCenter_mm / speed_mm_s * 1000); // ms

// ===== PID variables =====
float Kp = 15.0;  // Proportional gain
float Ki = 0.0;   // Integral gain
float Kd = 8.0;  // Derivative gain
float integral = 0;
float lastError = 0;

void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  pinMode(S1, INPUT); pinMode(S2, INPUT);
  pinMode(S3, INPUT); pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  Serial.begin(115200);

  // --- Add startup delay of 2 seconds ---
  delay(2000);

  // --- Print calculated pre-turn delay ---
  Serial.print("Calculated pre-turn delay (ms): ");
  Serial.println(preTurnDelay_ms);
}

// ===== Motor control helper =====
void setMotors(int leftSpeed, int rightSpeed){
  // left motor
  if(leftSpeed >= 0){
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, leftSpeed);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -leftSpeed);
  }
  // right motor
  if(rightSpeed >= 0){
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, rightSpeed);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -rightSpeed);
  }
}

// ===== Determine line type =====
void detectLineType() {
  // If uninitialized, detect line type by sampling center sensor
  if (lineType == 0) {
    int white_count = 0, black_count = 0;
    // Sample center sensor for a few cycles
    for (int i = 0; i < 10; i++) {
      int val = digitalRead(S3);
      if (val == 1) white_count++;
      else black_count++;
      delay(10);
    }
    // If majority are white, assume black line on white; else white line on black
    if (white_count > black_count) lineType = 1;
    else lineType = 2;
    Serial.print("Detected line type: ");
    if (lineType == 1) Serial.println("Black line on White background");
    else Serial.println("White line on Black background");
  }
}

// ===== Turn in the shortest way until center sensor detects line =====
void turnUntilCenter(int s1, int s2, int s4, int s5) {
  int turnSpeed = 100;
  int direction = 0;

  // Decide the direction based on sensor readings
  if ((s1 == 1 || s2 == 1) && (s4 == 0 && s5 == 0)) {
    direction = -1; // turn left
  } else if ((s4 == 1 || s5 == 1) && (s1 == 0 && s2 == 0)) {
    direction = 1; // turn right
  } else if ((s1 == 1 || s2 == 1) && (s4 == 1 || s5 == 1)) {
    // Both sides see line, prefer left
    direction = -1;
  } else {
    direction = 0; // no clear direction, stop
  }

  // Only turn if a direction is set
  while (direction != 0) {
    int s3 = digitalRead(S3);
    if (lineType == 2) s3 = !s3;
    if (s3 == 1) {
      // Found the line, break and continue normal operation
      break;
    }
    if (direction == -1) {
      setMotors(-turnSpeed, turnSpeed); // turn left
    } else if (direction == 1) {
      setMotors(turnSpeed, -turnSpeed); // turn right
    }
    delay(20);
  }
  // Once found, do NOT stop motors here - let normal loop code take over
}

// ===== Spin in place until any sensor detects the line =====
bool spinSearchForLine() {
  int spinSpeed = 100;
  unsigned long startTime = millis();
  unsigned long spinDuration = 1000; // Max 1 second spin (adjust as needed)
  while (millis() - startTime < spinDuration) {
    setMotors(spinSpeed, -spinSpeed); // spin right
    int s1 = digitalRead(S1);
    int s2 = digitalRead(S2);
    int s3 = digitalRead(S3);
    int s4 = digitalRead(S4);
    int s5 = digitalRead(S5);
    if (lineType == 2) { 
      s1 = !s1; s2 = !s2; s3 = !s3; s4 = !s4; s5 = !s5;
    }
    if (s1 == 1 || s2 == 1 || s3 == 1 || s4 == 1 || s5 == 1) {
      // Found a line
      return true;
    }
    delay(10);
  }
  setMotors(0, 0); // stop after spin
  return false;
}

void loop() {
  // Read all sensors
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);

  // Detect line type if needed (uses only S3)
  detectLineType();

  // Auto inversion according to detected line type
  // Sensor output: 1 = white, 0 = black
  // For black line: 0 = line, for white line: 1 = line
  if (lineType == 2) { 
    s1 = !s1; s2 = !s2; s3 = !s3; s4 = !s4; s5 = !s5;
  }

  // ===== No sensors see line: go forward for 500ms, then spin if still no line =====
  if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
    // Go forward for 500ms
    unsigned long startTime = millis();
    bool found = false;
    setMotors(baseSpeed, baseSpeed);
    while (millis() - startTime < 500) {
      int t1 = digitalRead(S1);
      int t2 = digitalRead(S2);
      int t3 = digitalRead(S3);
      int t4 = digitalRead(S4);
      int t5 = digitalRead(S5);
      if (lineType == 2) { 
        t1 = !t1; t2 = !t2; t3 = !t3; t4 = !t4; t5 = !t5;
      }
      if (t1 == 1 || t2 == 1 || t3 == 1 || t4 == 1 || t5 == 1) {
        found = true;
        break;
      }
      delay(10);
    }

    // If found during forward, continue normal operation (don't stop)
    if (found) {
      // Proceed as normal (PID will take over at end of loop)
    } else {
      // If not found, spin once to search for line
      if (spinSearchForLine()) {
        // Found during spin, proceed as normal
      } else {
        // Still not found, stop
        setMotors(0, 0);
        delay(100);
        return;
      }
    }
    // After recovery, re-read sensors
    s1 = digitalRead(S1);
    s2 = digitalRead(S2);
    s3 = digitalRead(S3);
    s4 = digitalRead(S4);
    s5 = digitalRead(S5);
    if (lineType == 2) { 
      s1 = !s1; s2 = !s2; s3 = !s3; s4 = !s4; s5 = !s5;
    }
  }

  // If center sensor does NOT see line, but either side does, pre-turn delay then turn until center sees line again
  // Now, check using all sensors for off-track detection
  if (s3 == 0) { // off track
    if ((s1 == 1 || s2 == 1) && (s4 == 0 && s5 == 0)) {
      // Left sensors see line, right sensors don't
      setMotors(baseSpeed, baseSpeed);
      delay(preTurnDelay_ms);
      turnUntilCenter(s1, s2, s4, s5);
    } else if ((s5 == 1 || s4 == 1) && (s1 == 0 && s2 == 0)) {
      // Right sensors see line, left sensors don't
      setMotors(baseSpeed, baseSpeed);
      delay(preTurnDelay_ms);
      turnUntilCenter(s1, s2, s4, s5);
    } else if ((s1 == 1 || s2 == 1) && (s4 == 1 || s5 == 1)) {
      // Both sides see line, prefer left
      setMotors(baseSpeed, baseSpeed);
      delay(preTurnDelay_ms);
      turnUntilCenter(s1, s2, s4, s5);
    }
    // After recovery, re-read sensors
    s1 = digitalRead(S1);
    s2 = digitalRead(S2);
    s3 = digitalRead(S3);
    s4 = digitalRead(S4);
    s5 = digitalRead(S5);
    if (lineType == 2) { 
      s1 = !s1; s2 = !s2; s3 = !s3; s4 = !s4; s5 = !s5;
    }
  }

  // ===== New error calculation (with lastError fallback) =====
  int error = 0;
  if (s1 == 1 && s2 == 0) error = -5;
  else if (s1 == 1 && s2 == 1) error = -3;
  else if (s2 == 1 && s3 == 0) error = -2;
  else if (s2 == 1 && s3 == 1) error = -1;
  else if (s3 == 1 && s2 == 0 && s4 == 0) error = 0;
  else if (s3 == 1 && s4 == 1) error = 1;
  else if (s4 == 1 && s3 == 0) error = 2;
  else if (s4 == 1 && s5 == 1) error = 3;
  else if (s5 == 1 && s4 == 0) error = 5;
  else error = lastError; // fallback if no match

  // ===== PID controller =====
  integral += error;
  float derivative = error - lastError;
  float pid = Kp*error + Ki*integral + Kd*derivative;
  lastError = error;

  int leftSpeed = baseSpeed + pid;
  int rightSpeed = baseSpeed - pid;

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -180, 180);     //150
  rightSpeed = constrain(rightSpeed, -180, 180);

  // Set motors
  setMotors(leftSpeed, rightSpeed);

  // --- Print for debug ---
  // Serial.print("S1:"); Serial.print(s1);
  // Serial.print(" S2:"); Serial.print(s2);
  // Serial.print(" S3:"); Serial.print(s3);
  // Serial.print(" S4:"); Serial.print(s4);
  // Serial.print(" S5:"); Serial.print(s5);
  // Serial.print(" | L:"); Serial.print(leftSpeed);
  // Serial.print(" R:"); Serial.print(rightSpeed);
  // Serial.print(" | err:"); Serial.print(error);
  // Serial.print(" | PID:"); Serial.print(pid);
  // Serial.print(" | LineType:"); Serial.println(lineType == 1 ? "Black" : "White");

  delay(50);
}
