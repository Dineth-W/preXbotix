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
#define S4 19   // right-mid (unreliable)
#define S5 18   // right-most

// ===== Motor base speed =====
int baseSpeed = 100;      // max speed
int maxTurnAdjustment = 100; // max speed adjustment for turns (0-255)

// ===== WiFi credentials =====
#include <WiFi.h>
const char* ssid = "Dineth";
const char* password = "not123456";

// ===== Line detection state =====
// 0 = unknown, 1 = black line, 2 = white line
int lineType = 0;

// ===== Robot dimensions and motion params =====
float sensorToCenter_mm = 160.0;    // distance from sensors to robot center (updated)
float wheelDiameter_mm = 43.0;      // wheel diameter
float wheelRadius_mm = wheelDiameter_mm/2.0;
float motorRPM = 500.0;             // motor RPM
float wheelCircum_mm = 3.1416 * wheelDiameter_mm;
float speed_mm_s = (motorRPM / 60.0) * wheelCircum_mm * (baseSpeed/255.0); // rough real speed estimation

// ===== Turning delay calculation =====
// For a gentle pre-turn delay: let robot move forward about the distance from sensor to center
int preTurnDelay_ms = int(sensorToCenter_mm / speed_mm_s * 1000); // ms

void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  pinMode(S1, INPUT); pinMode(S2, INPUT);
  pinMode(S3, INPUT); pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  Serial.begin(115200);

  // --- Connect to WiFi ---
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi failed to connect.");
  }

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

// ===== Turning until center sensor detects line =====
void turnUntilCenter(int direction) {
  // direction: -1 for left, 1 for right
  int turnSpeed = 80;
  bool foundLine = false;
  while (!foundLine) {
    int s3 = digitalRead(S3);
    if (lineType == 2) s3 = !s3;
    if (s3 == 1) {
      foundLine = true;
      break;
    }
    if (direction == -1) {
      setMotors(-turnSpeed, turnSpeed); // turn left
      Serial.println("Turning Left...");
    } else {
      setMotors(turnSpeed, -turnSpeed); // turn right
      Serial.println("Turning Right...");
    }
    delay(30);
  }
}

// ===== Check if robot is stopped =====
bool isRobotStopped(int leftSpeed, int rightSpeed) {
  return leftSpeed == 0 && rightSpeed == 0;
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

  // If center sensor does NOT see line, but either side does, pre-turn delay then turn until center sees line again
  // Now, check using all sensors for off-track detection
  if (s3 == 0) { // off track
    if ((s1 == 1 || s2 == 1) && (s4 == 0 && s5 == 0)) {
      // Left sensors see line, right sensors don't
      setMotors(baseSpeed, baseSpeed);
      delay(preTurnDelay_ms);
      turnUntilCenter(-1); // turn left
    } else if ((s5 == 1 || s4 == 1) && (s1 == 0 && s2 == 0)) {
      // Right sensors see line, left sensors don't
      setMotors(baseSpeed, baseSpeed);
      delay(preTurnDelay_ms);
      turnUntilCenter(1); // turn right
    } else if ((s1 == 1 || s2 == 1) && (s4 == 1 || s5 == 1)) {
      // Both sides see line, prefer left
      setMotors(baseSpeed, baseSpeed);
      delay(preTurnDelay_ms);
      turnUntilCenter(-1); // prefer left
    } else {
      // No line detected, stop or slow search
      setMotors(0, 0);
      Serial.println("Line lost. Stopping.");
      delay(100);
      // --- Sensor 4 unreliability handling ---
      // If S4 is different than all other sensors, and S3 does not see line, keep turning right until S3 aligns
      if ((s4 != s1 && s4 != s2 && s4 != s5) && s3 == 0) {
        Serial.println("Sensor 4 unreliable. Turning right until aligned.");
        turnUntilCenter(1); // turn right until center sees line
      }
      return;
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

  // Compute "position error" from center using all 5 sensors
  // Assign weights: S1=-4, S2=-2, S3=0, S4=2, S5=4
  int error = (-4)*s1 + (-2)*s2 + 0*s3 + 2*s4 + 4*s5;

  // Scale error to motor speed adjustment
  int adjust = map(error, -4, 4, -maxTurnAdjustment, maxTurnAdjustment);

  int leftSpeed = baseSpeed + adjust;
  int rightSpeed = baseSpeed - adjust;

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -150, 150);
  rightSpeed = constrain(rightSpeed, -150, 150);

  // Set motors
  setMotors(leftSpeed, rightSpeed);

  // --- Sensor 4 unreliability check when robot is stopped ---
  if (isRobotStopped(leftSpeed, rightSpeed)) {
    // If S4 is different from S1, S2, and S5, and S3 is not on line, turn right until S3 aligns
    if ((s4 != s1 && s4 != s2 && s4 != s5) && s3 == 0) {
      Serial.println("Sensor 4 unreliable (stopped). Turning right until aligned.");
      turnUntilCenter(1);
    }
  }

  // --- Print for debug ---
  Serial.print("S1:"); Serial.print(s1);
  Serial.print(" S2:"); Serial.print(s2);
  Serial.print(" S3:"); Serial.print(s3);
  Serial.print(" S4:"); Serial.print(s4);
  Serial.print(" S5:"); Serial.print(s5);
  Serial.print(" | L:"); Serial.print(leftSpeed);
  Serial.print(" R:"); Serial.print(rightSpeed);
  Serial.print(" | LineType:"); Serial.println(lineType == 1 ? "Black" : "White");

  delay(50);
}
