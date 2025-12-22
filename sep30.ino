#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ===== User WiFi Configuration =====
const char* ssid = "💀";
const char* password = "not123456";  

// ===== Motor driver pins (TB6612FNG) =====
#define AIN1 14
#define AIN2 12
#define PWMA 13
#define BIN1 26
#define BIN2 25
#define PWMB 33
#define STBY 27

// ===== IR sensor pins =====
#define S1 23  // left-most
#define S2 22  // left-mid
#define S3 21  // center
#define S4 19  // right-mid
#define S5 18  // right-most

// ===== Motor base speed and tuning =====
int baseSpeed = 80;
int maxTurnAdjustment = 50;

// ===== Line detection state =====
int lineType = 0;
unsigned long lastMotionTime = 0;
bool isStopped = false;
static bool initialRun = true;

// ===== Robot dimensions and motion params =====
float sensorToCenter_mm = 90.0;
float wheelDiameter_mm = 43.0;
float motorRPM = 500.0;
float wheelCircum_mm = 3.1416 * wheelDiameter_mm;
float speed_mm_s = (motorRPM / 60.0) * wheelCircum_mm * (baseSpeed / 255.0);
int preTurnDelay_ms = int(sensorToCenter_mm / speed_mm_s * 1000);

// ===== PID variables =====
float Kp = 13.0;
float Ki = 0.05;
float Kd = 18.0;
float integral = 0;
float lastError = 0;

// ===== Line confidence for dotted line filtering =====
int lineConfidence = 0;

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);  digitalWrite(STBY, HIGH);
 
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  Serial.begin(115200);
  delay(2000);
  Serial.print("Calculated pre-turn delay (ms): ");
  Serial.println(preTurnDelay_ms);
}

// ===== Motor control helper =====
void setMotors(int leftSpeed, int rightSpeed) {
 
  if (leftSpeed == 0 && rightSpeed == 0) {
    if (!isStopped) {
      lastMotionTime = millis();
      isStopped = true;
    }
  }
  else {
    isStopped = false;
  }

  // Existing motor control logic...

  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, leftSpeed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -leftSpeed);
  }
  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, rightSpeed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -rightSpeed);
  }
}

// ===== Read and invert sensors =====
void readSensors(int s[]) {
  s[0] = digitalRead(S1);
  s[1] = digitalRead(S2);
  s[2] = digitalRead(S3);
  s[3] = digitalRead(S4);
  s[4] = digitalRead(S5);

  if (lineType == 2) {
    for (int i = 0; i < 5; i++) s[i] = !s[i];
  }
}

// ===== Determine line type =====
void detectLineType() {
    int white_count = 0, black_count = 0;
    const int sensors[5] = {S1, S2, S3, S4, S5};

    for (int i = 0; i < 5; i++) {
        int val = digitalRead(sensors[i]);
        if (val == HIGH) white_count++;
        else black_count++;
        delay(10);
    }

    int lineType = (white_count > black_count) ? 1 : 2;

    Serial.print("Detected line type: ");
    Serial.println(lineType == 1 ? "Black line on White background" : "White line on Black background");
}

// ===== Turn until center sensor detects line =====
void turnUntilCenter(int s1, int s2, int s4, int s5) {
  int turnSpeed = 100;
  int direction = 0;
  unsigned long startTime = millis();
  unsigned long maxTurnTime = 800;

  if ((s1 || s2) && !(s4 || s5)) direction = -1;
  else if ((s4 || s5) && !(s1 || s2)) direction = 1;
  else if ((s1 || s2) && (s4 || s5)) direction = -1;

  while (direction != 0 && millis() - startTime < maxTurnTime) {
    int s3 = digitalRead(S3);
    if (lineType == 2) s3 = !s3;
    if (s3 == 1) break;

    if (direction == -1)
      setMotors(-turnSpeed + 30, turnSpeed + 30);
    else
      setMotors(turnSpeed + 30, -turnSpeed + 30);

    delay(20);
  }

  setMotors(0, 0);
}

// ===== Spin search for line =====
bool spinSearchForLine() {
  int spinSpeed = 100;
  unsigned long startTime = millis();
  int s[5];
  while (millis() - startTime < 1000) {
    setMotors(spinSpeed, -spinSpeed);
    readSensors(s);
    if (s[0] || s[1] || s[2] || s[3] || s[4]) return true;
    delay(10);
  }
  setMotors(0, 0);
  return false;


  // ===== Initial straight motion for `1.5 =====
void initial() {
  if (initialRun) {
    int forwardTime_ms = int((500.0 / speed_mm_s) * 1000);  // time in milliseconds
    unsigned long startTime = millis();
    while ((millis() - startTime) < forwardTime_ms) {
      setMotors(baseSpeed, baseSpeed);
      delay(10);
    }
    initialRun = false;
  }
}

}

void loop() {

  initial(); //runs only ones(hopefully).
  detectLineType();
  int s[5];
  readSensors(s);
  

  // ===== Dotted line confidence filter =====
  if (s[2] == 1) lineConfidence = min(lineConfidence + 1, 5);
  else lineConfidence = max(lineConfidence - 1, 0);

  // ===== Line lost recovery =====
  if (s[0] == 0 && s[1] == 0 && s[2] == 0 && s[3] == 0 && s[4] == 0) //All black (Biggining, Cross, T).
  {
    unsigned long startTime = millis();
    bool found = false;
    setMotors(baseSpeed, baseSpeed);
    while (millis() - startTime < 500) {
      readSensors(s);
      if (s[0] || s[1] || s[2] || s[3] || s[4]) {
        found = true;
        break;
      }
      delay(10);
    }
    if (!found && !spinSearchForLine()) {
      setMotors(0, 0);
      delay(100);
      return;
    }
    readSensors(s);
  }

  // ===== Off-track recovery =====
  if (lineConfidence == 0) {
    if ((s[0] || s[1]) && !(s[3] || s[4])) {
      setMotors(baseSpeed, baseSpeed);
      delay(preTurnDelay_ms);
      turnUntilCenter(s[0], s[1], s[3], s[4]);
    } else if ((s[3] || s[4]) && !(s[0] || s[1])) {
      setMotors(baseSpeed, baseSpeed);
      delay(preTurnDelay_ms);
      turnUntilCenter(s[0], s[1], s[3], s[4]);
    } else if ((s[0] || s[1]) && (s[3] || s[4])) {
      setMotors(baseSpeed, baseSpeed);
      delay(preTurnDelay_ms);
      turnUntilCenter(s[0], s[1], s[3], s[4]);
    }
    readSensors(s);
  }

  // ===== Hard turn detection =====
  bool hardTurnLeft = (s[0] == 1 && s[1] == 1 && s[2] == 0);
  bool hardTurnRight = (s[4] == 1 && s[3] == 1 && s[2] == 0);

  if (hardTurnLeft) {
    setMotors(-baseSpeed, baseSpeed);
    delay(200);
    return;
  } else if (hardTurnRight) {
    setMotors(baseSpeed, -baseSpeed);
    delay(200);
    return;
  }

  // ===== Weighted error calculation =====
  int weights[5] = { -4, -2, 0, 2, 4 };
  int total = 0, active = 0;
  for (int i = 0; i < 5; i++) {
    if (s[i] == 1) {
      total += weights[i];
      active++;
    }
  }
  int error = (active > 0) ? total / active : lastError;

  // ===== Dynamic PID tuning =====
  int absError = abs(error);
  if (absError >= 4) {
    Kp = 17.0;
    Kd = 22.0;
  } else if (absError >= 2) {
    Kp = 14.0;
    Kd = 18.0;
  } else {
    Kp = 11.0;
    Kd = 16.0;
  }
  Ki = 0.05;

  // ===== PID calculation =====
  integral += error;
  float derivative = error - lastError;
  float pid = Kp * error + Ki * integral + Kd * derivative;
  pid = constrain(pid, -maxTurnAdjustment, maxTurnAdjustment);
  lastError = error;
// === Recovery Shake if stalled ===
if ((isStopped && millis() - lastMotionTime) > 2000) {
  Serial.println("Stalled: initiating shake recovery");

  for (int i = 0; i < 6; i++) {
    setMotors(60, -60); delay(150);  // quick left
    setMotors(-60, 60); delay(150);  // quick right
    setMotors(0, 0); delay(100);

    int center = digitalRead(S3);
    if (lineType == 2) center = !center;
    if (center == 1) {
      Serial.println("Line re-centered under S3");
      break;
    }
  }

  lastMotionTime = millis();  // reset timer
}
  // ===== Motor speed adjustment =====
  int leftSpeed = baseSpeed + pid;
  int rightSpeed = baseSpeed - pid;
  leftSpeed = constrain(leftSpeed, -150, 150);
  rightSpeed = constrain(rightSpeed, -150, 150);

  setMotors(leftSpeed, rightSpeed);

  // ===== Optional debug output =====
  // Serial.print("Sensors: ");
  // for (int i = 0; i < 5; i++) Serial.print(s[i]);
  // Serial.print(" | Error: "); Serial.print(error);
  // Serial.print(" | PID: "); Serial.print(pid);
  // Serial.print(" | L: "); Serial.print(leftSpeed);
  // Serial.print(" R: "); Serial.println(rightSpeed);

  delay(30);  // fast loop for responsive control

}
