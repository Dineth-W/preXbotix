#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ===== User WiFi Configuration =====
const char* ssid = "💀";              
const char* password = "not123456";  

// Motor pins
#define AIN1 14
#define AIN2 12
#define PWMA 13
#define BIN1 26
#define BIN2 25
#define PWMB 33
#define STBY 27

// IR sensor pins
#define S1 23
#define S2 22
#define S3 21
#define S4 19 
#define S5 18  

// Motor base speed
int baseSpeed = 80;  // normal speed

// Line detection mode
bool LINE_IS_BLACK = true;

// PID parameters
float Kp = 40;
float Ki = 0;
float Kd = 20;
int lastError = 0;
int integral = 0;

// Robot states
enum RobotState { FOLLOW, TURN_RIGHT, TURN_LEFT, UTURN, STOP };
RobotState state = FOLLOW;
unsigned long stateStart = 0;
int turnTimeout = 1000; // max turn duration in ms

// ================== Functions ==================
int readSensors() {
  int value = 0;
  value |= (digitalRead(S1) == LINE_IS_BLACK ? 1 : 0) << 4;
  value |= (digitalRead(S2) == LINE_IS_BLACK ? 1 : 0) << 3;
  value |= (digitalRead(S3) == LINE_IS_BLACK ? 1 : 0) << 2;
  value |= (digitalRead(S4) == LINE_IS_BLACK ? 1 : 0) << 1;
  value |= (digitalRead(S5) == LINE_IS_BLACK ? 1 : 0) << 0;
  return value;
}

void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(STBY, HIGH);

  if (leftSpeed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); leftSpeed = -leftSpeed; }
  analogWrite(PWMA, constrain(leftSpeed, 0, 255));

  if (rightSpeed >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); rightSpeed = -rightSpeed; }
  analogWrite(PWMB, constrain(rightSpeed, 0, 255));
}

// PID line following
void followLinePID() {
  int sensors = readSensors();
  int error = 0;

  switch(sensors) {
    case 0b00100: error = 0; break;
    case 0b01100: error = -1; break;
    case 0b11000: error = -2; break;
    case 0b00110: error = 1; break;
    case 0b00011: error = 2; break;
    case 0b11111: error = 0; break;          // junction
    case 0b00000: error = lastError; break;  // lost line
    default: error = lastError; break;
  }

  integral += error;
  int derivative = error - lastError;
  int correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;
  setMotors(leftSpeed, rightSpeed);
}

// Flexible junction detection
bool isJunction(int sensors) {
  int count = 0;
  for(int i=0;i<5;i++) if(sensors & (1<<i)) count++;
  return count >= 3; // 3+ sensors active -> junction
}

// Detect U-turn (lost line)
bool isUTurn(int sensors) {
  return sensors == 0b00000 && lastError != 0;
}

// Turn helpers (non-blocking)
void turnRight() {
  setMotors(baseSpeed, -baseSpeed);
}

void turnLeft() {
  setMotors(-baseSpeed, baseSpeed);
}

void uTurn() {
  setMotors(-baseSpeed, baseSpeed);
}

// Auto-detect line color
void autoDetectLineColor() {
  static int lostCount = 0;
  int mid = digitalRead(S3);
  if(mid != LINE_IS_BLACK) lostCount++;
  else lostCount = 0;
  if(lostCount > 5) { LINE_IS_BLACK = !LINE_IS_BLACK; lostCount = 0; }
}

// ================== Setup ==================
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED){ delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());

  ArduinoOTA.setHostname("esp32-linefollower");
  ArduinoOTA.begin();
  Serial.println("OTA Ready!");

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(S1, INPUT); pinMode(S2, INPUT); pinMode(S3, INPUT); pinMode(S4, INPUT); pinMode(S5, INPUT);
}

// ================== Loop ==================
void loop() {
  ArduinoOTA.handle();
  autoDetectLineColor();
  int sensors = readSensors();

  switch(state) {
    case FOLLOW:
      followLinePID();
      if(isJunction(sensors)) { state = TURN_RIGHT; stateStart = millis(); }
      else if(isUTurn(sensors)) { state = UTURN; stateStart = millis(); }
      break;

    case TURN_RIGHT:
      turnRight();
      if(millis()-stateStart > turnTimeout || !isJunction(sensors)) state = FOLLOW;
      break;

    case TURN_LEFT:
      turnLeft();
      if(millis()-stateStart > turnTimeout || !isJunction(sensors)) state = FOLLOW;
      break;

    case UTURN:
      uTurn();
      if(millis()-stateStart > turnTimeout || !isUTurn(sensors)) state = FOLLOW;
      break;

    case STOP:
      setMotors(0,0);
      break;
  }
}
