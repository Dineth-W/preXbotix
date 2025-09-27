#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ===== User WiFi Configuration =====
const char* ssid = "💀";              // <--- CHANGE THIS
const char* password = "not123456";  // <--- CHANGE THIS


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
#define S4 19 //tx2
#define S5 18  //rx2

// Motor base speed
int baseSpeed = 80;  // 0-255 PWM

// Line detection mode
bool LINE_IS_BLACK = true;  // will auto-switch

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

  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PWMA, constrain(leftSpeed, 0, 255));

  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PWMB, constrain(rightSpeed, 0, 255));
}
float Kp = 40;
float Ki = 0;
float Kd = 20;
int lastError = 0;
int integral = 0;

void followLinePID() {
  int sensors = readSensors();
  int error = 0;

  // Assign weighted values to sensors for error calculation
  switch (sensors) {
    case 0b00100: error = 0; break;
    case 0b01100: error = -1; break;
    case 0b11000: error = -2; break;
    case 0b00110: error = 1; break;
    case 0b00011: error = 2; break;
    case 0b11111: error = 0; break;          // junction or circle
    case 0b00000: error = lastError; break;  // lost line, continue previous
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
bool isTJunction(int sensors) {
  return sensors == 0b11100 || sensors == 0b00111;
}
bool isCross(int sensors) {
  return sensors == 0b11111;
}
bool isDotLine(int sensors) {
  return sensors == 0b00010 || sensors == 0b01000;
}
bool isUturn(int sensors) {
  return sensors == 0b00000 && lastError != 0;
}
bool isCircle(int sensors) {
  return sensors == 0b11111 && lastError == 0;
}

void handleFeatures(int sensors) {
  if (isTJunction(sensors)) {
    setMotors(0, 0);
    delay(100);
    // example: always turn right at T-junction
    setMotors(baseSpeed, -baseSpeed);
    delay(300);
  } else if (isCross(sensors)) {
    // cross detected, go straight
    followLinePID();
  } else if (isDotLine(sensors)) {
    // slow down for dotted line
    followLinePID();
    delay(50);
  } else if (isUturn(sensors)) {
    setMotors(-baseSpeed, baseSpeed);  // U-turn
    delay(500);
  } else if (isCircle(sensors)) {
    followLinePID();  // circle
  } else {
    followLinePID();  // normal line
  }
}
void autoDetectLineColor() {
  // Read middle sensor only
  int mid = digitalRead(S3);
  // If lost line repeatedly, invert detection
  static int lostCount = 0;
  if (mid != LINE_IS_BLACK) lostCount++;
  else lostCount = 0;
  if (lostCount > 5) {
    LINE_IS_BLACK = !LINE_IS_BLACK;
    lostCount = 0;
  }
}
void setup() {
  // put your setup code here, to run once:
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname("esp32-linefollower");
  ArduinoOTA.begin();
  Serial.println("OTA Ready! You can upload new code over WiFi.");

  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
}



void loop() {
  ArduinoOTA.handle();
  autoDetectLineColor();
  int sensors = readSensors();
  handleFeatures(sensors);
}
