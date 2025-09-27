// ===== Required Libraries =====
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ===== User WiFi Configuration =====
const char* ssid = "💀";        // <--- CHANGE THIS
const char* password = "not123456"; // <--- CHANGE THIS

// ===== Pin Definitions =====

// IR Sensor Array (digital outputs) - Left to Right
#define IR_SENSOR_1  23
#define IR_SENSOR_2  22
#define IR_SENSOR_3  21
#define IR_SENSOR_4  19
#define IR_SENSOR_5  18

// TB6612FNG Motor Driver Pins
#define AIN1  14   // Motor A Direction 1
#define AIN2  12   // Motor A Direction 2
#define PWMA  13   // Motor A PWM

#define BIN1  26   // Motor B Direction 1
#define BIN2  25   // Motor B Direction 2
#define PWMB  33   // Motor B PWM

#define STBY  27   // Standby

// ===== Speed Constant =====
#define SPEED 100    // 65% of 255 (max for 8-bit PWM): 255 * 0.65 ≈ 166
#define DELAY 500

// ===== PWM Setup for ESP32 (New API) =====
void setupPWM() {
  ledcAttach(PWMA, 20000, 8); // Attach PWM to pin PWMA, 20kHz, 8-bit resolution
  ledcAttach(PWMB, 20000, 8); // Attach PWM to pin PWMB, 20kHz, 8-bit resolution
}

// ===== Helper: Read IR Sensors with Color Detection =====
void readIRSensors(bool isBlackLine, int sensors[5]) {
  sensors[0] = digitalRead(IR_SENSOR_1);
  sensors[1] = digitalRead(IR_SENSOR_2);
  sensors[2] = digitalRead(IR_SENSOR_3);
  sensors[3] = digitalRead(IR_SENSOR_4);
  sensors[4] = digitalRead(IR_SENSOR_5);

  // Invert readings for white line (on black background)
  if (!isBlackLine) {
    for (int i = 0; i < 5; i++) {
      sensors[i] = !sensors[i];
    }
  }
}

// ===== Print IR Sensor Values to Serial Terminal =====
void printIRSensors(const int sensors[5]) {
  Serial.print("IR: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(sensors[i]);
    if (i != 4) Serial.print(" ");
  }
  Serial.println();
}

// ===== Detect Line Color at Startup =====
bool detectLineColor() {
  int blackCount = 0, whiteCount = 0;
  int s[5];
  s[0] = digitalRead(IR_SENSOR_1);
  s[1] = digitalRead(IR_SENSOR_2);
  s[2] = digitalRead(IR_SENSOR_3);
  s[3] = digitalRead(IR_SENSOR_4);
  s[4] = digitalRead(IR_SENSOR_5);

  for (int i = 0; i < 5; i++) {
    if (s[i] == LOW) blackCount++;
    else whiteCount++;
  }
  return (blackCount > whiteCount); // true: black line, false: white line
}

// ===== Movement Functions =====
void moveForward() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, SPEED);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, SPEED);
}

void stopMotors() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
}

void turnRightUntilCenter(bool isBlackLine) {
  int sensors[5];
  unsigned long startTime = millis();
  const unsigned long timeout = 2000;
  do {
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, SPEED);
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); analogWrite(PWMB, SPEED);
    readIRSensors(isBlackLine, sensors);
    printIRSensors(sensors); // Print input values while turning
    delay(DELAY);
    if (millis() - startTime > timeout) break;
  } while (sensors[2] == 0);
  stopMotors();
  delay(DELAY);
  moveForward();
}

void turnLeftUntilCenter(bool isBlackLine) {
  int sensors[5];
  unsigned long startTime = millis();
  const unsigned long timeout = 2000;
  do {
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); analogWrite(PWMA, SPEED);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  analogWrite(PWMB, SPEED);
    readIRSensors(isBlackLine, sensors);
    printIRSensors(sensors); // Print input values while turning
    delay(DELAY);
    if (millis() - startTime > timeout) break;
  } while (sensors[2] == 0);
  stopMotors();
  delay(DELAY);
  moveForward();
}

// ===== Search for Line When Lost =====
void searchForLine(bool isBlackLine) {
  int sensors[5];
  // Move forward for 500ms
  moveForward();
  unsigned long startTime = millis();
  bool found = false;
  while (millis() - startTime < 500) {
    readIRSensors(isBlackLine, sensors);
    printIRSensors(sensors); // Print input values while searching
    // If sensor pattern changes, act accordingly
    if (!( // check for not all 0 or all 1
      (sensors[0]==0 && sensors[1]==0 && sensors[2]==0 && sensors[3]==0 && sensors[4]==0) ||
      (sensors[0]==1 && sensors[1]==1 && sensors[2]==1 && sensors[3]==1 && sensors[4]==1)
    )) {
      found = true;
      break;
    }
    delay(DELAY);
  }
  stopMotors();
  if (found) return; // Main loop will handle new input

  // If still lost, rotate 360° (spin in place)
  startTime = millis();
  unsigned long rotateTime = 1200; // adjust for robot to rotate full circle
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, SPEED);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, SPEED);
  while (millis() - startTime < rotateTime) {
    readIRSensors(isBlackLine, sensors);
    printIRSensors(sensors); // Print input values while spinning
    // If sensor detects line, decide which way
    if (sensors[2]==1) {
      stopMotors();
      // If left sensors active, initiate turn left; if right sensors, turn right
      if (sensors[0] || sensors[1]) turnLeftUntilCenter(isBlackLine);
      else if (sensors[3] || sensors[4]) turnRightUntilCenter(isBlackLine);
      else moveForward();
      return;
    }
    delay(DELAY);
  }
  stopMotors(); // If still lost, stop
}

// ===== Arduino Setup =====
void setup() {
  Serial.begin(115200);

  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);
  pinMode(IR_SENSOR_3, INPUT);
  pinMode(IR_SENSOR_4, INPUT);
  pinMode(IR_SENSOR_5, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  setupPWM();

  // ===== WiFi & OTA Setup =====
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
}

// ===== Main Loop =====
void loop() {
  ArduinoOTA.handle(); // Listen for OTA updates

  static bool isBlackLine = detectLineColor(); // Detect once at startup
  int sensors[5];

  readIRSensors(isBlackLine, sensors);
  printIRSensors(sensors); // Print input values every loop

  // If all sensors are 0 or all 1, perform search routine
  if (
    (sensors[0]==0 && sensors[1]==0 && sensors[2]==0 && sensors[3]==0 && sensors[4]==0) ||
    (sensors[0]==1 && sensors[1]==1 && sensors[2]==1 && sensors[3]==1 && sensors[4]==1)
  ) {
    searchForLine(isBlackLine);
    delay(DELAY);
    return;
  }

  // Decision Table
  if (sensors[0]==0 && sensors[1]==0 && sensors[2]==1 && sensors[3]==0 && sensors[4]==0) {
    moveForward();
  }
  else if (sensors[0]==0 && sensors[1]==1 && sensors[2]==1 && sensors[3]==0 && sensors[4]==0) {
    turnLeftUntilCenter(isBlackLine);
  }
  else if (sensors[0]==0 && sensors[1]==0 && sensors[2]==1 && sensors[3]==1 && sensors[4]==0) {
    turnRightUntilCenter(isBlackLine);
  }
  else if (sensors[0]==0 && sensors[1]==1 && sensors[2]==0 && sensors[3]==0 && sensors[4]==0) {
    turnLeftUntilCenter(isBlackLine);
  }
  else if (sensors[0]==0 && sensors[1]==0 && sensors[2]==0 && sensors[3]==1 && sensors[4]==0) {
    turnRightUntilCenter(isBlackLine);
  }
  else if (sensors[0]==1 && sensors[1]==1 && sensors[2]==0 && sensors[3]==0 && sensors[4]==0) {
    turnLeftUntilCenter(isBlackLine);
  }
  else if (sensors[0]==0 && sensors[1]==0 && sensors[2]==0 && sensors[3]==1 && sensors[4]==1) {
    turnRightUntilCenter(isBlackLine);
  }
  else if (sensors[0]==1 && sensors[1]==1 && sensors[2]==1 && sensors[3]==0 && sensors[4]==0) {
    turnLeftUntilCenter(isBlackLine);
  }
  else if (sensors[0]==0 && sensors[1]==0 && sensors[2]==1 && sensors[3]==1 && sensors[4]==1) {
    turnRightUntilCenter(isBlackLine);
  }
  else {
    moveForward();
  }

  delay(DELAY);
}