// Pin definitions
const int sensorPins[5] = { 4, 5, 18, 19, 21 }; // 5 TCRT5000 digital outputs connected to these GPIOs
const int motorA_PWM = 14;  // TB6612 PWM pin motor A
const int motorA_IN1 = 27;  // TB6612 IN1
const int motorA_IN2 = 26;  // TB6612 IN2
const int motorB_PWM = 25;  // TB6612 PWM pin motor B
const int motorB_IN3 = 33;  // TB6612 IN3
const int motorB_IN4 = 32;  // TB6612 IN4

// PID tuning parameters (tune these experimentally)
float Kp = 0.12;
float Ki = 0.0;
float Kd = 0.02;

// Motor base speed PWM (0-255)
const int baseSpeed = 160;

// Variables for PID
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// For line color detection (0=black line on white, 1=white line on black)
bool invertLogic = false;

// Read sensors and return position error
// Sensors order: Left to Right - weights are -2, -1, 0, 1, 2
// Sensor outputs HIGH=white, LOW=black (digital)
int readLinePosition() {
  int sensorValues[5];
  for (int i = 0; i < 5; i++) {
    int val = digitalRead(sensorPins[i]);
    // Invert logic dynamically based on line type
    sensorValues[i] = invertLogic ? !val : val;
  }
  // Calculate weighted average error
  int weightedSum = 0;
  int sumActive = 0;
  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] == 0) {  // sensor on line if LOW (black)
      weightedSum += (i - 2) * 1000;
      sumActive += 1000;
    }
  }
  if (sumActive == 0) {
    // Line lost, return large error based on last known error sign
    return lastError > 0 ? 3000 : -3000;
  }
  return weightedSum / sumActive;
}

// Set motor speeds with PWM and direction
// speed from -255 to 255, positive forward, negative backward
void setMotorSpeed(int motorPWM, int IN1, int IN2, int speed) {
  if (speed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(motorPWM, speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(motorPWM, -speed);
  }
}

// Setup function
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 5; i++)
    pinMode(sensorPins[i], INPUT);

  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorB_IN3, OUTPUT);
  pinMode(motorB_IN4, OUTPUT);

  ledcSetup(0, 20000, 8); // channel 0 PWM 20kHz 8-bit for motor A PWM pin
  ledcAttachPin(motorA_PWM, 0);

  ledcSetup(1, 20000, 8); // channel 1 PWM 20kHz 8-bit for motor B PWM pin
  ledcAttachPin(motorB_PWM, 1);

  lastTime = millis();
}

// Main loop
void loop() {
  int position = readLinePosition();

  // PID control calculation
  unsigned long now = millis();
  float timeChange = (float)(now - lastTime);
  if(timeChange == 0) timeChange = 1;

  float error = (float)position;
  integral += error * timeChange;
  float derivative = (error - lastError) / timeChange;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // Motor speeds adjusted for PID output
  int leftSpeed = baseSpeed + output;
  int rightSpeed = baseSpeed - output;

  // Constrain PWM signals
  leftSpeed = constrain(leftSpeed, -200, 200);
  rightSpeed = constrain(rightSpeed, -200, 200);

  // Set motors
  setMotorSpeed(0, motorA_IN1, motorA_IN2, leftSpeed);
  setMotorSpeed(1, motorB_IN3, motorB_IN4, rightSpeed);

  lastError = error;
  lastTime = now;

  // Detect track features (simple example using all sensors detection)
  int sensorSum = 0;
  for (int i = 0; i < 5; i++) {
    sensorSum += digitalRead(sensorPins[i]);
  }

  // Example: all sensors on line -> possible circle or cross detected
  if (invertLogic ? (sensorSum == 0) : (sensorSum == 5)) {
    // Example action: slow down or turn accordingly to avoid
    setMotorSpeed(0, motorA_IN1, motorA_IN2, 100);
    setMotorSpeed(1, motorB_IN3, motorB_IN4, 100);
    delay(100);
  }

  // Switch line color mode if needed (implement your own logic or external trigger)
  // invertLogic = conditionToSwitchLineColor;

  delay(10);
}
