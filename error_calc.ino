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
#define S2 22
#define S3 21   // center
#define S4 19
#define S5 18   // right-most

// ===== Motor base speed =====
int baseSpeed = 120;      // max speed
int maxTurnAdjustment = 100; // max speed adjustment for turns (0-255)

// ===== Setup =====
void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  pinMode(S1, INPUT); pinMode(S2, INPUT);
  pinMode(S3, INPUT); pinMode(S4, INPUT); pinMode(S5, INPUT);

  Serial.begin(115200);
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
//int s4_count = 0;
//int s4_state = 0;

// int readIRHysteresis(int pin){
//   int val = digitalRead(pin);
//   if(val != s4_state){
//     s4_count++;
//     if(s4_count >= 3){   // 3 consecutive different readings needed
//       s4_state = val;
//       s4_count = 0;
//     }
//   } else {
//     s4_count = 0;
//   }
//   return s4_state;
// }

// ===== Loop =====
void loop() {
  // Read sensors
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);

  // Invert for white line on black
  int total = s1+s2+s3+s4+s5;
  if(total >= 3){ s1=!s1; s2=!s2; s3=!s3; s4=!s4; s5=!s5; }

  // Compute "position error" from center (-2 to +2)
  int error = (-2)*s1 + (-1)*s2 + 0*s3 + 1*s4 + 2*s5;

  // Scale error to motor speed adjustment
  int adjust = map(error, -2, 2, -maxTurnAdjustment, maxTurnAdjustment);

  int leftSpeed = baseSpeed + adjust;
  int rightSpeed = baseSpeed - adjust;

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -150, 150);
  rightSpeed = constrain(rightSpeed, -150, 150);

  // Set motors
  setMotors(leftSpeed, rightSpeed);

  // --- Print for debug ---
  Serial.print("S1:"); Serial.print(s1);
  Serial.print(" S2:"); Serial.print(s2);
  Serial.print(" S3:"); Serial.print(s3);
  Serial.print(" S4:"); Serial.print(s4);
  Serial.print(" S5:"); Serial.print(s5);
  Serial.print(" | L:"); Serial.print(leftSpeed);
  Serial.print(" R:"); Serial.println(rightSpeed);
  //   Serial.print("Error: "); Serial.print(error);
  // Serial.print(" | Adjust: "); Serial.println(adjust);

  delay(200);
}
