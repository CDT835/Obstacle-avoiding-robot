#include <Arduino.h>
#include <Servo.h>

/* ===== HC-SR04 ===== */
const unsigned int TRIG_PIN = A4;
const unsigned int ECHO_PIN = A5;
const unsigned int BAUD_RATE = 9600;

/* ===== Motor pins L298N ===== */
const int ENA = 6;    // PWM Left
const int ENB = 5;    // PWM Right
const int IN1 = 7;    // Left DIR1
const int IN2 = 4;    // Left DIR2
const int IN3 = 8;    // Right DIR1
const int IN4 = 9;    // Right DIR2

int speed_left  = 115;
int speed_right = 115;

/* ===== Servo ===== */
const int SERVO_PIN = 13;
Servo myservo;

const int CENTER_ANGLE = 105;
const int LEFT_ANGLE   = 175;
const int RIGHT_ANGLE  = 15;

/* ===== Obstacle threshold ===== */
const int OBSTACLE_THRESHOLD = 20;

/* ===== Turning flag ===== */
bool isTurning = false;

/* ===== Encoder + hình học ===== */
const uint8_t PIN_ENC_L = 2;
const uint8_t PIN_ENC_R = 3;

const uint16_t PULSES_PER_REV = 20;

const float WHEEL_RADIUS_M = 0.0325f;
const float WHEEL_BASE_M   = 0.1150f;
const float CIRC_M = 2.0f * 3.1415926f * WHEEL_RADIUS_M;

volatile long encL_total = 0;
volatile long encR_total = 0;

/* ===== ISR Encoder ===== */
void encL_isr() {
  encL_total++;
}

void encR_isr() {
  encR_total++;
}

/* ===== Motor L298N helper ===== */
void motorWriteLR(int pwmL, int pwmR) {
  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);

  // Left motor
  if (pwmL > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwmL);
  } else if (pwmL < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwmL);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  // Right motor
  if (pwmR > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwmR);
  } else if (pwmR < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -pwmR);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

/* ===== Setup ===== */
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(BAUD_RATE);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stop();

  myservo.attach(SERVO_PIN);
  myservo.write(CENTER_ANGLE);
  delay(500);

  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), encL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), encR_isr, RISING);
}

/* ===== Loop ===== */
void loop() {
  if (isTurning) {
    delay(100);
    return;
  }

  int frontDistance = getDistance();

  if (frontDistance == 0) {
    // Không đọc được sensor
  } else {
    if (frontDistance < OBSTACLE_THRESHOLD) {
      stop();

      int leftDistance = scanDirection(LEFT_ANGLE);
      int rightDistance = scanDirection(RIGHT_ANGLE);

      myservo.write(CENTER_ANGLE);
      delay(500);

      if (leftDistance < OBSTACLE_THRESHOLD && rightDistance > OBSTACLE_THRESHOLD) {
        turn_right_90_Degrees();
      } 
      else if (rightDistance < OBSTACLE_THRESHOLD && leftDistance > OBSTACLE_THRESHOLD) {
        turn_left_90_Degrees();
      } 
      else if (rightDistance >= OBSTACLE_THRESHOLD && leftDistance >= OBSTACLE_THRESHOLD) {
        turn_right_90_Degrees();
      } 
      else {
        turn_left_90_Degrees();
      }
    } 
    else {
      forward();
    }
  }

  delay(200);
}

/* ===== Siêu âm + quét servo ===== */
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);

  if (duration == 0) return 0;

  int distance = duration / 29 / 2;
  return distance;
}

int scanDirection(int angle) {
  myservo.write(angle);
  delay(700);
  return getDistance();
}

/* ===== Motor functions dùng L298N ===== */
void stop() {
  motorWriteLR(0, 0);
}

void forward() {
  motorWriteLR(speed_left, speed_right);
}

void backward() {
  motorWriteLR(-speed_left, -speed_right);
}

void turn_left() {
  motorWriteLR(-speed_left, speed_right);
}

void turn_right() {
  motorWriteLR(speed_left, -speed_right);
}

/* ===== Quay 90° bằng encoder ===== */
long pulsesForSpinDeg(float deg) {
  float theta = fabs(deg) * PI / 180.0f;
  float s = theta * (WHEEL_BASE_M / 2.0f);
  float revs = s / CIRC_M;
  return (long)(revs * PULSES_PER_REV + 0.5f);
}

void turn_right_90_Degrees() {
  isTurning = true;

  long target = 3.0 * pulsesForSpinDeg(90.0f);

  long L0, R0;
  noInterrupts();
  L0 = encL_total;
  R0 = encR_total;
  interrupts();

  while (true) {
    turn_right();

    long Ld, Rd;
    noInterrupts();
    Ld = abs(encL_total - L0);
    Rd = abs(encR_total - R0);
    interrupts();

    if (Ld >= target && Rd >= target) break;

    delay(5);
  }

  stop();
  delay(200);
  isTurning = false;
}

void turn_left_90_Degrees() {
  isTurning = true;

  long target = 3.0 * pulsesForSpinDeg(90.0f);

  long L0, R0;
  noInterrupts();
  L0 = encL_total;
  R0 = encR_total;
  interrupts();

  while (true) {
    turn_left();

    long Ld, Rd;
    noInterrupts();
    Ld = abs(encL_total - L0);
    Rd = abs(encR_total - R0);
    interrupts();

    if (Ld >= target && Rd >= target) break;

    delay(5);
  }

  stop();
  delay(200);
  isTurning = false;
}