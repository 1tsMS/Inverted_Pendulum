#include <Arduino.h>
#include <Wire.h>

// --- HARDWARE ---
#define PIN_R_PWM 25
#define PIN_L_PWM 26
#define PIN_EN_R  27
#define PIN_EN_L  14
#define POLE_ENC_A 33
#define POLE_ENC_B 32
#define AS5600_ADDR 0x36
#define RAW_ANGLE_REG 0x0C

// --- CONSTANTS ---
const int PWM_FREQ = 18000;
const int PWM_RES  = 12;
const int PWM_CHANNEL_R = 0;
const int PWM_CHANNEL_L = 1;
const float POLE_PPR = 600.0; // x4 quadrature decoding -> 2400 effective ticks/rev
const float METERS_PER_REV = 0.032;
const float TRACK_LIMIT = 0.10;

// --- PID (TUNED FOR RESPONSE + STABILITY) ---
float Kp_angle = 350.0;
float Ki_angle = 0.0;
float Kd_angle = 120.0;

float Kp_pos = 0.0;
float Ki_pos = 0.0;
float Kd_pos = 0.0;

// --- STATE ---
volatile long poleTicks = 0;

float angleOffset = 0;
int offsetRawAngle = 0;

float currentPos = 0, targetAngle = 0;
float angleErrorI = 0, posErrorI = 0;
float lastAngleError = 0, lastPosError = 0;
float filteredD = 0, smoothPWM = 0;

int lastRawAngle = -1;
long fullRotations = 0;

unsigned long lastTime = 0;
unsigned long lastDebugTime = 0;

bool systemEnabled = false;
bool balanceActive = false;

// --- ENCODER ---
void IRAM_ATTR readPoleA() {
  if (digitalRead(POLE_ENC_A) == digitalRead(POLE_ENC_B)) poleTicks++;
  else poleTicks--;
}

void IRAM_ATTR readPoleB() {
  if (digitalRead(POLE_ENC_A) == digitalRead(POLE_ENC_B)) poleTicks--;
  else poleTicks++;
}

// --- AS5600 ---
uint16_t readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_REG);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() >= 2) return (Wire.read() << 8) | Wire.read();
  return 0;
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_R_PWM, PWM_CHANNEL_R);
  ledcAttachPin(PIN_L_PWM, PWM_CHANNEL_L);

  pinMode(POLE_ENC_A, INPUT_PULLUP);
  pinMode(POLE_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(POLE_ENC_A), readPoleA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(POLE_ENC_B), readPoleB, CHANGE);

  pinMode(PIN_EN_R, OUTPUT);
  pinMode(PIN_EN_L, OUTPUT);
  digitalWrite(PIN_EN_R, LOW);
  digitalWrite(PIN_EN_L, LOW);

  lastRawAngle = readAS5600();

  Serial.println("READY: Press S to zero & start");
}

// --- LOOP ---
void loop() {

  // --- COMMANDS ---
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'S' || cmd == 's') {

      angleErrorI = 0;
      posErrorI = 0;

      long tickSum = 0;
      for (int i = 0; i < 50; i++) {
        tickSum += poleTicks;
        delay(2);
      }
      angleOffset = -(tickSum / 50.0) * 360.0 / (POLE_PPR * 4.0);

      int raw = readAS5600();
      offsetRawAngle = raw;
      fullRotations = 0;
      lastRawAngle = raw;

      smoothPWM = 0;
      filteredD = 0;
      lastAngleError = 0;
      lastPosError = 0;

      systemEnabled = true;
      balanceActive = false;

      Serial.println("ZERO SET: ANGLE + POSITION");
    }

    if (cmd == 'X' || cmd == 'x') {
      systemEnabled = false;
      stopMotor();
      Serial.println("STOP");
    }
  }

  // --- TIMING ---
  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6;
  if (dt < 0.004) return;

  // --- ANGLE ---
  float rawAngle = -(poleTicks * 360.0 / (POLE_PPR * 4.0));
  float currentAngle = rawAngle - angleOffset;
  // --- POSITION ---
  int raw = readAS5600();
  if (lastRawAngle != -1) {
    int diff = raw - lastRawAngle;
    if (diff < -2048) fullRotations++;
    else if (diff > 2048) fullRotations--;
  }
  lastRawAngle = raw;

  float absolutePos = ((fullRotations * 4096.0 + raw) / 4096.0) * METERS_PER_REV;
  float zeroPos = (offsetRawAngle / 4096.0) * METERS_PER_REV;
  currentPos = -(absolutePos - zeroPos);
  // --- OUTER PID ---
  float posError = -currentPos;
  //if (abs(posError) < 0.02) posError *= 0.3;
  posErrorI = constrain(posErrorI + posError * dt, -0.2, 0.2);
  float posD = (posError - lastPosError) / dt;
  targetAngle = Kp_pos * posError + Ki_pos * posErrorI + Kd_pos * posD;
  targetAngle = constrain(targetAngle, -6.0, 6.0);

  // --- INNER PID ---
  float angleError = targetAngle - currentAngle;
  if (abs(angleError) < 1.0) angleError *= 0.5;
  angleErrorI = constrain(angleErrorI + angleError * dt, -10, 10);

  float rawD = (angleError - lastAngleError) / dt;
  filteredD = 0.45 * rawD + 0.55 * filteredD;

  float output = Kp_angle * angleError + Ki_angle * angleErrorI + Kd_angle * filteredD;

  // --- CONTROL ---
  if (systemEnabled) {

    if (!balanceActive && abs(currentAngle) < 10.0) {
      balanceActive = true;
      digitalWrite(PIN_EN_R, HIGH);
      digitalWrite(PIN_EN_L, HIGH);
    }

    if (balanceActive && abs(currentAngle) < 45 && abs(currentPos) < TRACK_LIMIT) {
      driveMotor(constrain(output, -4095, 4095));
    } else {
      stopMotor();
    }
  }

  // --- DEBUG OUTPUT ---
  if (millis() - lastDebugTime > 50) {

    
    Serial.print("Angle:"); Serial.print(currentAngle, 2); Serial.print("\t");
    Serial.print("Target:"); Serial.print(targetAngle, 2); Serial.print("\t");
    Serial.print("Pos_mm:"); Serial.print(currentPos * 1000.0, 1); Serial.print("\t");
    Serial.print("PWM:"); Serial.print(smoothPWM, 0); Serial.print("\t");
    Serial.print("Output:"); Serial.print(output, 2);
    Serial.println();

    lastDebugTime = millis();
  }

  lastAngleError = angleError;
  lastPosError = posError;
  lastTime = now;
}

// --- MOTOR ---
void driveMotor(int speed) {

  if (speed > 0 && speed < 40) speed = 40;
  else if (speed < 0 && speed > -40) speed = -40;
  else if (speed == 0) speed = 0;

  smoothPWM = 0.85 * speed + 0.15 * smoothPWM;
  static float lastPWM = 0;
  float maxStep = 400;  // tune this (200–500 range)

  float delta = smoothPWM - lastPWM;
  delta = constrain(delta, -maxStep, maxStep);

  smoothPWM = lastPWM + delta;
  lastPWM = smoothPWM;
  int pwm = constrain((int)smoothPWM, -4095, 4095);

  if (pwm > 0) {
    ledcWrite(PWM_CHANNEL_R, pwm);
    ledcWrite(PWM_CHANNEL_L, 0);
  } else if (pwm < 0) {
    ledcWrite(PWM_CHANNEL_R, 0);
    ledcWrite(PWM_CHANNEL_L, -pwm);
  } else {
    ledcWrite(PWM_CHANNEL_R, 0);
    ledcWrite(PWM_CHANNEL_L, 0);
  }
}

// --- STOP ---
void stopMotor() {
  digitalWrite(PIN_EN_R, LOW);
  digitalWrite(PIN_EN_L, LOW);
  ledcWrite(PWM_CHANNEL_R, 0);
  ledcWrite(PWM_CHANNEL_L, 0);

  smoothPWM = 0;
  angleErrorI = 0;
  posErrorI = 0;
  balanceActive = false;
}