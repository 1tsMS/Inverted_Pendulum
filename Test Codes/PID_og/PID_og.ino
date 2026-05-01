/* * ======================================================================================
 * PROJECT: FINAL ROBUST MICRO-PENDULUM (BTS7960) - I2C AS5600 VERSION
 * CONFIG: Pole (600PPR Optical) | Cart (AS5600 I2C Mode)
 * PINS: SDA(21), SCL(22) for AS5600 | R_PWM(25), L_PWM(26), R_EN(27), L_EN(14)
 * ======================================================================================
 */

#include <Arduino.h>
#include <Wire.h>

// --- BTS7960 PINS ---
#define PIN_R_PWM 25 
#define PIN_L_PWM 26
#define PIN_EN_R  27 
#define PIN_EN_L  14 

// --- POLE ENCODER PINS ---
#define POLE_ENC_A 33  
#define POLE_ENC_B 32  

// --- AS5600 I2C CONFIG ---
#define AS5600_ADDR 0x36
#define RAW_ANGLE_REG 0x0C

// --- PHYSICAL CONSTANTS ---
const float POLE_PPR = 600.0;           
const int   PULLEY_TEETH = 16;          
const float BELT_PITCH = 0.002;         
const float METERS_PER_REV = PULLEY_TEETH * BELT_PITCH;
const float TRACK_LIMIT = 0.06; 

// --- PID VARIABLES ---
float Kp_angle = 68.0, Ki_angle = 0.25, Kd_angle = 2.8;
float Kp_pos   = 0.10, Ki_pos   = 0.0,  Kd_pos   = 0.04;

// --- STATE VARIABLES ---
volatile long poleTicks = 0;
float currentPos = 0, lastPos = 0;
float targetAngle = 0, smoothPWM = 0;
float angleErrorI = 0, posErrorI = 0;
float lastAngleError = 0, lastPosError = 0;

// AS5600 Multi-turn tracking
int lastRawAngle = -1;
long fullRotations = 0;

unsigned long lastTime = 0, lastDebugTime = 0;
bool systemEnabled = false; 
bool balanceActive = false; 

// --- POLE INTERRUPT ---
void IRAM_ATTR readPole() { if (digitalRead(POLE_ENC_A) == digitalRead(POLE_ENC_B)) poleTicks++; else poleTicks--; }

// --- AS5600 I2C READER ---
uint16_t readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_REG);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() >= 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    return (highByte << 8) | lowByte;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL pins for ESP32
  
  pinMode(POLE_ENC_A, INPUT_PULLUP); pinMode(POLE_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(POLE_ENC_A), readPole, RISING);
  
  pinMode(PIN_R_PWM, OUTPUT); pinMode(PIN_L_PWM, OUTPUT); 
  pinMode(PIN_EN_R, OUTPUT);  pinMode(PIN_EN_L, OUTPUT);
  digitalWrite(PIN_EN_R, LOW); digitalWrite(PIN_EN_L, LOW);

  // Initialize AS5600 baseline
  lastRawAngle = readAS5600();

  Serial.println("\n--- I2C MICRO-PENDULUM READY ---");
  Serial.println("S = Start | X = Stop | P/I/D/p/d = Tune");
}

void loop() {
  // --- SERIAL COMMAND PARSER ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'S' || cmd == 's') { systemEnabled = true; Serial.println(">>> ARMED"); } 
    else if (cmd == 'X' || cmd == 'x') { systemEnabled = false; stopMotor(); Serial.println("!!! KILLED"); } 
    else {
      float val = Serial.parseFloat();
      switch (cmd) {
        case 'P': Kp_angle = val; break;
        case 'I': Ki_angle = val; break;
        case 'D': Kd_angle = val; break;
        case 'p': Kp_pos = val; break;
        case 'd': Kd_pos = val; break;
      }
    }
  }

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  if (dt < 0.01) return; // 100Hz Loop

  // 1. UNIT CONVERSION & AS5600 UNWRAPPING
  float currentAngle = -(poleTicks * 360.0 / POLE_PPR); 
  
  int rawAngle = readAS5600();
  // Detect full rotation jump (4096 to 0 or 0 to 4096)
  if (lastRawAngle != -1) {
    int diff = rawAngle - lastRawAngle;
    if (diff < -2048) fullRotations++;      // Clockwise wrap
    else if (diff > 2048) fullRotations--;   // Counter-clockwise wrap
  }
  lastRawAngle = rawAngle;
  
  long totalTicks = (fullRotations * 4096) + rawAngle;
  currentPos = (totalTicks / 4096.0) * METERS_PER_REV;

  // 2. OUTER LOOP (POSITION)
  float posError = 0.0 - currentPos; 
  posErrorI = constrain(posErrorI + (posError * dt), -0.5, 0.5);
  float posDeriv = (posError - lastPosError) / dt;
  targetAngle = (posError * Kp_pos) + (posErrorI * Ki_pos) + (posDeriv * Kd_pos);
  targetAngle = constrain(targetAngle, -10, 10); 

  // 3. INNER LOOP (BALANCE)
  float angleError = targetAngle - currentAngle;
  if (abs(angleError) < 0.05) angleError = 0; 
  angleErrorI = constrain(angleErrorI + (angleError * dt), -35, 35);
  float angleDeriv = (angleError - lastAngleError) / dt;
  float motorOutput = (angleError * Kp_angle) + (angleErrorI * Ki_angle) + (angleDeriv * Kd_angle);

  // 4. SAFETY & CONTROL
  if (systemEnabled) {
    if (!balanceActive && abs(currentAngle) < 2.0) {
      balanceActive = true;
      digitalWrite(PIN_EN_R, HIGH); digitalWrite(PIN_EN_L, HIGH);
    }
    if (balanceActive && abs(currentAngle) < 32.0 && abs(currentPos) < TRACK_LIMIT) {
      driveBTS7960(constrain(motorOutput, -255, 255));
    } else if (balanceActive) {
      stopMotor(); 
    }
  }

  // 5. DEBUGGING (10Hz)
  if (currentTime - lastDebugTime > 100) {
    if (systemEnabled) {
      Serial.print("A:"); Serial.print(currentAngle, 1);
      Serial.print(" P_mm:"); Serial.print(currentPos * 1000.0, 1);
      Serial.print(" T:"); Serial.print(targetAngle, 1);
      Serial.print(" O:"); Serial.println(smoothPWM, 0);
    }
    lastDebugTime = currentTime;
  }

  lastAngleError = angleError; lastPosError = posError; lastTime = currentTime;
}

void driveBTS7960(int speed) {
  if (abs(speed) > 1) speed += (speed > 0) ? 40 : -40; 
  smoothPWM = (0.45 * speed) + (0.55 * smoothPWM);    
  int finalPWM = constrain((int)smoothPWM, -255, 255);
  if (finalPWM > 0) { analogWrite(PIN_R_PWM, finalPWM); analogWrite(PIN_L_PWM, 0); }
  else { analogWrite(PIN_R_PWM, 0); analogWrite(PIN_L_PWM, abs(finalPWM)); }
}

void stopMotor() {
  digitalWrite(PIN_EN_R, LOW); digitalWrite(PIN_EN_L, LOW);
  analogWrite(PIN_R_PWM, 0); analogWrite(PIN_L_PWM, 0);
  balanceActive = false; smoothPWM = 0; angleErrorI = 0; posErrorI = 0;
}