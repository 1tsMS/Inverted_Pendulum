#include <Wire.h>

// ===== AS5600 =====
#define AS5600_ADDR 0x36

// ===== Encoder (600 PPR) =====
#define ENC_A 32
#define ENC_B 33

volatile long encoderCount = 0;

// ===== BTS7960 =====
#define RPWM 25
#define LPWM 26
#define REN  27
#define LEN  14

// ===== Encoder ISR =====
void IRAM_ATTR handleEncoderA() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B))
    encoderCount++;
  else
    encoderCount--;
}

// ===== Read AS5600 Angle =====
uint16_t readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); // RAW ANGLE register
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() == 2) {
    uint16_t high = Wire.read();
    uint16_t low = Wire.read();
    return (high << 8) | low;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);

  // I2C
  Wire.begin(21, 22);

  // Encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), handleEncoderA, CHANGE);

  // Motor Driver
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  Serial.println("System Test Start...");
}

void loop() {

  // ===== 1. Read AS5600 =====
  uint16_t angle = readAS5600();

  // Convert to degrees
  float degrees = angle * 360.0 / 4096.0;

  // ===== 2. Read Encoder =====
  long count = encoderCount;

  // ===== 3. Print Values =====
  Serial.print("AS5600 Angle: ");
  Serial.print(degrees);
  Serial.print(" deg | Encoder Count: ");
  Serial.println(count);

  // ===== 4. Motor Test (slow forward) =====
  analogWrite(RPWM, 80);  // forward
  analogWrite(LPWM, 0);

  delay(200);
}