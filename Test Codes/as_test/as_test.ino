#include <Wire.h>

#define AS5600_ADDR 0x36

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL

  Serial.println("AS5600 Test Starting...");
}

// Read raw angle (0–4095)
uint16_t readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); // RAW ANGLE register
  if (Wire.endTransmission() != 0) {
    return 0xFFFF; // error
  }

  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() == 2) {
    uint16_t high = Wire.read();
    uint16_t low = Wire.read();
    return (high << 8) | low;
  }

  return 0xFFFF;
}

void loop() {
  uint16_t raw = readAS5600();

  if (raw == 0xFFFF) {
    Serial.println("AS5600 NOT DETECTED");
  } else {
    float angle = raw * 360.0 / 4096.0;

    Serial.print("Raw: ");
    Serial.print(raw);
    Serial.print(" | Angle: ");
    Serial.print(angle);
    Serial.println(" deg");
  }

  delay(100);
}