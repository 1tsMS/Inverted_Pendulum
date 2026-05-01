#define RPWM 25
#define LPWM 26
#define REN 27
#define LEN 14

#define PWM_FREQ 20000
#define PWM_RES 8

void setup() {
  Serial.begin(115200);

  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcAttachPin(RPWM, 0);

  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(LPWM, 1);

  Serial.begin(115200);
}

void loop() {
  // Forward
  ledcWrite(0, 255);
  ledcWrite(1, 0);
  delay(40);
  Serial.println("Forward");

  // Stop
  // ledcWrite(0, 0);
  // ledcWrite(1, 0);
  // delay(10);
  // Serial.println("Stop");
  // Reverse
  ledcWrite(0, 0);
  ledcWrite(1, 255);
  delay(50);
  Serial.println("Reverse");

  // Stop
//   ledcWrite(0, 0);
//   ledcWrite(1, 0);
//   delay(10);
//   Serial.println("Stop");
 }