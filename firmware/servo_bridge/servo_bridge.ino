// Serial -> PCA9685 bridge for ESP32 / Arduino.
// Protocol (115200 baud, one line per frame):
//   S 0:1500 1:1620 2:980 ...   set pulse widths in microseconds
//   R                           relax (all outputs off)
//   P                           ping -> replies "OK"
// Failsafe: if no frame arrives for 3 s, outputs are switched off so a crashed host can't
// leave the servos straining against a pose.
//
// Wiring: PCA9685 SDA/SCL -> ESP32 GPIO21/22 (Uno: A4/A5). Servo power from a separate
// 5-6 V supply rated for 12 x MG996R stall current (budget 2.5 A each at peak; a 10 A+ BEC).
// Connect the grounds together.
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
const float SERVO_FREQ = 50.0;
const uint16_t MIN_US = 450, MAX_US = 2550;   // hard safety clamp
const unsigned long FAILSAFE_MS = 3000;
unsigned long lastFrame = 0;
bool relaxed = true;
char line[256];
size_t len = 0;

void setUs(uint8_t ch, uint16_t us) {
  if (ch > 15) return;
  us = constrain(us, MIN_US, MAX_US);
  pwm.writeMicroseconds(ch, us);
}

void relaxAll() {
  for (uint8_t ch = 0; ch < 16; ch++) pwm.setPWM(ch, 0, 4096);  // full off
  relaxed = true;
}

void handle(char *s) {
  if (s[0] == 'P') { Serial.println("OK"); return; }
  if (s[0] == 'R') { relaxAll(); return; }
  if (s[0] != 'S') return;
  char *tok = strtok(s + 1, " ");
  while (tok) {
    char *colon = strchr(tok, ':');
    if (colon) {
      *colon = 0;
      setUs((uint8_t)atoi(tok), (uint16_t)atoi(colon + 1));
    }
    tok = strtok(NULL, " ");
  }
  lastFrame = millis();
  relaxed = false;
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  relaxAll();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) { line[len] = 0; handle(line); len = 0; }
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
  if (!relaxed && millis() - lastFrame > FAILSAFE_MS) relaxAll();
}
