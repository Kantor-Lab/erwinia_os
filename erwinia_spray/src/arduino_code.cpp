#include <Servo.h>

Servo s;

const int SERVO_PIN = 9;

// Tune these for your mechanism
const int OFF_ANGLE = 0;
const int ON_ANGLE  = 90;

void setSpray(bool on) {
  s.write(on ? ON_ANGLE : OFF_ANGLE);
}

void setup() {
  Serial.begin(115200);
  s.attach(SERVO_PIN);
  setSpray(false);
  delay(300);
  Serial.println("OK");  // boot ack
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "S") {
    setSpray(true);
    Serial.println("OK");
  }
  else if (cmd == "R") {
    setSpray(false);
    Serial.println("OK");
  }
  else if (cmd.startsWith("P")) {
    long ms = cmd.substring(1).toInt();
    if (ms <= 0 || ms > 30000) {
      Serial.println("ERR");
      return;
    }
    setSpray(true);
    delay(ms);
    setSpray(false);
    Serial.println("OK");
  }
  else {
    Serial.println("ERR");
  }
}