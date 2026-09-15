#include <Arduino.h>

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  inputString.reserve(200);
  pinMode(11, OUTPUT);     // W101 IN ← D11 (PWM)
  analogWrite(11, 0);      // 0% 시작
}

static void setDutyPercent(int pin, float dutyPercent) {
  if (dutyPercent < 0) dutyPercent = 0;
  if (dutyPercent > 100) dutyPercent = 100;
  int pwmValue = (int)round(dutyPercent * 255.0 / 100.0);
  analogWrite(pin, pwmValue);
}

void loop() {
  if (stringComplete) {
    inputString.trim();
    if (inputString.startsWith("P")) {
      String cmd = inputString.substring(1);
      int comma = cmd.indexOf(',');
      if (comma > 0) {
        int pin = cmd.substring(0, comma).toInt();   // 예: 11
        float duty = cmd.substring(comma + 1).toFloat(); // 예: 75.5
        setDutyPercent(pin, duty);
        // 필요시 확인
        // Serial.println("OK");
      }
    }
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    inputString += c;
    if (c == '\n') stringComplete = true;
  }
}
