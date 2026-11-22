#include <Arduino.h>

constexpr int BAUD_RATE = 115200;

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("Hello World");
}

void loop() {
  // put your main code here, to run repeatedly:
}