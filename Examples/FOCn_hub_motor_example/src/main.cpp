#include <Arduino.h>
#include "SimpleFOC.h"

#define USRLED 46


void setup() {
  // put your setup code here, to run once:
  pinMode(USRLED, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(USRLED, HIGH);
  delay(500);
  digitalWrite(USRLED, LOW);
  delay(500);
}

