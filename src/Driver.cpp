#include "Driver.h"
#include <Arduino.h>
#include <SPI.h>

Driver::Driver(int pwm, int dir) : pwm(pwm), dir(dir) {}

void Driver::begin() {  
  // Set pin mode
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
}


float Driver::move(int speed, int direction) {
  // Reads encoder angle and outputs in degrees
  digitalWrite(dir, direction);
  analogWrite(pwm, speed);
  return speed, direction;
}