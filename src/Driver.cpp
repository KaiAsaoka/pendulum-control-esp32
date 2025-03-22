#include "Driver.h"
#include <Arduino.h>
#include <SPI.h>

Driver::Driver(int pwm, int dir) : pwm(pwm), dir(dir) {
  Serial.print("Driver constructor: PWM=");
  Serial.print(pwm);
  Serial.print(" DIR=");
  Serial.println(dir);
}

void Driver::begin() {  
  // Set pin mode
  Serial.print("Setting PWM pin ");
  Serial.println(pwm);
  Serial.flush();
  
  if(pwm < 0 || pwm > 39) {
    Serial.println("Invalid PWM pin number!");
    return;
  }
  pinMode(pwm, OUTPUT);
  Serial.println("PWM pin set");
  Serial.flush();
  
  Serial.print("Setting DIR pin ");
  Serial.println(dir);
  Serial.flush();
  
  if(dir < 0 || dir > 39) {
    Serial.println("Invalid DIR pin number!");
    return;
  }
  pinMode(dir, OUTPUT);
  Serial.println("DIR pin set");
  Serial.flush();
  
  Serial.println("Driver initialized");
  Serial.flush();
}

void Driver::move(int speed, bool direction) {
  // Reads encoder angle and outputs in degrees
  digitalWrite(dir, direction);
  analogWrite(pwm, speed);
}