#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>
#include <chrono>
#include "Driver.h"
#include "Move.h"

// Define encoder SPI pins
// #define ENC_MISO 12    // Encoder data output (MISO)
// #define ENC_CLK  14    // SPI clock (SCK)
// #define ENC_CS   15    // Chip Select (active LOW)
// #define ENC_MOSI 13    // MOSI pin for encoder communication

#define PWM1 35
#define DIR1 34
#define PWM2 7
#define DIR2 5
#define SPEED 10

// Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS, ENC_MOSI);
Driver DVR1(PWM1, DIR1);
Driver DVR2(PWM2, DIR2);
Move move(DVR1, DVR2);

unsigned long startTime;

void printBinary16(uint16_t n);
unsigned long getTime(unsigned long startTime);

void setup() {
  // Serial.begin(115200);
  // ENC1.begin(); // Initializes SPI for ENC
  DVR1.begin();
  DVR2.begin();
  // startTime = millis();
}

void loop() {
  // Read the 14-bit angle
  move.moveX(SPEED, 1);
  delay(200);
  move.stop();
  delay(200);

  move.moveY(SPEED, 1);
  delay(200);
  move.stop();
  delay(200);
  
  move.moveX(SPEED, 0);
  delay(200);
  move.stop();
  delay(200);
  
  move.moveY(SPEED, 0);
  delay(200);
  move.stop();
  delay(200);

  // int angle;
  // angle = ENC1.readAngle();
  
  // Serial.print("Angle 1: ");
  // Serial.println(angle);
  }

void printBinary16(uint16_t n) {
  for (int i = 15; i >= 0; i--) {
    Serial.print((n >> i) & 1);
  }
  Serial.println();
}

unsigned long getTime(unsigned long startTime) {
  unsigned long currentTime = millis();
  unsigned long duration = currentTime - startTime;
  return duration;
}
