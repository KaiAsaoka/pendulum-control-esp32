#include "Encoder.h"
#include <Arduino.h>
#include <SPI.h>

// Define the static member variable
int Encoder::firstReading = true;  // Initialize to 0


int CLOCK_SPEED = 10000000; // 10MHz


Encoder::Encoder(int miso, int clk, int cs, int mosi)
    : miso(miso), clk(clk), cs(cs), mosi(mosi), prevAngle(0), rotationCount(0), zeroAngle(-1)
{}

void Encoder::begin() {  
  // Configure encoder
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);  // Deselect encoder by default

  if (firstReading) {
    pinMode(mosi, OUTPUT);  // Explicitly set MOSI as output
    digitalWrite(mosi, HIGH);  // Set MOSI high initially

    SPI.begin(clk, miso, mosi);
    SPI.beginTransaction(SPISettings(CLOCK_SPEED, MSBFIRST, SPI_MODE1));
    firstReading = false;
  }

  zeroAngle = readAngle();
  Serial.println("AS5147 SPI Encoder Initialized");

}

int Encoder::readAngle() {
  // Reads encoder angle and converts to degrees (0-360)
  uint16_t response;

  // Read angle register
  digitalWrite(cs, LOW);
  delayMicroseconds(1);  // Small delay before reading
  response = SPI.transfer16(0x3FFF);  // Changed to single read with proper command
  digitalWrite(cs, HIGH);
  delayMicroseconds(1);  // Small delay between reads

  int currentAngle = int(response & 0x3FFF);
  
  // Detect rollover
  if (prevAngle > 0x3FFF * 0.75 && currentAngle < 0x3FFF * 0.25 && zeroAngle != -1) {
    // Rolled over from high to low
    rotationCount++;
  } else if (prevAngle < 0x3FFF * 0.25 && currentAngle > 0x3FFF * 0.75 && zeroAngle != -1) {
    // Rolled over from low to high
    rotationCount--;
  }
  
  prevAngle = currentAngle;
  return currentAngle;
}

long Encoder::getTotalAngle() {
  // Total angle is the number of rotations times 360 plus the current angle
  int currentAngle = readAngle();
  long totalAngle = (long)rotationCount * 16384L + (long)currentAngle - (long)zeroAngle;
  return totalAngle;
}

float Encoder::getTotalAngleFloat() {
  float totalAngleFloat = float(getTotalAngle()) * 360.0 / 16384.0;
  return totalAngleFloat;
}

void Encoder::zero() {
  zeroAngle = getTotalAngle();
  
  Serial.print("zero'd to: ");
  Serial.println(zeroAngle);
}
