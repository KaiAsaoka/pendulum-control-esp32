#include "Encoder.h"
#include <Arduino.h>
#include <SPI.h>

// Define the static member variable
int Encoder::firstReading = true;  // Initialize to 0


int CLOCK_SPEED = 1000000; // 1 MHz; Maximum per AS5147 datasheet is 10 MHz (100ns) but this was not shown to work

// RC : removes the trailing bits from mask E.g., numBitIgnore = 2 -> mask = 0011111111111100 ignores the last 2 bits
Encoder::Encoder(int miso, int clk, int cs, int mosi, int numBitIgnore)
    : miso(miso), clk(clk), cs(cs), mosi(mosi), mask(0x3FFF - (pow(2, numBitIgnore)-1)), prevAngle(0), rotationCount(0), zeroAngle(-1)
{}

Encoder::Encoder(int miso, int clk, int cs, int mosi)
    : Encoder(miso, clk, cs, mosi, 0) // Default to using all 14 bits for AS5147
{}

void Encoder::begin() {  
  // Configure encoder
  Serial.println(mask, HEX);
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

  int currentAngle = int(response & mask); // E.g., mask = 0011111111111100 -> Floor last 4 bits
  
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
  int currentAngle = readAngle();
  long totalAngleNet = (long)rotationCount * 16384L + (long)currentAngle;
  zeroAngle = totalAngleNet;
  
  Serial.print("zero'd to: ");
  Serial.println(totalAngleNet);
}
