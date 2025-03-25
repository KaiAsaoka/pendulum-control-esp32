#include "Encoder.h"
#include <Arduino.h>
#include <SPI.h>

// Define the static member variable
int Encoder::firstReading = true;  // Initialize to 0

int CLOCK_SPEED = 10000000; // 10MHz

Encoder::Encoder(int miso, int clk, int cs, int mosi)
    : miso(miso), clk(clk), cs(cs), mosi(mosi), prevAngle(0), rotationCount(0)
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

  return int(response & 0x3FFF);
}

float Encoder::getTotalAngle() {
  // Total angle is the number of rotations times 360 plus the current angle
  return rotationCount * 360.0 + prevAngle;
}