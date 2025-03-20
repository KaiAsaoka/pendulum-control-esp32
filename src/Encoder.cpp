#include "Encoder.h"
#include <Arduino.h>
#include <SPI.h>

int CLOCK_SPEED = 10000000; // 10MHz

Encoder::Encoder(int miso, int clk, int cs, int mosi)
    : miso(miso), clk(clk), cs(cs), mosi(mosi),
      firstReading(true), prevAngle(0), rotationCount(0)
{}

void Encoder::begin() {  
  // Configure encoder
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);  // Deselect encoder by default
  SPI.begin(clk, miso, mosi, cs);
  SPI.beginTransaction(SPISettings(CLOCK_SPEED, MSBFIRST, SPI_MODE1));
  Serial.println("AS5147 SPI Encoder Initialized");
}

int Encoder::readAngle() {
  // Reads encoder angle and converts to degrees (0-360)
  uint16_t response;
  
  // Read angle register
  digitalWrite(cs, LOW);
  // delayMicroseconds(1);  // Small delay before reading
  response = SPI.transfer16(0xFF);
  digitalWrite(cs, HIGH);
  // delayMicroseconds(1);  // Small delay between reads
  
  // Read error register (0x0001)
  // digitalWrite(cs, LOW);
  // delayMicroseconds(1);  // Small delay before reading
  // uint16_t errorReg = SPI.transfer16(0x0001); // Command to read error register with proper 16th bit MSB
  // digitalWrite(cs, HIGH);

  // Check specific error bits
  // if (errorReg & 0x0001) Serial.println("Framing Error" + String(response, BIN));
  // if (errorReg & 0x0002) Serial.println("Invalid Command" + String(response, BIN));
  // if (errorReg & 0x0004) Serial.println("Parity Error" + String(response, BIN));

  // Read diagnostic register (0x3FFC)
  // digitalWrite(cs, LOW);
  // delayMicroseconds(1);  // Small delay before reading
  // uint16_t diagReg = SPI.transfer16(0x3FFC);
  // digitalWrite(cs, HIGH);

  // Check diagnostic bits
  // if (diagReg & 0x2000) Serial.println("MAGL: Too Low Magnetic Field");
  // if (diagReg & 0x1000) Serial.println("MAGH: Too High Magnetic Field");
  // if (diagReg & 0x0800) Serial.println("COF: CORDIC Overflow");
  // if (diagReg & 0x0400) Serial.println("LF: Offset Compensation Failed");

  // Calculate the current angle between 0 and 360
  
  // if (firstReading) {
  //   prevAngle = angle;
  //   firstReading = false;
  // } else {
  //   float diff = angle - prevAngle;

  //   // Adjust for multiple rollovers/rollbacks
  //   while (diff > 180) {
  //     rotationCount--;  // A negative diff means we've rolled back past 0
  //     diff -= 360;
  //   }
  //   while (diff < -180) {
  //     rotationCount++;  // A large negative difference indicates a rollover
  //     diff += 360;
  //   }
    
  //   prevAngle = angle;
  // }

  return response;
}

float Encoder::getTotalAngle() {
  // Total angle is the number of rotations times 360 plus the current angle
  return rotationCount * 360.0 + prevAngle;
}