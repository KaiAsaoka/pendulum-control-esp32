#include "Encoder.h"
#include <Arduino.h>
#include <SPI.h>

Encoder::Encoder(int miso, int clk, int cs, int mosi) : miso(miso), clk(clk), cs(cs), mosi(mosi) {}

void Encoder::begin() {  
  // Configure encoder
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);  // Deselect encoder by default
  SPI.begin(clk, miso, mosi, cs);
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
  Serial.println("AS5147 SPI Encoder Initialized");
}


float Encoder::readAngle() {
  // Reads encoder angle and outputs in degrees
  uint16_t response;
  digitalWrite(cs, LOW);
  response = SPI.transfer16(0x3FFF);
  digitalWrite(cs, HIGH);
  float angle = float(response & 0x3FFF) / 16384.00 * 360.00;
  return angle;
}