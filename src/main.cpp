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

#define PWM1 32
#define DIR1 33
#define PWM2 26
#define DIR2 25
#define SPEED 20

// Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS, ENC_MOSI);
Driver DVR1(PWM1, DIR1);
Driver DVR2(PWM2, DIR2);
Move move(DVR1, DVR2);

unsigned long startTime;
unsigned long loopCount = 0;

void printBinary16(uint16_t n);
unsigned long getTime(unsigned long startTime);

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give serial connection time to establish
  Serial.println("ESP32 Motor Control Starting...");
  Serial.flush();
  
  // ENC1.begin(); // Initializes SPI for ENC
  DVR1.begin();
  delay(1000);  // Give serial connection time to establish
  Serial.println("Driver 1 initialized");
  Serial.flush();

  DVR2.begin();
  delay(1000);  // Give serial connection time to establish
  Serial.println("Driver 2 initialized");
  Serial.flush();

  Serial.println("Setup complete!");
  Serial.flush();
}

void loop() {
  loopCount++;
  Serial.print("Loop #");
  Serial.print(loopCount);
  Serial.println(" ----");

  Serial.println("X+");
  move.moveXY(SPEED, 1, 0, 0);
  delay(500);
  Serial.println("Stop");
  move.stop();
  delay(500);

  Serial.println("X-");
  move.moveXY(SPEED, 0, 0, 0);
  delay(500);
  Serial.println("Stop");
  move.stop();
  delay(500); 

  Serial.println("Y+");
  move.moveXY(0, 0, SPEED, 1);
  delay(500);
  Serial.println("Stop");
  move.stop();
  delay(500);

  Serial.println("Y-");
  move.moveXY(0, 0, SPEED, 0);
  delay(500);
  Serial.println("Stop");
  move.stop();
  delay(500);

  Serial.println("X+ Y+");
  move.moveXY(SPEED, 1, SPEED, 1);
  delay(500);
  Serial.println("Stop");
  move.stop();
  delay(500);

  Serial.println("X- Y-");
  move.moveXY(SPEED, 0, SPEED, 0);
  delay(500);
  Serial.println("Stop");
  move.stop();
  delay(500);

  Serial.println("X+ Y-");
  move.moveXY(SPEED, 1, SPEED, 0);
  delay(500);
  Serial.println("Stop");
  move.stop();
  delay(500);

  Serial.println("X- Y+");
  move.moveXY(SPEED, 0, SPEED, 1);
  delay(500);
  Serial.println("Stop");
  move.stop();
  delay(500);

  Serial.println("--------------------");
  Serial.flush();
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
