#include <Arduino.h>
#include <SPI.h>
#include "Encoder.h"
#include "Move.h"
#include "Driver.h"

// Define encoder SPI pins
// #define ENC_MISO 12    // Encoder data output (MISO)
// #define ENC_CLK  14    // SPI clock (SCK)
// #define ENC_CS   15    // Chip Select (active LOW)
// #define ENC_MOSI 13    // MOSI pin for encoder communication

#define PWM1 4
#define DIR1 2
#define PWM2 15
#define DIR2 13
#define SPEED 10
// Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS, ENC_MOSI);
Driver DVR1(PWM1, DIR1);
Driver DVR2(PWM2, DIR2);
Move move(DVR1, DVR2);

void setup() {
  DVR1.begin();
  DVR2.begin();
}

void loop() {
  // Read the 14-bit angle

  move.moveX(SPEED, 1);
  delay(500);
  move.stop();
  delay(1000);
  move.moveX(SPEED, 0);
  delay(500);
  move.stop();
  delay(1000);
  move.moveY(SPEED, 1);
  delay(500);
  move.stop();
  delay(1000);
  move.moveY(SPEED, 0);
  delay(500);
  move.stop();
  delay(1000);
}

// void printBinary16(uint16_t n) {
//   for (int i = 15; i >= 0; i--) {
//     Serial.print((n >> i) & 1);
//   }
//   Serial.println();
// }
