#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>
#include <chrono>
#include "Driver.h"
#include "Move.h"
#include "getMACAddress.h"
#include "ESPNow.h"

// Define ESP identifiers
#define ESP_GANTRY 1
#define ESP_PENDULUM 2

// Choose which ESP to compile for
#define CURRENT_ESP ESP_GANTRY  // Change this to ESP_PENDULUM when uploading to the pendulum ESP

// Define encoder SPI pins
#define ENC_MISO 12    // Encoder data output (MISO)
#define ENC_CLK  14    // SPI clock (SCK)
#define ENC_CS1  15    // Chip Select (active LOW)
#define ENC_CS2  13    // Chip Select (active LOW)
#define ENC_MOSI 5    // MOSI pin for encoder communication

#define PWM1 32
#define DIR1 33
#define PWM2 26
#define DIR2 25

#define SPEED 20

Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS1, ENC_MOSI);
Encoder ENC2(ENC_MISO, ENC_CLK, ENC_CS2, ENC_MOSI);

Driver DVR1(PWM1, DIR1);
Driver DVR2(PWM2, DIR2);
Move move(DVR1, DVR2);
ESPNowReceiver receiverESP;

uint8_t broadcastAddress[] = {0x64, 0xb7, 0x08, 0x9b, 0xaf, 0x88};
ESPNowSender senderESP(broadcastAddress);

unsigned long startTime;

void printBinary16(uint16_t n);
unsigned long getTime(unsigned long startTime);

// Gantry-specific setup
#if CURRENT_ESP == ESP_GANTRY
void setup() {
  Serial.begin(115200);
  Serial.println("Gantry ESP32 Starting...");

  ENC1.begin();
  Serial.println("Encoder 1 initialized (Gantry)");

  ENC2.begin();
  Serial.println("Encoder 2 initialized (Gantry)");

  // Initialize drivers
  DVR1.begin();
  delay(1000);
  Serial.println("Driver 1 initialized");
  Serial.flush();

  DVR2.begin();
  delay(1000);
  Serial.println("Driver 2 initialized");
  Serial.flush();

  // Initialize ESPNow communication
  receiverESP.setUp();
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    receiverESP.onDataRecv(mac, data, len);
  });

  Serial.println("Gantry setup complete!");
  Serial.flush();
}

// Gantry-specific loop
void loop() {
  // Gantry-specific control code
  // This will handle motor control and position management

  int angle1 = ENC1.readAngle();
  delay(1);
  Serial.print("G1: ");
  Serial.println(angle1);

  int angle2 = ENC2.readAngle();
  delay(1);
  Serial.print("G2: ");
  Serial.println(angle2);

  Serial.flush();
  // Example movement patterns (commented out for safety)
  /*

  Serial.println("X+ Y+");
  move.moveXY(SPEED, 1, SPEED, 1);
  delay(500);

  Serial.println("X- Y-");
  move.moveXY(SPEED, 0, SPEED, 0);
  delay(500);

  Serial.println("X+ Y-");
  move.moveXY(SPEED, 1, SPEED, 0);
  delay(500);

  Serial.println("X- Y+");
  move.moveXY(SPEED, 0, SPEED, 1);
  delay(500);

  */


}

  // Initialize pendulum-specific hardware
#elif CURRENT_ESP == ESP_PENDULUM
// Pendulum-specific setup
void setup() {
  Serial.begin(115200);
  Serial.println("Pendulum ESP32 Starting...");
  
  senderESP.setUp();
  
  ENC1.begin();
  Serial.println("Encoder 1 initialized (Pendulum)");
  
  ENC2.begin();
  Serial.println("Encoder 2 initialized (Pendulum)");

  Serial.println("Pendulum setup complete!");
  Serial.flush();
}

// Pendulum-specific loop
void loop() {
  // Pendulum-specific control code
  // This will handle sensor readings and send data to gantry
  
  int angle1 = ENC1.readAngle();
  delay(1);
  Serial.print("E1: ");
  Serial.println(angle1);

  int angle2 = ENC2.readAngle();
  delay(1);
  Serial.print("E2: ");
  Serial.println(angle2);

  senderESP.sendMessage(String("E1: " + String(angle1) + "\n" + "E2: " + String(angle2)).c_str());

}

#else
#error "Please select either ESP_GANTRY or ESP_PENDULUM for CURRENT_ESP"
#endif

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
