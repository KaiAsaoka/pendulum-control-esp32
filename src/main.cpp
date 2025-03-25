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
#define CURRENT_ESP ESP_PENDULUM  // Change this to ESP_PENDULUM when uploading to the pendulum ESP

// Define encoder SPI pins
#define ENC_MISO 12    // Encoder data output (MISO)
#define ENC_CLK  14    // SPI clock (SCK)
#define ENC_CS1  15    // Chip Select (active LOW)
#define ENC_CS2  13    // Chip Select (active LOW)
#define ENC_MOSI 9    // MOSI pin for encoder communication

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
unsigned long loopCount = 0;

void printBinary16(uint16_t n);
unsigned long getTime(unsigned long startTime);

#if CURRENT_ESP == ESP_GANTRY
void setup() {
  Serial.begin(115200);
  Serial.println("Gantry ESP32 Starting...");
  
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

void loop() {
  // Gantry-specific control code
  // This will handle motor control and position management
  loopCount++;
  Serial.print("Gantry Loop #");
  Serial.print(loopCount);
  Serial.println(" ----");

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

  Serial.println("--------------------");
  Serial.flush();
}

#elif CURRENT_ESP == ESP_PENDULUM
void setup() {
  Serial.begin(115200);
  Serial.println("Pendulum ESP32 Starting...");
  // Initialize pendulum-specific hardware
  
  // Initialize ESPNow communication
  senderESP.setUp();
  
  // Initialize SPI encoders with proper delays
  delay(100);  // Give SPI bus time to stabilize
  ENC1.begin();
  delay(100);
  Serial.println("Encoder 1 initialized");
  
  delay(100);
  ENC2.begin();
  delay(100);
  Serial.println("Encoder 2 initialized");

  Serial.println("Pendulum setup complete!");
  Serial.flush();
}

void loop() {
  // Pendulum-specific control code
  // This will handle sensor readings and send data to gantry
  
  int angle1 = ENC1.readAngle();
  Serial.print("Encoder 1 angle: ");
  Serial.println(angle1);
  delay(1000);

  int angle2 = ENC2.readAngle();
  Serial.print("Encoder 2 angle: ");
  Serial.println(angle2);
  delay(1000);

  // Add your pendulum sensor reading and control code here
  // Example: Read sensors and send data to gantry
  senderESP.sendMessage(String("Encoder 1 angle: " + String(angle1)).c_str());
  senderESP.sendMessage(String("Encoder 2 angle: " + String(angle2)).c_str());
  
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
