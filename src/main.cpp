#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>
#include <chrono>
#include "Driver.h"
#include "Move.h"
#include "getMACAddress.h"
#include "ESPNow.h"
#include "PID.h"

// Define ESP identifiers
#define ESP_GANTRY 1
#define ESP_PENDULUM 2

// Choose which ESP to compile for
#define CURRENT_ESP ESP_GANTRY// Change this to ESP_PENDULUM when uploading to the pendulum ESP

// // Define encoder SPI pins
// #define ENC_MISO 12    // Encoder data output (MISO)
// #define ENC_CLK  14    // SPI clock (SCK)
// #define ENC_CS1  15    // Chip Select (active LOW)
// #define ENC_CS2  13    // Chip Select (active LOW)
// #define ENC_MOSI 5    // MOSI pin for encoder communication

// Define encoder SPI pins
#define ENC_MISO 26    // Encoder data output (MISO)
#define ENC_CLK  25    // SPI clock (SCK)
#define ENC_CS1  32    // Chip Select (active LOW)
#define ENC_CS2  33    // Chip Select (active LOW)
#define ENC_MOSI 9    // MOSI pin for encoder communication




#define TARGET_POSX 0
#define TARGET_POSY 0




#define SPEED 20

#define pendKP 0.005
#define pendKI 0
#define pendKD 0

Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS1, ENC_MOSI);
Encoder ENC2(ENC_MISO, ENC_CLK, ENC_CS2, ENC_MOSI);

PID pendPID(pendKP, pendKI, pendKD);;

ESPNowReceiver receiverESP;

uint8_t broadcastAddress[] = {0x64, 0xb7, 0x08, 0x9c, 0x5b, 0xb0};
ESPNowSender senderESP(broadcastAddress);

unsigned long startTime;

void printBinary16(uint16_t n);
unsigned long getTime(unsigned long startTime);

// Flag to indicate button was pressed (must be volatile)
volatile bool buttonPressed = false;

// Time tracking for debouncing
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // milliseconds

// Interrupt Service Routine (ISR)
void IRAM_ATTR buttonISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > debounceDelay) {
    buttonPressed = true;
    lastDebounceTime = currentTime;
  }
}

// The function to run when button is pressed
void handleButtonPress() {
  // Your button handling code here
  Serial.println("Button was pressed!");
  ENC1.zero();
  ENC2.zero();
}

// Gantry-specific setup
#if CURRENT_ESP == ESP_GANTRY

#define ZERO_BTN 37
#define PWM2 19
#define DIR2 22
#define PWM1 21
#define DIR1 20

Driver DVR1(PWM1, DIR1);
Driver DVR2(PWM2, DIR2);

Move move(DVR1, DVR2, ENC1, ENC2);

void setup() {
  Serial.begin(115200);
  Serial.println("Gantry ESP32 Starting...");

  pinMode(ZERO_BTN, INPUT_PULLUP);
    
  // Attach interrupt (FALLING for normally-open button with pull-up resistor)
  attachInterrupt(digitalPinToInterrupt(ZERO_BTN), buttonISR, FALLING);
  
  Serial.println("Button interrupt initialized");

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
  
  int e1 = receiverESP.data.int_message_1;
  delay(1);
  Serial.print("E1: ");
  Serial.print(e1);

  int e2 = receiverESP.data.int_message_2;
  delay(1);
  Serial.print("E2: ");
  Serial.print(e2);

  int g1 = ENC1.getTotalAngle();
  delay(1);
  // Serial.print("G1: ");
  // Serial.println(g1);

  int g2 = ENC2.getTotalAngle();
  delay(1);
  // Serial.print("G2: ");
  // Serial.println(g2);

  int posX = move.returnPosX();
  int posY = move.returnPosY();

  float error1 = TARGET_POSX - posX;
  float error2 = TARGET_POSY - posY;

  Serial.print(", error1: ");
  Serial.print(error1);
  Serial.print(", error2: ");
  Serial.print(error2);

  // Calculate PID outputs
  float xVel = pendPID.calculate(error1);
  float yVel = pendPID.calculate(error2);

  // Serial.print(", xVel: ");
  // Serial.print(xVel);
  // Serial.print(", yVel: ");
  // Serial.println(yVel);

  // // Print debug info
  // Serial.print("xVel: ");
  // Serial.println(xVel);
  // Serial.print(" yVel: ");
  // Serial.println(yVel);

  // Extract direction (true for positive, false for negative)
  bool xDir = (xVel >= 0);
  bool yDir = (yVel >= 0);

  // Get absolute values for speed
  int xSpeed = abs(xVel);
  int ySpeed = abs(yVel);

  xSpeed = constrain(xSpeed, 0, 255);
  ySpeed = constrain(ySpeed, 0, 255);
  Serial.print(", xSpeed: ");
  Serial.print(xSpeed);
  Serial.print(", xDir: ");
  Serial.print(xDir);

  Serial.print(", ySpeed: ");
  Serial.print(ySpeed);
  Serial.print(", yDir: ");
  Serial.println(yDir);

  // Apply to motors
  move.moveXY(xSpeed, xDir, ySpeed, yDir);

  Serial.flush();
  // Example movement patterns (commented out for safety)
  
   // Check if button was pressed
  if (buttonPressed) {
    handleButtonPress();
    buttonPressed = false;  // Reset the flag
  }

}

  // Initialize pendulum-specific hardware
#elif CURRENT_ESP == ESP_PENDULUM
// Pendulum-specific setup
#define ZERO_BTN 37

void setup() {
  Serial.begin(115200);
  Serial.println("Pendulum ESP32 Starting...");
  
  senderESP.setUp();
  
  ENC1.begin();
  Serial.println("Encoder 1 initialized (Pendulum)");
  
  ENC2.begin();
  Serial.println("Encoder 2 initialized (Pendulum)");

  pinMode(ZERO_BTN, INPUT_PULLUP);
    
  // Attach interrupt (FALLING for normally-open button with pull-up resistor)
  attachInterrupt(digitalPinToInterrupt(ZERO_BTN), buttonISR, FALLING);
  
  Serial.println("Button interrupt initialized");

  Serial.println("Pendulum setup complete!");
  Serial.flush();
}

// Pendulum-specific loop
void loop() {
  // Pendulum-specific control code
  // This will handle sensor readings and send data to gantry
  
  int angle1 = ENC1.getTotalAngle();
  delay(1);
  Serial.print("E1: ");
  Serial.print(angle1);

  int angle2 = ENC2.getTotalAngle();
  delay(1);
  Serial.print(", E2: ");
  Serial.print(angle2);

  senderESP.sendMessage(String("E1: " + String(angle1) + "\n" + "E2: " + String(angle2)).c_str(), angle1, angle2);

  // Check if button was pressed
  if (buttonPressed) {
    handleButtonPress();
    buttonPressed = false;  // Reset the flag
  }
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
