#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>
#include <chrono>
#include <Driver.h>
#include <Move.h>
#include <getMACAddress.h>
#include <ESPNow.h>
#include <PID.h>

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

#define X_DEADZONE 5
#define Y_DEADZONE 3

#define SPEED 20

#define pendKPx 0.045
#define pendKIx 0.016
#define pendKDx 0

#define pendKPy 0.035
#define pendKIy 0.023
#define pendKDy 0

#define ganKPx 0.0050  // 0.05
#define ganKIx 0.0
#define ganKDx 0.0010

#define ganKPy 0.0050  // 0.05
#define ganKIy 0.0
#define ganKDy 0.0005

Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS1, ENC_MOSI);
Encoder ENC2(ENC_MISO, ENC_CLK, ENC_CS2, ENC_MOSI);

PID pendPIDx(pendKPx, pendKIx, pendKDx);
PID pendPIDy(pendKPy, pendKIy, pendKDy);

PID ganPIDx(ganKPx, ganKIx, ganKDx);
PID ganPIDy(ganKPy, ganKIy, ganKDy);

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
  pendPIDx.reset_I();
  pendPIDy.reset_I();
  ganPIDx.reset_I();
  ganPIDy.reset_I();
  ENC1.zero(); //Old zeroing button
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
  
  int e1 = - receiverESP.data.int_message_1;

  int e2 = receiverESP.data.int_message_2;

  // int g1 = ENC1.getTotalAngle();

  // int g2 = ENC2.getTotalAngle();


  int posX = move.returnPosX();
  int posY = move.returnPosY();

  float posError1 = (TARGET_POSX - posX);
  float posError2 = (TARGET_POSY - posY);


  auto [setPointAngle1, angle1p, angle1i, angle1d] = ganPIDx.calculate(posError1);
  auto [setPointAngle2, angle2p, angle2i, angle2d] = ganPIDy.calculate(posError2);
  
  setPointAngle1 = constrain(setPointAngle1, -11, 11);
  setPointAngle2 = constrain(setPointAngle2, -11, 11);

  float error1 = -(setPointAngle1 - e1);
  float error2 = -(setPointAngle2 - e2);


  auto [xVel, xVelp, xVeli, xVeld] = pendPIDx.calculate(error1);
  auto [yVel, yVelp, yVeli, yVeld] = pendPIDy.calculate(error2);

  if (error1 < 0) {
    xVel -= X_DEADZONE;
  } else if (error1 > 0) {
    xVel += X_DEADZONE ;
  }else{
    xVel += 0;
  }

  if (error2 < 0) {
    yVel -= Y_DEADZONE;
  } else if (error2 > 0) {
    yVel += Y_DEADZONE ;
  }else{
    yVel += 0;
  }

  // Extract direction (true for positive, false for negative)
  bool xDir = (xVel >= 0);
  bool yDir = (yVel >= 0);

  // Get absolute values for speed
  int xSpeed = round(abs(xVel));
  // int xSpeed = 0;

  int ySpeed = round(abs(yVel));
  // int ySpeed = 0;

  xSpeed = constrain(xSpeed, 0, 255);
  ySpeed = constrain(ySpeed, 0, 255);

  // Apply to motors
  if (abs(posX) < 8000 && abs(posY) < 10000 && abs(e1) && abs(e1) < 2000 && abs(e2) < 2000){
    // Calculate PID outputs
    move.moveXY(xSpeed, xDir, ySpeed, yDir);
  } else {
    move.moveXY(0, xDir, 0, yDir);
  }
  Serial.print("E1: ");
  Serial.print(e1);
  Serial.print(", E2: ");
  Serial.print(e2);
  Serial.print(", G1: ");
  Serial.print(posX);
  Serial.print(", G2: ");
  Serial.print(posY);
  Serial.print(", xV: ");
  Serial.print(xVel);
  Serial.print(", yV: ");
  Serial.print(yVel);
  Serial.print(", px: ");
  Serial.print(error1);
  Serial.print(", py: ");
  Serial.print(error2);
  Serial.print(", gx: ");
  Serial.print(posError1);
  Serial.print(", gy: ");
  Serial.print(posError2);
  Serial.print(", xVelp: ");
  Serial.print(xVelp);
  Serial.print(", xVeli: ");
  Serial.print(xVeli);
  Serial.print(", xVeld: ");
  Serial.print(xVeld);
  Serial.print(", yVelp: ");
  Serial.print(yVelp);
  Serial.print(", yVeli: ");
  Serial.print(yVeli);
  Serial.print(", yVeld: ");
  Serial.print(yVeld);
  Serial.print(", setPointAngle1: ");
  Serial.print(setPointAngle1);
  Serial.print(", angle1p: ");
  Serial.print(angle1p);
  Serial.print(", angle1i: ");
  Serial.print(angle1i);
  Serial.print(", angle1d: ");
  Serial.print(angle1d);
  Serial.print(", setPointAngle2: ");
  Serial.print(setPointAngle2);
  Serial.print(", angle2p: ");
  Serial.print(angle2p);
  Serial.print(", angle2i: ");
  Serial.print(angle2i);
  Serial.print(", angle2d: ");
  Serial.println(angle2d);


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
