#include <Arduino.h>
#include <SPI.h>
#include "Encoder.h" // Remove <> to avoid accidental using Arduino version
#include <chrono>
#include <Driver.h>
#include <Move.h>
#include <getMACAddress.h>
#include <ESPNow.h>
#include <PID.h>
#include <math.h>   


// Define ESP identifiers
#define ESP_GANTRY 1
#define ESP_PENDULUM 2


// Define 1 ms loop timing
constexpr uint32_t LOOP_US = 10000;     // 10 ms
static volatile uint32_t overrun_count = 0;


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

#define X_DEADZONE 4
#define Y_DEADZONE 2

#define SPEED 20

#define pendKPx 0.045
#define pendKIx 0
#define pendKDx 0

#define pendlpfx 0
#define pendintcutoffx (1000 / 0.018)

#define pendKPy 0.015
#define pendKIy 0.
#define pendKDy 0

#define pendlpfy 0
#define pendintcutoffy (2000 / 0.016)

#define ganKPx 0.0030  // 0.05
#define ganKIx 0
#define ganKDx 0

#define ganlpfx 0.75
#define ganintcutoffx 5

#define ganKPy 0.0055  // 0.05
#define ganKIy 0
#define ganKDy 0

#define ganlpfy 0.75
#define ganintcutoffy 5

Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS1, ENC_MOSI);
Encoder ENC2(ENC_MISO, ENC_CLK, ENC_CS2, ENC_MOSI);

// PID pendPIDx(pendKPx, pendKIx, pendKDx);
// PID pendPIDy(pendKPy, pendKIy, pendKDy);

// PID ganPIDx(ganKPx, ganKIx, ganKDx);
// PID ganPIDy(ganKPy, ganKIy, ganKDy);

PID pendPIDx(pendKPx, pendKIx, pendKDx, pendlpfx, pendintcutoffx);
PID pendPIDy(pendKPy, pendKIy, pendKDy, pendlpfy, pendintcutoffy);

PID ganPIDx(ganKPx, ganKIx, ganKDx, ganlpfx, ganintcutoffx);
PID ganPIDy(ganKPy, ganKIy, ganKDy, ganlpfy, ganintcutoffy);

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

  pinMode(ZERO_BTN, INPUT_PULLUP);          // or INPUT if using GPIO37 with external pull-up
    
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
  // ---- 1 kHz fixed-timestep cadence (wrap-safe, catch-up) ----
  static uint32_t next_tick = micros();
  uint32_t now = micros();

  // Sleep if early
  int32_t until_tick = (int32_t)(next_tick - now);
  if (until_tick > 0) {
    delayMicroseconds((uint32_t)until_tick);
    now = micros();
  }

  // Catch up if we’re late by >= 1 period (no drift even on overruns)
  uint32_t missed = 0;
  while ((int32_t)(now - next_tick) >= 0) {
    next_tick += LOOP_US;   // LOOP_US = 1000
    ++missed;
  }
  overrun_count += missed;

  // Fixed dt (exactly 10 ms)
  const float dt = 0.01f;

  // Snapshot inputs (avoid torn reads)
  const int e1 = -receiverESP.data.int_message_1;
  const int e2 =  receiverESP.data.int_message_2;

  // Read plant state
  const int posX = move.returnPosX();
  const int posY = move.returnPosY();

  // Outer-loop (position) errors
  const float posError1 = (TARGET_POSX - posX);
  const float posError2 = (TARGET_POSY - posY);

  // Outer PIDs -> desired angles
  auto [angle1p, angle1i, angle1d, setPointAngle1] = ganPIDx.calculate(posError1, dt);
  auto [angle2p, angle2i, angle2d, setPointAngle2] = ganPIDy.calculate(posError2, dt);

  // Angle limits (units must match e1/e2)
  setPointAngle1 = constrain(setPointAngle1, -8,  8);
  setPointAngle2 = constrain(setPointAngle2, -11, 11);

  // Inner-loop (angle) errors
  const float error1 = -(setPointAngle1 - e1);
  const float error2 = -(setPointAngle2 - e2);

  // Inner PIDs -> motor velocities
  auto [xVelp, xVeli, xVeld, xVel] = pendPIDx.calculate(error1, dt);
  auto [yVelp, yVeli, yVeld, yVel] = pendPIDy.calculate(error2, dt);

  // Deadzones
  if (error1 < 0) xVel -= X_DEADZONE;
  else if (error1 > 0) xVel += X_DEADZONE;

  if (error2 < 0) yVel -= Y_DEADZONE;
  else if (error2 > 0) yVel += Y_DEADZONE;

  // Directions and speed limits
  const bool xDir = (xVel >= 0);
  const bool yDir = (yVel >= 0);
  int xSpeed = (int)lroundf(fabsf(xVel));
  int ySpeed = (int)lroundf(fabsf(yVel));
  xSpeed = constrain(xSpeed, 0, 255);
  ySpeed = constrain(ySpeed, 0, 255);

  // Safety window + command
  if (abs(posX) < 8000 && abs(posY) < 10000 && abs(e1) < 2000 && abs(e2) < 2000) {
    move.moveXY(xSpeed, xDir, ySpeed, yDir);
  } else {
    move.moveXY(0, xDir, 0, yDir);
  }

  static uint32_t k=0;
  if ((k++ % 25) == 0) {
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
  }
  
  
   // Check if button was pressed
  if (buttonPressed) {
    handleButtonPress();
    buttonPressed = false;  // Reset the flag
  }
}


#elif CURRENT_ESP == ESP_PENDULUM

#define ZERO_BTN 37


void setup() {
  Serial.begin(115200);
  Serial.println("Pendulum ESP32 Starting...");

  senderESP.setUp();

  ENC1.begin();  Serial.println("Encoder 1 initialized (Pendulum)");
  ENC2.begin();  Serial.println("Encoder 2 initialized (Pendulum)");

  pinMode(ZERO_BTN, INPUT_PULLUP);           // or INPUT if using GPIO37 with external pull-up
  attachInterrupt(digitalPinToInterrupt(ZERO_BTN), buttonISR, FALLING);
  Serial.println("Button interrupt initialized");
  Serial.println("Pendulum setup complete!");
}

void loop() {
  // ---- 1 kHz fixed-timestep cadence (wrap-safe, catch-up) ----
  static uint32_t next_tick = micros();
  uint32_t now = micros();

  int32_t until_tick = (int32_t)(next_tick - now);
  if (until_tick > 0) { delayMicroseconds((uint32_t)until_tick); now = micros(); }

  uint32_t missed = 0;
  while ((int32_t)(now - next_tick) >= 0) { next_tick += 10000; ++missed; }
  // overrun_count += missed;  // optional

  // ---- pendulum work ----
  const int angle1 = ENC1.getTotalAngle();
  const int angle2 = ENC2.getTotalAngle();

  static uint32_t k = 0;
  if ((k++ % 25) == 0) {                   // ~40 Hz debug
    Serial.print("E1: "); Serial.print(angle1);
    Serial.print(", E2: "); Serial.println(angle2);
  }

  // Send at 200 Hz, avoid String allocations
  static uint32_t t = 0;
  if ((t++ % 5) == 0) {
    static char msg[40];
    snprintf(msg, sizeof(msg), "E1:%d\nE2:%d", angle1, angle2);
    senderESP.sendMessage(msg);
  }

  // Debounce example (optional)
  if (buttonPressed) {
    static uint32_t lastPressUs = 0;
    uint32_t usNow = micros();
    if ((int32_t)(usNow - lastPressUs) > 15000) { // ~15 ms
      handleButtonPress();
      lastPressUs = usNow;
    }
    buttonPressed = false;
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
