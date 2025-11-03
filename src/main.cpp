#include <Arduino.h>
#include <SPI.h>
#include "Encoder.h" // Remove <> to avoid accidental using Arduino version
#include <chrono>
#include <Driver.h>
#include <Move.h>
#include <getMACAddress.h>
#include <PL_Telemetry_ESP32.h>
#include <ESPNow.h>
#include <PID.h>
#include <math.h>   
#include <freertos/semphr.h>


// Define ESP identifiers
#define ESP_GANTRY 1
#define ESP_PENDULUM 2
#define ESP_GANTRY_IP 192,168,137,50

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

#define STACK_SIZE 10000 // Memory size for tasks, 10 000 words (32bits) each
#define CORE_0 0
#define CORE_1 1
#define TASK_PRIORITY 0

#define TARGET_POSX 0
#define TARGET_POSY 0

#define X_DEADZONE 4
#define Y_DEADZONE 2

<<<<<<< HEAD
#define SPEED 20

#define pendKPx 0.045
#define pendKIx 0
#define pendKDx 0

#define pendlpfx 0
#define pendintcutoffx (1000 / 0.018)

#define pendKPy 0.015
#define pendKIy 0
#define pendKDy 0

#define pendlpfy 0
#define pendintcutoffy (2000 / 0.016)

#define ganKPx 0  
#define ganKIx 0
#define ganKDx 0

#define ganlpfx 0.75
#define ganintcutoffx 5

#define ganKPy 0  
#define ganKIy 0
#define ganKDy 0

#define ganlpfy 0.75
#define ganintcutoffy 5

TaskHandle_t controlLoop;

=======
>>>>>>> 7693d39 (Changed loop variables to global to allow for the telemetry task to access them)
Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS1, ENC_MOSI);
Encoder ENC2(ENC_MISO, ENC_CLK, ENC_CS2, ENC_MOSI);

// PENDULUM (ANGLE ERROR) PID X
volatile float pendKPx = 0.045;
volatile float pendKIx = 0.05;
volatile float pendKDx = 0.00016;
volatile float pendLPFx = 0;
volatile float pendIntegralCutoffx = (2000 / 0.016);

// PENDULUM (ANGLE ERROR) PID Y
volatile float pendKPy = 0.015;
volatile float pendKIy = 0.15;
volatile float pendKDy = 0.0005;
volatile float pendLPFy = 0;
volatile float pendIntegralCutoffy = (1000 / 0.018);

PID pendPIDx(pendKPx, pendKIx, pendKDx, pendLPFx, pendIntegralCutoffx);
PID pendPIDy(pendKPy, pendKIy, pendKDy, pendLPFy, pendIntegralCutoffy);

// GANTRY (POSITION ERROR) PID X
volatile float ganKPx = 0;
volatile float ganKIx = 0;
volatile float ganKDx = 0;
volatile float ganLPFx = 0.75;
volatile float ganIntegralCutoffx = 5;

// GANTRY (POSITION ERROR) PID Y
volatile float ganKPy = 0;
volatile float ganKIy = 0;
volatile float ganKDy = 0;
volatile float ganLPFy = 0.75;
volatile float ganIntegralCutoffy = 5;

PID ganPIDx(ganKPx, ganKIx, ganKDx, ganLPFx, ganIntegralCutoffx);
PID ganPIDy(ganKPy, ganKIy, ganKDy, ganLPFy, ganIntegralCutoffy);

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

// Telemetry Globals

SemaphoreHandle_t xMyMutex;

//26 Telemetry variables
const char* telemVars[] = {
  "carriageXPosition", "carriageYPostion",
  "pendulumXAngle", "pendulumYAngle",
  "xPositionError", "xSetAngleP", "xSetAngleI", "xSetAngleD", "xSetPointAngle",
  "yPositionError", "ySetAngleP", "ySetAngleI", "ySetAngleD", "ySetPointAngle",
  "xAngleError", "xSetPWMP", "xSetPWMI", "xSetPWMD", "xPWM",
  "yAngleError", "ySetPWMP", "ySetPWMI", "ySetPWMD", "yPWM",
  "loopTime", "loopWaitTime"
};

volatile int posX;
volatile int posY;
volatile int angleX;
volatile int angleY;

volatile float positionErrorX;
volatile float setAnglePX;
volatile float setAngleIX;
volatile float setAngleDX;
volatile float setPointAngleX;

volatile float positionErrorY;
volatile float setAnglePY;
volatile float setAngleIY;
volatile float setAngleDY;
volatile float setPointAngleY;

volatile float angleErrorX;
volatile float setPWMPX;
volatile float setPWMIX;
volatile float setPWMDX;
volatile float pwmX;

volatile float angleErrorY;
volatile float setPWMPY;
volatile float setPWMIY;
volatile float setPWMDY;
volatile float pwmY;

volatile uint32_t loopTime;
volatile uint32_t loopWaitTime;

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

// Telemetry Setup - Please change when flashing before tests
#define PC_IP 192,168,137,1

const char* ssid = "Tjoe-Surface";
const char* password = "d70%2D23";

const char* telemVars[] = {
  "carriageXPosition", "carriageYPosition"
  "pendulumXAngle", "pendulumYAngle",
  "xPositionKP", "xPositionKI", "xPositionKD", "xSetPointAngle",
  "yPositionKP", "yPositionKI", "yPositionKD", "ySetPointAngle",
  "xAngleKP", "xAngleKI", "xAngleKD", "xPWM",
  "yAngleKP", "yAngleKI", "yAngleKD", "yPWM",
  "loopTime", "loopWaitTime"
  };

// PL_Telemetry_ESP32 telemetry(
//   ssid,
//   password,
//   IPAddress(ESP_GANTRY_IP),
//   IPAddress(PC_IP),
//   12345,
//   0,
//   telemVars
// );

PL_Telemetry_ESP32 telemetry(
  "na",
  "na",
  IPAddress(ESP_GANTRY_IP),
  IPAddress(PC_IP),
  12345,
  telemVars
);

void setup() {
  Serial.begin(115200);
  xMyMutex = xSemaphoreCreateMutex(); // Create mutex for variable sharing
  Serial.println("Gantry ESP32 Starting...");

  pinMode(ZERO_BTN, INPUT_PULLUP);          // or INPUT if using GPIO37 with external pull-up
    
  // Attach interrupt (FALLING for normally-open button with pull-up resistor)
  attachInterrupt(digitalPinToInterrupt(ZERO_BTN), buttonISR, FALLING);
  
  Serial.println("Button interrupt initialized");

  ENC1.begin();
  Serial.println("Encoder 1 initialized (Gantry)");
  ENC1.zero();

  ENC2.begin();
  Serial.println("Encoder 2 initialized (Gantry)");
  ENC2.zero();

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

  telemetry.begin();

  // ESP32 Should make loop on core 1 anyways, but just to be sure
  // xTaskCreatePinnedToCore(
  //   control,
  //   "Control Loop",
  //   STACK_SIZE,
  //   NULL,
  //   TASK_PRIORITY,
  //   &controlLoop,
  //   CORE_1
  // );
}

// This will handle motor control and position management
void loop() {
  // ---- 1 kHz fixed-timestep cadence (wrap-safe, catch-up) ----
  static uint32_t next_tick = micros();
  uint32_t now = micros();
  loopTime = now;

  // Sleep if early
  int32_t until_tick = (int32_t)(next_tick - now);
  loopWaitTime = until_tick;
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

<<<<<<< HEAD
  // // Fixed dt (exactly 10 ms)
  // const float dt = 0.01f;

  float telemetryVariables[22];

  // int posX = move.returnPosX();
  // int posY = move.returnPosY();

  // float posErrorX = (TARGET_POSX - posX);
  // float posErrorY = (TARGET_POSY - posY);

  // auto [setPointAngleX, setAngleXp, setAngleXi, setAngleXd] = ganPIDx.calculate(posErrorX);
  // auto [setPointAngleY, setAngleYp, setAngleYi, setAngleYd] = ganPIDy.calculate(posErrorY);
  
  // // setPointAngle1 = constrain(setPointAngle1, -8, 8);
  // // setPointAngle2 = constrain(setPointAngle2, -11, 11);

  // // int pendulumAngleX = -receiverESP.data.int_message_1;
  // // int pendulumAngleY = receiverESP.data.int_message_2;
  // int pendulumAngleX = 0;
  // int pendulumAngleY = 0;

  // float angleErrorX = -(setPointAngleX - pendulumAngleX);
  // float angleErrorY = -(setPointAngleY - pendulumAngleY);


  // auto [xVel, xVelp, xVeli, xVeld] = pendPIDx.calculate(angleErrorX);
  // auto [yVel, yVelp, yVeli, yVeld] = pendPIDy.calculate(angleErrorY);

  // // if (angleErrorX < 0) {
  // //   xVel -= X_DEADZONE;
  // // } else if (angleErrorX > 0) {
  // //   xVel += X_DEADZONE ;
  // // }else{
  // //   xVel += 0;
  // // }

  // // if (angleErrorY < 0) {
  // //   yVel -= Y_DEADZONE;
  // // } else if (angleErrorY > 0) {
  // //   yVel += Y_DEADZONE ;
  // // }else{
  // //   yVel += 0;
  // // }

  // // Extract direction (true for positive, false for negative)
  // bool xDir = (xVel >= 0);
  // bool yDir = (yVel >= 0);

  // // Get absolute values for speed
  // int xSpeed = round(abs(xVel));
  // int ySpeed = round(abs(yVel));

  // // Should these constraints be here?
  // // xSpeed = constrain(xSpeed, 0, 255);
  // // ySpeed = constrain(ySpeed, 0, 255);

  // Apply to motors
  // Need to change soft limits to match new coordinates
  // if (abs(posX) < 8000 && abs(posY) < 10000 && abs(pendulumAngleX) < 2000 && abs(pendulumAngleY) < 2000){
  //   // Calculate PID outputs
  //   move.moveXY(xSpeed, xDir, ySpeed, yDir);
  // } else {
  //   move.moveXY(0, xDir, 0, yDir);
  // }

  // // Set up and send Telemetry
  // // There is probably a better way to do this (global vars? set up the array beforehand, add read/write blocking for race)
  // telemetryVariables[0] = float(posX);
  // telemetryVariables[1] = float(posY);
  // telemetryVariables[2] = float(pendulumAngleX);
  // telemetryVariables[3] = float(pendulumAngleY);
  // telemetryVariables[4] = setAngleXp;
  // telemetryVariables[5] = setAngleXi;
  // telemetryVariables[6] = setAngleXd;
  // telemetryVariables[7] = setPointAngleX;
  // telemetryVariables[8] = setAngleYp;
  // telemetryVariables[9] = setAngleYi;
  // telemetryVariables[10] = setAngleYd;
  // telemetryVariables[11] = setPointAngleY;
  // telemetryVariables[12] = xVelp;
  // telemetryVariables[13] = xVeli;
  // telemetryVariables[14] = xVeld;
  // telemetryVariables[15] = xVel;
  // telemetryVariables[16] = yVelp;
  // telemetryVariables[17] = yVeli;
  // telemetryVariables[18] = yVeld;
  // telemetryVariables[19] = yVel;
  // //placeholders for now, until I get the stuff from Cyrus' branch
  // telemetryVariables[20] = 0;
  // telemetryVariables[21] = 0;

  for (int i = 0; i++; i < 22) {
    telemetryVariables[i] = i;
=======
  // Fixed dt (exactly 10 ms)
  const float dt = 0.01f;
  // Acquire the mutex after the loop wait time
  if (xSemaphoreTake(xMyMutex, portMAX_DELAY) == pdTRUE) {
    // Snapshot inputs (avoid torn reads)
    angleX = -receiverESP.data.int_message_1;
    angleY =  receiverESP.data.int_message_2;

    // Read plant state
    posX = move.returnPosX();
    posY = move.returnPosY();

    // Outer-loop (position) errors
    positionErrorX = (TARGET_POSX - posX);
    positionErrorY = (TARGET_POSY - posY);

    // Outer PIDs -> desired angles
    auto [setAnglePX, setAngleIX, setAngleDX, setPointAngleX] = ganPIDx.calculate(positionErrorX, dt);
    auto [setAnglePY, setAngleIY, setAngleDY, setPointAngleY] = ganPIDy.calculate(positionErrorY, dt);

    // Angle limits (units must match e1/e2)
    //setPointAngle1 = constrain(setPointAngle1, -8,  8);
    //setPointAngle2 = constrain(setPointAngle2, -11, 11);

    // Inner-loop (angle) errors
    angleErrorX = -(setPointAngleX - angleX);
    angleErrorY = -(setPointAngleY - angleY);

    // Inner PIDs -> motor velocities
    auto [setPWMPX, setPWMIX, setPWMDX, pwmX] = pendPIDx.calculate(angleErrorX, dt);
    auto [setPWMPY, setPWMIY, setPWMDX, pwmY] = pendPIDy.calculate(angleErrorY, dt);

    // Deadzones
    if (angleErrorX < 0) pwmX -= X_DEADZONE;
    else if (angleErrorX > 0) pwmX += X_DEADZONE;

    if (angleErrorY < 0) pwmY -= Y_DEADZONE;
    else if (angleErrorY > 0) pwmY += Y_DEADZONE;

    // Directions and speed limits
    const bool xDir = (pwmX >= 0);
    const bool yDir = (pwmY >= 0);
    int xSpeed = (int)lroundf(fabsf(pwmX));
    int ySpeed = (int)lroundf(fabsf(pwmY));
    xSpeed = constrain(xSpeed, 0, 255);
    ySpeed = constrain(ySpeed, 0, 255);

    // Safety window + command
    if (abs(posX) < 8000 && abs(posY) < 10000 && abs(angleX) < 2000 && abs(angleY) < 2000) {
      move.moveXY(xSpeed, xDir, ySpeed, yDir);
    } else {
      move.moveXY(0, xDir, 0, yDir);
      Serial.print("Out of bounds!");
    }
    // Give the mutex back after calculations - all telemetry should be able to run during this time
    xSemaphoreGive(xMyMutex);
>>>>>>> 7693d39 (Changed loop variables to global to allow for the telemetry task to access them)
  }

  telemetry.sendSnapshot(telemetryVariables, micros());

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
