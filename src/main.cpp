#include <Arduino.h>
#include <SPI.h>
#include "Encoder.h" // Remove <> to avoid accidental using Arduino version
#include <chrono>
#include <Driver.h>
#include <Move.h>
#include <PL_Telemetry_ESP32.h>
#include <PID.h>
#include <math.h>
#include <freertos/semphr.h>
#include <array>

// ADDED: Include the BNO085 Library
#include "SparkFun_BNO080_Arduino_Library.h"

// // Define ESP identifiers
// #define ESP_GANTRY 1
// #define ESP_PENDULUM 2

// // Define loop timing 
// constexpr uint32_t LOOP_US = 10000;     // 1.5 ms
// constexpr uint32_t MAX_GANTRY_LOOP_US = LOOP_US;
// static volatile uint32_t overrun_count = 0;
// int controlCycle = 0;
// const float dt = LOOP_US * 1e-6f; // Convert microseconds to seconds for PID calculations

// constexpr uint32_t POS_UPDATE_US = 10000;               // 1 ms
// constexpr int POS_UPDATE_CYCLES = POS_UPDATE_US / LOOP_US;

// #define CONTROL_LOOP_PIN 15

// SPI bus pins (shared)
#define ENC_MISO 25
#define ENC_MOSI 26
#define ENC_CLK  14

// // --- BNO085 Specific Pins ---
// // CHANGED: IMU_CS moved from 5 to 18 to avoid conflict with RED_LED
#define IMU_CS   32   
#define IMU_RST  33  // Reset pin
#define IMU_WAK  27  // Wake pin
#define IMU_BOOTN  22  // Boot pin
#define IMU_INTN 21 // Init pin

// // Gantry motor encoder chip-selects
// #define ENC_CS1  2
// #define ENC_CS2  4
// // Pendulum encoder chip-selects
// #define PEND_CS1 0
// #define PEND_CS2 19

// #define ZERO_BTN 37
// #define AUX_BTN 38        // Extra safety / aux button
// #define BLUE_LED 10       // "Armed" status LED
// #define RED_LED 5         // Out-of-bounds LED

// #define PWM2 7
// #define DIR2 9
// #define PWM1 8
// #define DIR1 20

// // Analog potentiometer tuning pins
// #define MOVE_TARGET_POSX_PIN 13 // Move target position with joystick (X)
// #define MOVE_TARGET_POSY_PIN 12 // Move target position with joystick (Y)
// // #define JOYSTICK_BUTTON_PIN 15
// #define MOVE_TARGET_POSX_SCALE_FACTOR 0.0004 // Tune sensitivity of joystick for target position (X)
// #define MOVE_TARGET_POSY_SCALE_FACTOR 0.0004 // Tune sensitivity of joystick for target position (Y)
// #define JOYSTICK_DEAD_ZONE 50 // To prevent drift when joystick is near neutral
// #define JOYSTICK_OFFSET_X -120 // Calibrate the joystick centre
// #define JOYSTICK_OFFSET_Y -110

// #define X_DEADZONE 12
// #define Y_DEADZONE 0

// #define STACK_SIZE 10000
// #define TASK_PRIORITY 0
// #define CORE_0 0
// #define CORE_1 1

// // Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS1, ENC_MOSI);
// // Encoder ENC2(ENC_MISO, ENC_CLK, ENC_CS2, ENC_MOSI);

// // Encoder PEND1(ENC_MISO, ENC_CLK, PEND_CS1, ENC_MOSI, 0); 
// // Encoder PEND2(ENC_MISO, ENC_CLK, PEND_CS2, ENC_MOSI, 0); 

// // ADDED: IMU Object
BNO080 myIMU;

// // // Param order: kp, ki, kd, ap, ai, ad, ao, iCutoff
// pidParams setAngleXParams = {15, 3, 48, 0, 0, 985, 0, 50000000}; 
// pidParams setAngleYParams = {15, 3, 25, 0, 0, 985, 0, 50000000};
// pidParams setPWMXParams = {600, 0, 7, 890, 0, 880, 0, 0};
// pidParams setPWMYParams = {600, 0, 7, 790, 0, 880, 0, 0};

// PID setPWMPIDX(setPWMXParams);
// PID setPWMPIDY(setPWMYParams);
// PID setAnglePIDX(setAngleXParams);
// PID setAnglePIDY(setAngleYParams);
// std::array<pidParams, 4> paramSet;

// pidOutputs setAngleXOutputs;
// pidOutputs setAngleYOutputs;
// pidOutputs setPWMXOutputs;
// pidOutputs setPWMYOutputs;

// unsigned long startTime;

// void printBinary16(uint16_t n);
// unsigned long getTime(unsigned long startTime);

// // Flag to indicate button was pressed (must be volatile)
// volatile bool buttonPressed = false;

// // Time tracking for debouncing
// volatile unsigned long lastDebounceTime = 0;
// const unsigned long debounceDelay = 300;  // milliseconds

// volatile bool auxButtonPressed = false;
// volatile bool zeroButtonState = false;   // false = not armed, true = armed

// // Interrupt Service Routine (ISR)
// void IRAM_ATTR buttonISR() {
//   unsigned long currentTime = millis();
//   if (currentTime - lastDebounceTime > debounceDelay) {
//     buttonPressed = true;
//     lastDebounceTime = currentTime;
//   }
// }

// void IRAM_ATTR auxButtonISR() {
//   unsigned long currentTime = millis();
//   if (currentTime - lastDebounceTime > debounceDelay) {
//     auxButtonPressed = true;
//     lastDebounceTime = currentTime;
//   }
// }

// struct stateErrs {
//   int positionErrorX;
//   int positionErrorY;
//   int angleErrorX;
//   int angleErrorY;
// };

// stateErrs stateErrors;

// // Telemetry Globals
// SemaphoreHandle_t pidValsMutex;
// bool pauseTesting = false;

// //Telemetry variable names
// const char* telemVars[] = {
//   "carriageXPosition", "pendulumXAngle",
//   "xAngleError", "xPWMp", "xPWMi", "xPWMd", "xPWMout", "xPWM",
//   "xPositionError", "angleXp", "angleXi", "angleXd", "setAngleXOut",
//   "carriageYPosition", "pendulumYAngle",
//   "yAngleError", "yPWMp", "yPWMi", "yPWMd", "yWMout", "yPWM",
//   "yPositionError", "angleYp", "angleYi", "angleYd", "setAngleYOut"
// };

// float telemVals[26];

// struct stateVars {
//   int posX;
//   int posY;
//   int angleX;
//   int angleY;
//   int joystick_reading_x;
//   int joystick_reading_y;
//   float targetPosX; 
//   float targetPosY; 
// };

// stateVars stateVariables;
// motorPWMs PWMOutputs;

// volatile uint32_t loopTime;
// volatile uint32_t loopWaitTime;

// PL_Telemetry_ESP32 telemetry(
//   telemVars,
//   setAngleXParams,
//   setAngleYParams,
//   setPWMXParams,
//   setPWMYParams
// );

// TaskHandle_t telemTask;

// Driver DVR1(PWM1, DIR1);
// Driver DVR2(PWM2, DIR2);

// // Move move(DVR1, DVR2, ENC1, ENC2);

// void updateTelemetry() {
//   telemVals[0] = stateVariables.posX;
//   telemVals[1] = stateVariables.angleX;
//   telemVals[2] = -stateErrors.angleErrorX;
//   telemVals[3] = setPWMXOutputs.pOut;
//   telemVals[4] = setPWMXOutputs.iOut;
//   telemVals[5] = setPWMXOutputs.dOut;
//   telemVals[6] = setPWMXOutputs.output;
//   telemVals[7] = PWMOutputs.xPWM;
//   telemVals[8] = stateErrors.positionErrorX;
//   telemVals[9] = setAngleXOutputs.pOut;
//   telemVals[10] = setAngleXOutputs.iOut;
//   telemVals[11] = setAngleXOutputs.dOut;
//   telemVals[12] = setAngleXOutputs.output;
//   telemVals[13] = stateVariables.posY;
//   telemVals[14] = stateVariables.angleY;
//   telemVals[15] = -stateErrors.angleErrorY;
//   telemVals[16] = setPWMYOutputs.pOut;
//   telemVals[17] = setPWMYOutputs.iOut;
//   telemVals[18] = setPWMYOutputs.dOut;
//   telemVals[19] = setPWMYOutputs.output;
//   telemVals[20] = PWMOutputs.yPWM;
//   telemVals[21] = stateErrors.positionErrorY;
//   telemVals[22] = setAngleYOutputs.pOut;
//   telemVals[23] = setAngleYOutputs.iOut;
//   telemVals[24] = setAngleYOutputs.dOut;
//   telemVals[25] = setAngleYOutputs.output;
// }

// void telemLoop(void *pvParameters){
//   for(;;){
//     uint32_t start_us = micros();

//     if (xSemaphoreTake(pidValsMutex, portMAX_DELAY) == pdTRUE) {
//       updateTelemetry();
//       telemetry.sendSnapshot(telemVals, start_us);
//       xSemaphoreGive(pidValsMutex);
//     }

//     uint32_t elapsed = (uint32_t)(micros() - start_us);

//     if (elapsed >= LOOP_US) {
//       overrun_count++;
//     } else {
//       while ((uint32_t)(micros() - start_us) < LOOP_US) {
//         if(telemetry.pauseTesting()) {
//           if(telemetry.updateGainVals()) {
//             if(xSemaphoreTake(pidValsMutex, portMAX_DELAY) == pdTRUE) {
//               setAnglePIDX.readNewGains(telemetry.setAngleXParams);
//               setAnglePIDY.readNewGains(telemetry.setAngleYParams);
//               setPWMPIDX.readNewGains(telemetry.setPWMXParams);
//               setPWMPIDY.readNewGains(telemetry.setPWMYParams);
//               xSemaphoreGive(pidValsMutex);
//             }
//           }
//         }
//       }
//     }
//   }
// }

void setup() {
  delay(5000);
  SPI.begin(ENC_CLK, ENC_MISO, ENC_MOSI);

  Serial.begin(115200);
  // telemetry.begin();
  // pidValsMutex = xSemaphoreCreateMutex(); // Create mutex for errors

  // pinMode(ZERO_BTN, INPUT_PULLUP);
  // pinMode(AUX_BTN, INPUT_PULLUP);
  // pinMode(BLUE_LED, OUTPUT);
  // pinMode(RED_LED, OUTPUT);
  // pinMode(CONTROL_LOOP_PIN, OUTPUT);

  // digitalWrite(BLUE_LED, LOW);
  // digitalWrite(RED_LED, LOW);
  // digitalWrite(CONTROL_LOOP_PIN, LOW);

  // attachInterrupt(digitalPinToInterrupt(ZERO_BTN), buttonISR, FALLING);
  // attachInterrupt(digitalPinToInterrupt(AUX_BTN), auxButtonISR, FALLING);

  // stateVariables.targetPosX = 0;
  // stateVariables.targetPosY = 0;
  // stateVariables.joystick_reading_x = 0;
  // stateVariables.joystick_reading_y = 0;

  // ENC1.begin();
  // ENC2.begin();
  
  // // NOTE: Pendulum Encoders left enabled in setup in case you still need them, 
  // // but they are no longer updating stateVariables.angle in readState()
  // PEND1.begin();
  // PEND2.begin();

  // ADDED: Initialize BNO085 IMU
  // myIMU.enableDebugging(Serial);

  // DVR1.begin();
  // delay(1000);
  // Serial.flush();

  // DVR2.begin();
  // delay(1000);
  // Serial.flush();

  // Serial.flush();

  // xTaskCreatePinnedToCore(
  //   telemLoop,
  //   "Telemetry Loop",
  //   STACK_SIZE,
  //   NULL,
  //   TASK_PRIORITY,
  //   &telemTask,
  //   CORE_0
  // );

  Serial.println("Starting BNO085 over SPI...");
  if (myIMU.beginSPI(IMU_CS, IMU_WAK, IMU_INTN, IMU_RST) == false) {
    Serial.println("BNO085 not detected. Check wiring!");
    // while (1); // Commented out so gantry won't completely freeze if IMU wires slip
  } else {
    Serial.println("Trying to enable rotation vector");
    myIMU.enableRotationVector(1); // 400Hz update rate
    Serial.println("BNO085 Initialized!");
  }
}

// void readState() { 
//   // 1. UPDATE PENDULUM ANGLES VIA BNO085 IMU
//   if (myIMU.dataAvailable() == true) {
//     float roll  = (myIMU.getRoll()) * 180.0 / PI;   
//     float pitch = (myIMU.getPitch()) * 180.0 / PI;  
    
//     // Cast float to int to match existing struct. 
//     // WARNING: Your PID gains must be retuned since these are now 
//     // degrees (-180 to 180) instead of raw 14-bit encoder ticks!
//     stateVariables.angleX = (int)roll;
//     stateVariables.angleY = (int)pitch;
//   }

//   // Serial.println(stateVariables.angleX);
//   // Serial.println(stateVariables.angleY);

//   // Old encoder reads commented out:
//   // stateVariables.angleX = -PEND1.getTotalAngle();
//   // stateVariables.angleY =  PEND2.getTotalAngle();

//   // 2. UPDATE GANTRY POSITIONS VIA MOTORS
//   // stateVariables.posX = move.returnPosX();
//   // stateVariables.posY = move.returnPosY();
// }

// // Custom signum function where sgn(0) = 1 instead of 0
// int sgn(int val) {
//   return (0 <= val) - (val < 0);
// }

// // Ensure pendulum is at rest against one side of the mount beforehand
// void swingUp() {
//   int REPOSITION_SPEED = 7; 
//   int SWINGUP_SPEED_X = 10;   
//   int SWINGUP_SPEED_Y = 10;
//   int EXCESS_REPOSITION_TIME_MS = 1000; 
//   int SWINGUP_TIME_MS = 250; 

//   // move.moveXY(0, 0);
//   int now = millis();
//   while (millis() - now < 4000) {delayMicroseconds(100);} 
//   readState();

//   int x_dir = sgn(stateVariables.angleX);
//   int y_dir = sgn(stateVariables.angleY);

//   uint32_t loop_timer = micros();

//   while (x_dir * stateVariables.posX < 2750 && y_dir * stateVariables.posY < 4000) {
//     // move.moveXY(REPOSITION_SPEED * -x_dir, REPOSITION_SPEED * -y_dir); 
//     readState();
//     while (micros() - loop_timer < LOOP_US) {delayMicroseconds(100);} 
//     loop_timer = micros();
//   }

//   int start_reposition_time = millis();
//   while (millis() - start_reposition_time < 100) { 
//     // move.moveXY(REPOSITION_SPEED * -x_dir, REPOSITION_SPEED * -y_dir);
//     while (micros() - loop_timer < LOOP_US) {delayMicroseconds(100);}
//     loop_timer = micros();
//   }

//   // move.moveXY(0, 0);
//   delay(3000); 

//   loop_timer = micros();
//   int swingup_start = millis();
  
//   while (millis() - swingup_start < SWINGUP_TIME_MS) {
//     // move.moveXY(SWINGUP_SPEED_X * x_dir, SWINGUP_SPEED_Y * y_dir); 
//     readState();
//     while (micros() - loop_timer < LOOP_US) {delayMicroseconds(100);}
//     loop_timer = micros();
//   }

//   while (abs(stateVariables.angleX) > 10 && abs(stateVariables.angleY) > 10) { 
//     // move.moveXY(0, 0);
//     readState();
//     while (micros() - loop_timer < LOOP_US) {delayMicroseconds(100);}
//     loop_timer = micros();
//   }
// }

// void updateTargetPos() {
//   WiFi.mode(WIFI_OFF);
//   stateVariables.joystick_reading_x = analogRead(MOVE_TARGET_POSX_PIN) - 2048 - JOYSTICK_OFFSET_X; 
//   stateVariables.joystick_reading_y = -1*(analogRead(MOVE_TARGET_POSY_PIN) - 2048 - JOYSTICK_OFFSET_Y); 

//   if (abs(stateVariables.joystick_reading_x) > JOYSTICK_DEAD_ZONE) { 
//     stateVariables.targetPosX += MOVE_TARGET_POSX_SCALE_FACTOR*stateVariables.joystick_reading_x; 
//     stateVariables.targetPosX = constrain(stateVariables.targetPosX, -2750, 2750); 
//   }
//   if (abs(stateVariables.joystick_reading_y) > JOYSTICK_DEAD_ZONE) {
//     stateVariables.targetPosY += MOVE_TARGET_POSY_SCALE_FACTOR*stateVariables.joystick_reading_y; 
//     stateVariables.targetPosY = constrain(stateVariables.targetPosY, -4000, 4000); 
//   }
// }

// void runControl(float dt, int controlCycle) {
//   if (controlCycle == POS_UPDATE_CYCLES) {
//     stateErrors.positionErrorX = (stateVariables.posX - (int)stateVariables.targetPosX);
//     stateErrors.positionErrorY = ((int)stateVariables.targetPosY - stateVariables.posY);
//   }

//   setAngleXOutputs = setAnglePIDX.calculate(stateErrors.positionErrorX, dt);
//   setAngleYOutputs = setAnglePIDY.calculate(stateErrors.positionErrorY, dt);

//   stateErrors.angleErrorX = (setAngleXOutputs.output - stateVariables.angleX);
//   stateErrors.angleErrorY = -(setAngleYOutputs.output - stateVariables.angleY);

//   setPWMXOutputs = setPWMPIDX.calculate(stateErrors.angleErrorX, dt);
//   setPWMYOutputs = setPWMPIDY.calculate(stateErrors.angleErrorY, dt);

//   PWMOutputs = {setPWMXOutputs.output, setPWMYOutputs.output};
// }

// void handleButtonPress() {
//   setPWMPIDX.reset();
//   setPWMPIDY.reset();
//   setAnglePIDX.reset();
//   setAnglePIDY.reset();
//   stateErrors.positionErrorX = 0;
//   stateErrors.positionErrorY = 0;
//   stateVariables.targetPosX = 0;
//   stateVariables.targetPosY = 0;
//   // ENC1.zero(); 
//   // ENC2.zero();
//   // PEND1.zero();
//   // PEND2.zero();

//   zeroButtonState = !zeroButtonState;
//   digitalWrite(BLUE_LED, zeroButtonState ? HIGH : LOW);
// }

// void handleAuxButtonPress() {
//   setPWMPIDX.reset();
//   setPWMPIDY.reset();
//   setAnglePIDX.reset();
//   setAnglePIDY.reset();
//   stateErrors.positionErrorX = 0;
//   stateErrors.positionErrorY = 0;
//   stateVariables.targetPosX = 0;
//   stateVariables.targetPosY = 0;
// }

void loop() {
    if (myIMU.dataAvailable()) {
      float roll  = myIMU.getRoll()  * 180.0 / PI;
      float pitch = myIMU.getPitch() * 180.0 / PI;
      float yaw   = myIMU.getYaw()   * 180.0 / PI;

      Serial.print("Roll: ");  Serial.print(roll);
      Serial.print(" Pitch: "); Serial.print(pitch);
      Serial.print(" Yaw: ");   Serial.println(yaw);
  }

//   static uint32_t start_us = micros(); 

//   if (micros() - start_us >= LOOP_US) {
//     overrun_count++;
//     digitalWrite(CONTROL_LOOP_PIN, !digitalRead(CONTROL_LOOP_PIN)); 
//     while(micros() - start_us > LOOP_US) {
//       start_us += LOOP_US;
//     }
//   }
  
//   // The BNO085's INT pin is continuously checked right here
//   while(micros() - start_us < LOOP_US) {
//     // readState();
//   }
//   readState();
//   start_us += LOOP_US;

//   updateTargetPos();

//   if(!zeroButtonState || telemetry.pauseTesting()) {
//     // move.moveXY(0, 0);
//     if(!zeroButtonState) {
//       setPWMPIDX.reset();
//       setPWMPIDY.reset();
//       setAnglePIDX.reset();
//       setAnglePIDY.reset();
//     }
//   }
//   else {
//     if (xSemaphoreTake(pidValsMutex, portMAX_DELAY) == pdPASS) {

//       runControl(dt, controlCycle);
//       controlCycle++;
//       if(controlCycle > POS_UPDATE_CYCLES) controlCycle = 1;

//       PWMOutputs.xPWM = constrain(PWMOutputs.xPWM, -255, 255);
//       PWMOutputs.yPWM = constrain(PWMOutputs.yPWM, -255, 255);

//       if (abs(stateVariables.posX) < 2750 && abs(stateVariables.posY) < 4000) { 
//         // move.moveXY(PWMOutputs.xPWM, PWMOutputs.yPWM);
//         digitalWrite(RED_LED, LOW);
//       } else {
//         PWMOutputs.xPWM = 0;
//         PWMOutputs.yPWM = 0;
//         // move.moveXY(0, 0);
//         digitalWrite(RED_LED, HIGH);
//       }

//       xSemaphoreGive(pidValsMutex);
//     }
//   }

//   if (buttonPressed) {
//     buttonPressed = false;
//     if (digitalRead(ZERO_BTN) == LOW) {
//       handleButtonPress();
//     }
//   }

//   if (auxButtonPressed) {
//     auxButtonPressed = false;
//     if (digitalRead(AUX_BTN) == LOW) {
//       handleAuxButtonPress();
//     }
//   }
}