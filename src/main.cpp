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

// Define ESP identifiers
#define ESP_GANTRY 1
#define ESP_PENDULUM 2

// Define 1 ms loop timing 
constexpr uint32_t LOOP_US = 1000;     // 1 ms
constexpr uint32_t MAX_GANTRY_LOOP_US = LOOP_US;
static volatile uint32_t overrun_count = 0;
int controlCycle = 0;
const float dt = LOOP_US * 1e-6f; // Convert microseconds to seconds for PID calculations

constexpr uint32_t POS_UPDATE_US = 1000;               // 1 ms
constexpr int POS_UPDATE_CYCLES = POS_UPDATE_US / LOOP_US;

#define CONTROL_LOOP_PIN 15

// SPI bus pins (shared)
#define ENC_MISO 27
#define ENC_MOSI 0
#define ENC_CLK  14

// Gantry motor encoder chip-selects
#define ENC_CS1  33
#define ENC_CS2  32
// Pendulum encoder chip-selects
#define PEND_CS1 26
#define PEND_CS2 25

#define ZERO_BTN 37
#define AUX_BTN 38        // Extra safety / aux button
#define BLUE_LED 10       // "Armed" status LED
#define RED_LED 5         // Out-of-bounds LED

// Analog potentiometer tuning pins
#define MOVE_TARGET_POSX_PIN 12 // Move target position with joystick (X)
#define MOVE_TARGET_POSY_PIN 13 // Move target position with joystick (Y)
// #define JOYSTICK_BUTTON_PIN 15
#define MOVE_TARGET_POSX_SCALE_FACTOR 0.0004 // Tune sensitivity of joystick for target position (X)
#define MOVE_TARGET_POSY_SCALE_FACTOR 0.0004 // Tune sensitivity of joystick for target position (Y)
#define JOYSTICK_DEAD_ZONE 50 // To prevent drift when joystick is near neutral
#define JOYSTICK_OFFSET_X -63 // Calibrate the joystick centre
#define JOYSTICK_OFFSET_Y -53

#define X_DEADZONE 12
#define Y_DEADZONE 0

#define STACK_SIZE 10000
#define TASK_PRIORITY 0
#define CORE_0 0
#define CORE_1 1

Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS1, ENC_MOSI);
Encoder ENC2(ENC_MISO, ENC_CLK, ENC_CS2, ENC_MOSI);

Encoder PEND1(ENC_MISO, ENC_CLK, PEND_CS1, ENC_MOSI, 0); // RC: Pend angle tends to spike between 0 when angle > 0 and -2*numBitIgnore when angle < 0 as expected 
Encoder PEND2(ENC_MISO, ENC_CLK, PEND_CS2, ENC_MOSI, 0); // RC: (but not desired). Ignore Greg's suggestion and use filtering instead for now

// Param order: kp, ki, kd, ap, ai, ad, ao, iCutoff
pidParams setAngleXParams = {0, 0, 0, 0, 0, 0, 0, 0};
// {45, 50, 0.16, 0, 125000000}
pidParams setAngleYParams = {0, 0, 0, 0, 0, 0, 0, 0};
// {15, 150, 0.5, 0, 55555555}
pidParams setPWMXParams = {0, 0, 0, 0, 0, 0, 0, 0};
// {0, 0, 0, 750, 1000}
pidParams setPWMYParams = {0, 0, 0, 0, 0, 0, 0, 0};

PID setPWMPIDX(setPWMXParams);
PID setPWMPIDY(setPWMYParams);
PID setAnglePIDX(setAngleXParams);
PID setAnglePIDY(setAngleYParams);
std::array<pidParams, 4> paramSet;

pidOutputs setAngleXOutputs;
pidOutputs setAngleYOutputs;
pidOutputs setPWMXOutputs;
pidOutputs setPWMYOutputs;

unsigned long startTime;

void printBinary16(uint16_t n);
unsigned long getTime(unsigned long startTime);

// Flag to indicate button was pressed (must be volatile)
volatile bool buttonPressed = false;

// Time tracking for debouncing
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 300;  // milliseconds

volatile bool auxButtonPressed = false;
volatile bool zeroButtonState = false;   // false = not armed, true = armed

// Interrupt Service Routine (ISR)
void IRAM_ATTR buttonISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > debounceDelay) {
    buttonPressed = true;
    lastDebounceTime = currentTime;
  }
}

void IRAM_ATTR auxButtonISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > debounceDelay) {
    auxButtonPressed = true;
    lastDebounceTime = currentTime;
  }
}

struct stateErrs {
  int positionErrorX;
  int positionErrorY;
  int angleErrorX;
  int angleErrorY;
};

stateErrs stateErrors;

// The function to run when button is pressed
void handleButtonPress() {
  //serial.println("Button was pressed!");
  setPWMPIDX.reset();
  setPWMPIDY.reset();
  setAnglePIDX.reset();
  setAnglePIDY.reset();
  stateErrors.positionErrorX = 0;
  stateErrors.positionErrorY = 0;
  ENC1.zero(); //Old zeroing button
  ENC2.zero();
  PEND1.zero();
  PEND2.zero();

// Toggle armed state and update BLUE status LED
zeroButtonState = !zeroButtonState;
digitalWrite(BLUE_LED, zeroButtonState ? HIGH : LOW);
}

void handleAuxButtonPress() {
  //setPWMPIDX.reset();
  //setPWMPIDY.reset();
  setAnglePIDX.reset();
  setAnglePIDY.reset();
  stateErrors.angleErrorX = 0;
  stateErrors.angleErrorY = 0;
  PEND1.zero();
  PEND2.zero();
}

// Telemetry Globals
SemaphoreHandle_t pidValsMutex;
bool pauseTesting = false;

//Telemetry variable names
const char* telemVars[] = {
  "carriageXPosition", "pendulumXAngle",
  "xAngleError", "xPWMp", "xPWMi", "xPWMd", "xPWMout", "xPWM",
  "pendulumYAngle",
  "xPositionError", "angleXp", "angleXi", "angleXd", "setAngleXOut"
};

float telemVals[14];

struct stateVars {
  int posX;
  int posY;
  int angleX;
  int angleY;
  int joystick_reading_x;
  int joystick_reading_y;
  float targetPosX; //RC: doubles since we need the resolution to be fine for the
  float targetPosY; //RC: joystick adjustment. Casted to int for PID calculations
};

stateVars stateVariables;
motorPWMs PWMOutputs;

volatile uint32_t loopTime;
volatile uint32_t loopWaitTime;

PL_Telemetry_ESP32 telemetry(
  telemVars,
  setAngleXParams,
  setAngleYParams,
  setPWMXParams,
  setPWMYParams
);

TaskHandle_t telemTask;

#define PWM2 19
#define DIR2 22
#define PWM1 21
#define DIR1 20

Driver DVR1(PWM1, DIR1);
Driver DVR2(PWM2, DIR2);

Move move(DVR1, DVR2, ENC1, ENC2);

void updateTelemetry() {
  telemVals[0] = stateVariables.posX;
  telemVals[1] = stateVariables.angleX;
  telemVals[2] = -stateErrors.angleErrorX;
  telemVals[3] = setPWMXOutputs.pOut;
  telemVals[4] = setPWMXOutputs.iOut;
  telemVals[5] = setPWMXOutputs.dOut;
  telemVals[6] = setPWMXOutputs.output;
  telemVals[7] = PWMOutputs.xPWM;
  telemVals[8] = stateVariables.angleY;
  telemVals[9] = stateErrors.positionErrorX;
  telemVals[10] = setAngleXOutputs.pOut;
  telemVals[11] = setAngleXOutputs.iOut;
  telemVals[12] = setAngleXOutputs.dOut;
  telemVals[13] = setAngleXOutputs.output;
}

void telemLoop(void *pvParameters){
  for(;;){
    uint32_t start_us = micros();

    if (xSemaphoreTake(pidValsMutex, portMAX_DELAY) == pdTRUE) {
      updateTelemetry();
      telemetry.sendSnapshot(telemVals, start_us);
      xSemaphoreGive(pidValsMutex);
    }

    uint32_t elapsed = (uint32_t)(micros() - start_us);

    if (elapsed >= LOOP_US) {
      overrun_count++;
    } else {
      while ((uint32_t)(micros() - start_us) < LOOP_US) {
        if(telemetry.pauseTesting()) {
          if (telemetry.updateGainVals()) {
            if(xSemaphoreTake(pidValsMutex, portMAX_DELAY) == pdTRUE) {
              setAnglePIDX.readNewGains(telemetry.setAngleXParams);
              setAnglePIDY.readNewGains(telemetry.setAngleYParams);
              setPWMPIDX.readNewGains(telemetry.setPWMXParams);
              setPWMPIDY.readNewGains(telemetry.setPWMYParams);
              xSemaphoreGive(pidValsMutex);
            }
          }
        }
      }
    }
  }
}

void setup() {

  SPI.begin(ENC_CLK, ENC_MISO, ENC_MOSI);

  Serial.begin(115200);
  telemetry.begin();
  pidValsMutex = xSemaphoreCreateMutex(); // Create mutex for errors

  pinMode(ZERO_BTN, INPUT_PULLUP);
  pinMode(AUX_BTN, INPUT_PULLUP);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(CONTROL_LOOP_PIN, OUTPUT);

  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(CONTROL_LOOP_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(ZERO_BTN), buttonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(AUX_BTN), auxButtonISR, FALLING);

  stateVariables.targetPosX = 0;
  stateVariables.targetPosY = 0;
  stateVariables.joystick_reading_x = 0;
  stateVariables.joystick_reading_y = 0;

  ENC1.begin();
  ENC2.begin();
  PEND1.begin();
  PEND2.begin();

  DVR1.begin();
  delay(1000);
  Serial.flush();

  DVR2.begin();
  delay(1000);
  Serial.flush();

  Serial.flush();

  xTaskCreatePinnedToCore(
    telemLoop,
    "Telemetry Loop",
    STACK_SIZE,
    NULL,
    TASK_PRIORITY,
    &telemTask,
    CORE_0
  );
}

void readState() { //140us empirically with scope at 1MHz clock speed
  stateVariables.angleX = -PEND1.getTotalAngle();
  stateVariables.angleY =  PEND2.getTotalAngle();
  stateVariables.posX = move.returnPosX();
  stateVariables.posY = move.returnPosY();
}

void updateTargetPos() {
  //RC: analogReads are the issue. TODO: bugfix
  //RC: It turns out GPIO 0, 2, 4, 12-15, 25-27 are on ADC2 which are shared with Wi-Fi/Bluetooth
  //RC: Unlike GPIO 32-39 which are on ADC1 and are always free
  //RC: First test change: turn WiFi off with WiFi.mode(WIFI_OFF) - CHANGE WORKS
  //RC: Alternatively, try jumping the connections to pins 34, 35 which are not GPIO (as discovered with the LEDs) but are ADC1 (untested)
  WiFi.mode(WIFI_OFF);
  stateVariables.joystick_reading_x = analogRead(MOVE_TARGET_POSX_PIN) - 2048 - JOYSTICK_OFFSET_X; // Get value between [0, 4095] and divide by 2
  stateVariables.joystick_reading_y = analogRead(MOVE_TARGET_POSY_PIN) - 2048 - JOYSTICK_OFFSET_Y; // Note that due to offset, min_value != -1*max_value
  
  if (abs(stateVariables.joystick_reading_x) > JOYSTICK_DEAD_ZONE) { //RC: Experiment with dead-zone value
    stateVariables.targetPosX += MOVE_TARGET_POSX_SCALE_FACTOR*stateVariables.joystick_reading_x; //RC: TODO: Find good scale factor (movement speed)
    stateVariables.targetPosX = constrain(stateVariables.targetPosX, -2750, 2750); //RC: TODO: replace all instances of dead zone magic numbers with constants
  }
  if (abs(stateVariables.joystick_reading_y) > JOYSTICK_DEAD_ZONE) {
    stateVariables.targetPosY += MOVE_TARGET_POSY_SCALE_FACTOR*stateVariables.joystick_reading_y; //RC: ""
    stateVariables.targetPosY = constrain(stateVariables.targetPosY, -4000, 4000); //RC: TODO: also consider limiting target to just shy of dead zone as it is impossible to control at dead zone exactly anyway
  }
}

void runControl(float dt, int controlCycle) {
  // position PID may run on slower loop time
  if (controlCycle == POS_UPDATE_CYCLES) {
    stateErrors.positionErrorX = (stateVariables.posX - (int)stateVariables.targetPosX);
    stateErrors.positionErrorY = ((int)stateVariables.targetPosY - stateVariables.posY);
  }

  setAngleXOutputs = setAnglePIDX.calculate(stateErrors.positionErrorX, dt);
  setAngleYOutputs = setAnglePIDY.calculate(stateErrors.positionErrorY, dt);

  stateErrors.angleErrorX = (setAngleXOutputs.output - stateVariables.angleX);
  stateErrors.angleErrorY = -(setAngleYOutputs.output - stateVariables.angleY);

  setPWMXOutputs = setPWMPIDX.calculate(stateErrors.angleErrorX, dt);
  setPWMYOutputs = setPWMPIDY.calculate(stateErrors.angleErrorY, dt);

  PWMOutputs = {setPWMXOutputs.output, setPWMYOutputs.output};
}

// Gantry-specific loop
void loop() {
  static uint32_t start_us = micros(); //RC: Initialize start time ONCE only

  if (micros() - start_us >= LOOP_US) {
    overrun_count++;
    // Serial.println("Loop overrun! Total overruns: " + String(overrun_count));
    digitalWrite(CONTROL_LOOP_PIN, !digitalRead(CONTROL_LOOP_PIN)); // Toggle pin to measure loop timing
    while(micros() - start_us > LOOP_US) {
      start_us += LOOP_US;
    }
  }
  while(micros() - start_us < LOOP_US) {
    readState();
  }
  start_us += LOOP_US;

  updateTargetPos();

  if(!zeroButtonState || telemetry.pauseTesting()) {
    move.moveXY(0, 0);
    if(!zeroButtonState) {
      setPWMPIDX.reset();
      setPWMPIDY.reset();
      setAnglePIDX.reset();
      setAnglePIDY.reset();
    }
  }
  else {
    if (xSemaphoreTake(pidValsMutex, portMAX_DELAY) == pdPASS) {

      runControl(dt, controlCycle);
      controlCycle++;
      if(controlCycle > POS_UPDATE_CYCLES) controlCycle = 1;

      // if (abs(PWMOutputs.xPWM) < X_DEADZONE) {
      //   PWMOutputs.xPWM = int(X_DEADZONE * std::tanh(PWMOutputs.xPWM/(float)(3)));
      // }

      // if (stateErrors.angleErrorY < 0) PWMOutputs.yPWM -= Y_DEADZONE;
      // else if (stateErrors.angleErrorY > 0) PWMOutputs.yPWM += Y_DEADZONE;

      PWMOutputs.xPWM = constrain(PWMOutputs.xPWM, -255, 255);
      PWMOutputs.yPWM = constrain(PWMOutputs.yPWM, -255, 255);

      if (abs(stateVariables.posX) < 2750 && abs(stateVariables.posY) < 4000 &&
          abs(stateVariables.angleX) < 1400 && abs(stateVariables.angleY) < 1500) {
        move.moveXY(PWMOutputs.xPWM, PWMOutputs.yPWM);
        digitalWrite(RED_LED, LOW);
      } else {
        PWMOutputs.xPWM = 0;
        PWMOutputs.yPWM = 0;
        move.moveXY(0, 0);
        digitalWrite(RED_LED, HIGH);
      }

      xSemaphoreGive(pidValsMutex);
    }
  }

  if (buttonPressed) {
    buttonPressed = false;
    if (digitalRead(ZERO_BTN) == LOW) {
      handleButtonPress();
    }
  }

  if (auxButtonPressed) {
    auxButtonPressed = false;
    if (digitalRead(AUX_BTN) == LOW) {
      handleAuxButtonPress();
    }
  }
}