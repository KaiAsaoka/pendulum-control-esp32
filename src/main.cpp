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

// Define 10 ms loop timing
constexpr uint32_t LOOP_US = 10000;     // 10 ms
constexpr uint32_t MAX_GANTRY_LOOP_US = LOOP_US;
static volatile uint32_t overrun_count = 0;
int controlCycle = 0;

// Choose which ESP to compile for
#define CURRENT_ESP ESP_PENDULUM // Change this to ESP_PENDULUM when uploading to the pendulum ESP

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

#if CURRENT_ESP == ESP_GANTRY
#define ZERO_BTN 37
#define AUX_BTN 38        // Extra safety / aux button
#define BLUE_LED 10       // "Armed" status LED
#define RED_LED 5         // Out-of-bounds LED
#endif


#define TARGET_POSX 0
#define TARGET_POSY 0

#define X_DEADZONE 0
#define Y_DEADZONE 0

#define STACK_SIZE 10000
#define TASK_PRIORITY 0
#define CORE_0 0
#define CORE_1 1

Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS1, ENC_MOSI);
Encoder ENC2(ENC_MISO, ENC_CLK, ENC_CS2, ENC_MOSI);

pidParams setAngleXParams = {0, 0, 0, 0, 0};
// {45, 50, !!0.16!!, 0, 125000000}
pidParams setAngleYParams = {0, 0, 0, 0, 0};
// {15, 150, 0.5, 0, 55555555}
pidParams setPWMXParams = {0, 0, 0, 0, 0};
// {0, 0, 0, 750, 1000}
pidParams setPWMYParams = {0, 0, 0, 0, 0};
// {0, 0, 0, 750, 1000}

PID setPWMPIDX(setPWMXParams);
PID setPWMPIDY(setPWMYParams);
PID setAnglePIDX(setAngleXParams);
PID setAnglePIDY(setAngleYParams);
std::array<pidParams, 4> paramSet;

pidOutputs setAngleXOutputs;
pidOutputs setAngleYOutputs;
pidOutputs setPWMXOutputs;
pidOutputs setPWMYOutputs;

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
const unsigned long debounceDelay = 300;  // milliseconds

#if CURRENT_ESP == ESP_GANTRY
volatile bool auxButtonPressed = false;
volatile bool zeroButtonState = false;   // false = not armed, true = armed
#endif

int buttonHysterisisTestVar1 = 0; // To test button hysterisis (remove when done)
int buttonHysterisisTestVar2 = 0; // To test button hysterisis (remove when done)
// Interrupt Service Routine (ISR)
void IRAM_ATTR buttonISR() {
  unsigned long currentTime = millis();
  buttonHysterisisTestVar1++; // Counts # of times entered ISR function
  if (currentTime - lastDebounceTime > debounceDelay) {
    buttonPressed = true;
    lastDebounceTime = currentTime;
    buttonHysterisisTestVar2++; // Counts # of times button press registered
  }
}

#if CURRENT_ESP == ESP_GANTRY
void IRAM_ATTR auxButtonISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > debounceDelay) {
    auxButtonPressed = true;
    lastDebounceTime = currentTime;
  }
}
#endif

struct stateErrs {
  int positionErrorX;
  int positionErrorY;
  int angleErrorX;
  int angleErrorY;
};

stateErrs stateErrors;

// The function to run when button is pressed
// The function to run when button is pressed
void handleButtonPress() {
  //Serial.println("Button was pressed!");
  setPWMPIDX.reset();
  setPWMPIDY.reset();
  setAnglePIDX.reset();
  setAnglePIDY.reset();
  stateErrors.positionErrorX = 0;
  stateErrors.positionErrorY = 0;
  ENC1.zero(); //Old zeroing button
  ENC2.zero();
  //Serial.println("ISR Loops: " + String(buttonHysterisisTestVar1));
  //Serial.println("Registered Presses: " + String(buttonHysterisisTestVar2));

#if CURRENT_ESP == ESP_GANTRY
  // Toggle armed state and update BLUE status LED
  zeroButtonState = !zeroButtonState;
  digitalWrite(BLUE_LED, zeroButtonState ? HIGH : LOW);
#endif
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

// Gantry-specific setup
#if CURRENT_ESP == ESP_GANTRY

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
//   // //Serial.printf("Telemetry loop running on core: %d\n", xPortGetCoreID());
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
//       //Serial.println("Telemetry Overtime!");
//     } else {
//       // Busy --> wait until full 10 ms period has elapsed
//       while ((uint32_t)(micros() - start_us) < LOOP_US) {
//         if(telemetry.pauseTesting()) {
//           if (telemetry.updateGainVals()) {
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
}


void setup() {
  Serial.begin(115200);
  //telemetry.begin();
  pidValsMutex = xSemaphoreCreateMutex(); // Create mutex for errors
  //Serial.println("Gantry ESP32 Starting...");

  pinMode(ZERO_BTN, INPUT_PULLUP);          // or INPUT if using GPIO37 with external pull-up
  pinMode(AUX_BTN, INPUT_PULLUP);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(BLUE_LED, LOW);  // Start unarmed
  digitalWrite(RED_LED, LOW);   // No fault initially
    
  // Attach interrupts (FALLING for normally-open button with pull-up resistor)
  attachInterrupt(digitalPinToInterrupt(ZERO_BTN), buttonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(AUX_BTN), auxButtonISR, FALLING);
  
  //Serial.println("Button interrupt initialized");

  ENC1.begin();
  //Serial.println("Encoder 1 initialized (Gantry)");

  ENC2.begin();
  //Serial.println("Encoder 2 initialized (Gantry)");

  // Initialize drivers
  DVR1.begin();
  delay(1000);
  //Serial.println("Driver 1 initialized");
  Serial.flush();

  DVR2.begin();
  delay(1000);
  //Serial.println("Driver 2 initialized");
  Serial.flush();

  // Initialize ESPNow communication
  receiverESP.setUp();
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    receiverESP.onDataRecv(mac, data, len);
  });

  //Serial.println("Gantry setup complete!");
  Serial.flush();

  // ESP32 Should make loop on core 1 anyways, but just to be sure
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

void readState() {
  stateVariables.angleX = -receiverESP.data.int_message_1;
  stateVariables.angleY = receiverESP.data.int_message_2;
  stateVariables.posX = move.returnPosX();
  stateVariables.posY = move.returnPosY();
}

void runControl(float dt, int controlCycle) {
  // Calculate positional error 
  // position PID should only occur every 100ms
  if (controlCycle == 10) {
    stateErrors.positionErrorX = (stateVariables.posX-TARGET_POSX);
    stateErrors.positionErrorY = (TARGET_POSY - stateVariables.posY);
  }
  // stateErrors.positionErrorX = 0;
  // stateErrors.positionErrorY = 0;

  // Calculate desired angle
  setAngleXOutputs = setAnglePIDX.calculate(stateErrors.positionErrorX, dt);
  setAngleYOutputs = setAnglePIDY.calculate(stateErrors.positionErrorY, dt);

  // Calculate angular error
  // Edited the angleErrorX to make the pwm go the right way
  stateErrors.angleErrorX = (setAngleXOutputs.output - stateVariables.angleX);
  stateErrors.angleErrorY = -(setAngleYOutputs.output - stateVariables.angleY);

  // Calculate desired PWMs
  setPWMXOutputs = setPWMPIDX.calculate(stateErrors.angleErrorX, dt);
  setPWMYOutputs = setPWMPIDY.calculate(stateErrors.angleErrorY, dt);

  PWMOutputs = {setPWMXOutputs.output, setPWMYOutputs.output};
}


// Gantry-specific loop
void loop() {

  const float dt = 0.01f;

  uint32_t start_us = micros();
  loopTime = start_us;

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
    readState();

    //xSemaphoreTake(pidValsMutex, portMAX_DELAY) == pdPASS
    if (true) {
      
      runControl(dt, controlCycle);
      controlCycle++;
      if(controlCycle > 10) controlCycle = 1;
        // Deadzones
      if (stateErrors.angleErrorX < 0) PWMOutputs.xPWM -= X_DEADZONE;
      else if (stateErrors.angleErrorX > 0) PWMOutputs.xPWM += X_DEADZONE;

      if (stateErrors.angleErrorY < 0) PWMOutputs.yPWM -= Y_DEADZONE;
      else if (stateErrors.angleErrorY > 0) PWMOutputs.yPWM += Y_DEADZONE;

      PWMOutputs.xPWM = constrain(PWMOutputs.xPWM, -255, 255);
      PWMOutputs.yPWM = constrain(PWMOutputs.yPWM, -255, 255);

      // Safety window + command
      if (abs(stateVariables.posX) < 2750 && abs(stateVariables.posY) < 4000 && 
          abs(stateVariables.angleX) < 1400 && abs(stateVariables.angleY) < 1500) {
        //move.moveXY(10, 0);
        move.moveXY(PWMOutputs.xPWM, PWMOutputs.yPWM);
        digitalWrite(RED_LED, LOW); // Turn out-of-bounds LED off
      } else {
        PWMOutputs.xPWM = 0;
        PWMOutputs.yPWM = 0;
        move.moveXY(0, 0);
        digitalWrite(RED_LED, HIGH); // Turn out-of-bounds LED on
        // //Serial.print("Out of bounds!");
      }
      xSemaphoreGive(pidValsMutex);
    }
  }

  // Button handling block stays as-is
  if (buttonPressed) {
    handleButtonPress();
    buttonPressed = false;
  }

  // Measure elapsed time and wait if needed
  uint32_t current_time_us = micros();
  uint32_t elapsed_time_us = current_time_us - start_us;

  if (elapsed_time_us >= MAX_GANTRY_LOOP_US) {
    overrun_count++;
    loopWaitTime = 0;
    //Serial.printf("Overtime (Gantry): %d us\n", elapsed_time_us);
  } else {
    loopWaitTime = MAX_GANTRY_LOOP_US - elapsed_time_us;
    while (elapsed_time_us < MAX_GANTRY_LOOP_US) {
      // Busy wait
      current_time_us = micros();
      elapsed_time_us = current_time_us - start_us;
      ENC1.getTotalAngle();
      ENC2.getTotalAngle();
    }
  }
}

#elif CURRENT_ESP == ESP_PENDULUM

// Pendulum-specific setup
#define ZERO_BTN 37

void setup() {
  Serial.begin(115200);
  //Serial.println("Pendulum ESP32 Starting...");
  
  senderESP.setUp();
  
  ENC1.begin();
  //Serial.println("Encoder 1 initialized (Pendulum)");
  
  ENC2.begin();
  //Serial.println("Encoder 2 initialized (Pendulum)");

  pinMode(ZERO_BTN, INPUT_PULLUP);
    
  // Attach interrupt (FALLING for normally-open button with pull-up resistor)
  attachInterrupt(digitalPinToInterrupt(ZERO_BTN), buttonISR, FALLING);
  
  //Serial.println("Button interrupt initialized");

  //Serial.println("Pendulum setup complete!");
  Serial.flush();
}

// Pendulum-specific loop
void loop() {
  // Pendulum-specific control code
  // This will handle sensor readings and send data to gantry
  
  int angle1 = ENC1.getTotalAngle();
  //delay(1);
  // Serial.print("E1: ");
  // Serial.print(angle1);

  int angle2 = ENC2.getTotalAngle();
  //delay(1);
  // Serial.print(", E2: ");
  // Serial.print(angle2);

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