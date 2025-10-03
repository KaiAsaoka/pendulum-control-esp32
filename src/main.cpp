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
#define CURRENT_ESP ESP_GANTRY// Change this to ESP_\PENDULUM when uploading to the pendulum ESP

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
#define pendKIx 0.018
#define pendKDx 0 

#define pendlpfx 0
#define pendintcutoffx 1000 / 0.018

#define pendKPy 0.015
#define pendKIy 0.018
#define pendKDy 0

#define pendlpfy 0
#define pendintcutoffy 2000 / 0.016

#define ganKPx 0.0030  // 0.05
#define ganKIx 0.00000
#define ganKDx 0.1100

#define ganlpfx 0.75
#define ganintcutoffx 5

#define ganKPy 0.0055  // 0.05
#define ganKIy 0.00000
#define ganKDy 0.0300

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

struct movePacket {
    int x_pwm;
    bool x_dir;
    int y_pwm;
    bool y_dir;
};

void moveTest(int x_pwm, bool x_dir, int y_pwm, bool y_dir, int waitTime) {
    move.moveXY(x_pwm, x_dir, y_pwm, y_dir);
    move.brake();

    int posX = move.returnPosX();
    int posY = move.returnPosY();

    Serial.print("Move outwards: (");
    Serial.print(posX);
    Serial.print(",");
    Serial.print(posY);
    Serial.println(")");
    delay(waitTime);

    move.moveXY(x_pwm, !x_dir, y_pwm, !y_dir);
    move.brake();

    posX = move.returnPosX();
    posY = move.returnPosY();

    Serial.print("Move inwards: (");
    Serial.print(posX);
    Serial.print(",");
    Serial.print(posY);
    Serial.println(")");
    delay(waitTime);
}

void loop() {
    bool validPWM = false;
    int waitTime = 2000; //ms
    int baseMotorPWM = 0;

    while (!validPWM) {
        Serial.println("Input base PWM Speed (0 < int < 50): ");
        while (Serial.available() == 0) {}

        baseMotorPWM = Serial.parseInt();

        if (baseMotorPWM <= 0 || baseMotorPWM > 50) {
            Serial.println("Invalid Input!");
        }
        else {
            Serial.print("Speeds: ");
            Serial.print(baseMotorPWM);
            Serial.print(baseMotorPWM * 2);
            Serial.print(baseMotorPWM * 3);
            Serial.print(baseMotorPWM * 4);
            Serial.println(baseMotorPWM * 5);
            Serial.println("Beginning motion test...");
            validPWM = true;
        }
    }

    Serial.flush();

    movePacket y;
    movePacket plusXY;
    movePacket x;
    movePacket plusXMinusY;

    y.x_pwm = 0;
    y.x_dir = 0;
    y.y_pwm = baseMotorPWM;
    y.y_dir = 1;

    plusXY.x_pwm = baseMotorPWM;
    plusXY.x_dir = 1;
    plusXY.y_pwm = baseMotorPWM;
    plusXY.y_dir = 1;

    x.x_pwm = baseMotorPWM;
    x.x_dir = 1;
    x.y_pwm = 0;
    x.y_dir = 1;

    plusXMinusY.x_pwm = baseMotorPWM;
    plusXMinusY.x_dir = 1;
    plusXMinusY.y_pwm = baseMotorPWM;
    plusXMinusY.y_dir = 0;

  std::array<movePacket, 4> directions = {y, plusXY, x, plusXMinusY};

  for (int i = 1; i < 6; i ++) {
    for (auto& directionTest : directions) {
        moveTest(directionTest.x_pwm * i, directionTest.x_dir, directionTest.y_pwm * i, directionTest.y_dir, waitTime);
        moveTest(directionTest.x_pwm * i, !directionTest.x_dir, directionTest.y_pwm * i, !directionTest.y_dir, waitTime);
    }
  }

  validPWM = false;

   // Check if button was pressed
  if (buttonPressed) {
    handleButtonPress();
    buttonPressed = false;  // Reset the flag
  }


//   // Below this write position stuff
//   int xVel = 0;
//   int yVel = 0;
//   bool xDir = 0;
//   bool yDir = 0;
//   int xPos = 0;
//   int yPos = 0;
//   int xError = 0;
//   int yError = 0;
//   int netError = 0;

//   bool validPos = false;

//   Serial.println("Input PWM Speed (0 < int < 255): ");

//   while (Serial.available() == 0) {}

//   int motorPWM = Serial.parseInt();

//   if (motorPWM <= 0 || motorPWM > 255) {
//     Serial.println("Invalid Input!");
//   }
//   else {
//     Serial.println("Setting PWM speed...");
//   }

//   while (!validPos) {
//     Serial.println("Input position X,Y ( -450 < X < 450, -600 < Y < 600):");
//     while (Serial.available() == 0) {}

//     String posInput = Serial.readStringUntil('\n');
//     posInput.trim();

//     int comma = posInput.indexOf(",");
//     int xPos = int(posInput.substring(0, comma).toInt());
//     int yPos = int(posInput.substring(comma, -1).toInt());

//     if (-450 < xPos < 450 && -600 < yPos < 600) {
//         validPos == true;
//         Serial.println("Moving...");

//         xError = xPos - move.returnPosX();
//         yError = yPos - move.returnPosY();
//         netError = abs(xError) + abs(yError);

//         xDir = (xError >= 0);
//         yDir = (yError >= 0);
//     }
//     else {
//         Serial.println("Invalid Position!");
//     }
//  }

//  while(netError >= 15) {
//     if (abs(xError) < 5) {
//         xVel = 0;
//     }
//     else {
//         xVel = motorPWM;
//     }
//     if (abs(yError) < 5) {
//         yVel = 0;
//     }
//     else {
//         yVel = motorPWM;
//     }

//     move.moveXY(xVel, xDir, yVel, yDir);

//     xError = xPos - move.returnPosX();
//     yError = yPos - move.returnPosY();
//     netError = abs(xError) + abs(yError);

//     xDir = (xError >= 0);
//     yDir = (yError >= 0);
//  }
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
