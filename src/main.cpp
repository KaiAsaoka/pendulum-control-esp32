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

// Choose which ESP to compile for
#define CURRENT_ESP ESP_GANTRY // Change this to ESP_PENDULUM when uploading to the pendulum ESP

// Gantry-specific setup
#if CURRENT_ESP == ESP_GANTRY
ESPNowReceiver receiverESP;
#define RECIEVER_PIN 7

void setup() {
  Serial.begin(115200);
  pinMode(RECIEVER_PIN, OUTPUT);
  digitalWrite(RECIEVER_PIN, LOW);

  // Initialize ESPNow communication
  receiverESP.setUp();
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    receiverESP.onDataRecv(mac, data, len);
    digitalWrite(RECIEVER_PIN, !digitalRead(RECIEVER_PIN));
    Serial.println(receiverESP.data.int_message_3);
  });

  Serial.flush();
}

// Gantry-specific loop
void loop() {
}

#elif CURRENT_ESP == ESP_PENDULUM

// Pendulum-specific setup
#define SENDER_PIN 14

uint8_t broadcastAddress[] = {0x64, 0xb7, 0x08, 0x9c, 0x5b, 0xb0};
ESPNowSender senderESP(broadcastAddress);

#define ENC_MISO 26    // Encoder data output (MISO)
#define ENC_CLK  25    // SPI clock (SCK)
#define ENC_CS1  32    // Chip Select (active LOW)
#define ENC_CS2  33    // Chip Select (active LOW)
#define ENC_MOSI 9    // MOSI pin for encoder communication

Encoder ENC1(ENC_MISO, ENC_CLK, ENC_CS1, ENC_MOSI);
Encoder ENC2(ENC_MISO, ENC_CLK, ENC_CS2, ENC_MOSI);

int count = 0;

void setup() {
  senderESP.setUp();
  
  ENC1.begin();  
  ENC2.begin();

  pinMode(SENDER_PIN, OUTPUT);
  digitalWrite(SENDER_PIN, LOW);
}

// Pendulum-specific loop
void loop() {
  // Pendulum-specific control code
  // This will handle sensor readings and send data to gantry
  digitalWrite(SENDER_PIN, !digitalRead(SENDER_PIN));

  int angle1 = ENC1.getTotalAngle();
  // delay(1);

  int angle2 = ENC2.getTotalAngle();
  //delay(1);

  digitalWrite(SENDER_PIN, HIGH);
  senderESP.sendMessage(angle1, angle2, count);
  digitalWrite(SENDER_PIN, LOW);

  count++;
  if (count > 100) {
    count = 0;
  }

  delay(1);
}
#else
#error "Please select either ESP_GANTRY or ESP_PENDULUM for CURRENT_ESP"
#endif