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
  });

  Serial.flush();
}

// Gantry-specific loop
void loop() {
  Serial.println(receiverESP.data.int_message_3);
}

#elif CURRENT_ESP == ESP_PENDULUM

// Pendulum-specific setup
#define SENDER_PIN 14
ESPNowSender senderESP;

void setup() {
  senderESP.setUp();
  
  ENC1.begin();  
  ENC2.begin();

  pinMode(SENDER_PIN, OUTPUT);
  digitalWrite(SENDER_PIN, LOW)
}

// Pendulum-specific loop
void loop() {
  // Pendulum-specific control code
  // This will handle sensor readings and send data to gantry
  digitalWrite(SENDER_PIN, !digitalRead(SENDER_PIN));

  int angle1 = ENC1.getTotalAngle();
  //delay(1);

  int angle2 = ENC2.getTotalAngle();
  //delay(1);

  digitalWrite(SENDER_PIN, HIGH);
  senderESP.sendMessage(, angle1, angle2);
  digitalWrite(SENDER_PIN, LOW);
}
#else
#error "Please select either ESP_GANTRY or ESP_PENDULUM for CURRENT_ESP"
#endif