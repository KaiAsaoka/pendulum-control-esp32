#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESPNow.h>

// #define SENDER_PIN 14
// #define RECIEVER_PIN 7

ESPNowSender::ESPNowSender(uint8_t broadcastAddress[]){
    memcpy(this->broadcastAddress, broadcastAddress, 6);
}

void ESPNowSender::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
};

void ESPNowSender::setUp(){
    // pinMode(SENDER_PIN, OUTPUT);
    // digitalWrite(SENDER_PIN, LOW);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }  
      
    esp_now_register_send_cb(ESPNowSender::onDataSent);
    memcpy(this->peerInfo.peer_addr, this->broadcastAddress, 6);
    this->peerInfo.channel = 0;
    this->peerInfo.encrypt = false;

    if(esp_now_add_peer(&this->peerInfo)!= ESP_OK){
        Serial.println("Failed to add peer.");
        return;
    }
    Serial.println("Added peer successfully!");
};

void ESPNowSender::sendMessage(int int_message_1, int int_message_2, int int_message_3){

    this->data.int_message_1 = int_message_1;
    this->data.int_message_2 = int_message_2;
    this->data.int_message_3 = int_message_3;
    
    //Send message!
    esp_err_t result = esp_now_send(this->broadcastAddress, (uint8_t *) &this->data, sizeof(this->data));
    // digitalWrite(SENDER_PIN, !digitalRead(SENDER_PIN));
    // if (result == ESP_OK) {
    //     Serial.println("Sent with success.");
    // }
    // else {
    //     Serial.println("Error sending the data.");
    // }
};

ESPNowReceiver::ESPNowReceiver(){
    //Nothing needed.
};

void ESPNowReceiver::setUp(){
    // pinMode(RECIEVER_PIN, OUTPUT);
    // digitalWrite(RECIEVER_PIN, LOW);
    WiFi.mode(WIFI_STA);

    //Initialize ESP-NOW.
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
};

void ESPNowReceiver::onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
    memcpy(&this->data, incomingData, sizeof(this->data));
    // digitalWrite(RECIEVER_PIN, !digitalRead(RECIEVER_PIN));
    // Serial.print("Bytes received: ");
    // Serial.println(len);

    // Serial.print("Message: \n");
    // Serial.println(this->data.message);
    // Serial.println(this->data.int_message_1);
    // Serial.println(this->data.int_message_2);
    //Read messages by placing esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv)) in the setup of main.cpp after setting up receiver object.
};