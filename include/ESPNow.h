#ifndef ESP_NOW_H
#define ESP_NOW_H

#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message{
    char message[32];
    int int_message_1;
    int int_message_2;
} struct_message;

class ESPNowSender{
    public:
        ESPNowSender(uint8_t broadcastAddress[]); //Address of the receiver ESP32.
        uint8_t broadcastAddress[6];  // Changed from pointer to array
        struct_message data;
        esp_now_peer_info_t peerInfo;
    
        static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
        void setUp();
        void sendMessage(const char* message, int int_message_1, int int_message_2);
};

class ESPNowReceiver{
    public:
        ESPNowReceiver();
        struct_message data;
        void onDataRecv(const uint8_t*mac, const uint8_t *incomingData, int len);
        void setUp();
};

#endif
