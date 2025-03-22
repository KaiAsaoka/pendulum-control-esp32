#include <WiFi.h>
#include <esp_wifi.h>


// Code to get MAC address of ESP32.

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void begin(){
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
<<<<<<< HEAD
=======
  WiFi.begin();
>>>>>>> 3c1d6f40ec4bb8f38aeaf9bc19badc5d17594224

  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
}