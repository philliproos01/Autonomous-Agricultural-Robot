#include <Arduino.h>
#include <WiFi.h>

#define WIFI_SSID "Tufts_Robot"
#define WIFI_PASSWORD ""

void setup() {
  Serial.begin(115200);
  

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("starting");
}

bool isConnected = false;

void loop() {
  if (WiFi.status() == WL_CONNECTED && !isConnected) {
    Serial.println("Connected");
    
    isConnected = true;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
    
    delay(1000);
    isConnected = false;
  }
}
