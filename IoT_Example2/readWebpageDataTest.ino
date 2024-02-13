#include <WiFiNINA.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>


#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

#define sensorPin A5

char ssid[] = "TMOBILE-3422";
char pass[] = "chip018big6268hits";

int status = WL_IDLE_STATUS;

//char server[] = "www.google.com";
char server[] = "pcr.bounceme.net";

String postData;
String postVariable = "temp=";

String postData1;
String postVariable1 = "ppm=";

String postData2;
String postVariable2 = "pressure=";

String postData3;
String postVariable3 = "tvoc=";



WiFiClient client;

void setup() {
  Serial.begin(9600);
  unsigned status_sensor;
  
  while (!Serial) {
     delay(100); // wait for serial port to connect. Needed for native USB port only
  }
  
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  IPAddress gateway = WiFi.gatewayIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  printWifiStatus();
  if (client.connect(server, 80)) {
    Serial.println("connected to server");
    // Make a HTTP request:
    client.println("GET /test/tempC.html HTTP/1.1");
    client.println("Host: pcr.bounceme.net");
    client.println("Connection: close");
    client.println();
    
  }


  
}

void loop() {
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  // if the server's disconnected, stop the client:
  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting from server.");
    client.stop();

    // do nothing forevermore:
    while (true);
  }


}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
