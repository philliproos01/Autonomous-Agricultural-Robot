#include <WiFi.h>
#include <WebServer.h>
#include <iostream>
#include <string>
#include <ctype.h>

// obviously change this to your own SSID and password
#define ssid "Tufts_Robot"
#define pass ""
#define server "pcr.bounceme.net"

struct coord {
    double latitude;
    double longitude;
};

WiFiClient client;

void setup() {
  int status = WL_IDLE_STATUS;
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
    delay(1000);
  }
  printWifiStatus();
  if (client.connect(server, 80)) {
    Serial.println("connected to server");
    // Make a HTTP request:
    client.println("GET /test/GUI/test/robot.txt HTTP/1.1");
    client.println("Host: pcr.bounceme.net");
    client.println("Connection: close");
    client.println();
  }
 
}

void loop() {
  int i = 0;
  coord lat_longs[5] = { 0 };

  while (client.available()) {
    String line = client.readStringUntil('\n');
    Serial.println(line);
    // if first character is not numeric, skip this line, it's the HTTP header or fluff before our coordinates
    if (line.length() == 0 || !isdigit(line[0])) {
      continue;
    }
    //char* line_cstr = line.c_str();
    char *line_cstr = new char[line.length()+1];
    strcpy(line_cstr, line.c_str());
    char* lat = strtok(line_cstr, ", ");
    char* lng = strtok(NULL, ", ");
    if (i < 5) {
      lat_longs[i++] = (coord) {
        .latitude = strtod(lat, NULL),
        .longitude = strtod(lng, NULL),
      };
    }
    delete[] line_cstr;
  }

  for (int j = 0; j < i; j++) {
      Serial.print("lat long #");
      Serial.print(j+1);
      Serial.print(": ");
      Serial.print(lat_longs[j].latitude, 10);
      Serial.print(", ");
      Serial.println(lat_longs[j].longitude, 10);
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
